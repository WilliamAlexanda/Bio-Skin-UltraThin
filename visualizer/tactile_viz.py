#!/usr/bin/env python3
"""
Ultra-thin tactile sensor visualizer.

  32 piezoresistive taxels  : drawn on a replica of the PCB, colour = pressure
                              (4095 no load = light yellow -> green -> orange -> red)
  MLX90393 magnetometer     : arrow from U3 pointing in the shear direction

Data comes from the ESP32-S3 firmware in ../tactile_sensor. On connect the
visualizer sends 'b' so the firmware streams binary packets at 100 Hz; with old
firmware it falls back to parsing the 5 Hz text printout.

MLX zero: whenever the four taxels around U3 (15, 16, 17, 18) are all at full
scale (no load) the current field becomes the new origin and the arrow is held
at the centre. While any of them is pressed the origin is frozen.

    .venv/bin/python tactile_viz.py            # pick the port in the window
    .venv/bin/python tactile_viz.py --demo     # simulated data, no hardware
"""
import argparse
import json
import math
import os
import re
import struct
import sys
import threading
import time
from collections import deque

import serial
import serial.tools.list_ports
from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import QPointF, QRectF, Qt

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, "viz_config.json")
BAUD = 115200
DEMO_PORT = "演示模式（无硬件）"

# ---------------------------------------------------------------------------
# PCB layout, traced from the board render (units: render pixels, ~48 per mm)
# ---------------------------------------------------------------------------
BOARD_W, BOARD_H = 1480, 1215
PAD_W, PAD_H = 118, 56
COLS = [100, 340, 605, 855, 1110, 1365]

TAXELS = {}                                     # label -> pad centre
for i, lab in enumerate(range(27, 33)):
    TAXELS[lab] = (COLS[i], 105)
for i, lab in enumerate(range(26, 20, -1)):
    TAXELS[lab] = (COLS[i], 330)
TAXELS.update({13: (COLS[0], 625), 14: (COLS[1], 625), 19: (COLS[4], 625), 20: (COLS[5], 625),
               15: (520, 530), 16: (520, 735), 17: (912, 530), 18: (912, 735)})
for i, lab in enumerate(range(12, 6, -1)):       # second row from the bottom: 7-12 run right to left
    TAXELS[lab] = (COLS[i], 890)
for i, lab in enumerate(range(1, 7)):
    TAXELS[lab] = (COLS[i], 1090)

CHIP_CENTER = (716, 632)
CHIP_SIZE = 150
HOLES = [(83, 495, 46), (1390, 790, 48)]
NOTCH = [(1190, 20), (1460, 25), (1457, 62), (1292, 68), (1288, 135)]
SLOT = (680, 1168, 310, 36)

# label -> (mux index, channel): red 1-16 = MUX1 CH0-15, 17-32 = MUX2 CH0-15
CHANNEL_MAP = {lab: ((lab - 1) // 16, (lab - 1) % 16) for lab in range(1, 33)}
ZERO_GATE_LABELS = (15, 16, 17, 18)


def adc_index(label):
    m, c = CHANNEL_MAP[label]
    return m * 16 + c


# light yellow -> green -> orange -> red, by pressure 0..1
COLOR_STOPS = [(0.00, (255, 241, 186)),
               (0.35, (92, 206, 108)),
               (0.68, (255, 150, 40)),
               (1.00, (229, 45, 58))]


def pressure_color(p):
    """Interpolated in HSV (shortest hue path), so green -> orange passes through
    yellow-green instead of turning grey/olive."""
    p = min(max(p, 0.0), 1.0)
    for (p0, c0), (p1, c1) in zip(COLOR_STOPS, COLOR_STOPS[1:]):
        if p <= p1:
            a, b = QtGui.QColor(*c0), QtGui.QColor(*c1)
            k = (p - p0) / (p1 - p0)
            h0, h1 = a.hsvHueF(), b.hsvHueF()
            dh = (h1 - h0 + 0.5) % 1.0 - 0.5
            return QtGui.QColor.fromHsvF((h0 + dh * k) % 1.0,
                                         a.hsvSaturationF() + (b.hsvSaturationF() - a.hsvSaturationF()) * k,
                                         a.valueF() + (b.valueF() - a.valueF()) * k)
    return QtGui.QColor(*COLOR_STOPS[-1][1])


# ---------------------------------------------------------------------------
# Settings
# ---------------------------------------------------------------------------
DEFAULT_CFG = {
    "noload_adc": 4050,     # >= this counts as no load (full scale)
    "full_adc": 1000,       # <= this is drawn as full red
    "arrow_full_ut": 500.0, # shear |dXY| that gives a full-length arrow
    "deadband_ut": 20.0,    # smaller shear is treated as zero
    "rotation": 270,        # MLX X/Y -> screen: rotate (deg, counter-clockwise; 270 = 90 deg clockwise) ...
    "flip_x": False,        # ... after flipping
    "flip_y": False,
    "show_values": True,
    "show_text": False,     # raw data text panel under the PCB
    "port": "",
    # direction calibration from the wizard; overrides rotation/flip. Default =
    # calibration of the MLX90393 board on 2026-10-25 ("清除校准" falls back to rotation/flip)
    "dir_cal": {
        "kx": 0.5607, "ky": -0.2937,
        "M": [[0.0871, 0.9849], [-0.6201, 0.1432]],
        "err_deg": 35.4,
    },
}


def load_config():
    cfg = dict(DEFAULT_CFG)
    try:
        with open(CONFIG_PATH, encoding="utf-8") as f:
            cfg.update({k: v for k, v in json.load(f).items() if k in DEFAULT_CFG})
    except (OSError, ValueError):
        pass
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
    except OSError:
        pass


# ---------------------------------------------------------------------------
# Data sources. Each frame: (t, adc[32] indexed mux*16+ch, (x, y, z, T) or None)
# ---------------------------------------------------------------------------
PKT_SYNC = b"\xA5\x5A"
PKT_FMT = "<2sBHB32H4fB"            # must match stream_pkt_t in tactile_sensor/main/main.c
PKT_SIZE = struct.calcsize(PKT_FMT)
PKT_VERSION = 1
MLX_RE = re.compile(r"MLX\s+X=\s*(-?[\d.]+)\s+Y=\s*(-?[\d.]+)\s+Z=\s*(-?[\d.]+).*T=\s*(-?[\d.]+)")


class SerialSource(threading.Thread):
    def __init__(self, port, sink):
        super().__init__(daemon=True)
        self.port = port
        self.sink = sink
        self.stop_evt = threading.Event()
        self.error = None
        self.binary = False
        self.lost = 0           # packets missing according to seq
        self.bad = 0            # checksum errors
        self._text = bytearray()
        self._text_adc = [4095] * 32
        self._text_mlx = None
        self._last_seq = None

    def stop(self):
        self.stop_evt.set()

    def run(self):
        try:
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = BAUD
            ser.timeout = 0.02
            ser.dtr = False         # do not trigger the auto-reset circuit
            ser.rts = False
            ser.open()
        except (serial.SerialException, OSError) as e:
            self.error = str(e)
            return

        buf = bytearray()
        last_bin = last_cmd = 0.0
        try:
            while not self.stop_evt.is_set():
                now = time.monotonic()
                self.binary = now - last_bin < 1.0
                if not self.binary and now - last_cmd > 0.5:
                    ser.write(b"b")         # ask for binary stream (again after a reset)
                    last_cmd = now
                buf += ser.read(ser.in_waiting or 1)

                while True:
                    i = buf.find(PKT_SYNC)
                    if i < 0:
                        keep = 1 if buf.endswith(PKT_SYNC[:1]) else 0
                        self._feed_text(buf[:len(buf) - keep])
                        del buf[:len(buf) - keep]
                        break
                    if i > 0:
                        self._feed_text(buf[:i])
                        del buf[:i]
                    if len(buf) < PKT_SIZE:
                        break
                    pkt = bytes(buf[:PKT_SIZE])
                    if pkt[2] == PKT_VERSION and sum(pkt[2:-1]) & 0xFF == pkt[-1]:
                        del buf[:PKT_SIZE]
                        self._packet(pkt)
                        last_bin = time.monotonic()
                    else:
                        self.bad += 1
                        del buf[:1]
        except (serial.SerialException, OSError) as e:
            self.error = str(e)
        finally:
            try:
                ser.write(b"t")             # back to text mode for the VS Code monitor
                ser.flush()
            except (serial.SerialException, OSError):
                pass
            ser.close()

    def _packet(self, pkt):
        v = struct.unpack(PKT_FMT, pkt)
        seq, flags = v[2], v[3]
        if self._last_seq is not None:
            self.lost += (seq - self._last_seq - 1) & 0xFFFF
        self._last_seq = seq
        mlx = tuple(v[36:40]) if flags & 0x01 else None
        self.sink((time.monotonic(), list(v[4:36]), mlx))

    def _feed_text(self, data):
        """Fallback for firmware without stream mode: parse the text printout."""
        self._text += data
        *lines, rest = self._text.split(b"\n")
        self._text = bytearray(rest[-512:])
        for raw in lines:
            line = raw.decode("ascii", "ignore").strip()
            m = MLX_RE.match(line)
            if m:
                self._text_mlx = tuple(float(g) for g in m.groups())
            elif line.startswith("MLX") and ("N.A." in line or "not available" in line):
                self._text_mlx = None
            elif line.startswith(("MUX1", "MUX2")):
                vals = line[4:].split()
                if len(vals) == 16 and all(s.isdigit() for s in vals):
                    base = 0 if line[3] == "1" else 16
                    self._text_adc[base:base + 16] = [int(s) for s in vals]
                    if base == 16 and not self.binary:
                        self.sink((time.monotonic(), list(self._text_adc), self._text_mlx))


class DemoSource(threading.Thread):
    """Simulated finger: wanders over the board, presses and drags."""

    def __init__(self, sink):
        super().__init__(daemon=True)
        self.sink = sink
        self.stop_evt = threading.Event()
        self.error = None
        self.binary = True
        self.lost = self.bad = 0

    def stop(self):
        self.stop_evt.set()

    def run(self):
        import random
        t0 = next_t = time.monotonic()
        prev = None
        while not self.stop_evt.is_set():
            t = time.monotonic() - t0
            # alternate between roaming the board and pressing around U3
            if int(t / 6) % 2 == 0:
                fx = 716 + 560 * math.sin(0.41 * t)
                fy = 610 + 420 * math.sin(0.67 * t + 1.0)
            else:
                fx = 716 + 170 * math.cos(1.3 * t)
                fy = 632 + 170 * math.sin(1.3 * t)
            amp = max(0.0, math.sin(1.1 * t)) ** 0.6
            adc = [4095] * 32
            for lab, (x, y) in TAXELS.items():
                g = math.exp(-((x - fx) ** 2 + (y - fy) ** 2) / (2 * 170 ** 2))
                v = 4095 - 3500 * amp * g + random.gauss(0, 6)
                adc[adc_index(lab)] = int(min(4095, max(0, v)))
            vx, vy = (0.0, 0.0) if prev is None else (fx - prev[0], prev[1] - fy)
            prev = (fx, fy)
            n = math.hypot(vx, vy) or 1.0
            near = math.exp(-((fx - 716) ** 2 + (fy - 632) ** 2) / (2 * 260 ** 2))
            shear = 900 * amp * near
            mlx = (-42 + shear * vx / n + random.gauss(0, 3),
                   -153 + shear * vy / n + random.gauss(0, 3),
                   -82 - 400 * amp * near + random.gauss(0, 5),
                   29.5)
            self.sink((time.monotonic(), adc, mlx))
            next_t += 0.01
            time.sleep(max(0.0, next_t - time.monotonic()))


# ---------------------------------------------------------------------------
# Shear estimate with FSR-gated zero
# ---------------------------------------------------------------------------
RELEASE_DEBOUNCE_S = 0.15   # gate taxels must stay unloaded this long before re-zeroing
ZERO_WINDOW_S = 0.30        # origin = mean field over this unloaded window
SMOOTH_ALPHA = 0.5


class ShearEstimator:
    def __init__(self):
        self.baseline = None
        self.window = deque()
        self.released_since = None
        self.zeroed = False         # True while the gate holds the arrow at the origin
        self.zero_count = 0
        self.delta = (0.0, 0.0, 0.0)
        self.smooth = (0.0, 0.0, 0.0)

    def manual_zero(self, b):
        if b is not None:
            self.baseline = b[:3]
            self.window.clear()
            self.smooth = (0.0, 0.0, 0.0)
            self.zero_count += 1

    def update(self, t, b, released):
        if b is None:
            return
        b = b[:3]
        if self.baseline is None:
            self.baseline = b
        if released:
            if self.released_since is None:
                self.released_since = t
            self.window.append((t, b))
            while self.window[0][0] < t - ZERO_WINDOW_S:
                self.window.popleft()
            if t - self.released_since >= RELEASE_DEBOUNCE_S:
                n = len(self.window)
                self.baseline = tuple(sum(s[1][i] for s in self.window) / n for i in range(3))
                if not self.zeroed:
                    self.zero_count += 1
                self.zeroed = True
        else:
            self.released_since = None
            self.window.clear()
            self.zeroed = False
        self.delta = tuple(b[i] - self.baseline[i] for i in range(3))
        target = (0.0, 0.0, 0.0) if self.zeroed else self.delta
        self.smooth = tuple(s + SMOOTH_ALPHA * (v - s) for s, v in zip(self.smooth, target))


def to_screen(dx, dy, dz, cfg):
    """MLX change -> screen vector (x right, y up).

    Calibrated: remove the X/Y change caused by pressing (k * dZ), then map with
    the fitted 2x2 matrix (any angle, unequal axis gains, skew). Otherwise use
    the rotation/flip settings."""
    cal = cfg.get("dir_cal")
    if cal:
        cx, cy = dx - cal["kx"] * dz, dy - cal["ky"] * dz
        (a, b), (c, d) = cal["M"]
        return a * cx + b * cy, c * cx + d * cy
    if cfg["flip_x"]:
        dx = -dx
    if cfg["flip_y"]:
        dy = -dy
    a = math.radians(cfg["rotation"])
    return dx * math.cos(a) - dy * math.sin(a), dx * math.sin(a) + dy * math.cos(a)


def draw_arrow(p, origin, vec, length, width, color):
    """Arrow from origin along unit vector vec (screen y down) of total length."""
    if length < 1:
        return
    ux, uy = vec
    head = min(length * 0.45, width * 3.2)
    tip = QPointF(origin.x() + ux * length, origin.y() + uy * length)
    base = QPointF(tip.x() - ux * head, tip.y() - uy * head)
    grad = QtGui.QLinearGradient(origin, tip)
    c0 = QtGui.QColor(color)
    c0.setAlpha(40)
    grad.setColorAt(0, c0)
    grad.setColorAt(1, color)
    p.setPen(QtGui.QPen(QtGui.QBrush(grad), width, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
    p.drawLine(origin, base)
    nx, ny = -uy, ux
    hw = width * 1.6
    poly = QtGui.QPolygonF([tip,
                            QPointF(base.x() + nx * hw, base.y() + ny * hw),
                            QPointF(base.x() - nx * hw, base.y() - ny * hw)])
    p.setPen(Qt.PenStyle.NoPen)
    p.setBrush(color)
    p.drawPolygon(poly)


# ---------------------------------------------------------------------------
# Widgets
# ---------------------------------------------------------------------------
class PcbView(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.hover = None
        self.setMouseTracking(True)
        self.setMinimumSize(400, 330)
        self._board_path = self._make_board_path()

    @staticmethod
    def _make_board_path():
        path = QtGui.QPainterPath()
        path.addRoundedRect(QRectF(0, 0, BOARD_W, BOARD_H), 22, 22)
        cut = QtGui.QPainterPath()
        for x, y, r in HOLES:
            cut.addEllipse(QPointF(x, y), r, r)
        cut.addPolygon(QtGui.QPolygonF([QPointF(*pt) for pt in NOTCH]))
        cut.closeSubpath()
        cut.addRoundedRect(QRectF(*SLOT), 6, 6)
        return path.subtracted(cut)

    def _geometry(self):
        m = 28
        s = min((self.width() - 2 * m) / BOARD_W, (self.height() - 2 * m) / BOARD_H)
        return s, (self.width() - BOARD_W * s) / 2, (self.height() - BOARD_H * s) / 2

    def paintEvent(self, _):
        win = self.win
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        bg = QtGui.QRadialGradient(self.width() / 2, self.height() / 2, max(self.width(), self.height()) * 0.7)
        bg.setColorAt(0, QtGui.QColor("#1a2233"))
        bg.setColorAt(1, QtGui.QColor("#0b0f17"))
        p.fillRect(self.rect(), bg)

        s, ox, oy = self._geometry()
        p.translate(ox, oy)
        p.scale(s, s)

        # board: shadow, body, edge, pour border
        for i in range(6, 0, -1):
            p.fillPath(self._board_path.translated(6 + i * 3, 10 + i * 3), QtGui.QColor(0, 0, 0, 22))
        body = QtGui.QLinearGradient(0, 0, BOARD_W, BOARD_H)
        body.setColorAt(0, QtGui.QColor("#2d66cf"))
        body.setColorAt(1, QtGui.QColor("#1c4597"))
        p.fillPath(self._board_path, body)
        p.setPen(QtGui.QPen(QtGui.QColor("#10306b"), 6))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPath(self._board_path)
        p.setPen(QtGui.QPen(QtGui.QColor(120, 170, 255, 45), 3))
        p.drawRoundedRect(QRectF(16, 16, BOARD_W - 32, BOARD_H - 32), 14, 14)
        p.setPen(QtGui.QPen(QtGui.QColor("#0d2350"), 10))
        for x, y, r in HOLES:
            p.drawEllipse(QPointF(x, y), r + 5, r + 5)

        # zero gate area around U3
        gate_rect = QRectF(520 - 95, 530 - 62, 912 - 520 + 190, 735 - 530 + 150)
        if win.mlx is None:
            gate_col = QtGui.QColor(255, 255, 255, 55)
        elif win.est.zeroed:
            gate_col = QtGui.QColor(110, 230, 140, 150)
        else:
            gate_col = QtGui.QColor(255, 170, 60, 170)
        pen = QtGui.QPen(gate_col, 3, Qt.PenStyle.DashLine)
        p.setPen(pen)
        p.drawRoundedRect(gate_rect, 30, 30)

        # chip
        cx, cy = CHIP_CENTER
        half = CHIP_SIZE / 2
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor("#d4b56a"))
        for k in range(4):
            off = -45 + k * 30
            for rect in (QRectF(cx + off - 8, cy - half - 18, 16, 22), QRectF(cx + off - 8, cy + half - 4, 16, 22),
                         QRectF(cx - half - 18, cy + off - 8, 22, 16), QRectF(cx + half - 4, cy + off - 8, 22, 16)):
                p.drawRoundedRect(rect, 3, 3)
        chip_grad = QtGui.QLinearGradient(cx - half, cy - half, cx + half, cy + half)
        chip_grad.setColorAt(0, QtGui.QColor("#4a4b50"))
        chip_grad.setColorAt(1, QtGui.QColor("#2b2c30"))
        p.setBrush(chip_grad)
        p.drawRoundedRect(QRectF(cx - half, cy - half, CHIP_SIZE, CHIP_SIZE), 8, 8)
        p.setBrush(QtGui.QColor("#d8d8d8"))
        p.drawEllipse(QPointF(cx + half - 22, cy - half + 22), 9, 9)
        font = QtGui.QFont()
        font.setPixelSize(19)
        p.setFont(font)
        p.setPen(QtGui.QColor(200, 200, 210, 150))
        if win.mlx is not None:
            p.drawText(QRectF(cx - half, cy + 18, CHIP_SIZE, 30), Qt.AlignmentFlag.AlignCenter, "MLX90393")
        else:
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QtGui.QColor(11, 15, 23, 150))
            p.drawRoundedRect(QRectF(cx - half - 20, cy - half - 20, CHIP_SIZE + 40, CHIP_SIZE + 40), 12, 12)
            na_font = QtGui.QFont()
            na_font.setPixelSize(30)
            na_font.setWeight(QtGui.QFont.Weight.Bold)
            p.setFont(na_font)
            p.setPen(QtGui.QColor("#ff8fa3"))
            p.drawText(QRectF(cx - 150, cy - 40, 300, 44), Qt.AlignmentFlag.AlignCenter, "N.A.")
            p.setFont(font)
            p.setPen(QtGui.QColor(255, 200, 210, 200))
            p.drawText(QRectF(cx - 150, cy + 4, 300, 30), Qt.AlignmentFlag.AlignCenter, "shear not available")

        # taxels
        cfg = win.cfg
        span = max(1, cfg["noload_adc"] - cfg["full_adc"])
        value_font = QtGui.QFont()
        value_font.setPixelSize(24)
        value_font.setWeight(QtGui.QFont.Weight.DemiBold)
        label_font = QtGui.QFont()
        label_font.setPixelSize(30)
        label_font.setWeight(QtGui.QFont.Weight.Bold)
        levels = {}
        for lab in TAXELS:
            v = win.adc[adc_index(lab)]
            levels[lab] = (v, (cfg["noload_adc"] - v) / span)
        p.setPen(Qt.PenStyle.NoPen)
        for lab, (x, y) in TAXELS.items():     # glows first so they never cover a pad
            pr = min(levels[lab][1], 1.0)
            if pr > 0.02:
                r = 70 + 110 * pr
                glow = QtGui.QRadialGradient(x, y, r)
                g = pressure_color(pr)
                g.setAlpha(int(170 * pr))
                glow.setColorAt(0, g)
                g.setAlpha(0)
                glow.setColorAt(1, g)
                p.setBrush(glow)
                p.drawEllipse(QPointF(x, y), r, r)
        for lab, (x, y) in TAXELS.items():
            v, pr = levels[lab]
            col = pressure_color(pr)
            rect = QRectF(x - PAD_W / 2, y - PAD_H / 2, PAD_W, PAD_H)
            p.setBrush(col)
            p.setPen(QtGui.QPen(col.darker(150), 3))
            p.drawRoundedRect(rect, 9, 9)
            p.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 45), 4))
            p.drawLine(QPointF(x, y - PAD_H / 2 + 5), QPointF(x, y + PAD_H / 2 - 5))
            if lab == self.hover:
                p.setPen(QtGui.QPen(QtGui.QColor("white"), 4))
                p.setBrush(Qt.BrushStyle.NoBrush)
                p.drawRoundedRect(rect.adjusted(-7, -7, 7, 7), 13, 13)
            if cfg["show_values"]:
                p.setFont(value_font)
                p.setPen(QtGui.QColor("#ffffff") if pr > 0.6 else QtGui.QColor("#3a3320"))
                p.drawText(rect, Qt.AlignmentFlag.AlignCenter, str(v))
            p.setFont(label_font)
            p.setPen(QtGui.QColor(255, 255, 255, 215))
            p.drawText(QRectF(x - 70, y + PAD_H / 2 + 2, 140, 40), Qt.AlignmentFlag.AlignCenter, str(lab))

        # shear arrow from U3
        sx, sy = to_screen(*win.est.smooth, cfg)
        mag = math.hypot(sx, sy)
        if win.mlx is not None and mag > cfg["deadband_ut"]:
            k = min(1.0, mag / max(cfg["arrow_full_ut"], 1e-6))
            length = 60 + 300 * k
            draw_arrow(p, QPointF(cx, cy), (sx / mag, -sy / mag), length, 16, QtGui.QColor("#21e6ff"))
            p.setFont(value_font)
            p.setPen(QtGui.QColor("#ccf9ff"))
            tip = QPointF(cx + sx / mag * (length + 38), cy - sy / mag * (length + 38))
            p.drawText(QRectF(tip.x() - 80, tip.y() - 18, 160, 36), Qt.AlignmentFlag.AlignCenter, f"{mag:.0f} µT")
        elif win.mlx is not None:
            p.setPen(QtGui.QPen(QtGui.QColor(33, 230, 255, 200), 4))
            p.setBrush(QtGui.QColor(33, 230, 255, 70))
            p.drawEllipse(QPointF(cx, cy), 14, 14)
        p.end()

    def mouseMoveEvent(self, e):
        s, ox, oy = self._geometry()
        bx, by = (e.position().x() - ox) / s, (e.position().y() - oy) / s
        hit = None
        for lab, (x, y) in TAXELS.items():
            if abs(bx - x) <= PAD_W / 2 + 10 and abs(by - y) <= PAD_H / 2 + 10:
                hit = lab
        self.hover = hit
        if hit is None:
            QtWidgets.QToolTip.hideText()
        else:
            m, c = CHANNEL_MAP[hit]
            v = self.win.adc[adc_index(hit)]
            tag = "（MLX 归零门控）" if hit in ZERO_GATE_LABELS else ""
            QtWidgets.QToolTip.showText(e.globalPosition().toPoint(),
                                        f"触点 {hit}{tag}\nMUX{m + 1} CH{c}\nADC {v}", self)
        self.update()

    def leaveEvent(self, _):
        self.hover = None
        self.update()


class ShearCompass(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.trail = deque(maxlen=40)
        self.setMinimumSize(180, 180)

    def paintEvent(self, _):
        win, cfg = self.win, self.win.cfg
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        side = min(self.width(), self.height())
        c = QPointF(self.width() / 2, self.height() / 2)
        r = side / 2 - 18
        p.setPen(Qt.PenStyle.NoPen)
        g = QtGui.QRadialGradient(c, r)
        g.setColorAt(0, QtGui.QColor("#1d2840"))
        g.setColorAt(1, QtGui.QColor("#121826"))
        p.setBrush(g)
        p.drawEllipse(c, r, r)
        p.setBrush(Qt.BrushStyle.NoBrush)
        for k in (0.25, 0.5, 0.75, 1.0):
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 60 if k == 1.0 else 28), 1))
            p.drawEllipse(c, r * k, r * k)
        p.drawLine(QPointF(c.x() - r, c.y()), QPointF(c.x() + r, c.y()))
        p.drawLine(QPointF(c.x(), c.y() - r), QPointF(c.x(), c.y() + r))
        font = QtGui.QFont()
        font.setPixelSize(11)
        p.setFont(font)
        p.setPen(QtGui.QColor(255, 255, 255, 110))
        p.drawText(QRectF(c.x() - r, c.y() - r - 16, 2 * r, 14), Qt.AlignmentFlag.AlignCenter, "↑")
        if win.mlx is not None:
            p.drawText(QRectF(c.x() + r - 60, c.y() + 4, 58, 14), Qt.AlignmentFlag.AlignRight,
                       f"{cfg['arrow_full_ut']:.0f} µT")

        sx, sy = to_screen(*win.est.smooth, cfg)
        mag = math.hypot(sx, sy)
        full = max(cfg["arrow_full_ut"], 1e-6)
        k = min(1.0, mag / full)
        if mag > cfg["deadband_ut"]:
            self.trail.append((sx / mag * k, -sy / mag * k))
        elif self.trail:
            self.trail.popleft()
        for i, (tx, ty) in enumerate(self.trail):
            a = int(140 * (i + 1) / len(self.trail))
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QtGui.QColor(33, 230, 255, a // 2))
            p.drawEllipse(QPointF(c.x() + tx * r, c.y() + ty * r), 3, 3)
        if win.mlx is None:
            self.trail.clear()
            f2 = QtGui.QFont()
            f2.setPixelSize(15)
            f2.setWeight(QtGui.QFont.Weight.Bold)
            p.setFont(f2)
            p.setPen(QtGui.QColor("#ff8fa3"))
            p.drawText(QRectF(c.x() - r, c.y() - 30, 2 * r, 60), Qt.AlignmentFlag.AlignCenter,
                       "Shear\nnot available")
            p.end()
            return
        if mag > cfg["deadband_ut"]:
            draw_arrow(p, c, (sx / mag, -sy / mag), max(k * r, 14), 6, QtGui.QColor("#21e6ff"))
        p.setPen(QtGui.QPen(QtGui.QColor("#21e6ff"), 2))
        p.setBrush(QtGui.QColor("#0b0f17"))
        p.drawEllipse(c, 5, 5)
        p.end()


class GradientBar(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.setFixedHeight(46)

    def paintEvent(self, _):
        cfg = self.win.cfg
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        bar = QRectF(4, 4, self.width() - 8, 16)
        g = QtGui.QLinearGradient(bar.left(), 0, bar.right(), 0)
        for i in range(11):
            g.setColorAt(i / 10, pressure_color(i / 10))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(g)
        p.drawRoundedRect(bar, 8, 8)
        font = QtGui.QFont()
        font.setPixelSize(11)
        p.setFont(font)
        p.setPen(QtGui.QColor("#9aa6bd"))
        p.drawText(QRectF(4, 24, 120, 18), Qt.AlignmentFlag.AlignLeft, f"无压 ≥{cfg['noload_adc']}")
        p.drawText(QRectF(self.width() - 124, 24, 120, 18), Qt.AlignmentFlag.AlignRight, f"满压 ≤{cfg['full_adc']}")
        p.end()


CAL_STEPS = [
    # kind, instruction, seconds, target direction on screen (x right, y up)
    ("zero", "完全松开传感器，不要碰它。", 1.5, None),
    ("normal", "垂直往下按，再松开，重复 2～3 次。\n只按、不要往任何方向推。", 4.0, None),
    ("dir", "按住后朝  →  右  推，并保持住。", 2.5, (1.0, 0.0)),
    ("dir", "按住后朝  ↑  上  推，并保持住。", 2.5, (0.0, 1.0)),
    ("dir", "按住后朝  ←  左  推，并保持住。", 2.5, (-1.0, 0.0)),
    ("dir", "按住后朝  ↓  下  推，并保持住。", 2.5, (0.0, -1.0)),
]
CAL_LEAD_S = 1.0        # time to get into position after clicking, not recorded


def _mean(rows):
    n = len(rows)
    return tuple(sum(r[i] for r in rows) / n for i in range(len(rows[0])))


def _std(rows, m):
    n = len(rows)
    return tuple(math.sqrt(sum((r[i] - m[i]) ** 2 for r in rows) / n) for i in range(len(m)))


class DirectionCalibration(QtWidgets.QDialog):
    """Wizard: zero -> press straight down (normal-force crosstalk k) -> push in
    four directions (2x2 matrix). Directions are as seen on the PCB drawing."""

    def __init__(self, win):
        super().__init__(win)
        self.win = win
        self.setWindowTitle("剪切力方向校准")
        self.setMinimumWidth(430)
        v = QtWidgets.QVBoxLayout(self)
        self.step_label = QtWidgets.QLabel(objectName="muted")
        self.text = QtWidgets.QLabel(objectName="big")
        self.text.setWordWrap(True)
        self.text.setStyleSheet("font-size: 18px;")
        self.progress = QtWidgets.QProgressBar()
        self.progress.setTextVisible(False)
        self.progress.setFixedHeight(8)
        self.info = QtWidgets.QLabel(objectName="muted")
        self.info.setWordWrap(True)
        row = QtWidgets.QHBoxLayout()
        self.clear_btn = QtWidgets.QPushButton("清除校准", objectName="ghost", clicked=self.clear)
        self.go_btn = QtWidgets.QPushButton("开始记录", clicked=self.go)
        row.addWidget(self.clear_btn)
        row.addStretch()
        row.addWidget(self.go_btn)
        for w in (self.step_label, self.text, self.progress, self.info):
            v.addWidget(w)
        v.addLayout(row)
        self.timer = QtCore.QTimer(self, interval=30, timeout=self._progress)
        self.restart()

    def restart(self):
        self.step = 0
        self.samples = []
        self.t_start = None
        self.base = self.sigma = None
        self.k = (0.0, 0.0)
        self.pushes = []            # (target, mean corrected vector)
        self.result = None
        self.go_btn.setText("开始记录")
        self.go_btn.setEnabled(True)
        cal = self.win.cfg.get("dir_cal")
        self.info.setText("当前：" + (f"已校准（误差约 ±{cal['err_deg']:.0f}°）" if cal else "未校准（使用旋转/翻转设置）")
                          + "\n左右上下以界面上 PCB 图的方向为准。每一步点“开始记录”后有 1 秒准备时间。")
        self._show_step()

    def _show_step(self):
        kind, text, secs, _ = CAL_STEPS[self.step]
        self.step_label.setText(f"第 {self.step + 1} / {len(CAL_STEPS)} 步")
        self.text.setText(text)
        self.progress.setValue(0)

    def go(self):
        if self.result is not None:         # finished: save
            self.win.set_cfg("dir_cal", self.result)
            self.accept()
            return
        if self.win.mlx is None:
            self.info.setText("没有 MLX90393 数据，无法校准。")
            return
        self.samples = []
        self.t_start = time.monotonic()
        self.go_btn.setEnabled(False)
        self.timer.start()

    def feed(self, mlx):
        if self.t_start is None or mlx is None:
            return
        dt = time.monotonic() - self.t_start
        if dt >= CAL_LEAD_S:
            self.samples.append(mlx[:3])

    def _progress(self):
        secs = CAL_STEPS[self.step][2]
        dt = time.monotonic() - self.t_start
        if dt < CAL_LEAD_S:
            self.step_label.setText(f"第 {self.step + 1} / {len(CAL_STEPS)} 步 · 准备…")
            return
        self.step_label.setText(f"第 {self.step + 1} / {len(CAL_STEPS)} 步 · 记录中")
        self.progress.setValue(int(100 * min(1.0, (dt - CAL_LEAD_S) / secs)))
        if dt - CAL_LEAD_S >= secs:
            self.timer.stop()
            self.t_start = None
            err = self._finish_step()
            self.go_btn.setEnabled(True)
            if err:
                self.info.setText("⚠ " + err + "  请重做这一步。")
                self.go_btn.setText("重做这一步")
                self.progress.setValue(0)
                return
            self.step += 1
            if self.step < len(CAL_STEPS):
                self.go_btn.setText("开始记录")
                self._show_step()
            else:
                self._fit()

    def _finish_step(self):
        kind, _, _, target = CAL_STEPS[self.step]
        s = self.samples
        if len(s) < 20:
            return f"只收到 {len(s)} 个 MLX 数据。"
        if kind == "zero":
            self.base = _mean(s)
            self.sigma = _std(s, self.base)
            self.info.setText(f"零点 OK，噪声 X {self.sigma[0]:.1f} / Y {self.sigma[1]:.1f} / Z {self.sigma[2]:.1f} µT")
            return None
        d = [tuple(r[i] - self.base[i] for i in range(3)) for r in s]
        if kind == "normal":
            zz = sum(r[2] ** 2 for r in d)
            if max(abs(r[2]) for r in d) < max(30.0, 8 * self.sigma[2]):
                return "没检测到明显的按压（dZ 太小）。"
            self.k = (sum(r[0] * r[2] for r in d) / zz, sum(r[1] * r[2] for r in d) / zz)
            self.info.setText(f"法向串扰 kx={self.k[0]:+.3f}  ky={self.k[1]:+.3f}"
                              "（每 1 µT 的 dZ 带来的假 dX/dY）")
            return None
        c = [(r[0] - self.k[0] * r[2], r[1] - self.k[1] * r[2]) for r in d]
        mags = [math.hypot(*v) for v in c]
        top = max(mags)
        if top < max(30.0, 8 * max(self.sigma[0], self.sigma[1])):
            return "剪切信号太小，推得再用力一点。"
        held = [v for v, m in zip(c, mags) if m >= 0.5 * top]     # the steady, pushed part
        mv = _mean(held)
        self.pushes.append((target, mv))
        self.info.setText(f"记录到 dX'={mv[0]:+.0f}  dY'={mv[1]:+.0f} µT")
        return None

    def _fit(self):
        # M minimises sum |M c_i - m u_i|^2 : M = (T C^T)(C C^T)^-1, with one common
        # target length m so unequal axis gains are corrected, not baked in
        m = sum(math.hypot(*c) for _, c in self.pushes) / len(self.pushes)
        a11 = a12 = a22 = b11 = b12 = b21 = b22 = 0.0
        for (ux, uy), (cx, cy) in self.pushes:
            tx, ty = ux * m, uy * m
            a11 += cx * cx; a12 += cx * cy; a22 += cy * cy
            b11 += tx * cx; b12 += tx * cy; b21 += ty * cx; b22 += ty * cy
        det = a11 * a22 - a12 * a12
        if abs(det) < 1e-9 * max(a11 * a22, 1e-9):
            self.info.setText("⚠ 四个方向的信号几乎在一条线上，无法拟合，请点“清除校准”后重新校准。")
            self.step = 0
            self.restart()
            return
        i11, i12, i22 = a22 / det, -a12 / det, a11 / det
        M = [[b11 * i11 + b12 * i12, b11 * i12 + b12 * i22],
             [b21 * i11 + b22 * i12, b21 * i12 + b22 * i22]]
        errs, before = [], []
        for (ux, uy), (cx, cy) in self.pushes:
            sx, sy = M[0][0] * cx + M[0][1] * cy, M[1][0] * cx + M[1][1] * cy
            errs.append(abs((math.degrees(math.atan2(sy, sx) - math.atan2(uy, ux)) + 180) % 360 - 180))
            ox, oy = to_screen(cx, cy, 0.0, {**self.win.cfg, "dir_cal": None})
            before.append(abs((math.degrees(math.atan2(oy, ox) - math.atan2(uy, ux)) + 180) % 360 - 180))
        self.result = {"kx": self.k[0], "ky": self.k[1], "M": M, "err_deg": max(errs)}
        self.step_label.setText("完成")
        self.text.setText(f"校准完成：方向误差最大约 {max(errs):.0f}°")
        self.info.setText(f"校准前（当前旋转/翻转设置）误差最大约 {max(before):.0f}°。\n"
                          "点“保存并使用”后，旋转/翻转设置将不再生效。")
        self.progress.setValue(100)
        self.go_btn.setText("保存并使用")

    def clear(self):
        self.timer.stop()
        self.win.set_cfg("dir_cal", None)
        self.restart()

    def closeEvent(self, e):
        self.timer.stop()
        super().closeEvent(e)


class DataText(QtWidgets.QLabel):
    """Fixed-width text panel. Every field has a fixed column width so the
    numbers never shift when values change length (e.g. 4095 -> 812 -> N.A.).
    The font shrinks with the panel width so the widest line always fits."""
    PAD_X, PAD_Y = 16, 10

    def __init__(self, lines, cols):
        super().__init__(objectName="datatext")
        self.lines, self.cols = lines, cols
        self.setTextFormat(Qt.TextFormat.PlainText)
        self.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop)
        # never let the text width push the window wider than the screen
        self.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored, QtWidgets.QSizePolicy.Policy.Fixed)
        self.setMinimumWidth(0)
        self._fit(900)

    def _fit(self, width):
        # set through the widget's own style sheet: the app style sheet's
        # font-size would otherwise override setFont()
        font = QtGui.QFont()
        font.setFamilies(["Menlo", "SF Mono", "Monaco", "Courier New"])
        font.setStyleHint(QtGui.QFont.StyleHint.Monospace)
        size = 14
        while size > 7:
            font.setPixelSize(size)
            if QtGui.QFontMetrics(font).horizontalAdvance("0" * self.cols) <= width - 2 * self.PAD_X - 8:
                break
            size -= 1
        self.setStyleSheet(f'font-family: "{QtGui.QFontInfo(font).family()}"; font-size: {size}px;')
        self.setFixedHeight(QtGui.QFontMetrics(font).lineSpacing() * self.lines + 2 * self.PAD_Y + 12)

    def resizeEvent(self, e):
        if e.size().width() != e.oldSize().width():
            self._fit(e.size().width())
        super().resizeEvent(e)


STYLE = """
QMainWindow, QDialog, QWidget#side { background: #0b0f17; }
QProgressBar { background: #1d263a; border: none; border-radius: 4px; }
QProgressBar::chunk { background: #21e6ff; border-radius: 4px; }
QWidget { color: #dfe6f3; font-size: 13px; }
QGroupBox { background: #131a28; border: 1px solid #222c40; border-radius: 12px;
            margin-top: 16px; padding: 12px 10px 10px 10px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; color: #8fb4ff; }
QPushButton { background: #2457c5; border: none; border-radius: 8px; padding: 7px 12px; font-weight: 600; }
QPushButton:hover { background: #2e68e0; }
QPushButton:pressed { background: #1d48a6; }
QPushButton#ghost { background: #1d263a; }
QPushButton#ghost:hover { background: #26324c; }
QComboBox, QSpinBox, QDoubleSpinBox { background: #0f1520; border: 1px solid #2a354d; border-radius: 6px;
            padding: 4px 6px; }
QLabel#big { font-size: 26px; font-weight: 700; color: #ffffff; }
QLabel#muted { color: #8792a8; }
QLabel#chip { border-radius: 10px; padding: 3px 10px; font-weight: 700; }
QLabel#datatext { background: #0f1520; color: #cfe3ff; border-top: 1px solid #222c40;
            padding: 10px 16px; }
QScrollArea#sidescroll { background: #0b0f17; border: none; }
QPushButton#ghost:checked { background: #2457c5; }
QStatusBar { background: #0b0f17; color: #8792a8; }
"""


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, demo=False, port=None):
        super().__init__()
        self.setWindowTitle("Ultra-thin Tactile Sensor")
        self.cfg = load_config()
        self.frames = deque(maxlen=5000)
        self.source = None
        self.adc = [4095] * 32
        self.mlx = None
        self.est = ShearEstimator()
        self.frame_times = deque(maxlen=300)
        self.cal_dialog = None

        self.pcb = PcbView(self)
        self.data_text = self._build_data_text()
        side = self._build_side()
        left = QtWidgets.QWidget()
        lv = QtWidgets.QVBoxLayout(left)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(0)
        lv.addWidget(self.pcb, 1)
        lv.addWidget(self.data_text)
        central = QtWidgets.QWidget()
        lay = QtWidgets.QHBoxLayout(central)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(left, 1)
        scroll = QtWidgets.QScrollArea(objectName="sidescroll")    # side panel scrolls on short screens
        scroll.setWidget(side)
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll.setFixedWidth(side.width() + 12)
        lay.addWidget(scroll)
        self.setCentralWidget(central)
        self.status = self.statusBar()
        avail = QtGui.QGuiApplication.primaryScreen().availableGeometry()
        self.resize(min(1360, avail.width() - 20), min(860, avail.height() - 40))
        self.setMinimumSize(760, 520)

        QtGui.QShortcut(QtGui.QKeySequence("Z"), self, activated=self.manual_zero)

        self.timer = QtCore.QTimer(self, interval=16, timeout=self.tick)
        self.timer.start()
        if demo:
            self.port_box.setCurrentText(DEMO_PORT)
            self.toggle_connect()
        elif port:
            self.port_box.setCurrentText(port)
            self.toggle_connect()

    # ----- side panel -----
    def _build_side(self):
        side = QtWidgets.QWidget(objectName="side")
        side.setFixedWidth(340)
        v = QtWidgets.QVBoxLayout(side)
        v.setContentsMargins(10, 6, 14, 10)

        conn = QtWidgets.QGroupBox("串口")
        g = QtWidgets.QGridLayout(conn)
        self.port_box = QtWidgets.QComboBox()
        refresh = QtWidgets.QPushButton("刷新", objectName="ghost", clicked=self.refresh_ports)
        self.conn_btn = QtWidgets.QPushButton("连接", clicked=self.toggle_connect)
        self.conn_label = QtWidgets.QLabel("未连接", objectName="muted")
        g.addWidget(self.port_box, 0, 0, 1, 2)
        g.addWidget(refresh, 0, 2)
        g.addWidget(self.conn_btn, 1, 0, 1, 3)
        g.addWidget(self.conn_label, 2, 0, 1, 3)
        text_btn = QtWidgets.QPushButton("显示数据文本", objectName="ghost", checkable=True)
        text_btn.setChecked(self.cfg["show_text"])
        text_btn.toggled.connect(self.toggle_data_text)
        g.addWidget(text_btn, 3, 0, 1, 3)
        v.addWidget(conn)
        self.refresh_ports()

        shear = QtWidgets.QGroupBox("剪切力（MLX90393）")
        sv = QtWidgets.QVBoxLayout(shear)
        self.compass = ShearCompass(self)
        sv.addWidget(self.compass)
        row = QtWidgets.QHBoxLayout()
        self.mag_label = QtWidgets.QLabel("0 µT", objectName="big")
        self.state_label = QtWidgets.QLabel("—", objectName="chip")
        row.addWidget(self.mag_label)
        row.addStretch()
        row.addWidget(self.state_label)
        sv.addLayout(row)
        self.shear_info = QtWidgets.QLabel("", objectName="muted")
        sv.addWidget(self.shear_info)
        zero_btn = QtWidgets.QPushButton("手动归零 (Z)", objectName="ghost", clicked=self.manual_zero)
        cal_btn = QtWidgets.QPushButton("方向校准…", objectName="ghost", clicked=self.open_calibration)
        btns = QtWidgets.QHBoxLayout()
        btns.addWidget(zero_btn)
        btns.addWidget(cal_btn)
        sv.addLayout(btns)
        v.addWidget(shear)

        sett = QtWidgets.QGroupBox("显示设置")
        f = QtWidgets.QFormLayout(sett)
        f.setLabelAlignment(Qt.AlignmentFlag.AlignRight)

        def spin(key, lo, hi, step, dbl=False, suffix=""):
            w = QtWidgets.QDoubleSpinBox() if dbl else QtWidgets.QSpinBox()
            w.setRange(lo, hi)
            w.setSingleStep(step)
            w.setValue(self.cfg[key])
            if suffix:
                w.setSuffix(suffix)
            w.valueChanged.connect(lambda val, k=key: self.set_cfg(k, val))
            return w

        f.addRow("无压阈值 ADC", spin("noload_adc", 100, 4095, 10))
        f.addRow("满压 ADC", spin("full_adc", 0, 4000, 50))
        f.addRow("箭头满量程", spin("arrow_full_ut", 10, 50000, 50, True, " µT"))
        f.addRow("死区", spin("deadband_ut", 0, 5000, 5, True, " µT"))
        rot = QtWidgets.QComboBox()
        rot.addItems(["0°", "90°", "180°", "270°"])
        rot.setCurrentIndex(int(self.cfg["rotation"]) // 90 % 4)
        rot.currentIndexChanged.connect(lambda i: self.set_cfg("rotation", i * 90))
        f.addRow("箭头旋转", rot)
        flips = QtWidgets.QHBoxLayout()
        for key, text in (("flip_x", "翻转 X"), ("flip_y", "翻转 Y")):
            cb = QtWidgets.QCheckBox(text)
            cb.setChecked(self.cfg[key])
            cb.toggled.connect(lambda val, k=key: self.set_cfg(k, val))
            flips.addWidget(cb)
        f.addRow("", flips)
        show = QtWidgets.QCheckBox("在触点上显示 ADC 数值")
        show.setChecked(self.cfg["show_values"])
        show.toggled.connect(lambda val: self.set_cfg("show_values", val))
        f.addRow("", show)
        self.legend = GradientBar(self)
        f.addRow(self.legend)
        v.addWidget(sett)
        v.addStretch()
        return side

    # ----- raw data text -----
    TEXT_LINES = 5
    TEXT_COLS = 108     # widest line: 12-char row label + 16 x 6-char ADC columns

    def _build_data_text(self):
        w = DataText(self.TEXT_LINES, self.TEXT_COLS)
        w.setVisible(self.cfg["show_text"])
        self._text_tick = 0
        return w

    def toggle_data_text(self, on):
        self.data_text.setVisible(on)
        self.set_cfg("show_text", on)

    def _format_data_text(self):
        def num(v, fmt):
            return format(v, fmt) if v is not None else format("N.A.", ">" + fmt.lstrip("+").split(".")[0])
        if self.mlx is None:
            x = y = z = t = dx = dy = dz = None
        else:
            x, y, z, t = self.mlx
            dx, dy, dz = self.est.delta
        lines = [
            f"MLX90393     X:{num(x, '9.1f')}  Y:{num(y, '9.1f')}  Z:{num(z, '9.1f')} uT"
            f"     T:{num(t, '6.1f')} C",
            f"  vs zero   dX:{num(dx, '+9.1f')} dY:{num(dy, '+9.1f')} dZ:{num(dz, '+9.1f')} uT",
            "CH          " + "".join(f"{c:>6}" for c in range(16)),
            "MUX1 ( 1-16)" + "".join(f"{v:>6}" for v in self.adc[:16]),
            "MUX2 (17-32)" + "".join(f"{v:>6}" for v in self.adc[16:]),
        ]
        return "\n".join(lines)

    def set_cfg(self, key, val):
        self.cfg[key] = val
        save_config(self.cfg)
        self.legend.update()

    # ----- connection -----
    def refresh_ports(self):
        cur = self.port_box.currentText() or self.cfg["port"]
        ports = [p.device for p in serial.tools.list_ports.comports()
                 if "Bluetooth" not in p.device and "debug-console" not in p.device]
        ports.sort(key=lambda d: ("usbserial" not in d and "usbmodem" not in d, d))
        self.port_box.clear()
        self.port_box.addItems(ports + [DEMO_PORT])
        if cur in ports + [DEMO_PORT]:
            self.port_box.setCurrentText(cur)

    def toggle_connect(self):
        if self.source:
            self.source.stop()
            self.source.join(timeout=1.0)
            self.source = None
            self.conn_btn.setText("连接")
            self.conn_label.setText("未连接")
            return
        port = self.port_box.currentText()
        if not port:
            self.conn_label.setText("没有找到串口，插上开发板后点“刷新”")
            return
        self.frames.clear()
        self.frame_times.clear()
        self.est = ShearEstimator()
        self.mlx = None
        sink = self.frames.append
        self.source = DemoSource(sink) if port == DEMO_PORT else SerialSource(port, sink)
        self.source.start()
        self.conn_btn.setText("断开")
        if port != DEMO_PORT:
            self.cfg["port"] = port
            save_config(self.cfg)

    def manual_zero(self):
        self.est.manual_zero(self.mlx)

    def open_calibration(self):
        if self.cal_dialog is None:
            self.cal_dialog = DirectionCalibration(self)
            self.cal_dialog.finished.connect(lambda _: setattr(self, "cal_dialog", None))
        self.cal_dialog.show()
        self.cal_dialog.raise_()

    # ----- update loop -----
    def tick(self):
        gate = [adc_index(lab) for lab in ZERO_GATE_LABELS]
        while self.frames:
            t, adc, mlx = self.frames.popleft()
            self.adc = adc
            self.mlx = mlx
            released = all(adc[i] >= self.cfg["noload_adc"] for i in gate)
            self.est.update(t, mlx, released)
            self.frame_times.append(t)
            if self.cal_dialog is not None:
                self.cal_dialog.feed(mlx)
        self._update_status()
        self.pcb.update()
        self.compass.update()
        self._text_tick += 1
        if self.data_text.isVisible() and self._text_tick % 6 == 0:     # ~10 Hz, readable
            self.data_text.setText(self._format_data_text())

    def _update_status(self):
        src = self.source
        if src is None:
            return
        if src.error:
            self.conn_label.setText(f"串口错误：{src.error}")
            self.source = None
            self.conn_btn.setText("连接")
            return
        now = time.monotonic()
        recent = [t for t in self.frame_times if t > now - 1.0]
        rate = len(recent)
        if isinstance(src, DemoSource):
            mode = "演示数据"
        elif src.binary:
            mode = "二进制流"
        elif rate:
            mode = "文本模式（固件未更新？）"
        else:
            mode = "等待数据…"
        self.conn_label.setText(f"{mode} · {rate} 帧/秒 · 丢包 {src.lost} · 校验错 {src.bad}")

        if self.mlx is None:
            self.mag_label.setText("N.A.")
            self.state_label.setText("剪切力 not available")
            self.state_label.setStyleSheet("background:#3a2230; color:#ff8fa3;")
            self.shear_info.setText("未检测到 MLX90393（无磁传感器版本）\n"
                                    "dX N.A.  dY N.A.  dZ N.A.   T N.A.")
            self.status.showMessage("MLX  X N.A.  Y N.A.  Z N.A.  T N.A.")
            return
        sx, sy = to_screen(*self.est.smooth, self.cfg)
        mag = math.hypot(sx, sy)
        ang = math.degrees(math.atan2(sy, sx)) if mag > self.cfg["deadband_ut"] else float("nan")
        self.mag_label.setText(f"{mag:.0f} µT")
        if self.est.zeroed:
            self.state_label.setText("无接触 · 已归零")
            self.state_label.setStyleSheet("background:#163a2a; color:#7ef0a6;")
        else:
            self.state_label.setText("接触中")
            self.state_label.setStyleSheet("background:#3d2a12; color:#ffb35c;")
        dx, dy, dz = self.est.delta
        ang_txt = "—" if math.isnan(ang) else f"{ang:+.0f}°"
        self.shear_info.setText(f"方向 {ang_txt}   dZ {dz:+.0f} µT   归零 {self.est.zero_count} 次\n"
                                f"dX {dx:+.0f}  dY {dy:+.0f} µT   T {self.mlx[3]:.1f} °C   "
                                + ("方向已校准" if self.cfg.get("dir_cal") else "方向未校准"))
        x, y, z, _ = self.mlx
        self.status.showMessage(f"MLX 原始  X {x:.1f}  Y {y:.1f}  Z {z:.1f} µT    "
                                f"门控触点 15/16/17/18 ADC: "
                                + " ".join(str(self.adc[adc_index(l)]) for l in ZERO_GATE_LABELS))

    def closeEvent(self, e):
        if self.source:
            self.source.stop()
            self.source.join(timeout=1.0)
        super().closeEvent(e)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--demo", action="store_true", help="simulated data, no hardware")
    ap.add_argument("--port", help="connect to this serial port at start")
    ap.add_argument("--screenshot", help=argparse.SUPPRESS)
    ap.add_argument("--screenshot-delay", type=float, default=4.0, help=argparse.SUPPRESS)
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(STYLE)
    win = MainWindow(demo=args.demo, port=args.port)
    win.show()
    if args.screenshot:
        QtCore.QTimer.singleShot(int(args.screenshot_delay * 1000), lambda: (win.grab().save(args.screenshot), app.quit()))
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
