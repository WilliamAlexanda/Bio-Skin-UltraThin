# Bio-Skin-UltraThin

超薄触觉传感器：32 个压阻触点 + 可选的 MLX90393 三轴磁传感器（用于剪切力方向），由 ESP32-S3 以 100 Hz 采样。本仓库包含使用说明、ESP32-S3 固件和实时可视化程序。

Ultra-thin tactile sensor: 32 piezoresistive taxels plus an optional MLX90393 3-axis magnetometer (for shear direction), sampled at 100 Hz by an ESP32-S3. This repository contains the user guides, the ESP32-S3 firmware and a real-time visualizer.

![visualizer](docs/images/viz_demo.jpg)

## 文档 / Documentation

| | 中文 / English |
|---|---|
| 传感器使用说明 / Sensor guide | [docs/SENSOR_GUIDE.md](docs/SENSOR_GUIDE.md) |
| 可视化程序使用说明 / Visualizer guide | [docs/VISUALIZER_GUIDE.md](docs/VISUALIZER_GUIDE.md) |

## 目录 / Layout

```
tactile_sensor/        ESP32-S3 主固件 (ESP-IDF v5.5) / main firmware
tests/mlx90393_test/   MLX90393 焊接与通信诊断 / MLX90393 soldering & I2C diagnostics
tests/mux_adc_test/    74HC4067 + ADC 诊断 / multiplexer & ADC diagnostics
visualizer/            Python 可视化程序 (PyQt6) / Python visualizer (PyQt6)
docs/                  使用说明与图片 / guides and images
```

## 快速开始 / Quick start

1. 按 [传感器使用说明](docs/SENSOR_GUIDE.md#1-连接--connection) 连接 FPC 排线，用 USB 线连接开发板的 **COM** 口。
   Connect the FPC cables and plug the board's **COM** (USB-UART) port into the computer, see the [sensor guide](docs/SENSOR_GUIDE.md#1-连接--connection).
2. 烧录固件 / Flash the firmware:
   ```
   cd tactile_sensor
   idf.py -p <PORT> flash monitor
   ```
3. 启动可视化 / Start the visualizer:
   ```
   cd visualizer
   ./run.command            # macOS / Linux
   ./run.command --demo     # 无硬件演示 / demo without hardware
   ```
