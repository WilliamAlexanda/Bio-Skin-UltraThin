# Ultra-thin tactile sensor firmware (ESP32-S3, ESP-IDF v5.5.6)

> 完整的中英双语说明见 / Full bilingual guide: [SENSOR_GUIDE.md](../docs/SENSOR_GUIDE.md)

## 硬件连接
| 功能 | GPIO |
|---|---|
| MLX90393 SDA / SCL（I2C 地址 0x0C） | 8 / 9 |
| MUX1 (74HC4067) S0–S3 / SIG | 4,5,6,7 / 1 (ADC1_CH0) |
| MUX2 (74HC4067) S0–S3 / SIG | 11,12,13,14 / 2 (ADC1_CH1) |

## 采样
- 32 路 ADC 和 MLX90393 都是 **100 Hz**（`main.c` 里的 `SAMPLE_HZ`）
- 串口监视器每秒刷新 5 次（`PRINT_HZ`），显示最新一帧；串口波特率 115200
- ADC：12 位，0–4095，输入 ≥3.1 V 时读数为 4095。切换通道后等待 100 µs，并丢掉第一次读数（`mux_adc.c` 里的 `SETTLE_US`、`DUMMY_READS`），用来适应 100k 以上的分压源
- MLX90393：GAIN_SEL=0、RES=2（最大量程，XY 约 ±66 mT，3.0 µT/LSB；Z 约 ±106 mT，4.8 µT/LSB），DIG_FILT=5。上电后取 20 次读数的平均作为零点，dX/dY/dZ 是相对这个零点的变化量。上电时**不要碰传感器**

- 不带 MLX90393 的版本：启动时会提示一次 `MLX90393: not detected`，之后 MLX 那一行全部显示 `N.A.`，固件每秒在后台重试一次检测

## 串口输出示例
```
---- #40  t=0.77 s  100.0 Hz  frame 6.1 ms  missed 0
MLX   X=   -42.1 Y=  -153.2 Z=   -82.3 uT | dX=   +1.8 dY=   +0.3 dZ=   -0.2 uT | T=29.7 C
CH      0    1    2    3 ...   15
MUX1 4095 4095 4095 4095 ...
MUX2 ...
```
- `frame`：采一帧（32 路 ADC 加 MLX）实际花费的时间，必须小于 10 ms
- `missed`：因为来不及处理而丢掉的帧数，正常情况下是 0

## 可视化用的二进制流
在串口里收到 `b` 后，固件改为每帧发送一个 87 字节的二进制包（100 Hz），收到 `t` 后恢复文本输出，每次复位后都是文本模式。这个功能给 `../visualizer` 使用，包格式见 `main.c` 里的 `stream_pkt_t`。在 VS Code 监视器里不要输入 `b`。

## 使用
VS Code 打开本文件夹 → 底部状态栏点 🔥（编译、烧录并打开监视器），或者点 🖥️（只打开监视器）。

## tests/
- `mlx90393_test`：MLX90393 焊接和通信诊断（检测引脚短路、扫描 I2C、读取寄存器）
- `mux_adc_test`：74HC4067 和 ADC 诊断（检测选择线短路，用上拉/下拉判断 SIG 是否悬空，统计每个通道的最小值、最大值、平均值）
