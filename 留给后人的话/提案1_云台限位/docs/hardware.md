# 硬件、接线与供电

## 1. NUCLEO-G431RB 引脚

| 功能 | STM32 引脚 | Nucleo 标识 | 连接 |
|---|---:|---:|---|
| ICM SPI SCK | PA5 | D13 | IMU SCLK |
| ICM SPI MISO | PA6 | D12 | IMU SDO |
| ICM SPI MOSI | PA7 | D11 | IMU SDI |
| ICM SPI CS | PB6 | D10 | IMU nCS |
| Roll PWM | PA0 / TIM2_CH1 | A0 | Roll 舵机信号 |
| Pitch PWM | PA1 / TIM2_CH2 | A1 | Pitch 舵机信号 |
| 调试 UART TX | PA2 / LPUART1_TX | 板载 VCP | 到 NUC USB |
| 调试 UART RX | PA3 / LPUART1_RX | 板载 VCP | 从 NUC USB |
| IMU 电源 | 3V3 | 3V3 | IMU VCC（确认模块无 5 V 要求） |
| 公共地 | GND | GND | IMU、BEC、两舵机共地 |

PA5 同时连接 Nucleo 用户 LED，SPI 时 LED 可能发暗光，这是正常现象。若具体扩展板在 PA5 上有较重负载，改用另一组 SPI 引脚并同步修改固件。

## 2. 电源拓扑

```text
无人机电池
  ├─> 飞控电源模块 ─> CUAV
  ├─> NUC 稳压电源  ─> Intel NUC
  └─> 独立 BEC 5~8.4 V ─┬─> Roll 舵机 V+
                         └─> Pitch 舵机 V+

BEC GND ─ STM32 GND ─ IMU GND ─ CUAV/NUC 接口地（单点/低阻连接）
```

禁止从 Nucleo 5 V 引脚给两个舵机供电。舵机启动和堵转电流会导致 MCU 复位、IMU 毛刺甚至烧毁 USB 端口。BEC 输出电压必须落在舵机额定范围内，靠近舵机电源分支放置低 ESR 470～1000 µF 电解电容和 0.1 µF 陶瓷电容。

初次上电使用带电流限制的台式电源，限流从 0.5 A 起，仅接控制板和 IMU；接舵机后根据铭牌逐步放宽。确认所有地线连接后再接 PWM 信号。

## 3. 机械要求

- 先做静态配平：断电时相机托架在各轴不应明显自行坠落。配平差会直接变成舵机持续电流和发热。
- IMU 要刚性固定在内框，远离舵机齿轮冲击和 BEC 电感；不要把 IMU 装在软泡棉上造成相位滞后。
- 无人机与云台之间可做整体减振，但控制用 IMU 与被控载荷之间必须刚性连接。
- 两轴全行程手动转动，确认线束不会拉扯、卡住或进入桨叶区域。
- 软件默认只允许 ±45°，机械上仍需保留至少 5° 裕量。

## 4. CUAV 与 NUC

推荐 CUAV 通过独立 USB 或隔离/正确电平的 TELEM UART 接 NUC，STM32 Nucleo 通过板载 ST-LINK USB 接 NUC。不要把 CUAV 的 5 V UART 电源与 BEC 输出硬并联。若使用裸 UART：先核对具体 CUAV 型号端口是否为 3.3 V TTL，TX/RX 交叉并共地。

NUC 同时会看到两个设备，例如：

- `/dev/ttyACM0`：NUCLEO 虚拟串口
- `/dev/ttyACM1` 或 `/dev/ttyUSB0`：CUAV MAVLink

用 `/dev/serial/by-id/` 的稳定设备名部署，避免重启后 ACM 编号互换。

NUCLEO-G431RB 默认焊桥把 ST-LINK VCP 接到 LPUART1 的 PA2/PA3，本工程按该默认配置编写。若板卡焊桥已被改到 USART1 PC4/PC5，需恢复默认焊桥或同步修改 `main.cpp` 的 UART 实例、GPIO 复用和中断入口。
