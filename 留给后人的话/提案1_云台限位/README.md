# AKPF 两轴无人机水平云台

这是一个可落地验证的两轴（Roll/Pitch）水平云台草案。云台挂在无人机下方，控制器读取安装在相机托架上的 ICM-42688-P IMU，驱动两个位置舵机，使托架相对地面保持水平。飞控使用 CUAV，机载计算机使用 Intel NUC；稳定闭环完全运行在 STM32 上，飞控或 NUC 暂时掉线时仍可工作。

## 选型

- 控制器：NUCLEO-G431RB（STM32G431RBT6，170 MHz Cortex-M4F）
- 云台 IMU：ICM-42688-P SPI 模块，必须使用 3.3 V 逻辑
- 执行器：2 个金属齿数字舵机，建议 7.4 V、响应时间不大于 0.12 s/60°、堵转扭矩按载荷留 2 倍以上裕量
- 舵机电源：独立 5～8.4 V BEC，建议持续 5 A 以上；BEC GND、STM32 GND、IMU GND 共地
- 上位机接口：NUCLEO 板载 ST-LINK 虚拟串口，115200 8N1，连接 Intel NUC

> 重要：普通位置舵机存在齿隙、死区和低刷新率，本方案适合“保持水平/低频扰动”的原型。若要求航拍级隔振和快速姿态补偿，应换成带编码器的无刷云台电机与 FOC 驱动，控制软件架构仍可复用，但执行器控制层需要替换。

## 工程结构

```text
firmware/                  STM32 PlatformIO 固件
  include/                 驱动、滤波、控制、协议接口
  src/                     固件实现
  test/                    可在 PC 上跑的算法单元测试
tools/                     Intel NUC 串口命令与日志工具
docs/architecture.md       坐标系、控制环与安全状态机
docs/hardware.md           接线、电源和机械要求
docs/commissioning.md      台架到实机的分阶段调试方案
docs/protocol.md           NUC 与 MCU 文本协议
```

## 快速开始

1. 按 [硬件接线](docs/hardware.md) 接 IMU，暂时不要安装舵臂。
2. 安装 VS Code + PlatformIO，或直接安装 PlatformIO Core。
3. 在 `firmware` 目录执行：

   ```powershell
   platformio run
   platformio run --target upload
   ```

4. NUC/PC 安装串口工具依赖并查看状态：

   ```bash
   python3 -m pip install -r tools/requirements.txt
   python3 tools/gimbal_cli.py --port /dev/ttyACM0 status
   python3 tools/gimbal_cli.py --port /dev/ttyACM0 monitor
   ```

5. 上电后保持云台托架静止约 2 秒。看到 `READY` 后，用 `ARM` 才会进入稳定状态。正式装舵臂前先用示波器或舵机测试仪确认脉宽和方向。

## 当前版本的边界

- 两个转轴必须近似正交，默认 Roll 为外框、Pitch 为内框。
- 航向不稳定，随无人机机体转动。
- 加速度计只能观测重力方向；持续水平加速会产生短时倾角误差。滤波器会在加速度模长偏离 1 g 时降低加速度修正权重。
- 默认最大机械行程为中心点两侧 45°。必须根据实际机构修改 `firmware/include/app_config.h` 中的脉宽、方向和角度限位。
- 当前校准保存在 RAM；断电后需要重新做陀螺零偏校准。安装误差可通过 `ZERO` 临时归零，量产版建议保存到 MCU Flash。

## 验收目标（建议）

- 静态水平误差：Roll/Pitch 均小于 1.0°
- 输入 ±20°、频率 0.5 Hz 的台架扰动：RMS 误差小于 2.0°
- IMU 通信连续失败 10 次：保持最后安全舵机位置并退出 ARMED
- 串口断开：不影响本地稳定闭环

设计依据与后续升级路径见 [系统架构](docs/architecture.md)，完整操作见 [调试方案](docs/commissioning.md)。
