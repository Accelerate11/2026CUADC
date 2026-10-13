# CUADC 全任务工作区 · V16

此文件夹用来保存 2026 年多旋翼无人机侦察与救援项目的 V16 完整代码。以提供的 `CUADCv16.zip` 为源码基线，将原来分散的任务入口、标定、视觉、舵机测试和复盘工具整理到同一个 `ws_offboard` 工作区，方便后人学习和复现。

这次完整公开任务状态机、视觉处理、相机时间同步、目标选择、分段投放判定、飞机参数以及全部模型。核心 C++、头文件、包内 Python 算法、原始参数和模型均保留原文件字节，没有删掉关键逻辑，也没有用占位实现替换。来源和调整逐项记录在 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。

任务流程为：起飞前准备 → 等待 GUIDED → 解锁起飞 → 搜索桶 → 两次投放 → 灾情区域航点侦察 → 返航 → 降落上锁。侦察阶段沿用 V16 的航点覆盖，没有接入危险物识别节点。

## 先看版本

工作区名称保留本次整理开始时的 `2026_10_11_v16`；最终验证记录更新于 2026-10-13。代码基线是 V16，不能与目录中的 V17 或旧版 `ws_offboard_2026_10_05` 混用。

| 项目 | 本版内容 |
| --- | --- |
| 系统 | Ubuntu 22.04、ROS 2 Humble、Python 3.10、C++17 |
| 飞控通信 | ArduPilot / MAVROS，原 V16 时间同步配置 |
| 相机 | Intel RealSense D435i |
| 默认任务模型 | `basket_detect.onnx`；同时提供对应 OpenVINO 模型 |
| 原任务配置 | Intel 核显推理，允许启动失败或推理故障后回退 CPU |
| 源码许可 | AGPL-3.0-only，第三方保留各自许可 |
| 本次验证 | ROS 编译、离线回归、安装入口、模型完整性及 ONNX CPU 合成输入推理；详见验证记录 |

## 目录说明

```text
ws_offboard_2026_10_11_v16/
├── src/cuadc_full_mission_20260803/  # V16 原 ROS 包，包名保留
│   ├── src/                         # 完整状态机、分段投放判定
│   ├── scripts/                     # 视觉、相机时间、遥测、消息频率工具
│   ├── models/                      # ONNX、PT、OpenVINO 权重
│   ├── config/                      # 原包参数、MAVROS 时间同步参数
│   ├── launch/                      # 任务和 MAVROS 启动
│   └── test/                        # 原回归测试及独立投放门限测试
├── full_mission/                    # 完整任务与对应 mission.yaml
├── drop_only/                       # 仅投放与对应 mission.yaml
├── fixed_release_test/              # 固定坐标投放入口
├── indoor_projection/               # 室内投影检查入口
├── servo_test/                      # 双舵机检查入口
├── routes/                          # 标定工具及合成格式示例
├── tools/                           # 启动守护、预检、时钟诊断、复盘
├── scripts/                         # 编译、测试、校验、环境记录
├── environment/                     # 安装依赖、环境检查与模型验证记录
├── tests/                           # 部署回归
└── docs/                            # 原理、参数、迁移、复现与验证
```

## 依赖与编译

先按 [环境说明](environment/README.md) 准备 ROS、MAVROS 和 Python 依赖。以下命令从工作区根目录执行，不依赖原机器的用户名或绝对路径：

```bash
cd ~/ws_offboard_2026_10_11_v16
python3 scripts/validate_source.py
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
bash scripts/build_onboard.sh
bash scripts/verify.sh
```

校验脚本只依赖 Python 标准库。编译脚本使用已有环境，不会自动安装软件，也不会连接飞控。`verify.sh` 执行离线测试和安装入口检查，不启动任务。只有源码检查环境时可用 `bash scripts/verify.sh --source-only`，其中视觉单元测试仍需要 NumPy 和 OpenCV。

本包已经提供模型，正常运行不需要现场重新训练或转换。原始训练数据、完整训练工程、飞控固件和原机参数导出没有包含在这次输入中；本包的复现范围是提供的 V16 运行代码，详见 [复现说明](docs/REPRODUCIBILITY.md)。

## 每轮独立标定

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 routes/calibrate_route.py \
  --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200
```

把 `YOUR_FCU` 替换为本机飞控设备，波特率与实际配置一致。按提示记录两点，间距必须大于 10 m，保存为 `round_01`。`routes/example.yaml` 仅用于说明格式，不能当成本轮标定结果。

## 运行完整任务

先核对 [飞机参数说明](docs/CONFIGURATION.md)。`full_mission/mission.yaml` 保留原 V16 的相机、投放口外参、舵机通道、PWM、速度、高度和自动解锁设置，必须与实际安装一致。

```bash
# 静态预检，不启动 MAVROS 或任务节点
bash full_mission/run_full_mission.sh round_01 \
  --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 \
  --camera-serial YOUR_D435I_SERIAL --preflight

# 启动完整任务，自动曝光
bash full_mission/run_full_mission.sh round_01 \
  --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 \
  --camera-serial YOUR_D435I_SERIAL expo=0
```

`round_01` 是路线名，不是数字航向。相机序列号用本机实际值替换。准备完成并显示 `READY_FOR_GUIDED` 后，由飞手切入 GUIDED，原状态机继续自动解锁起飞。首次适配按拆桨检查、仿真、外场的顺序完成，飞手保留人工接管能力。

没有 Intel 核显或需要对照 CPU 时，显式在启动命令前加 `CUADC_VISION_DEVICE=cpu`。`expo=20` 可指定曝光，省略则沿用本入口 YAML。无桌面时将对应 YAML 的 `live_view_enabled` 改为 `false`。所有入口都支持用 `bash` 调用，不要求解压后另行恢复执行权限。

## 其他入口

| 功能 | 说明 |
| --- | --- |
| 仅投放 | [drop_only/README.md](drop_only/README.md) |
| 固定坐标投放 | [fixed_release_test/README.md](fixed_release_test/README.md) |
| 室内投影检查 | [indoor_projection/README.md](indoor_projection/README.md) |
| 舵机测试 | [servo_test/README.md](servo_test/README.md) |
| 航向标定 | [routes/README.md](routes/README.md) |
| 时钟诊断与本地复盘 | [tools/README_flight_summary.md](tools/README_flight_summary.md) |

完整任务每轮日志在 `full_mission/logs/flight_<时间>/`，仅投放日志在 `drop_only/logs/`。保留运行参数、飞行遥测、视觉与投影 CSV、录像、投放快照和本地总结。邮件功能保留为手动 `--send`，默认不发送。

更多说明：[系统与算法](docs/ARCHITECTURE.md) · [参数](docs/CONFIGURATION.md) · [V16 来源对照](docs/MIGRATION.md) · [复现范围](docs/REPRODUCIBILITY.md) · [验证结果](docs/VALIDATION.md) · [常见问题](docs/TROUBLESHOOTING.md)。
