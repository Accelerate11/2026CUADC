# CUADC 全任务工作区 · 2026-10-05

任务流程：等待飞控与 GUIDED → 锁定坐标系 → 预发送 → 解锁起飞 → 搜索桶 → 对准与两次投放 → 航点侦察 → 爬升返航 → 降落上锁。任务节点包含导航、舵机控制和状态检查；侦察阶段使用航点，没有危险物识别节点。

## 目录

```text
ws_offboard_2026_10_05/
├── src/
│   ├── cuadc_mission/       # 原 C++ 任务状态机
│   ├── cuadc_perception/    # D435i 视觉、分析模块和三个模型
│   ├── cuadc_tools/         # 航向标定、MAVLink 频率设置、遥测
│   └── cuadc_bringup/       # 全任务 launch、参数、航向加载
├── scripts/                # 编译、标定、运行、校验入口
├── routes/                 # 本轮标定文件；随包只有合成示例
├── docs/                   # 架构、迁移、参数、验证、来源
├── tests/                  # 迁移后的路径、航向和 launch 回归
├── requirements-onboard.txt
├── dependencies.repos
├── SOURCE_MANIFEST.json    # 原文件与新文件路径、SHA-256 对照
├── MODEL_SHA256.json       # 三个原模型的 SHA-256
├── SHA256SUMS              # 发布源码文件校验清单
├── VERSION.txt
└── LICENSE                 # AGPL-3.0-only
```

## 依赖和编译

目标环境沿用原包：Ubuntu 22.04、ROS 2 Humble、ArduPilot/MAVROS、Intel RealSense D435i。默认视觉参数为 CPU，原依赖清单按 NUC11 整理。Jetson 或其他架构需要匹配该平台的 PyTorch、OpenCV 和 RealSense 运行环境。

ROS 依赖在四个 `package.xml` 中声明，Python 运行依赖在 `requirements-onboard.txt` 中列出。该清单保留原包版本，不会安装其注释列出的 NumPy、OpenCV、PyTorch 和 pyrealsense2；这些也需要在机载环境准备好。编译脚本不联网安装软件。

```bash
cd ~/ws_offboard_2026_10_05
python3 scripts/validate_source.py
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
# 使用已经准备好的机载 Python 环境：
python3 -m pip install -r requirements-onboard.txt
bash scripts/build_onboard.sh
```

若首次使用 rosdep，先按 [rosdep 文档](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) 初始化并更新其数据库。已独立准备好依赖、但测试环境没有 rosdep 数据库时，可用 `CUADC_SKIP_ROSDEP_CHECK=1 bash scripts/build_onboard.sh` 明确跳过这一步。跳过不代表依赖检查通过。

Humble 的 Python 包通过 colcon 和 setuptools 构建。若用户目录中的新版 setuptools 造成 `option --editable not recognized`，可用 `PYTHONNOUSERSITE=1 bash scripts/build_onboard.sh` 选择 Ubuntu 自带的 Python 构建环境；该设置只影响当前命令，不修改系统或已有 Python 环境。

## 每轮独立标定

查看设备，再用实际串口 URL 执行标定；不要同时运行另一个占用同一串口的 MAVROS。

```bash
bash scripts/check_devices.sh
bash scripts/calibrate_route.sh \
  serial:///dev/serial/by-id/YOUR_FCU:115200
```

按工具提示记录两点，间距必须大于 10 m，再输入文件名，例如 `round_01`。保存到 `routes/round_01.yaml`；可追加 `--output-dir /your/routes`。串口、波特率需要与实际飞控一致，示例中的 `YOUR_FCU` 必须替换。

标定得到的 `heading_deg` 是正北为 0°、顺时针增加的地理航向。两个 GPS 点用于计算航向；本版本启动器只读取航向，不实现旧 `routes/README.md` 所述的点 1 GPS 偏移检查。实际任务原点和返航点仍由原任务节点锁定。

## 运行全任务

先核对 [配置说明](docs/CONFIGURATION.md) 中的飞机安装参数。随包保留的是原飞机的实机配置：`flight_enable: true`、`auto_arm_on_guided: true`、舵机 7/8；满足原节点条件后，切 GUIDED 会进入自动解锁起飞流程。

```bash
bash scripts/run_flight.sh routes/round_01.yaml \
  serial:///dev/serial/by-id/YOUR_FCU:115200 \
  camera_serial:=YOUR_D435I_SERIAL
```

也可直接使用 ROS 入口：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cuadc_bringup flight.launch.py \
  route_file:=/absolute/path/round_01.yaml \
  fcu_url:=serial:///dev/serial/by-id/YOUR_FCU:115200
```

`route_file` 和 `fcu_url` 是必填参数。launch 在进程启动前读取并校验航向，然后覆盖原参数中的 `geographic_heading_deg: -1.0`。不会自动选择压缩包里的旧测试航线。

可用 `params_file:=/absolute/path/mission.yaml` 选择参数副本，`model_path:=/absolute/path/model.onnx` 选择模型，`log_dir:=/absolute/path/logs` 选择日志目录。`camera_serial` 留空时沿用配置中的相机设置；原配置也为空时由原视觉节点选择设备。已有 MAVROS 时可传 `start_mavros:=false`。无图形桌面时，在参数副本中将 `live_view_enabled` 设为 `false`。

默认模型仍是 `basket_detect.onnx`；单独运行视觉模块时的默认模型为原 `basket_v3.pt`。两种默认值均沿用原包设计。不要仅因文件名相似就替换模型。

日志默认写入当前工作区的 `log/flight_时间戳/`，包含配置与航向快照、启动覆盖值、遥测、检测/投影 CSV、视觉录像及投放快照。任务节点退出时 launch 关闭其余进程。

## 开源和验证

本版项目代码采用 **AGPL-3.0-only**，第三方依赖保持各自许可；详见 [LICENSES.md](LICENSES.md)。原压缩包中的个人串口标识和真实 GPS 测试航线已从发布内容中移除。三个模型保留完整二进制文件，未改为下载占位符。

```bash
python3 scripts/validate_source.py  # 无需 ROS，检查源码和模型完整性
bash scripts/verify.sh             # 需要 Humble、NumPy、OpenCV、PyYAML
sha256sum -c SHA256SUMS
```

验证范围和结果见 [docs/VALIDATION.md](docs/VALIDATION.md)。本次整理验证源码、编译和安装入口；没有连接飞机或相机进行实机验证。

更多说明：[包职责与接口](docs/ARCHITECTURE.md) · [迁移对照](docs/MIGRATION.md) · [参数](docs/CONFIGURATION.md) · [模型来源](docs/MODELS.md)。
