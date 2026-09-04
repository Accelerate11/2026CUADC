# Vision-Servo Payload Alignment Toolkit

一个面向 ROS 2 的**视觉对准 / 投放中心标定框架**。核心只保留最有复用价值的部分：

- 把视觉节点输出统一成一个很薄的 ROS 2 接口；
- 把“目标中心准星”和“载荷真实投放中心准星”从视觉算法里拆出来；
- 支持低位铅垂线人工对准，稳健求解每个载荷在 FCU `FRD` 机体系中的有效 `X/Y`；
- 支持高位真实落点试验，用中位数 + MAD 修正静态投放中心；
- 可选保留 MAVROS 舵机地面循环测试。

> 这个仓库的原则是：**视觉节点只负责“我看到了什么、目标在机体系哪里”；准星、人工标定和投放中心几何由独立节点负责。**

## 1. 架构

```text
camera driver / your image source
       │
       ├── sensor_msgs/Image ───────────────┐
       ├── sensor_msgs/CameraInfo ──────────┤
       │                                    │
       ▼                                    ▼
YOUR VISION NODE                    alignment_viewer
(完全由使用者实现)                  绿色：目标中心
       │                             彩色：各载荷投放中心投影
       │ PoseArray / FRD                    │
       └── /vision_servo/targets_body ──────┘
                      │
                      └── 可同时给你自己的飞行/伺服控制节点使用
```

公开版不再包含旧工程中的检测模型、检测器实现和比赛专用全流程状态机。视觉接口和准星显示彻底解耦，因此别人可以直接接自己的 YOLO、传统 CV、AprilTag、深度相机、双目、VIO 或其他目标定位节点。

## 2. ROS 2 视觉接口

默认目标话题：

```text
/vision_servo/targets_body
```

类型：

```text
geometry_msgs/msg/PoseArray
```

约定：

- `header.stamp`：该帧图像的**采集时间**，必须非零；
- `header.frame_id`：默认 `fcu_body_frd`；
- `pose.position.{x,y,z}`：目标参考点在 FCU body FRD 下的位置，单位 m；
- `pose.orientation.x`：目标直径/尺度，可不用，填 `0.0`；
- `pose.orientation.y`：置信度 `[0,1]`，不知道时填 `1.0`；
- `pose.orientation.z`：可选深度诊断值，不用填 `0.0`；
- `pose.orientation.w`：接口版本哨兵，填 `1.0`。

坐标固定使用 FCU/MAVLink `FRD`：

- `+X`：机头前；
- `+Y`：机体右；
- `+Z`：向下。

完整说明见 [`docs/VISION_INTERFACE.md`](docs/VISION_INTERFACE.md)。

## 3. 为什么准星必须独立于视觉节点

原来最有价值的思路并不是某一个检测模型，而是两套几何量同时显示：

1. **目标中心**：视觉节点给出的目标三维位置投影到当前相机画面；
2. **Payload projection**：每个载荷在机体上的标定 `X/Y` 垂线，与当前目标对准平面的交点再投回相机画面。

两者只有在真实载荷投放中心位于目标正上方时才应重合。载荷准星不能从目标 `x/y` 生成，否则准星会“跟着目标跑”，反而把对准误差隐藏掉。

公开版中的 `alignment_viewer` 只依赖：

- 相机图像 + `CameraInfo`；
- `camera -> FCU body FRD` 外参；
- 视觉节点发布的目标 FRD 坐标；
- 每个载荷的 FRD 投放中心。

因此检测算法可以被完全替换。

## 4. 快速开始

环境建议：Ubuntu 22.04 + ROS 2 Humble。

### 4.1 准备配置

```bash
cp config/aircraft.example.yaml config/aircraft.yaml
# 编辑 config/aircraft.yaml
python3 tools/validate_config.py --config config/aircraft.yaml --mode alignment
python3 tools/generate_config.py
```

最关键的配置是：

```yaml
camera:
  image_topic: /camera/color/image_raw
  camera_info_topic: /camera/color/camera_info
  rotation_camera_to_frd: [... 9 values ...]
  translation_frd_m: [x, y, z]

vision:
  detections_topic: /vision_servo/targets_body
  frame_id: fcu_body_frd

payloads:
  release_offsets_frd_m:
    [x1, y1, z1,
     x2, y2, z2]

target:
  # 视觉发布的参考点到真正“对准平面”的竖直高度。
  # 例如视觉发布桶底/地面中心，而桶口高 0.30 m，则填 0.30。
  # 若视觉已经直接发布桶口平面上的 3D 点，则填 0.0。
  height_m: 0.30
```

### 4.2 编译

```bash
./bin/build.sh
```

### 4.3 启动你的相机和视觉节点

公开版**不会替你启动相机，也不会加载任何模型**。先启动你自己的相机节点，再启动你自己的视觉节点，使其持续发布上述 `PoseArray`。

仓库内提供一个空模板：

```bash
./bin/run_vision_template.sh
```

真正接入时通常直接复制 `vision_provider_template.py`，把 `detect_targets_body_frd()` 函数替换成自己的算法即可；如果你已经有独立节点，只要话题格式满足契约，模板完全不用运行。

先检查接口：

```bash
./bin/check_vision_contract.sh
```

### 4.4 启动准星画面

```bash
./bin/visual_alignment.sh
```

画面会显示：

- 绿色十字：视觉目标中心；
- 不同颜色十字：各 Payload 的真实投放中心在当前目标平面上的投影；
- `+X FWD` / `+Y RIGHT`：当前配置对应的机体系方向；
- 每个 Payload 的 `dF / dR` 误差。

本脚本只启动准星显示节点，不启动飞控、不解锁、不飞行。

## 5. 静态投放中心标定

以 Payload 1 为例：

1. 低位固定飞机并保持姿态稳定；
2. 从真实投放机构中心垂下铅垂线；
3. 人工移动整架飞机，让铅垂线落到目标中心；
4. 保持 3 秒以上；
5. 退出 `visual_alignment.sh`；
6. 使用这一轮自动保存的检测 CSV：

```bash
python3 tools/estimate_static_offsets.py \
  logs/static_alignment_YYYYMMDD_HHMMSS/detections.csv \
  --payload 1 \
  --aircraft config/aircraft.yaml \
  --last-seconds 3 \
  --write
```

脚本用最近稳定窗口的 FRD `X/Y` 中位数作为该载荷的静态投放中心，**不会修改 Z**。Z 仍应由机械测量获得。

然后重新生成准星参数：

```bash
python3 tools/generate_config.py
```

对 Payload 2 重复一次。

## 6. 高位真实落点修正

静态几何不能覆盖舵机释放延迟、瓶体摆动、残余水平速度、气流等误差。真实投放后按 FRD 方向记录第一落点相对目标中心的偏差：

```bash
python3 tools/record_drop_trial.py \
  --payload 1 \
  --height 0.8 \
  --miss-forward 0.03 \
  --miss-right -0.02 \
  --output logs/drop_trials.csv
```

每个载荷、每个最终使用高度建议至少 3 次，再求解：

```bash
python3 tools/solve_drop_trials.py \
  logs/drop_trials.csv \
  --aircraft config/aircraft.yaml \
  --output logs/drop_solution.yaml
```

规则：

```text
recommended XY = static XY + median landing miss XY
```

同时输出 MAD 作为重复性指标。

## 7. 舵机地面检查（可选）

如果你仍使用 MAVROS/ArduPilot，并在 `aircraft.yaml` 中填写了 FCU 与舵机参数，可在**卸桨、飞控未解锁**状态运行：

```bash
./bin/servo_ground_check.sh
```

这部分与准星/视觉接口相互独立，不需要使用。

## 8. 目录

```text
config/
  aircraft.example.yaml       用户唯一需要维护的物理/话题配置模板
  alignment_base.yaml         准星节点基础参数
bin/
  build.sh                    编译 ROS 2 工作区
  visual_alignment.sh         只启动独立准星节点
  run_vision_template.sh      启动“空视觉节点”模板
  check_vision_contract.sh    检查别人接入的视觉节点是否满足接口
  servo_ground_check.sh       可选 MAVROS 舵机地面测试
tools/
  generate_config.py
  validate_config.py
  estimate_static_offsets.py
  record_drop_trial.py
  solve_drop_trials.py
  servo_cycle_test.py
ros2_ws/src/vision_servo_calibration/
  vision_servo_calibration/alignment_viewer_node.py
  vision_servo_calibration/vision_provider_template.py
  vision_servo_calibration/vision_contract_check.py
  vision_servo_calibration/projection.py
  launch/alignment.launch.py
docs/
  VISION_INTERFACE.md
  DESIGN.md
  MIGRATION_FROM_OLD_WORKSPACE.md
```

## 9. 发布到 GitHub 前

`.gitignore` 已排除：

- `build/install/log`；
- `config/aircraft.yaml`；
- 生成参数；
- 飞行/标定日志；
- Python 缓存。

这个公开版也已经移除原始检测模型、ONNX/PT 文件、原视觉算法和比赛专用全流程节点。请在正式发布前自行选择并写入开源许可证；本整理版没有替项目作者擅自改变许可证。

## 10. 安全边界

本仓库的核心功能是**测量、可视化和标定**。任何自动飞行、自动投放、舵机动作都应由使用者自己的飞行控制程序承担，并自行设置硬件急停、围栏、人工接管和测试流程。
