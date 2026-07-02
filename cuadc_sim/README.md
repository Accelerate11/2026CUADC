# CUADC Simulation

面向 2026 中国大学生飞行器设计创新大赛“多旋翼无人机侦察与救援”赛项的 Gazebo + ArduPilot SITL + ROS2 仿真环境。

本仓库目标是复现比赛场地和任务流程，用于训练与验证：

- 多旋翼全自动飞行流程
- 投放区圆筒定位与虚拟投放
- 侦察区危险化学品标识识别
- 下视 RGB-D 相机接口
- 后续实机 D435i 与投放机构的软件接口迁移

当前版本不模拟水瓶真实下落，不模拟舵机机械结构，只提供统一的虚拟投放事件 `/drop_controller/release`。实机阶段可将该事件替换为飞控 PWM 或舵机控制输出。

## 当前状态

已完成：

- 独立 ROS2 包 `cuadc_rescue_sim`
- 规则尺寸场地：起降区、起飞线、投放区、侦察区
- 派生 Iris 模型 `iris_d435i`
- 下视 RGB-D 相机传感器
- 3 个随机投放桶
- 5 个随机侦察桶
- 3 个随机危险化学品标识贴图
- 附件 11 的 10 类真实危险化学品标识纹理
- Gazebo world 自动生成脚本
- 虚拟投放裁判节点骨架
- 任务演示节点骨架

## 目录

```text
cuadc_rescue_sim/
├── config/                 # 场景、任务、桥接配置
├── docs/                   # 详细教程和归档说明
├── launch/                 # ROS2 launch
├── models/                 # Gazebo 模型和纹理
├── scripts/                # 生成、启动、停止、归档脚本
├── src/                    # ROS2 C++ 节点
├── worlds/                 # Gazebo world
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 快速开始

构建：

```bash
cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cuadc_rescue_sim
source install/setup.bash
```

生成一轮随机场景：

```bash
cd ~/cuadc_ws/src/cuadc_rescue_sim
python3 scripts/generate_scene.py
cd ~/cuadc_ws
colcon build --packages-select cuadc_rescue_sim
```

启动 Gazebo：

```bash
cd ~/cuadc_ws
source install/setup.bash
ros2 launch cuadc_rescue_sim cuadc_sim.launch.py
```

另开终端启动 ArduPilot SITL：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

后台启动 Gazebo：

```bash
~/cuadc_ws/src/cuadc_rescue_sim/scripts/start_cuadc_sim.sh
```

停止 Gazebo：

```bash
~/cuadc_ws/src/cuadc_rescue_sim/scripts/stop_cuadc_sim.sh
```

## 详细文档

- [完整配置教程](docs/SETUP.md)
- [配置文件说明](docs/CONFIGURATION.md)
- [规则映射说明](docs/RULE_MAPPING.md)
- [GitHub 归档说明](docs/GITHUB_ARCHIVE.md)

## 重要路径

```text
~/cuadc_ws/src/cuadc_rescue_sim
~/cuadc_ws/src/cuadc_rescue_sim/worlds/cuadc_rescue_single.sdf
~/cuadc_ws/src/cuadc_rescue_sim/config/generated_scene.yaml
~/cuadc_ws/src/cuadc_rescue_sim/models/hazard_marker/materials/textures
```

## 上游来源说明

`models/iris_d435i` 与 `models/iris_d435i_airframe` 基于本机 `~/ardupilot_gazebo` 中的 Iris 模型派生，仅增加下视 RGB-D 相机并修改模型命名。上游 `ardupilot_gazebo/LICENSE.md` 为 LGPL v3，归档到 GitHub 时请保留来源说明。

危险化学品标识纹理来自比赛附件 11，用于本赛项仿真训练。
<img width="3837" height="2157" alt="6<img width="3837" height="2157" alt="a864ee056c372db0712d3d0863653156" src="https://github.com/user-attachments/assets/5210dc1b-a3b4-4b75-aadb-2510ce471028" />
8edb154916b6c0f021813433ca1e971" src="https://github.com/user-attachments/assets/77935fdc-058f-4251-999c-0f17d24ac886" />
