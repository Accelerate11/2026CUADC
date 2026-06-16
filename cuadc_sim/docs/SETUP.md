# 完整配置教程

本文档说明如何在一台已经具备 Ubuntu 22.04、ROS2 Humble、Gazebo Harmonic、ArduPilot SITL 的机器上恢复并运行本仿真环境。

## 1. 环境要求

推荐环境：

```text
Ubuntu 22.04
ROS2 Humble
Gazebo Harmonic / gz sim 8.x
ArduPilot SITL
ardupilot_gazebo
MAVROS for ROS2
```

本项目当前按以下本机路径组织：

```text
~/ardupilot
~/ardupilot_gazebo
~/cuadc_ws/src/cuadc_rescue_sim
```

如果你放在其他路径，需要同步修改：

```text
launch/cuadc_sim.launch.py
README.md 中的命令示例
```

## 2. 基础依赖检查

检查 ROS2：

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
ros2 --help
```

检查 Gazebo：

```bash
gz sim --versions
gz sim --help
```

检查 ArduPilot Gazebo 插件：

```bash
ls ~/ardupilot_gazebo/build
ls ~/ardupilot_gazebo/models
ls ~/ardupilot_gazebo/worlds
```

需要能看到：

```text
models/iris_with_ardupilot
models/iris_with_standoffs
worlds/iris_runway.sdf
```

检查 MAVROS：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix mavros
ros2 pkg prefix mavros_msgs
```

## 3. 放置源码

推荐建立工作区：

```bash
mkdir -p ~/cuadc_ws/src
```

将本仓库放到：

```text
~/cuadc_ws/src/cuadc_rescue_sim
```

目录应类似：

```bash
ls ~/cuadc_ws/src/cuadc_rescue_sim
```

输出应包含：

```text
CMakeLists.txt
package.xml
config
docs
launch
models
scripts
src
worlds
```

## 4. 构建

```bash
cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cuadc_rescue_sim
source install/setup.bash
```

成功时会看到：

```text
Summary: 1 package finished
```

## 5. 生成随机比赛场景

场景由：

```text
config/scene.yaml
scripts/generate_scene.py
```

生成：

```text
config/generated_scene.yaml
worlds/cuadc_rescue_single.sdf
```

执行：

```bash
cd ~/cuadc_ws/src/cuadc_rescue_sim
python3 scripts/generate_scene.py
```

重新安装到 `install/`：

```bash
cd ~/cuadc_ws
colcon build --packages-select cuadc_rescue_sim
```

查看当前随机真值：

```bash
cat ~/cuadc_ws/src/cuadc_rescue_sim/config/generated_scene.yaml
```

## 6. 启动 Gazebo

前台启动：

```bash
cd ~/cuadc_ws
source install/setup.bash
ros2 launch cuadc_rescue_sim cuadc_sim.launch.py
```

后台启动：

```bash
~/cuadc_ws/src/cuadc_rescue_sim/scripts/start_cuadc_sim.sh
```

查看日志：

```bash
tail -100 /tmp/cuadc_rescue_sim.log
```

停止：

```bash
~/cuadc_ws/src/cuadc_rescue_sim/scripts/stop_cuadc_sim.sh
```

## 7. 启动 ArduPilot SITL

另开一个终端：

```bash
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
```

如果 Gazebo 与 SITL 已连接，ArduPilot 控制台会进入正常仿真状态。

## 8. 可选：安装 ROS-Gazebo 图像桥接

当前本机尚未安装 `ros_gz_bridge`，所以 `cuadc_sim.launch.py` 默认不开桥接：

```bash
ros2 launch cuadc_rescue_sim cuadc_sim.launch.py
```

如果后续安装了桥接包，可启用：

```bash
ros2 launch cuadc_rescue_sim cuadc_sim.launch.py use_bridge:=true
```

安装包名可能因 Gazebo/ROS 源不同而不同，常见选择：

```bash
sudo apt install ros-humble-ros-gz-bridge
```

或使用 Gazebo Harmonic 组合包：

```bash
sudo apt install ros-humble-ros-gzharmonic
```

安装后验证：

```bash
source /opt/ros/humble/setup.bash
ros2 pkg executables ros_gz_bridge
```

## 9. 常见问题

### Gazebo 找不到模型

确认资源路径：

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/cuadc_ws/install/cuadc_rescue_sim/share/cuadc_rescue_sim/models:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

如果用 `ros2 launch` 启动，launch 文件会自动设置这些路径。

### ArduPilotPlugin 找不到

确认：

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

### 生成了新场景但 Gazebo 里没变

生成后需要重新 build，并重启 Gazebo：

```bash
cd ~/cuadc_ws/src/cuadc_rescue_sim
python3 scripts/generate_scene.py
cd ~/cuadc_ws
colcon build --packages-select cuadc_rescue_sim
~/cuadc_ws/src/cuadc_rescue_sim/scripts/stop_cuadc_sim.sh
~/cuadc_ws/src/cuadc_rescue_sim/scripts/start_cuadc_sim.sh
```
