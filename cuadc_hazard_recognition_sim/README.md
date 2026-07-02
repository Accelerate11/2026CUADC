# CUADC Hazard Recognition Simulation

这是一个独立的 Gazebo 识别小场景，用于在 sim-to-real 流程中单独验证危险化学品标识识别。

它不包含无人机、ArduPilot、MAVROS 或任务状态机，只保留：

- 规则尺寸的侦察识别区：`5m x 8m`
- 5 个随机侦察桶
- 3 个随机危险化学品标识
- 附件 11 的 10 类真实危险化学品标识纹理
- 一个固定下视 RGB-D 相机
- 当前场景真值 `config/generated_scene.yaml`

这个包与全流程仿真包 `cuadc_rescue_sim` 相互独立。创建本包时只复制了一份危险品标识纹理，没有修改全流程环境。

## 快速开始

构建：

```bash
cd ~/cuadc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cuadc_hazard_recognition_sim
source install/setup.bash
```

生成随机识别场景：

```bash
cd ~/cuadc_ws/src/cuadc_hazard_recognition_sim
python3 scripts/generate_scene.py
cd ~/cuadc_ws
colcon build --packages-select cuadc_hazard_recognition_sim
```

启动：

```bash
cd ~/cuadc_ws
source install/setup.bash
ros2 launch cuadc_hazard_recognition_sim hazard_recognition.launch.py
```

后台启动：

```bash
~/cuadc_ws/src/cuadc_hazard_recognition_sim/scripts/start_hazard_recognition_sim.sh
```

停止：

```bash
~/cuadc_ws/src/cuadc_hazard_recognition_sim/scripts/stop_hazard_recognition_sim.sh
```

查看当前真值：

```bash
cat ~/cuadc_ws/src/cuadc_hazard_recognition_sim/config/generated_scene.yaml
```

## 相机话题

Gazebo 侧：

```text
/hazard_d435i/image
/hazard_d435i/depth_image
/hazard_d435i/camera_info
```

如果安装了 `ros_gz_bridge`，可启用 ROS2 桥接：

```bash
ros2 launch cuadc_hazard_recognition_sim hazard_recognition.launch.py use_bridge:=true
```

ROS2 侧目标话题：

```text
/perception/color/image
/perception/depth/image
/perception/color/camera_info
```

## 随机化

修改：

```text
config/scene.yaml
```

然后运行：

```bash
python3 scripts/generate_scene.py
```

`seed` 控制随机结果。改变 `seed` 后可得到新的桶位和标识类别。
