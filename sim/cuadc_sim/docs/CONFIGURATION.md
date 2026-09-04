# 配置文件说明

## config/scene.yaml

`scene.yaml` 是场景源配置。它定义规则尺寸、随机种子和任务区域。

关键字段：

```yaml
seed: 2026
field:
  takeoff_center: [0.0, 0.0, 0.0]
  takeoff_line_x: 2.0
  takeoff_area_size: [33.0, 8.0]
  drop_area_center: [30.0, 0.0, 0.0]
  drop_area_size: [5.0, 8.0]
  recon_area_center: [55.0, 0.0, 0.0]
  recon_area_size: [5.0, 8.0]
```

坐标约定：

```text
x: 比赛前进方向
y: 场地横向
z: 高度
H 点中心: (0, 0, 0)
起飞线: x = 2 m
投放区中心: x = 30 m
侦察区中心: x = 55 m
```

注意：规则写“8m 长、5m 宽”，结合场地示意和“投放区前方 20m 为侦察区、侦察区距起降点 55m”，在本 ENU 坐标中实现为：

```text
x 方向 5 m
y 方向 8 m
```

因此 `drop_area_size` 与 `recon_area_size` 为 `[5.0, 8.0]`。

## config/generated_scene.yaml

`generated_scene.yaml` 是生成后的真值文件。它记录当前 world 中每个桶和危险品标识的实际位置。

示例：

```yaml
drop_targets:
  drop_1:
    x: 28.44
    y: 0.02
    radius: 0.075
    score: 500
recon_targets:
  recon_1:
    x: 55.41
    y: 0.4
    marker: biohazard
```

用途：

- 第一阶段任务节点可直接读取真值验证飞控流程
- 虚拟投放裁判可根据桶位置判定误差
- 后续视觉识别结果可与真值对比

## config/mission_params.yaml

任务演示参数：

```yaml
mission:
  takeoff_altitude: 5.0
  cruise_altitude: 5.0
  waypoint_hold_seconds: 2.0
  drop_targets: [drop_1, drop_2]
  recon_hover_seconds: 8.0
  landing_altitude: 1.0
```

第一版节点还没有完全消费这些参数，后续状态机迭代时应逐步接入。

## config/camera_bridge.yaml

Gazebo 到 ROS2 的图像桥接目标：

```text
Gazebo:
/d435i/image
/d435i/depth_image
/d435i/camera_info

ROS2:
/perception/color/image
/perception/depth/image
/perception/color/camera_info
```

当前 `ros_gz_bridge` 没安装时，不启用桥接。

## scripts/generate_scene.py

该脚本负责：

- 读取 `config/scene.yaml`
- 在投放区随机生成 3 个桶
- 在侦察区随机生成 5 个桶
- 从 10 类危险化学品标识中随机选 3 类
- 随机选择 3 个侦察桶贴标识
- 输出 `config/generated_scene.yaml`
- 重写 `worlds/cuadc_rescue_single.sdf`

重新生成：

```bash
cd ~/cuadc_ws/src/cuadc_rescue_sim
python3 scripts/generate_scene.py
```

改变随机结果：

```yaml
seed: 2027
```

改完后重新执行脚本即可。

## models/hazard_marker/materials/textures

包含附件 11 的 10 类危险化学品标识：

```text
explosive.png
nonflammable_gas.png
irritant.png
radioactive.png
corrosive.png
biohazard.png
dangerous_when_wet.png
toxic.png
spontaneously_combustible.png
flammable.png
```

后续如果你裁剪了更干净的训练图，只要保持文件名不变即可替换。
