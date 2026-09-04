# Vision Provider Interface

## 1. 目标

视觉提供者（Vision Provider）与准星/标定逻辑之间只通过标准 ROS 2 消息连接。视觉提供者可以是任何算法，只要最终把目标三维点转换到 FCU body FRD 并发布。

默认：

- topic: `/vision_servo/targets_body`
- type: `geometry_msgs/msg/PoseArray`
- frame: `fcu_body_frd`

## 2. PoseArray 字段

| 字段 | 含义 | 必需 |
|---|---|---:|
| `header.stamp` | 原始图像采集时间 | 是 |
| `header.frame_id` | `fcu_body_frd` | 是 |
| `pose.position.x` | 目标参考点，FRD 前向，m | 是 |
| `pose.position.y` | 目标参考点，FRD 右向，m | 是 |
| `pose.position.z` | 目标参考点，FRD 下向，m | 是 |
| `pose.orientation.x` | 目标尺度/直径，m；未知可 0 | 否 |
| `pose.orientation.y` | 置信度 `[0,1]`；未知填 1 | 建议 |
| `pose.orientation.z` | 视觉节点自定义深度诊断值 | 否 |
| `pose.orientation.w` | 契约版本哨兵 `1.0` | 是 |

`orientation` **不是四元数**，只借用 `PoseArray` 的四个标量避免引入自定义消息包。

## 3. 空检测帧

每处理一帧有效图像，都建议发布一次 `PoseArray`。没有目标时：

```text
poses = []
```

仍应保留正确的 `header.stamp` 和 `header.frame_id`。这样下游可以区分“视觉节点活着但没目标”与“视觉节点已经停止”。

## 4. 时间戳

`header.stamp` 应对应产生这些检测结果的**图像采集时间**，不要在推理结束时重新取 `now()`。如果后续把同一话题用于飞行控制，这一点关系到机体运动补偿和历史里程计插值。

## 5. 坐标变换

若算法先得到相机光学坐标 `p_camera`，配置定义：

```text
p_body_frd = R_body_camera @ p_camera + t_body_camera
```

其中：

- `R_body_camera` 为 `camera.rotation_camera_to_frd`；
- `t_body_camera` 为 `camera.translation_frd_m`；
- 相机光学坐标通常为 `+X` 图像右、`+Y` 图像下、`+Z` 光轴前；
- body FRD 为 `+X` 前、`+Y` 右、`+Z` 下。

仓库中的 `projection.py` 提供了对应函数。

## 6. 对准平面

视觉发布的是“目标参考点”。准星节点用：

```text
alignment_plane_z = target_reference_z - target.height_m
```

例如：

- 视觉 3D 点是桶底/地面中心，桶高 0.30 m：`target.height_m = 0.30`；
- 视觉 3D 点已经在桶口平面：`target.height_m = 0.0`。

Payload 准星使用 `[payload_x, payload_y, alignment_plane_z]`，因此准星与目标 `x/y` 无关，不会自动跟着目标跑。

## 7. 图像接口

独立准星节点还需要：

- `sensor_msgs/msg/Image`
- `sensor_msgs/msg/CameraInfo`

二者话题由 `config/aircraft.yaml` 指定。相机内参直接来自 `CameraInfo.K`；不把焦距/主点写死在代码里。

## 8. 接入已有视觉节点

已有节点只需增加一个发布器，把内部结果转换成上述 `PoseArray`。不需要复制本仓库的模板，也不需要修改 `alignment_viewer`。

接入后运行：

```bash
./bin/check_vision_contract.sh
```

检查时间戳、frame、有限数值、置信度和版本哨兵。
