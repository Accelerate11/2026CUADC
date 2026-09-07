# Design Notes

## 1. 解耦边界

公开版把原先单个“大视觉节点”的职责拆成三层：

1. **Camera / image source**：只负责标准图像和 CameraInfo；
2. **Vision Provider**：只负责目标识别和 3D 目标坐标；
3. **Alignment Viewer**：只负责几何投影、准星、人工标定日志。

这样目标识别网络和准星逻辑不会互相污染。

## 2. 两个准星的几何意义

目标准星：

```text
pixel_target = project(body_to_camera(target_body))
```

Payload 准星：

```text
plane_z = target_body.z - target_height
payload_plane_body = [payload_offset.x, payload_offset.y, plane_z]
pixel_payload = project(body_to_camera(payload_plane_body))
```

关键约束是 `payload_plane_body.x/y` 始终来自**飞机固有标定值**，不能来自目标检测值。

## 3. 静态标定为什么可以直接取目标 FRD X/Y

人工用铅垂线把真实 Payload 投放点压到目标中心时，从飞机坐标系观察，目标的 `X/Y` 就等于真实 Payload 垂线的 `X/Y`。因此只要机体稳定，多个视觉样本的目标 `X/Y` 中位数就是新的 Payload 静态偏移。

这一步只识别水平位置，`Z` 仍属于机械几何测量。

## 4. 为什么用中位数而不是单帧

单帧视觉容易受深度噪声、检测框抖动和像素量化影响。静态标定工具取时间窗口内 `X/Y` 中位数，并报告 IQR；真实落点工具也用中位数和 MAD，避免少量异常试投主导参数。

## 5. 为什么不用 TF 强绑定

为了让其他人接入最少依赖，公开版不要求把相机外参发布进 TF 树。准星节点直接读取一组经过验证的 `camera -> body FRD` 旋转和平移。如果项目已有完整 TF，也可以在自己的视觉节点中完成变换后按契约发布 FRD。
