# 当前 CUADC 视觉节点集成约定

正式节点：

```text
~/cuadc_mission/src/cuadc_visual_drop_flight/scripts/drop_bucket_realsense_node.py
```

正式状态机接口必须保留：

```text
/perception/drop_buckets_body
geometry_msgs/PoseArray
frame_id = base_link
坐标 = FLU
```

标定扩展推荐提供：

```text
/cuadc/calib/annotated             sensor_msgs/Image
/cuadc/calib/color/image_raw       sensor_msgs/Image
/cuadc/calib/color/camera_info     sensor_msgs/CameraInfo
/vision_servo/targets_body         geometry_msgs/PoseArray, FRD
```

## 时间戳约定

必须继续使用相机 `wait_for_frames()` 后立即取得的同一个 `capture_stamp`：

```python
frames = pipeline.wait_for_frames(timeout_ms=5000)
capture_stamp = self.get_clock().now().to_msg()
```

图像、CameraInfo、正式 FLU 检测和 FRD 标定检测都应尽量使用同一采集时间戳。不要在推理完成后重新 `now()`。

## FLU -> FRD

```python
x_frd = x_flu
y_frd = -y_flu
z_frd = -z_flu
```

## 直接叠加模式

当前使用的参数约定：

```text
calibration_overlay_enabled
calibration_overlay_topic=/cuadc/calib/annotated
calibration_overlay_width
calibration_release_offsets_body_m
```

当前启动脚本见：

```text
bin/run_current_cuadc_visual_calibration.sh
```

## 相机独占

不要同时启动：

```text
drop_bucket_realsense_node.py
realsense2_camera
vision_provider_template（如果它自己打开相机）
```

D435i 只允许一个进程直接占用。
