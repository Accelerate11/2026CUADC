# Migration from the original portable workspace

本公开版不是简单删除模型，而是重新划分了模块边界。

## 已删除

- `models/basket_detect.onnx`
- `basket_detect_seg_analysis.py`
- 原 `basket_vision_ros_node.py`
- 与特定模型、RealSense 直连、检测质量门限相关的参数
- 比赛专用 `cuadc_full_mission_node.cpp` 和灾情侦察全流程
- 原工作空间中的 `__pycache__` / `.pyc`

## 已抽取

原视觉节点中的以下通用能力被独立成 `alignment_viewer`：

- Target center 准星；
- 每个 Payload projection 准星；
- FCU FRD `+X/+Y` 方向箭头；
- 相机外参反投影；
- 检测坐标 CSV；
- 实时画面与可选视频记录。

## 新增

- `vision_provider_template.py`：明确的空视觉节点模板；
- `vision_contract_check.py`：第三方视觉节点接入自检；
- `projection.py`：可单元测试的纯几何函数；
- `CameraInfo` 驱动内参，不再把 RealSense 内参绑在检测器内部；
- GitHub 发布用 `.gitignore`；
- 独立视觉接口文档。

## 旧接口映射

旧：

```text
/perception/drop_buckets_body
```

新默认：

```text
/vision_servo/targets_body
```

消息仍为 `geometry_msgs/PoseArray`，FRD 坐标约定保持不变。若已有旧节点，可把 `vision.detections_topic` 配回旧话题，无需改源码。
