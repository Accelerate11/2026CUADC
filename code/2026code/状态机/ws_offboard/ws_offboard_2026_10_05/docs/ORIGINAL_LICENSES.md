# 原包许可与来源说明（中文译文）

以下为 `cuadc_2.zip` 内 `LICENSES.md` 的中文历史译文。原文件 SHA-256 记录在 `SOURCE_MANIFEST.json` 中，本版许可见根目录的 `LICENSES.md`。

- 原说明记录：`basket_v3.pt` 和 `basket_detect_seg_analysis.py` 由用户于 2026-08-03 提供，原始 SHA-256 记入当时的交付清单。
- 原说明报告：检查点静态元数据含 Ultralytics 8.4.104 和 `AGPL-3.0`。使用 Ultralytics 运行环境或分发衍生系统可能涉及 AGPL 义务；原说明建议在周边飞行程序不拟采用 AGPL 分发时，核对项目的许可安排。
- PyTorch `.pt` 检查点包含基于 pickle 的数据，反序列化可能执行代码。原说明要求运行时只加载经校验的随包模型。
- ArduPilot、MAVROS、ROS 2、Gazebo、RealSense、PyTorch 等运行组件保留各自许可；它们属于依赖，原包未重新授予其许可。
- 新编写的任务连接代码、启动文件、部署脚本和文档用于用户的项目。原说明不作适航性或飞行适用性保证。
