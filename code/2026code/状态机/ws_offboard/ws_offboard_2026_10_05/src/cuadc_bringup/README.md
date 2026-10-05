# cuadc_bringup

`ament_python` 包，提供 `flight.launch.py`、原实机参数和纯 Python 航向加载器。

launch 启动 MAVROS、消息频率配置工具、遥测记录、桶视觉和 C++ 任务节点。必填参数为 `fcu_url`、`route_file`。后者必须是含有效 `heading_deg` 的 YAML 文件；航向必须是有限数值，范围 `[0, 360)`。

ROS 节点名保持原值，所以原参数文件无需改节点键。只覆盖本轮航向、选择的模型/相机和日志输出路径。详见工作区 README。
