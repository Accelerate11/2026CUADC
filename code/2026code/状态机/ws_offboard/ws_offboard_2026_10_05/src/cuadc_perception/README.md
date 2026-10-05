# cuadc_perception

`ament_python` 包，包含原 D435i 视觉适配器、检测/分割分析模块与三个模型。相机仍由原 Python 代码通过 pyrealsense2 直接读取，无需新增 RealSense ROS 图像驱动。

安装后的入口：

```bash
ros2 run cuadc_perception basket_vision_ros_node
ros2 run cuadc_perception basket_detect_seg_analysis --model /path/to/basket_v3.pt
ros2 run cuadc_perception preflight_camera_view --exposure 0
```

前两个入口需要视觉运行依赖和相机；地面预览也沿用原分析模块依赖。`preflight_camera_view` 不启动任务或操作舵机。完整飞行使用 `cuadc_bringup/flight.launch.py` 传入实机参数。

模型安装到 `share/cuadc_perception/models/`；包内 Python 模块采用完整包名导入。发布话题与消息字段见工作区的 `docs/ARCHITECTURE.md`。
