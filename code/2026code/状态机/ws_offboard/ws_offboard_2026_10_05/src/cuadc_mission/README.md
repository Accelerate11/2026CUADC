# cuadc_mission

`ament_cmake` 包，构建并安装 `cuadc_full_mission_node`。`src/cuadc_full_mission_node.cpp` 保留原压缩包的全部代码逻辑，只将原英文注释译为中文。

它继续承担任务状态机、坐标转换、桶跟踪与选择、导航、舵机 7/8 投放、航点侦察、返航降落和落地上锁。没有新增独立载荷节点或危险物视觉节点。

参数的 ROS 节点名保持 `cuadc_full_mission_node`，参数文件移到 `cuadc_bringup/config/mission_real.yaml`。全任务入口见工作区 README。
