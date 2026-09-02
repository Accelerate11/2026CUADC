# 2026 年备赛代码说明

## 文件夹说明

此文件夹用于储存 2026 年多旋翼无人机侦察与救援项目的主要备赛代码。

相比 2025 年代码，2026 年的开发环境由 **Ubuntu 20.04 + ROS 1** 更新为：

* Ubuntu 22.04
* ROS 2 Humble
* MAVROS
* ArduPilot / ArduCopter
* Gazebo Harmonic

2025 年代码仍然具有较高的参考价值，尤其是状态机设计、视觉引导、目标搜索和投放流程等部分。但是 ROS 1 与 ROS 2 的节点、话题、服务和编译方式存在较大差异，移植时不要直接复制代码，应当先理解原有逻辑，再按照 ROS 2 的接口重新实现。

目前 2026 年代码主要按照功能划分为 `状态机` 和 `视觉` 两部分。后续版本建议继续采用“模块名称 + 版本号”的方式命名，例如 `状态机1.0`、`视觉1.1`，并在每个版本开头注明代码功能、仿真验证情况和真机验证情况。

---

## 目录结构

```text
2026code/
├── 状态机/
│   ├── README.md
│   ├── cuadc_full_mission_node_1.cpp
│   ├── cuadc_full_mission_node_2.cpp
│   ├── cuadc_full_mission_node_3_v0_public.cpp（北部赛区冠军机组代码）
│   ├── cuadc_full_mission_node_3_v1_public.cpp
│   ├── cuadc_full_mission_node_3_v2_public.cpp
│   ├── offb_node.cpp
│   ├── recon_state_machine_node_1.cpp
│   ├── takeoff_drop_land_node_1.cpp
│   ├── takeoff_drop_land_node_2.cpp
│   └── ws_offboard/
|       ├── ws_offboard_2026_6_20
│       └── ws_offboard_2026_8_20
│
└── 视觉/
    ├── README.md
    └── 视觉1.0/
    |   ├── dangerous_target.onnx
    |   ├── labeling.py
    |   ├── labels.docx
    |   └── single_demo_en.py
    └── 视觉2.0/drop
        ├── basket_detect_seg.pt
        └── basket_detect_seg.py

## 注意事项

自动飞行代码必须先经过 SITL 仿真验证，再进行真机测试。

真机测试前应当确认：

* 飞控模式和固件正确；
* MAVROS 已连接；
* 本地位置数据正常；
* 高度和坐标系方向正确；
* 起飞区域无人员和障碍物；
* 遥控器能够随时切换模式并人工接管；
* 已准备紧急降落和上锁命令；
* 首次测试应拆除桨叶检查服务和状态切换逻辑。

不要在没有安全员、没有人工接管手段或坐标系尚未验证的情况下直接运行自动飞行程序。
