# 仿真工程区

`sim/` 用于集中归档和管理本仓库中的 CUADC 仿真工程。这里保留不同用途、不同阶段的仿真实现，便于独立查阅、复现和对比；各工程的具体安装、构建与运行方法以其目录内的 `README.md` 为准。

## 目录说明

| 目录 | 作用 |
| --- | --- |
| `cuadc_hazard_recognition_sim-v0.2.0/` | 危险品侦察仿真 v0.2.0，包含 ROS 2、Gazebo Sim、ArduPilot SITL、MAVROS、ONNX 视觉识别以及覆盖搜索状态机，用于较完整地验证危险品侦察流程。 |
| `cuadc_hazard_recognition_sim/` | 轻量级危险品标识识别场景，主要用于单独验证随机场景、危险品标识纹理和固定下视 RGB-D 相机，不包含无人机、ArduPilot、MAVROS 或任务状态机。 |
| `cuadc_sim/` | 面向多旋翼无人机侦察与救援赛项的全流程仿真环境，覆盖场地、无人机、投放区、侦察区、虚拟投放和危险品标识等内容；其 ROS 2 包名为 `cuadc_rescue_sim`。 |

## 使用注意事项

- `cuadc_hazard_recognition_sim-v0.2.0/` 和 `cuadc_hazard_recognition_sim/` 的 ROS 2 包名均为 `cuadc_hazard_recognition_sim`，它们是不同阶段的实现，不能同时加入同一个 `colcon` 工作区构建。使用时请选择其中一个。
- 本目录首先承担集中归档作用。建议把需要运行的单个工程复制或链接到 ROS 2 工作区的 `src/` 下，再按该工程自己的文档构建。
- 三个工程仅从仓库根目录迁移至此，迁移过程中未修改其内部文件。迁移完成后已按相对路径、目录、文件大小和逐文件 SHA-256 校验，内容保持一致。
