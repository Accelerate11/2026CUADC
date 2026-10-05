# 参数与飞机适配

默认文件：`src/cuadc_bringup/config/mission_real.yaml`。文件分为 `cuadc_full_mission_node` 和 `basket_vision_ros_node` 两个节点参数块。除了将注释译为中文，本版没有改动原参数值。

| 项目 | 原值或位置 | 含义 |
| --- | --- | --- |
| 任务开关 | `flight_enable: true` | 原包处于实机任务配置 |
| 自动解锁 | `auto_arm_on_guided: true` | 满足原节点条件且进入 GUIDED 后自动解锁 |
| 地理航向 | `geographic_heading_deg: -1.0` | 无效占位；由本轮标定文件在 launch 中覆盖 |
| 投放试验 | `coarse_release_trial_mode: true` | 原包当前启用粗定位释放试验模式 |
| 舵机通道 | `[7, 8]` | 两个载荷输出通道 |
| 收拢与释放 PWM | `[1100,1100]` → `[1900,1900]` | 两通道原 PWM 值 |
| 释放持续时间 | `[0.7,0.7]` 秒 | 原执行器动作时间 |
| 起飞、搜索高度 | 均为 3.0 m | 原任务高度 |
| 粗、精高度 | 2.0 m、0.8 m | 原对准阶段参数 |
| 返航高度 | 2.5 m | 原返航高度 |
| 相机外参 | `camera_to_body_rotation` / `translation_m` | 必须对应相机的实际安装 |
| 两投放口偏移 | `payload_release_offsets_body_m` | 任务和视觉两处分别配置，值应一致 |
| 视觉设备 | `device: cpu`、`half: false` | 原 CPU 推理配置 |
| 实时窗口 | `live_view_enabled: true` | 无图形桌面时使用参数副本关闭 |

这些参数属于原飞机，不是所有机型的通用安装值。尤其是舵机、相机旋转和平移、投放口位置、场地几何、速度和高度，需要与实际飞机匹配。

可以将 YAML 复制到工作区外编辑，再通过 `params_file:=/absolute/path/mission.yaml` 选择它。源文件中的参数名与节点名无需更改。

## 启动参数

| 参数 | 必填/默认 | 用途 |
| --- | --- | --- |
| `fcu_url` | 必填 | MAVROS 设备 URL |
| `route_file` | 必填 | 本轮地理航向 YAML |
| `params_file` | 随包实机 YAML | 任务和视觉参数 |
| `model_path` | 随包 `basket_detect.onnx` | 视觉模型 |
| `camera_serial` | 空 | 非空时覆盖 YAML 的相机序列号 |
| `gcs_url` | 空 | MAVROS 地面站转发 URL |
| `start_mavros` | `true` | 是否由本 launch 启动 MAVROS |
| `start_vision` | `true` | 是否启动视觉；关闭后任务仍受原视觉检查约束 |
| `log_dir` | 当前目录 `log/flight_时间戳` | 本架次记录目录 |

可选环境变量 `CUADC_RUNTIME_EXPOSURE` 保留原包运行时曝光覆盖功能，值必须是有限正数。自动曝光使用 YAML 的 `exposure: 0.0`。
