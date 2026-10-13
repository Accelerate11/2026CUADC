# 参数与飞机适配

本版保留 V16 参数原文件。运行完整任务读取 `full_mission/mission.yaml`，仅投放读取 `drop_only/mission.yaml`。包内 `src/cuadc_full_mission_20260803/config/mission_real.yaml` 是原包基础参数，不等同于完整任务最终配置。

启动器选择任务入口 YAML，根据本轮路线计算航向，再将航向、曝光和日志路径等写入本轮 `mission_runtime.yaml`。相机序列号、模型路径由 launch 参数覆盖，`CUADC_VISION_DEVICE` 由 launch 读取环境变量覆盖，它们不会全部写回这个运行副本。复现本轮任务时，需要同时保存 runtime YAML、实际启动命令、环境变量和日志。

| 项目 | 完整任务 V16 原值 / 行为 |
| --- | --- |
| 自动任务 | `flight_enable: true`、`auto_arm_on_guided: true` |
| 时间同步与投放门限 | `measurement_time_sync: true`、`staged_release_gate: true` |
| 起飞 / 搜索 / 粗对准 / 投放高度 | 3.0 / 3.0 / 2.0 / 1.0 m |
| 搜索速度 | 1.75 m/s |
| 两目标转场速度 | 4.8 m/s |
| 下降 / 水平对准 / 释放定位限速 | 0.16 / 0.375 / 0.15 m/s |
| 侦察高度 / 航线 | 2.5 m，2条覆盖线，无额外航点停留 |
| 目标桶径分类 | 0.15 / 0.20 / 0.25 m |
| 搜索稳定目标 | 至少6个不同帧、15样本中位数、位置离散度≤0.20 m |
| 投放口 | 原 FRD 杆臂原样保留，任务和视觉两处应保持一致 |
| 舵机 | CH7、CH8；收拢1100、释放1900；保持0.7 s |
| 视觉设备 | 原值 `intel:gpu`，可显式覆盖为 `cpu` |
| 相机 | 原YAML序列号保留；运行时通过 `--camera-serial` 或 `CUADC_CAMERA_SERIAL` 指定本机设备 |
| 航向 | 原文件 -1 为待注入值；启动器读取本轮两点路线计算实际航向 |
| 总任务超时 | 原值0；投放阶段单独配置150 s |

相机旋转、平移、两个投放口的杆臂、PWM 行程、飞控串口和相机序列号，都是飞机相关参数。直接换一架机体时，先完成相机与投放口标定，再复用任务策略。数据单位在 YAML 注释中标出。

## 命令行与环境变量

| 设置 | 用途 |
| --- | --- |
| `--fcu-url` / `CUADC_FCU_URL` | 显式传入飞控串口 URL，包装入口只支持 serial URL |
| `--camera-serial` / `CUADC_CAMERA_SERIAL` | 指定本机相机 |
| `CUADC_VISION_DEVICE=cpu` | 本次任务使用CPU推理 |
| `expo=0` / `expo=20` | 自动曝光 / 手动曝光覆盖 |
| `CUADC_ROUTES_DIR` | 使用其他目录的本轮路线 |
| `CUADC_SERIAL_LOCK` | 多个入口统一使用的串口锁文件 |

未指定相机时，包装入口先尝试枚举当前设备；若没有取得序列号，launch 传入空字符串，由 RealSense 自动选择设备，并不会沿用 YAML 中的原序列号。多相机或正式任务应显式指定序列号。

包内原注释有历史遗留，例如“默认179.9°”“三条搜索线”“粗定位试验高度”等；此次为保留源文件字节没有逐条改写。实际入口要求路线名，实际搜索两条线，完整任务释放高度1.0 m。需要追踪行为时，结合 C++、本轮 runtime YAML、启动参数和环境记录核对。
