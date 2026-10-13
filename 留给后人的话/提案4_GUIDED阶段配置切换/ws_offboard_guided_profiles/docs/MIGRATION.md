> 历史基线文档：下文记录原公开 V16 的设计、配置或验证结果。本提案新增阶段配置切换，并将默认飞行开关关闭；当前说明以工作区 README 和上级提案 docs 为准。原 V16 的测试结果不表示本提案已经实飞验收。

# V16 来源与整理说明

来源是本次提供的 `CUADCv16.zip`。原压缩包中使用的55个源文件已逐个读取并通过CRC及SHA256核对。`SOURCE_MANIFEST.json` 保存来源路径、原SHA256、新路径、新SHA256及是否改变，模型另有 `MODEL_SHA256.json`。

| 原位置（相对 CUADCv16） | 新位置 |
| --- | --- |
| cuadc_full_mission_release/src/cuadc_full_mission_20260803 | src/cuadc_full_mission_20260803 |
| cuadc_full_mission_release/scripts | scripts |
| cuadc_full_mission_release/tools | tools |
| cuadc_full_mission_release/docs | docs |
| full_mission / drop_only | 同名目录 |
| indoor_projection / fixed_release_test / servo_test | 同名目录 |
| routes/calibrate_route.py | routes/calibrate_route.py |

## 原样保留

- 任务 C++、`release_gate.hpp`，包括完整目标选择、搜索、对准、投放、侦察、返航与故障分支。
- 六个包内 Python 文件，包括视觉分析、ROS适配、曝光时间、相机预览、消息频率和遥测。
- 原始完整任务、仅投放、包内基础参数及MAVROS时间同步配置。
- 三个原权重文件，以及OpenVINO的XML、BIN和metadata，共六个模型资源。
- 三个既有测试、原投放舵机测试程序、路线预检与启动守护核心程序。

## 部署调整

- 统一为标准 `build/`、`install/`、`log/`，原构建入口转发到根脚本。
- 任务入口从工作区相对路径寻找文件，飞控连接由参数或环境变量显式提供；相机可显式覆盖，原YAML不变。
- 包装入口和直接任务launch均使用任务专用MAVROS入口，加载原V16时间同步配置。
- 航向标定的定位/航向数学不变，调整设备参数、输出目录和串口锁路径。
- 飞行总结的统计与解析不变，邮件改为显式手动 `--send`；任务结束默认只生成本地报告。
- 包元数据统一声明AGPL-3.0-only；加入独立投放门限测试、部署回归、环境核验、发布校验和中文说明。

完整算法已经提供，未将上一轮代码审查建议直接改成新的飞行策略。原版六秒强制投放、150秒固定点兜底、目标重捕获及超时策略均可在源码中检查。

## 未复制的运行文件

原 `build* / install* / log*`、缓存、旧视频和飞行日志不作为源码发布；原机真实路线和SMTP账号配置不复制。使用者重新标定本轮路线、填写自己的设备参数即可。包中保留合成路线格式示例和无账号密钥的SMTP模板。
