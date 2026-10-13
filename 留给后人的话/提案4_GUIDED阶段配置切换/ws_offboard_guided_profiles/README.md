# GUIDED 阶段配置切换工作空间

这是 [提案4](../README.md) 的完整 ROS 2 工作区。在公开 V16 的双投、侦察、返航、降落流程上增加阶段配置管理。源码、原视觉模型、路线标定、相机时间对齐、投放机构和复盘工具均随包提供。

运行平台：Ubuntu 22.04、ROS 2 Humble、MAVROS、Intel NUC、D435i。包名保持 `cuadc_full_mission_20260803`，使用本目录独立的 `build/` 和 `install/`；必须编译本版源码。

## 编译与离线检查

从本目录执行：

```bash
python3 scripts/validate_source.py
source /opt/ros/humble/setup.bash
bash scripts/build_onboard.sh
bash scripts/verify.sh
```

这些命令不启动 MAVROS、相机、任务飞行或舵机。`verify.sh` 的 ROS 测试使用本地隔离域。依赖配置见 [环境说明](environment/README.md)。只检查源码时可执行 `bash scripts/verify.sh --source-only`。

## 配置入口

完整任务读取 `full_mission/mission.yaml`，仅投放读取 `drop_only/mission.yaml`，直接 ROS launch 默认读取 `src/cuadc_full_mission_20260803/config/mission_real.yaml`。三个文件都提供阶段配置；请修改实际使用的入口。

交付默认 `flight_enable: false`、`auto_arm_on_guided: false`，飞控改速和参数写入开关也关闭。普通启动脚本要求飞行开关与自动解锁开关均开启，因此默认会拒绝正式启动。完成 [实验计划](../docs/03_接入与实验计划.md) 中的适配后，再在所用 YAML 中明确开启任务运行。静态、离线验证无须打开这些开关。

完整任务启动沿用 V16 接口，路线、串口、相机序列号必须指定本机实际值：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 routes/calibrate_route.py --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200
bash full_mission/run_full_mission.sh round_01 --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 --camera-serial YOUR_D435I_SERIAL --preflight
# 启用任务配置后，使用同样参数去掉 --preflight 启动正式任务。
```

`routes/example.yaml` 是格式示例，需要先完成本轮标定。完整参数、动态切换条件和实验接口见 [提案参数说明](../docs/02_参数与接口.md)。模型、视觉外参、投放口偏移、舵机通道及原分段投放判据来自 V16，仍需与实际飞机核对。

## 验证与来源

[本提案验证记录](../docs/04_验证记录.md) 说明本版实际执行的检查。`docs/` 中原 V16 文档已标注历史基线，保留供阅读原始算法与部署背景。基线编译和实飞记录不构成本提案的实飞验收。

`BASELINE_SOURCE.json` 记录公开仓库提交与原文件摘要，`SOURCE_MANIFEST.json` 列出保留、修改和新增文件，`SHA256SUMS` 用于检查交付完整性。修改源码/YAML 后哈希差异是预期结果；维护者可用 `python3 scripts/refresh_manifest.py` 生成自己的新清单。

源码沿用 [AGPL-3.0-only](LICENSE)，第三方模型和依赖许可见 [LICENSES.md](LICENSES.md)。
