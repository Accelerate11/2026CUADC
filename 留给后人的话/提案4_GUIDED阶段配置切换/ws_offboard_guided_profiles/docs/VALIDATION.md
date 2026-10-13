> 历史基线文档：下文记录原公开 V16 的设计、配置或验证结果。本提案新增阶段配置切换，并将默认飞行开关关闭；当前说明以工作区 README 和上级提案 docs 为准。原 V16 的测试结果不表示本提案已经实飞验收。

# 本次验证记录

验证日期：2026-10-13。输入基线为提供的 `CUADCv16.zip`，工作区名保留整理开始时的 `2026_10_11_v16`。

## 源码与模型

原压缩包中用于本次发布的 55 个文件已逐个读取，通过 ZIP CRC 与 SHA256 核对。32 个文件原样保留，另外 23 个文件涉及部署、构建元数据或文档整理，逐项记录在 [SOURCE_MANIFEST.json](../SOURCE_MANIFEST.json)。新增文件不计入这 55 个来源文件。

任务 C++、投放门限头文件、六个包内 Python 文件、原始任务参数、既有三个测试及六个模型资源均未改动。模型逐项摘要见 [MODEL_SHA256.json](../MODEL_SHA256.json)。完整发布文件由 `SHA256SUMS` 校验。

## 干净目录编译与离线回归

先在整理工作区编译，再将候选发布文件复制到 WSL 的新临时目录，从零生成 `build/`、`install/`、`log/`。第二次编译没有复用原工作区的构建结果，并清除了旧工作区的 ROS 路径环境变量。

环境为 Ubuntu 22.04 / WSL2、ROS 2 Humble、Python 3.10.12、GCC 11.4.0、CMake 3.22.1。实际软件记录见 [环境快照](../environment/verification_build_environment.json)。这个快照来自本次验证机器，不代表原比赛飞机。

| 检查 | 结果 |
| --- | --- |
| `bash scripts/build_onboard.sh` | 干净目录编译通过，1 个 ROS 包完成 |
| `bash scripts/verify.sh` | 完整离线验证通过，退出码 0 |
| 部署回归 | 11/11，通过设备参数、共享锁、路线预检、帮助入口及本地报告等检查 |
| CTest | 4/4：time_alignment、capture_clock、vision_backend、release_gate |
| 原 Python 回归 | capture_clock 6/6、vision_backend 4/4；它们也包含在上述 CTest 中，不重复计为额外覆盖 |
| 安装入口 | 可枚举 7 个 ROS 可执行入口；两个 launch 的 `--show-args` 均通过 |
| 源码与脚本 | 来源摘要、模型摘要、发布摘要、Python 语法及 Shell 语法检查通过 |

可读原始记录：[编译日志](validation/build.log)、[离线验证日志](validation/verify.log)、[执行结果](validation/result.json)。日志中的临时目录是验证位置，使用者不需要创建同名目录。日志记录的文件数量来自当时的候选包；本页和验证日志在测试完成后加入，最终文件清单以随包 `SHA256SUMS` 为准。

报告发送相关测试使用模拟对象，没有发送邮件。上述验证没有启动真实任务、MAVROS、相机或舵机。

## 模型推理

已使用 ONNX Runtime 1.23.2、NumPy 1.26.4，在 CPU 上加载随包 `basket_detect.onnx` 并执行一次合成输入推理：输入 `[1, 3, 640, 640]`，输出 `[1, 300, 6]`，输出数值有限。原始结果见 [ONNX 验证记录](../environment/verification_onnx_wsl.json)。

这项检查确认主 ONNX 文件能够加载和执行，没有验证真实图像检测精度，也不能把一次合成输入的耗时当作机载帧率。其余模型完成完整性检查，没有逐个执行推理。

固定依赖版本的官方 wheel 元数据已核对，记录见 [依赖发行记录](../environment/verification_dependency_metadata.json)。这不等同于在一台全新机器上完成全部视觉依赖安装与运行；完整适配器、Intel GPU、RealSense 仍需按 [环境说明](../environment/README.md) 在目标机验证。

## 复现命令与范围

从本工作区根目录执行：

```bash
python3 scripts/validate_source.py
bash scripts/build_onboard.sh
bash scripts/verify.sh
python3 environment/check_environment.py --mode onnx --json log/environment_onnx.json
```

前三条使用已准备好的 ROS 编译及离线测试环境，最后一条还需要 NumPy 和 ONNX Runtime。依赖安装步骤见环境说明。

本次没有完成 Intel GPU、D435i 采集时序、飞控固件与参数、实际投放、外场飞行或完整 SITL 场景复验。外层提供 GitHub Actions 配置，本次没有在 GitHub 上运行该工作流。原文件中的历史实机结论保留为原始记录，不算本次验证结果。
