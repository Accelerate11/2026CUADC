# V16 环境复现

先把依赖、模型和源码测试跑通，再接相机和飞控。以下命令在工作区根目录执行，也就是包含 `src/`、`environment/` 和 `full_mission/` 的目录。

目标平台是 Ubuntu 22.04、ROS 2 Humble、系统 Python 3.10。原部署说明写的是 Intel NUC11，视觉相机是 D435i。V16 参数默认选 `intel:gpu`，同时保留 ONNX CPU 路径。

## 1. 这份清单能复现到哪里

| 项目 | 已有依据 | 本包处理 |
| --- | --- | --- |
| Ultralytics、ONNX、ONNX Runtime | 原 `requirements-onboard.txt` | 保留 `8.4.41`、`1.17.0`、`1.23.2` |
| NumPy | 原说明要求 `>=1.24,<2.0` | 保留这个范围，兼容 Humble 的系统二进制模块 |
| OpenCV | 原说明未锁定具体版本 | 选择 `>=4.6,<4.12`；避免 Python 3.10 下的新 wheel 要求 NumPy 2 |
| PyTorch | 原注释只有 `torch 2.11 CPU` | 安装清单选择 `torch==2.11.0`、`torchvision==0.26.0` |
| OpenVINO | 随包 XML 记录导出运行时 `2026.4.1-22982-e213a147257-releases/2026/4` | GPU 清单选择 `openvino==2026.4.1` |
| RealSense SDK、GPU 驱动 | 原包未记录完整版本 | 在本机安装、检查并保存实际版本 |
| 飞控固件、飞控完整参数、相机固件 | 原包未提供完整版本快照 | 需要从实际设备另行导出 |

PyTorch 组合来自[官方版本配对表](https://pytorch.org/get-started/previous-versions/#v2110)。OpenVINO XML 记录的是模型导出版本，不能据此认定赛场机器安装的就是同一版本。

原赛场机器的完整 `pip freeze` 没有随 V16 留下。本目录提供的是有来源的安装清单和检查方法；对未记录的依赖，不补写一个看似精确的历史版本。完成本机验证后，按第 6 节保存新环境。

## 2. 系统与 ROS 依赖

先按 [ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) 配置 Ubuntu 22.04 的 ROS 软件源。已有 Humble 的机器直接执行下面的依赖安装。

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git curl unzip \
  python3-dev python3-pip python3-venv python3-yaml \
  python3-numpy python3-opencv python3-colcon-common-extensions python3-rosdep \
  libgl1 libglib2.0-0 libusb-1.0-0 geographiclib-tools \
  ros-humble-ros-base ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-cv-bridge ros-humble-ament-index-python \
  ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-sensor-msgs \
  ros-humble-diagnostic-msgs ros-humble-std-msgs ros-humble-std-srvs

source /opt/ros/humble/setup.bash
```

`rosdep` 第一次使用时执行一次 `sudo rosdep init`。已经初始化过的机器跳过这一条，再执行：

```bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

最后一条为 MAVROS 安装 GeographicLib 数据，会联网下载，不连接飞控。只有 Python 依赖安装成功还不够；ROS 接口、系统动态库和这份地理数据需要一并准备。

## 3. Python 与 CPU 推理

使用系统 Python 3.10 建立虚拟环境，并允许读取 ROS 所需的系统 Python 包。虚拟环境放在工作区外，避免 `colcon` 将依赖目录当作待构建源码。

```bash
/usr/bin/python3 -m venv --system-site-packages "$HOME/.venvs/cuadc-v16"
source "$HOME/.venvs/cuadc-v16/bin/activate"
source /opt/ros/humble/setup.bash
python3 -m pip install --upgrade pip

python3 -m pip install -r environment/requirements-pytorch-cpu.txt
python3 -m pip install -r environment/requirements-cpu.txt
python3 -m pip check
```

`requirements-pytorch-cpu.txt` 使用 `https://download.pytorch.org/whl/cpu`，先安装 CPU 版 PyTorch 和对应 torchvision。之后的清单补齐原版视觉依赖、NumPy、OpenCV 和 `pyrealsense2`。Ultralytics 的 ONNX/OpenVINO 路径仍会导入 PyTorch；不能因为模型文件是 ONNX 就省掉它。

后续每次开终端，都先激活这个虚拟环境，再加载 Humble 和本工作区的 `install/setup.bash`。ROS 可执行脚本通过 `/usr/bin/env python3` 选择解释器，因此检查、相机预览和任务启动要使用同一套环境。

```bash
python3 environment/check_environment.py --mode python --require-ros
python3 environment/check_environment.py --mode cpu --require-ros \
  --json log/environment_cpu.json
```

CPU 检查通过后，运行入口可用 `export CUADC_VISION_DEVICE=cpu` 显式选择 CPU。原始 YAML 的默认值仍是 `intel:gpu`。

`pyrealsense2` 的导入检查只确认 SDK 和时间戳 API 存在。D435i 的 USB3 连接、权限、固件、曝光元数据仍需在目标机器上检查。需要安装或更新系统 librealsense 时，按 [RealSense 官方 Linux 安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md) 选择与本机内核对应的方式，并记录最终版本。

## 4. Intel GPU 推理

先完成上面的 CPU 环境，再装 GPU 清单：

```bash
python3 -m pip install -r environment/requirements-intel-gpu.txt
python3 -m pip check
```

OpenVINO 的 Python 包不包含完整的 Intel GPU 驱动。按 [OpenVINO 官方 GPU 配置说明](https://docs.openvino.ai/2026/get-started/install-openvino/configurations/configurations-intel-gpu.html) 安装适合当前显卡、内核和 Ubuntu 版本的驱动，并检查当前用户对 `/dev/dri/render*` 的访问权限。驱动包名与版本随平台变化，不能用一个未经核实的固定版本覆盖所有机器。

```bash
python3 -c 'import openvino as ov; print(ov.__version__); print(ov.Core().available_devices)'
python3 environment/check_environment.py --mode intel-gpu --require-ros \
  --json log/environment_intel_gpu.json
```

设备列表应出现 `GPU` 或 `GPU.0` 等 GPU 设备。检查工具会加载随包的 XML/BIN，用原适配器执行一次合成图像推理；若原适配器回退到 CPU，工具会判为失败。这样可以确认此次检查实际用了 GPU。

确认通过后，可用 `export CUADC_VISION_DEVICE=intel:gpu` 选择 GPU。这里使用已有 OpenVINO 模型，不需要重新导出或覆盖原始 ONNX。

## 5. 检查模式与已有结果

```bash
python3 environment/check_environment.py --help
python3 environment/check_environment.py --mode files
python3 environment/check_environment.py --mode onnx --json log/environment_onnx.json
```

| 模式 | 做什么 | 主要依赖 |
| --- | --- | --- |
| `files` | 检查 6 个模型资产存在、非空，计算 SHA256，读取 XML 的导出版本 | Python 标准库 |
| `python` | 导入视觉依赖，核对原文件记载的精确版本和 NumPy 范围 | CPU 清单 |
| `onnx` | 直接用 ONNX Runtime CPU 对主 ONNX 模型执行全零张量推理 | NumPy、ONNX Runtime |
| `cpu` | 完成 Python 检查，再用原适配器执行 CPU 合成图像推理 | CPU 清单 |
| `intel-gpu` | 完成 Python 检查、列出 OpenVINO 设备，再执行原适配器 GPU 推理 | CPU/GPU 清单及 GPU 驱动 |

`--require-ros` 增加 ROS 接口导入检查。`--package-root` 可指定另一个源码包目录。`--json` 保存本机结果。退出码 `0` 表示当前模式没有 `FAIL`，退出码 `1` 表示检查失败；`WARN` 仍需阅读，不能把它当作原环境完全一致。

工具不构造 ROS 节点，不初始化 D435i，不访问飞控串口，不发布飞行指令。CPU/GPU 模式会使用相应计算设备执行推理。工具关闭 Ultralytics 自动装依赖，临时配置写入系统临时目录；只有指定 `--json` 时才保存检查报告。

`files` 会计算模型摘要，但不与发布清单逐项比对。完整文件一致性由工作区的 `python3 scripts/validate_source.py` 检查。

随包的 `verification_onnx_wsl.json` 是已完成的检查：WSL Ubuntu、Python 3.10.12、NumPy 1.26.4、ONNX Runtime 1.23.2。主模型输入为 `[1, 3, 640, 640]`，输出为 `[1, 300, 6]`，数值有限，CPU 推理通过。报告中的路径和平台来自当时的验证机器。

这份报告证明该 ONNX 文件能加载和执行。它没有验证真实图像检测精度、Intel GPU、D435i 时序或实飞效果；记录的耗时包含模型会话初始化，也不能直接当作机载帧率。

`verification_dependency_metadata.json` 记录了另一次官方发行元数据检查：上述 6 个固定版本均有适用于 Python 3.10、Linux x86_64 的 wheel，PyTorch 与 torchvision 使用官方 CPU 索引。这项检查只读取发行元数据，没有下载大体积 wheel，没有完成全量依赖解析，也没有把完整视觉运行时安装后实测。部署时仍要执行 `pip check` 和所选后端的检查命令。

## 6. 编译、测试与保存本机环境

依赖检查完成后，回到工作区根目录运行：

```bash
bash scripts/build_onboard.sh
bash scripts/verify.sh
source install/setup.bash

mkdir -p log
python3 -m pip freeze --all > log/pip-freeze.txt
python3 scripts/capture_environment.py --output log/environment.json
dpkg-query -W > log/dpkg-versions.txt
uname -a > log/kernel.txt
```

`build_onboard.sh` 编译源码和回归测试，不连接设备。`verify.sh` 执行源码校验与离线测试。仅检查源码时可运行 `bash scripts/verify.sh --source-only`。

环境快照记录当前机器。实际完成相机和飞控验证后，再补存相机固件、GPU 驱动、飞控固件、飞控参数和本轮标定文件，并与本轮日志放在一起。下一台机器才有明确的版本对照。
