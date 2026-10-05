# 许可与来源

本版项目代码、配置、脚本和文档采用 **GNU Affero General Public License v3.0 only（AGPL-3.0-only）**。这是本次整理时由项目使用者明确选择的开源许可；完整文本见 `LICENSE`。ROS 包的许可声明与此一致。

原压缩包的 `package.xml` 同时标注了 `LicenseRef-Proprietary` 和 `AGPL-3.0-only`。本版更新了项目许可声明；[docs/ORIGINAL_LICENSES.md](docs/ORIGINAL_LICENSES.md) 是原包说明的中文历史译文，不是本版新的私有许可条款。

三个模型与视觉分析模块来自用户提供的 `cuadc_2.zip`，文件对应与校验值见 `SOURCE_MANIFEST.json` 和 `MODEL_SHA256.json`。原包说明报告 `basket_v3.pt` 的元数据含 Ultralytics 及 AGPL 信息；本次没有反序列化 `.pt` 检查点来重新核验元数据。Ultralytics 的许可信息见其 [官方说明](https://www.ultralytics.com/license) 与 [许可文本](https://github.com/ultralytics/ultralytics/blob/main/LICENSE)。

ROS 2、MAVROS、ArduPilot、RealSense、PyTorch、OpenCV、ONNX Runtime 等是外部运行依赖，保持各自许可，未将其源码或二进制打包到本项目中。参考仓库只用作目录组织参考，没有复制其任务、载荷、安全或飞控实现。

压缩包未提供模型训练代码、训练数据集或数据集授权文件，本版也未补造这些材料。模型文件来源说明保留在 [docs/MODELS.md](docs/MODELS.md)。
