# CUADC 视觉：从零到投放定位与 O4 侦察

**面向多旋翼无人机侦察与救援的完整学习教程**  
学习工具：Python / NumPy / OpenCV / RealSense D435i / ROS 2 / C++任务状态机 / DJI O4。

学习定位：从离线实验到可解释的视觉观测

> 视觉不是“识别出一个框就结束”。本教程从新人能独立运行的小程序开始，逐步建立“图像 → 几何 → 时间 → 实体 → 任务证据”的完整链路。

## 这份教程与前两份有什么不同

C++教程教你表达程序逻辑；ROS教程教你组织节点与通信；这一份教你获得**可测量、可解释、可拒绝错误的视觉观测**，再把它交给任务状态机。

正文68章、7个附录。实践代码嵌入正文，按章节给出的相对路径在自己的练习目录中整理即可。数学推导与代码逐步出现，末尾提供函数表、接口检查表、术语、测试和官方资料入口。

## 教学范围与示例约定

本教程从图像处理、相机几何、模型训练讲到时间对齐、目标关联与任务接口。所有算法示例都是独立练习，不要求阅读任何任务工程，也不把某个工程的函数、参数或运行结果当作教材前提。

圆筒尺寸、场地布局、速度、误差门和超时值用于说明方法；应先写清假设，再用自己的测量选择参数。示例内参、外参和深度不代表真实相机标定。与赛事有关的实际尺寸、流程及判分，应另行核对当届公开规则。

技术接口以官方资料为查阅入口；教学推导与工程建议在明确条件下成立，设备性能需要通过实验确认。

## 三条学习路线

| 读者 | 建议先读 | 第一阶段交付 |
|---|---|---|
| 完全零基础 | 1～10、15、23～25、附录A/C | 能读图、找合成桶、解释像素与深度 |
| 已会YOLO但不会控制接口 | 12～22、32～50 | 有坐标、时间、质量与拒绝原因的ROS观测 |
| 负责比赛集成/侦察 | 2、39、43、47～58、59～66 | 实际链路、可读覆盖、参数边界与验证报告 |

每完成一个阶段，先解释输入、输出、单位、坐标、时间和失效情况，再增加功能。不要跳过合成与台架验收，直接把教学目标发给实机状态机。

## 如何组织练习

可以在自己的学习目录中建立 `cuadc_vision_lab/`，把正文标为“完整文件”的代码按对应路径保存；目录名是练习约定，本文无需任何额外 ZIP。附录 D 列出整理和运行顺序。

先运行离线练习，再安装相机、训练和 ROS 所需依赖。相机标定、真实模型和硬件测试由学习者完成，并使用附录 F 的记录表保存结果。所有 `/lab` 话题均用于隔离实验。

---

## 目录

### 第一部分：从像素与相机开始

- [1. CUADC 的视觉任务不是“在画面上画三个框”](#ch-01)
- [2. 先定义任务、观测和实验假设](#ch-02)
- [3. 学习顺序、环境与安全隔离](#ch-03)
- [4. 写视觉程序需要的 Python 和 NumPy 基础](#ch-04)
- [5. 颜色、编码与图像预处理](#ch-05)
- [6. 曝光、运动模糊、帧率与有效观察时间](#ch-06)
- [7. D435i：它给你什么，也不给你什么](#ch-07)
- [8. 实践：只读采集一组可复现的 RGB-D 数据](#ch-08)
- [9. 内参、针孔投影与畸变：把像素变成射线](#ch-09)
- [10. 实践：从拍棋盘到一份可审计的内参文件](#ch-10)

### 第二部分：标定、坐标、深度与投放几何

- [11. O4 的广角画面：FOV、去畸变和电子增稳必须分开](#ch-11)
- [12. 坐标系：先确定“相机看到的前方”是哪一条轴](#ch-12)
- [13. 相机到机体外参：测量、估计与验证](#ch-13)
- [14. 完整刚体变换：相机 → 机体 → 本地世界](#ch-14)
- [15. 实践：反投影、平面交点、三维圆和投放口几何](#ch-15)
- [16. 准星与人工对准标定：把可视化变成测量工具](#ch-16)
- [17. 深度的单位和含义：Z、斜距、相对高度不是同一个量](#ch-17)
- [18. 彩色与深度配准：同一个像素编号不一定看同一个地方](#ch-18)
- [19. 深度鲁棒统计：先判有效，再谈平滑](#ch-19)
- [20. 空心圆筒：框中心、桶沿、桶底与三维筒心](#ch-20)
- [21. 平面、三维圆与直径估计](#ch-21)
- [22. 误差预算：厘米级偏差到底从哪里来](#ch-22)

### 第三部分：传统视觉、数据集与模型

- [23. 第一套传统视觉算法：颜色候选、轮廓与几何筛选](#ch-23)
- [24. 轮廓层级、圆度、椭圆与透视偏差](#ch-24)
- [25. 实践：生成教学 RGB-D 场景并找出三个环](#ch-25)
- [26. 深度学习入门：模型究竟在学习什么](#ch-26)
- [27. 数据集设计：怎样避免“训练很好，室外不行”](#ch-27)
- [28. 标注规范：桶口、桶身和可见区域必须统一](#ch-28)
- [29. 实践：写一个数据集审计器](#ch-29)
- [30. 实践：训练一份自己的分割基线](#ch-30)
- [31. 超参数和增强：不要只靠“加 epochs”](#ch-31)
- [32. 评价模型：mAP 之外还要看投放任务指标](#ch-32)
- [33. 推理输出、letterbox 与坐标还原](#ch-33)
- [34. 从实例 mask 到可用于几何的测量区域](#ch-34)
- [35. 把检测与几何组成一个可拒绝的观测流水线](#ch-35)

### 第四部分：实时性、时间、跟踪与ROS对接

- [36. 实时视觉程序：先设计数据流，再讨论多线程](#ch-36)
- [37. 让模型更快：优化顺序与测量方法](#ch-37)
- [38. 视觉系统至少有四种时间](#ch-38)
- [39. 把图像与导航测量对齐到同一时刻](#ch-39)
- [40. 从检测到跟踪：同一个桶为什么会有不同编号](#ch-40)
- [41. EMA、中值、MAD：分别解决什么问题](#ch-41)
- [42. 三桶选二、冻结目标与受约束重捕获](#ch-42)
- [43. ROS 视觉接口：先把语义写清楚](#ch-43)
- [44. Image、CameraInfo 与深度订阅](#ch-44)
- [45. 阶段实践：一个能隔离运行的 ROS 视觉小工程](#ch-45)
- [46. 视觉健康：没有目标与没有相机不是一回事](#ch-46)
- [47. 把视觉接入任务节点：接收端检查清单](#ch-47)
- [48. 粗对准、细对准：视觉与姿态补偿的分工](#ch-48)
- [49. RELEASE：目标冻结、姿态更新与最后一道门](#ch-49)
- [50. 几何补偿的边界与误差来源](#ch-50)

### 第五部分：搜索与O4侦察

- [51. 投放区搜索：视场、航线和多帧确认如何一起设计](#ch-51)
- [52. O4 侦察：先画真实硬件链路](#ch-52)
- [53. O4 高清图像如何变成可判读证据](#ch-53)
- [54. 关键几何：筒壁会挡住标识，宽视场不能绕过遮挡](#ch-54)
- [55. 怎样重新求侦察高度、航线与速度](#ch-55)
- [56. 危险化学品标识识别：怎样确定类别与拒识](#ch-56)
- [57. 标识透视校正、模板与时序融合](#ch-57)
- [58. 人员判读与任务证据接口](#ch-58)

### 第六部分：验证、部署与团队交付

- [59. 记录什么数据，才能真正改进视觉](#ch-59)
- [60. rosbag 与离线回放：让错误可重复](#ch-60)
- [61. 故障注入与自动测试](#ch-61)
- [62. 不飞也能完成的大部分视觉验收](#ch-62)
- [63. 从台架到实飞：逐级放行](#ch-63)
- [64. 常见故障：按症状定位到哪一层](#ch-64)
- [65. 部署时把模型、标定和运行环境绑定](#ch-65)
- [66. 团队可以直接使用的三张记录表](#ch-66)
- [67. 综合实践：从空目录做到可解释视觉系统](#ch-67)
- [68. 学习验收题与参考答案](#ch-68)

### 查阅附录

- [附录 A：常用函数与数据结构速查](#appendix-a)
- [附录 B：视觉接口与参数设计速查](#appendix-b)
- [附录 C：给零基础读者补齐数学与深度学习](#appendix-c)
- [附录 D：正文练习代码的使用地图](#appendix-d)
- [附录 E：中英文术语表](#appendix-e)
- [附录 F：练习验证方法与记录](#appendix-f)
- [附录 G：来源、版本与延伸阅读](#appendix-g)


---

<a id="ch-01"></a>

# 1. CUADC 的视觉任务不是“在画面上画三个框”

## 1.1 从实际任务倒推系统输出

本教程以多旋翼寻找圆筒、对准投放与观察筒内标识为学习场景。下表固定一组练习条件，便于推导几何与讨论接口；它不是当届竞赛规则摘要。改变场景尺寸或判定条件时，应重新计算覆盖、精度和时间需求。

| 项目 | 本教程练习条件 | 给视觉系统带来的要求 |
|---|---|---|
| 投放容器 | 3 个白色圆筒，高 30 cm，直径分别 15、20、25 cm | 找到不同实体，并估计可用于投放的筒心 |
| 标准载荷 | 两瓶未开封 550 ml 饮用水 | 两次任务目标和两个投放口不能混淆 |
| 有效区域 | 桶内 A 区；筒心周围直径 1 m 的桶外 B 区 | 误差目标不能只用“看着差不多”描述 |
| 侦察容器 | 5 个白色圆筒，高 15 cm、直径 20 cm | 不能把投放桶参数直接搬到侦察桶 |
| 侦察标识 | 其中 3 个筒内放置 12×12 cm 危险化学品标识 | 看见白桶不等于看清筒内标识 |
| 侦察计分 | 正确 +100，错误 −100，空白 0 | 不确定时拒识是有意义的策略 |
| 后续任务条件 | 两瓶均在投放区，且至少一瓶在有效区 | 舵机 ACK 不是这个条件的物理证明 |

后续覆盖计算可取 8 m × 5 m 的矩形练习区。先在纸面或仿真中画清区域边界、坐标原点与目标摆放位置，再讨论路线；实际任务使用实地测量值。

## 1.2 把一个“识别结果”拆成六个问题

一个桶被框出来后，还必须回答：它是不是桶？是哪一个实体？桶口中心在哪里？位置属于什么坐标系？这个位置对应哪个时刻？质量是否足以让控制器使用？

例如 `x=0.2, y=-0.1, z=-1.8` 只是三个数。没有坐标系、单位和时间戳，它不比一张截图更适合自动飞行。

一个可交接的观测至少应该具有这样的语义：

```text
在 observation_stamp 对应的图像中，
识别到一个桶口，
相对指定机体参考原点的 FLU 坐标为 position_body_m，
实测直径为 diameter_m，
并附带检测分数、定位质量、无效原因。
```

## 1.3 成功指标要分层

| 层级 | 例子 | 不能替代什么 |
|---|---|---|
| 图像可用 | 有完整颜色帧、曝光正常 | 不代表有有效深度 |
| 检测可用 | 找到桶，轮廓合理 | 不代表筒心三维位置正确 |
| 定位可用 | 坐标、尺度、时间都通过检查 | 不代表飞控能稳定对准 |
| 控制可用 | 投放点误差和运动状态达标 | 不代表机构已经脱瓶 |
| 比赛有效 | 满足规则的第一落点、侦察、着陆条件 | 不能由软件日志单独保证 |

本教程的目标是建立前四层的可解释链路，并说明第五层需要什么额外证据，而不是许诺一个模型就能保证得分。

**练习 1**：为一次自己设计的离线或台架实验各定义一条“有图”“检测成功”“定位可信”“允许释放”“物理脱离”的证据。找不到的写“目前不可观测”，不要用相邻层的证据代替。

<a id="ch-02"></a>

# 2. 先定义任务、观测和实验假设

## 2.1 先写输入与输出

为视觉练习写一页任务说明：输入来自图片、RGB-D 相机还是视频；目标是框、分割区域还是三维中心；输出给显示界面、记录程序还是控制程序。三个消费者需要的证据强度不同。

对三维观测，至少定义坐标参考原点、轴向、单位、测量时间、有效范围和失败语义。检测模型名称属于实现选择，不应代替这些契约。

## 2.2 两条不同的视觉链路

```text
三维定位练习：
RGB-D → 颜色与深度校验 → 候选检测 → 几何定位
      → 相机到机体变换 → 带测量时间的观测 → 隔离接收器

图传判读练习：
相机 → 编码与传输 → 地面显示 → 人员判读与证据记录

自动识别扩展：
设备明确支持的采集接口 → 解码 → 标识定位 → 分类与拒识
```

图传链路不必天然提供 USB 摄像头、RTSP 或 ROS 图像话题。以 O4 为例，设备端、接收端、固件和输出方式应逐项核查。[R03][R46]

## 2.3 学习代码的技术选择

图像实验使用 Python、NumPy 和 OpenCV，便于检查中间结果；关键几何增加 C++17 对照。ROS 练习选择 Ubuntu 22.04 / ROS 2 Humble 作为固定教学环境。

数学、时间和协议正确时，Python 与 C++ 都能实现同一条测量链。先测瓶颈，再决定是否更换语言或推理后端。

阅读时区分三类内容：**数学推导**说明成立条件，**官方接口**说明类型与 API，**实验参数**需要用自己的数据选择。

<a id="ch-03"></a>

# 3. 学习顺序、环境与安全隔离

## 3.1 为什么先离线，不先接飞控

视觉节点通过目标话题影响任务判断；即使节点本身没有解锁或机构客户端，连接任务图后也可能改变后续动作。实验阶段先验证只读接收器，再考虑与真实任务接口连接。

本文 ROS 练习使用 `/lab` 命名空间，不含飞行命令。仍然应断开真实飞控及其 MAVLink 路由，并使用独立 ROS Domain。命名空间和 Domain 只是隔离措施的一部分，不是物理安全保证。

学习顺序为：离线图片 → 合成 RGB-D → 桌面相机 → 无动力标定 → 隔离 ROS 接口 → 软件仿真 → 经团队批准的低风险实机测试。

## 3.2 普通离线环境

在独立学习环境运行，并记录依赖版本：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy opencv-python pytest

python -c "import sys,numpy,cv2; print(sys.version); print(numpy.__version__,cv2.__version__)"
```

没有图形桌面且只读写文件时，可用 `opencv-python-headless` 替代 `opencv-python`，不要在同一环境混装多个提供 `cv2` 的 wheel。

上述命令没有锁定“全世界通用最佳版本”。安装通过之后应记录实际版本并保留锁定文件；比赛环境不应在赛前随意升级。[R04][R05]

## 3.3 ROS 环境与训练环境分开

ROS 2 的 `cv_bridge`、系统 OpenCV 和 Python ABI 有配套关系。训练环境可能使用另一套 PyTorch、CUDA 和 NumPy。不要为了跑一个训练脚本就全局升级 `/usr/bin/python3` 的包，也不要让 Conda 环境覆盖正常的 ROS 运行环境。[R27]

一个稳妥的组织方式是：

```text
训练电脑/独立虚拟环境：数据集、训练、导出、离线评估
机载运行环境：固定 ROS、相机 SDK、推理后端、标定和模型
纯算法测试环境：不依赖 ROS/相机的数学与协议单元测试
```

交付时保存 `python --version`、`pip freeze`、ROS 发行版、SDK/固件版本、模型 SHA256 和配置 SHA256，而不是只写“环境没问题”。

## 3.4 第一条验收命令

按附录 D 保存正文代码，建立空的 `cuadc_vision_lab/__init__.py` 和 `tools/__init__.py`，在练习根目录运行：

```bash
python -m pytest -q
python -m tools.make_scene
python -m tools.classical_detect
```

这三条只创建教学数据和离线结果，不打开相机或连接飞控。后文会把每个环节拆开。

<a id="ch-04"></a>

# 4. 写视觉程序需要的 Python 和 NumPy 基础

## 4.1 把图像当数组，而不是当文件名

```python
import cv2
image = cv2.imread("outputs/scene/color.png")
if image is None:
    raise FileNotFoundError("图像读取失败")
print(image.shape, image.dtype)
height, width = image.shape[:2]
```

彩色图像常见形状为 `(height, width, 3)`。访问一个像素用 `image[v, u]`，不是 `image[u, v]`。二维坐标常写 `(u,v)`，数组索引却先行后列，这是最常见的静默错误之一。

## 4.2 数据类型影响数值含义

`uint8` 适合颜色值，`uint16` 常用于原始深度编码，`float32/float64` 适合米制坐标和矩阵。`uint8` 不是可以随意相减的有符号数：

```python
import numpy as np
left = np.array([10], dtype=np.uint8)
right = np.array([20], dtype=np.uint8)
difference = left.astype(np.int16) - right.astype(np.int16)
```

训练输入经常要求浮点数与特定归一化，但深度转换必须按其编码单位，不能把所有矩阵都除以 255。[R04]

## 4.3 切片、视图与复制

```python
roi = image[100:200, 50:150]       # 往往是原数组的视图
roi_copy = roi.copy()             # 独立副本
mask = np.zeros((height, width), dtype=bool)
mask[100:200, 50:150] = True
```

修改 `roi` 可能影响原图。把 SDK 帧缓冲交给另一个线程时，还要保证帧对象存活；必要时复制数据，但要测量复制的成本。

## 4.4 常用语法如何落到任务中

| Python 结构 | 视觉用途 |
|---|---|
| `dataclass` | 定义内参、检测框、深度质量结构 |
| `list` | 一帧多个桶观测 |
| `deque(maxlen=N)` | 有界历史窗口 |
| `with open(...)` | 可靠关闭文件 |
| `try/finally` | 无论异常与否都停止相机 |
| `Path` | 路径拼接、目录创建和文件存在检查 |
| `None` / 异常 | 明确表达没有有效估计 |
| 类型标注 | 帮助交接，不代替运行时校验 |

**练习 2**：从合成图裁出三个桶，打印每块 ROI 的尺寸；在副本画十字，证明没有污染原图。随后把 `u,v` 交换一次，观察程序是否会报错——很多时候不会，这正是它危险的原因。

<a id="ch-05"></a>

# 5. 颜色、编码与图像预处理

## 5.1 BGR、RGB、灰度不是同一种输入

OpenCV 的常见彩色读取结果是 BGR；某些模型前处理使用 RGB。颜色顺序错了，通常不会崩溃，只会使结果变差。

```python
rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
```

不确定模型 API 是否已经做通道转换时，不要再手动交换一次。对 Ultralytics 等高层 API，应按其输入约定传入，不把自写 ONNX 前处理与高层 `predict()` 前处理叠加。[R18]

## 5.2 HSV 的作用与边界

HSV 可以把颜色类型、饱和度和亮度分开。OpenCV 常见 8 位 HSV 中，H 的有效范围是 0～179，S、V 是 0～255。[R07]

白桶常表现为较低饱和度和较高亮度，但白纸、过曝地面、反光塑料也会满足。白色阈值是候选生成手段，不是桶身份的充分条件。

```python
white = cv2.inRange(hsv,
                    np.array([0, 0, 160], np.uint8),
                    np.array([179, 70, 255], np.uint8))
```

这里的 `160/70` 是教学合成图阈值，不是比赛通用参数。

## 5.3 预处理必须保留几何记录

裁剪、缩放、旋转、去畸变和 letterbox 都改变了像素与原图的关系。可视化图与用于测量的图可以不同，但必须保存变换关系。

建议每帧记录：`source_shape`、`processed_shape`、ROI 原点、缩放比例、padding、是否去畸变、使用的内参版本。只有截图上画了框而没有这些信息，通常不足以追查厘米级偏差。

## 5.4 不要“增强出”用于判定的细节

对比度增强、锐化、插值放大可能帮助人看图，但不会凭空创造可验证的文字笔画。生成式超分辨率尤其不能替代原始证据。标识判读应保留原帧与处理帧并排检查。

<a id="ch-06"></a>

# 6. 曝光、运动模糊、帧率与有效观察时间

## 6.1 帧率不等于快门速度

30 fps 表示每秒输出约 30 帧，不表示每帧只曝光 `1/1000 s`。同样，100 fps 的图传也不能证明每帧都没有运动模糊。

在近似正视平面、横向匀速运动的小视角模型中：

$$
b_{px}\approx\frac{f_{px}v_{\perp}t_{exp}}{H}
$$

其中 `H` 是相机光心到被观察平面的距离，`v_perp` 是相对视线横向速度，`t_exp` 是曝光时间。公式暂时忽略旋转、滚动快门和振动。[教学推导]

假设 `f=960 px`、`H=3 m`、`v=6 m/s`、`t_exp=1 ms`，模糊约 `1.92 px`。改为 `1/250 s`，约 `7.68 px`。对二三十像素宽的小标识，这种差别可能决定能否判读。

## 6.2 高速转场对视觉时延的惩罚

由位置时刻用错产生的一阶误差约为：

$$
e_{xy}\approx v\Delta t
$$

`1.4 m/s × 0.12 s = 0.168 m`；`12 m/s × 0.12 s = 1.44 m`。这不是说已有系统必然有这么大误差，而是说明必须测量剩余时序误差，不能用“导航 30 Hz”来替代。[教学推导]

## 6.3 自动曝光与固定曝光如何选择

先观察自动曝光在太阳、阴影、地面材质和高度变化中的行为，再决定是否限制曝光上限、增益或锁定曝光。曝光过短会使图像变暗、噪声上升；曝光过长会拖影。固定曝光也可能在进阴影时失效。

正确的测试对象是“最终判读/定位成功率随曝光的变化”，不是最亮、最干净的一帧。[R02]

## 6.4 能用的帧数不是 FPS 乘一段随便选的时间

$$
N_{useful}\approx f_{effective}\,T_{visible}
$$

`f_effective` 应排除重复帧、丢帧、模糊帧、遮挡帧和无法解码帧。`T_visible` 要用标识实际可见的时间，而不是飞机穿过整个场地的时间。后文会从圆筒遮挡重新计算它。

**练习 3**：对同一目标固定 ROI，比较多组曝光的原图、Laplacian 方差、人工读对率。方差只作诊断指标，不独立决定合格。

<a id="ch-07"></a>

# 7. D435i：它给你什么，也不给你什么

## 7.1 双目深度与 RGB 是两条测量链

D435i 的深度来自双目立体测量，并具有 IMU；它不是“RGB 图像中心天然附带一个绝对高度”。产品资料和 SDK 描述了深度、颜色、内外参及 IMU 的不同数据来源。[R01][R06]

不要把产品页上深度传感器的 FOV、全局快门描述推广到所有颜色流。实际使用的 RGB/深度分辨率、曝光、快门行为和相互关系，应按具体型号资料与运行配置核验。

## 7.2 双目深度的基础关系

对于理想校正双目：

$$
Z=\frac{fB}{d},\qquad
\sigma_Z\approx\frac{Z^2}{fB}\sigma_d
$$

`B` 为基线，`d` 为视差。距离越远，同样的视差误差通常会造成更大 Z 误差。这是公式揭示的几何趋势，不是某个厂商给所有场景保证的误差曲线。

## 7.3 室外深度为什么需要重新验收

白色低纹理表面、反光、强太阳、边缘遮挡、草地纹理、运动、曝光和深度算法设置都会改变有效深度分布。IR 发射器不是室外所有问题的万能补丁。[R02][R09]

一张颜色图看起来正常时，深度仍可能空洞或取到了别的表面。你的算法必须能返回“检测到桶，但现在不能定位”。

## 7.4 IMU 不等于已经有 VIO

D435i 有 IMU，不代表使用它的程序已经实现视觉惯性里程计，也不代表飞控自动融合它。学习时把相机 IMU、视觉里程计估计器和飞控导航输入画成三个独立环节，逐一确认是否存在以及时间和坐标是否一致。

**验收输出**：相机型号、序列号、固件、SDK、深度/颜色 profile、depth scale、每路时间戳域、内外参来源。没有这些，不进入标定与飞行对接。

<a id="ch-08"></a>

# 8. 实践：只读采集一组可复现的 RGB-D 数据

## 8.1 两种相机接入方式不要同时抢设备

可以直接用 `pyrealsense2` 独占相机，也可以由 RealSense ROS wrapper 统一采集、其他节点订阅。教学采集器采用前者；机载架构一般应明确一个设备所有者。[R06][R10]

先确认没有其他进程占用相机，再运行：

```bash
python -m tools.capture_realsense --list
python -m tools.capture_realsense --serial 实际序列号 --frames 10 --out outputs/capture_001
```

脚本导入 SDK 失败时，说明该环境尚未安装对应 `pyrealsense2`，不能用伪造深度替代。按照官方安装资料与设备兼容矩阵固定 SDK；不要把任意 `pip install` 成功等同于 USB、固件和驱动全部兼容。[R06]

## 8.2 每帧保存的不止是图片

第8章采集器保存 `bgr`、对齐后的原始深度整数、`depth_scale`、实际内参、颜色/深度帧号、设备时间戳、设备时钟域、主机接收 Unix 时间和主机单调时间。

主机接收时间字段明确叫 `host_receipt`，不叫 `hardware_capture`。拿到一帧的时间晚于曝光时刻，USB 缓冲与调度延迟不能凭命名消失。[R11][R12]

## 8.3 深度配准与采样时间分开记录

`align` 改变空间配准，不使两路曝光成为同一物理时刻。正文代码保留原始颜色、深度帧的时间信息，再保存对齐结果。[R08]

硬件练习需先核查 SDK 和设备能力，再执行下列采集与验收。

**完整文件：`tools/capture_realsense.py`。**

```python
"""只读 D435i 采集。保存原始 depth units、scale、实际内参和多种时间戳。

硬件练习需核查 SDK 支持并记录本机结果；独占使用相机，不要同时启动 ROS wrapper。
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np


def intrinsics_dict(frame):
    i = frame.profile.as_video_stream_profile().get_intrinsics()
    return {'width': i.width, 'height': i.height, 'fx': i.fx, 'fy': i.fy,
            'cx': i.ppx, 'cy': i.ppy, 'model': str(i.model), 'coeffs': list(i.coeffs)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--list', action='store_true')
    p.add_argument('--serial')
    p.add_argument('--frames', type=int, default=10)
    p.add_argument('--out', default='outputs/camera_capture')
    args = p.parse_args()
    import pyrealsense2 as rs
    context = rs.context()
    if args.list:
        for device in context.query_devices():
            print(device.get_info(rs.camera_info.name), device.get_info(rs.camera_info.serial_number))
        return
    if not args.serial or args.frames <= 0:
        p.error('采集需要 --serial 和正的 --frames')
    out = Path(args.out); out.mkdir(parents=True, exist_ok=False)
    pipe = rs.pipeline(context); config = rs.config()
    config.enable_device(args.serial)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    started = False
    try:
        profile = pipe.start(config); started = True
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align = rs.align(rs.stream.color)
        # 预热只是教学设定，不是自动曝光必然收敛的证明。
        for _ in range(30):
            pipe.wait_for_frames(3000)
        for index in range(args.frames):
            frames = pipe.wait_for_frames(3000)
            host_receipt_ns = time.time_ns()
            host_monotonic_ns = time.monotonic_ns()
            color_raw, depth_raw = frames.get_color_frame(), frames.get_depth_frame()
            if not color_raw or not depth_raw:
                raise RuntimeError('原始帧组不完整')
            aligned = align.process(frames)
            color, depth = aligned.get_color_frame(), aligned.get_depth_frame()
            if not color or not depth:
                raise RuntimeError('没有完整颜色/对齐深度帧；不生成假数据')
            bgr = np.asanyarray(color.get_data()).copy()
            raw_units = np.asanyarray(depth.get_data()).copy()
            np.savez_compressed(out/f'frame_{index:05d}.npz', bgr=bgr, depth_units=raw_units)
            metadata = {'index': index, 'depth_scale_m_per_unit': scale,
                        'host_receipt_unix_ns': host_receipt_ns,
                        'host_receipt_monotonic_ns': host_monotonic_ns,
                        'color_sensor_timestamp_ms': color_raw.get_timestamp(),
                        'depth_sensor_timestamp_ms': depth_raw.get_timestamp(),
                        'color_timestamp_domain': str(color_raw.get_frame_timestamp_domain()),
                        'depth_timestamp_domain': str(depth_raw.get_frame_timestamp_domain()),
                        'color_frame_number': color_raw.get_frame_number(),
                        'depth_frame_number': depth_raw.get_frame_number(),
                        'aligned_depth_intrinsics': intrinsics_dict(depth),
                        'color_intrinsics': intrinsics_dict(color),
                        'note': 'Host receipt is NOT guaranteed exposure time; no ROS stamps here.'}
            (out/f'frame_{index:05d}.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
            print(index, raw_units.shape, scale)
    finally:
        if started:
            pipe.stop()


if __name__ == '__main__':
    main()
```

## 8.4 验收不是“看到了十张文件”

应检查每帧分辨率与内参一致，深度整数乘 scale 后尺度正确，帧号递增，颜色/深度时间差合理，文件没有写盘失败。在已知距离平板上采集多个位置，比较统计量，不只点测一个像素。

<a id="ch-09"></a>

# 9. 内参、针孔投影与畸变：把像素变成射线

## 9.1 内参矩阵的每个元素

$$
K=\begin{bmatrix} f_x&0&c_x\\0&f_y&c_y\\0&0&1\end{bmatrix}
$$

`fx/fy` 是以像素为单位的焦距；`cx/cy` 是主点。它们属于指定分辨率、指定图像处理模式，不能跨 profile 随便复用。[R13]

针孔关系为：

$$
u=f_x X/Z+c_x,\quad v=f_yY/Z+c_y
$$

反过来：

$$
X=(u-c_x)Z/f_x,\quad Y=(v-c_y)Z/f_y
$$

没有深度或平面等约束时，一个像素只能确定射线，不能唯一确定三维点。

## 9.2 畸变不是一个“偏移常数”

径向畸变通常随离主点距离改变；切向畸变反映另一类几何偏差。普通 Brown 模型与鱼眼模型不能把同样的系数数组互换使用。[R13][R14]

用 SDK 投影/反投影时，应使用该流实际 distortion model。自己写针孔反投影前，必须确认输入已被正确去畸变，或先由相应模型把原像素变为无畸变射线。[R08]

## 9.3 ROI、resize、letterbox 的反向映射

如果原图 ROI 左上角为 `(x0,y0)`，先裁剪再按比例 `s` 缩放、padding 为 `(px,py)`，网络坐标需要按相反顺序还原：

$$
u_{src}=(u_{net}-p_x)/s+x_0
$$

垂直方向同理。更严格的像素中心约定与插值实现有关；要求亚像素一致时应把整条前处理映射写成测试，而不是凭记忆套公式。

正文中的 `resize_intrinsics()` 采用 OpenCV resize 的像素中心约定：`c'=(c+0.5)s−0.5`。其他采样方式可能采用不同约定。[教学实现]

## 9.4 最基础的往返测试

给一个已知相机点 `p_c`，先投影，再用同一 Z 反投影，应该回到原点。这个测试只验证数学实现，不能证明实际相机标定准确。

**练习 4**：`fx=600, cx=319.5, u=379.5, Z=2`，手算 `X=0.2 m`；然后用代码验证。

<a id="ch-10"></a>

# 10. 实践：从拍棋盘到一份可审计的内参文件

## 10.1 棋盘尺寸的两个常见错误

`9×6` 一般指内角点的列数和行数，不是棋盘方格总数。`square_m` 指实测一个格子的边长，单位米。打印缩放、纸张弯曲和棋盘不平会进入标定结果。[R15]

采集时覆盖画面中央、四周、不同距离和不同倾斜角。十几张几乎同姿态的图片不能替代有几何变化的数据。

## 10.2 完整标定脚本

```bash
python -m tools.calibrate_chessboard \
  --images datasets/calibration/train \
  --cols 9 --rows 6 --square-m 0.025 \
  --out outputs/calibration.json
```

`0.025` 仅表示一块实测格长为 25 mm 的教学棋盘。换板后必须换成真实值。

**完整文件：`tools/calibrate_chessboard.py`。**

```python
"""普通针孔/Brown 模型棋盘内参标定，不适用于未经验证的 O4 超广角模型。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--images', required=True)
    p.add_argument('--cols', type=int, default=9, help='内角点列数，不是方格数')
    p.add_argument('--rows', type=int, default=6)
    p.add_argument('--square-m', type=float, required=True)
    p.add_argument('--out', default='outputs/calibration.json')
    args = p.parse_args()
    if min(args.cols, args.rows) < 3 or args.square_m <= 0:
        p.error('棋盘尺寸无效')
    pattern = (args.cols, args.rows)
    obj = np.zeros((args.cols*args.rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)*args.square_m
    objects, pixels, used = [], [], []
    image_size = None
    for path in sorted(Path(args.images).glob('*')):
        if path.suffix.lower() not in {'.png', '.jpg', '.jpeg'}:
            continue
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            print('unreadable:', path); continue
        size = (gray.shape[1], gray.shape[0])
        if image_size is not None and size != image_size:
            raise ValueError('不能混合不同分辨率标定')
        image_size = size
        found, corners = cv2.findChessboardCorners(gray, pattern)
        if not found:
            print('no board:', path); continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                  (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4))
        objects.append(obj.copy()); pixels.append(corners); used.append(str(path))
    if len(used) < 10:
        raise RuntimeError('教学最低门禁：至少 10 张可用、姿态和位置有变化的棋盘图')
    rms, k, d, rvecs, tvecs = cv2.calibrateCamera(objects, pixels, image_size, None, None)
    per_view = []
    for op, ip, rv, tv in zip(objects, pixels, rvecs, tvecs):
        projected, _ = cv2.projectPoints(op, rv, tv, k, d)
        err = projected.reshape(-1, 2)-ip.reshape(-1, 2)
        per_view.append(float(np.sqrt(np.mean(np.sum(err*err, axis=1)))))
    if not np.isfinite(k).all() or min(k[0, 0], k[1, 1]) <= 0:
        raise RuntimeError('标定结果无效')
    result = {'width': image_size[0], 'height': image_size[1], 'K': k.tolist(),
              'D': d.ravel().tolist(), 'model': 'opencv_brown', 'rms_px': rms,
              'per_view_rms_px': per_view, 'images': used,
              'square_m': args.square_m, 'opencv_version': cv2.__version__,
              'verified_on_holdout': False}
    out = Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2), encoding='utf-8')
    print('RMS:', rms, 'pixels; must validate on holdout images:', out)


if __name__ == '__main__':
    main()
```

## 10.3 RMS 小不等于一切正确

标定脚本输出总体及逐图重投影 RMS。还应检查留出图片、画面边缘直线、已知长度、主点和焦距是否合理，以及不同数据子集的结果稳定性。[R15]

同一批图片既标定又验收，只能检查拟合，不足以评价泛化。相机内部 RGB-depth 标定与机体外参也是两件事；这一份棋盘脚本不解决全部问题。

## 10.4 哪些变化需要重新核验

换分辨率/裁剪模式、镜头或镜座变化、相机拆装、明显机械冲击、启用新的图像稳定/畸变校正模式，都应重新核验适用的标定。原内参是否仍可复用，要看变化发生在哪一层，而不是一律重标或一律沿用。


---

<a id="ch-11"></a>

# 11. O4 的广角画面：FOV、去畸变和电子增稳必须分开

## 11.1 不把型号混在一起

DJI 官方规格区分 O4 Air Unit 与 O4 Air Unit Pro，列出的镜头 FOV 分别为 117.6°、155°；实时图传分辨率/帧率还与眼镜和工作模式有关。型号和模式必须记录，不能从“用了 O4”推断完整参数。[R03]

这些是厂商规格字段，不是已测得的有效水平可读视场。本教程不把它们直接代入一个直线针孔模型，宣称某高度一定覆盖全场。

## 11.2 四个不同的“视场”

| 名称 | 含义 |
|---|---|
| 镜头光学视场 | 镜头和传感器可以成像的范围 |
| 录制视场 | 机内录制模式实际保留的画面 |
| 实时显示视场 | 图传与眼镜/显示设备实际显示的区域 |
| 有效识别视场 | 标识在其中仍可见、清晰、足够大且不被筒壁遮挡的区域 |

比赛规划最终需要第四个。图像边缘看到一个白圆，并不能证明里面的图案可读。

## 11.3 增稳对测量的影响

电子增稳、裁剪、数字变焦和自动畸变校正可能改变逐帧像素到视线的对应关系。对人工判读，它们可能改善观感；对三维测量，它们可能使固定内参不再适用。[R03][R46]

应记录并固定实际使用模式。需要几何测量时，优先获取可建模的数据链，或测量处理后画面的有效映射；不要在稳定后的截图上套未稳定原图的 K。

## 11.4 如何实测可用视场

在训练场用已知尺寸地面网格，固定相机姿态和模式，从最终地面显示链读取画面。分别测出画面中心、中间区域与边缘能可靠读出的标识范围，保留原始录屏。

标定工具和网格用于训练与校准，不得据此提前确定比赛中随机桶的实际位置，或擅自改变比赛目标。

<a id="ch-12"></a>

# 12. 坐标系：先确定“相机看到的前方”是哪一条轴

## 12.1 五种坐标不能混用

| 坐标 | 本文记号 | 约定 |
|---|---|---|
| 图像像素 | `(u,v)` | 右、下，单位 px |
| 相机光学坐标 | `C` | 右、下、前，单位 m |
| 机体坐标 | `B` | 前、左、上，即 FLU |
| 本地坐标 | `W` | 与 MAVROS local 数据契约一致，本文按 ENU 理解 |
| 场地坐标 | `F` | 起飞锁定机头方向为 +X，左侧为 +Y |

ROS 的常用单位与 optical/body 轴约定见 REP 103；但命名符合约定仍不能证明你的数据真的做过正确变换。[R29]

## 12.2 外参应写成有方向的符号

本文约定：

$$
p_B=R_{BC}p_C+t_{BC}
$$

`R_BC` 把 C 系向量旋转成 B 系表达；`t_BC` 是相机光心在 B 系的位置。符号下标顺序明确后，就不会把变换方向和逆变换混用。

逆变换为：

$$
p_C=R_{BC}^{T}(p_B-t_{BC})
$$

不能简单把平移取负却不旋转。

## 12.3 机体原点不等于“随便选一个中心”

视觉输出的 body 原点、odom 所代表的机体参考点、投放口外参参考点，必须是同一刚体基准或有已知变换。飞控安装中心、IMU、重心、RTK 天线、机架几何中心不是天然相同点。

在纸上标出相机光心、机体参考原点、导航估计原点和释放口。仅凭 `position` 这样的变量名无法判断它对应哪一个物理点；导航原点与机体原点不一致时，应单独建模杠杆臂。

## 12.4 相机向下安装的轴向练习

假设相机严格向下、图像上方指向机头、无镜像，则可使用：

$$
R_{BC}=\begin{bmatrix}0&-1&0\\-1&0&0\\0&0&-1\end{bmatrix}
$$

这只是教学假设。把一块标记板依次放到飞机前、左、右，检查输出符号。显示程序旋转图片不会自动旋转算法坐标。

如果曾约定“图像上方对应飞机右侧”，在严格下视、无镜像的额外假设下，旋转会变成另一种关系。必须用当前安装实测，不能凭旧对话自动沿用。

<a id="ch-13"></a>

# 13. 相机到机体外参：测量、估计与验证

## 13.1 外参包含六个自由度

平移三项和旋转三项都需要确定。尺量能给光心位置近似，却很难单独给出高精度安装角。将外壳边缘当光心，又会引入固定偏差。

标定记录至少包含：相机流、光学坐标、机体参考原点、旋转表达、长度单位、标定日期、设备标识、安装照片、误差与适用模式。

## 13.2 基于已知点的刚体配准

在无动力台架上设置多个已知的机体系三维点 `p_Bi`，用相机测得对应 `p_Ci`，求：

$$
\min_{R,t}\sum_i\|p_{Bi}-(Rp_{Ci}+t)\|^2,
\quad R^TR=I,\ \det R=1
$$

先去中心，利用 SVD 求旋转，再恢复平移。点不能全重合或几乎共线；深度失真和已知点测量误差也会影响结果。[教学推导]

只用一个桶的一次对准，不足以约束所有外参。尤其难以区分安装角误差、XY 平移误差与深度尺度偏差。

## 13.3 PnP 解出来的是哪一个变换

OpenCV `solvePnP` 估计的是物体坐标到相机坐标的旋转与平移，不是直接输出相机到机体。使用它后还要结合标定板到机体的已知关系，按明确的链式变换求外参。[R16]

对于平面标定板，可能出现姿态歧义或弱约束；应利用正深度、重投影误差、安装先验和多视角一致性，而不是只看 `success=True`。

## 13.4 标定与验收必须用不同数据

用一批姿态拟合外参，另一批姿态检查已知点位置。至少覆盖不同高度、画面边缘、左右和前后倾斜。只在水平中央验证，容易漏掉 Z 力臂和旋转符号错误。

**验收表建议**：真实位置、估计位置、XY 误差、Z 误差、roll/pitch、像素位置、深度质量、外参版本。阈值由任务误差预算决定，不在本教程中虚构一套“保证命中”的数值。

<a id="ch-14"></a>

# 14. 完整刚体变换：相机 → 机体 → 本地世界

## 14.1 先写出数学，再写代码

$$
p_W=p_{WB}+R_{WB}(R_{BC}p_C+t_{BC})
$$

先做相机到机体，再用观测时刻机体姿态、位置转到世界。顺序不可以互换。

用齐次变换表达：

$$
T_{WC}=T_{WB}T_{BC},\qquad
T=\begin{bmatrix}R&t\\0&1\end{bmatrix}
$$

这里所有量必须对应一致的时间与原点。`R` 对，不代表 `t` 对；二者都对，不代表时间对。

## 14.2 欧拉角旋转顺序

本章规定 `R = Rz(yaw) Ry(pitch) Rx(roll)`，对列向量先做 roll，再做 pitch，最后做 yaw。其他库可能使用不同的轴与乘法约定，应先用单位轴测试核对。

角度输入必须明确是弧度还是度。`std::sin(8)` 不是 `sin(8°)`。矩阵正交性和行列式检查可以排除部分错误，但不能证明轴语义正确。

## 14.3 四元数与插值

四元数适合表达完整姿态；使用前检查有限性与范数。单位四元数 `q` 和 `−q` 表示同一旋转，因此相邻姿态插值需要处理符号一致性。

第39章为便于观察角度回绕，先演示欧拉角按最短角差逐项线性插值。它只适用于远离奇异姿态的短间隔练习；一般三维姿态历史应使用归一化四元数与 SLERP，并处理四元数双覆盖造成的符号差异。

## 14.4 最少要有的单元测试

零旋转保持向量；绕 Z 轴 +90° 把 X 轴转向 Y 轴；变换后再逆变换回原点；反射矩阵应拒绝；NaN 应拒绝；A1/A2 外参应产生不同期望机体位置。

这些测试是把“方向应该对”变成可运行的约束。

<a id="ch-15"></a>

# 15. 实践：反投影、平面交点、三维圆和投放口几何

第15章核心文件把数学从 ROS 回调中拆出来，以便不接相机和飞机也能测试。下面是完整实现。它不会自动判断输入是不是正确的桶沿；输入语义仍由上层负责。

**完整文件：`cuadc_vision_lab/geometry.py`。**

```python
"""带输入校验的几何练习。针孔函数只接受已校正、匹配内参的像素。"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np


def finite_array(value, shape=None) -> np.ndarray:
    out = np.asarray(value, dtype=np.float64)
    if shape is not None and out.shape != shape:
        raise ValueError(f"shape {out.shape} != {shape}")
    if not np.isfinite(out).all():
        raise ValueError("输入包含 NaN/Inf")
    return out


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        finite_array([self.fx, self.fy, self.cx, self.cy])
        if self.width <= 0 or self.height <= 0 or min(self.fx, self.fy) <= 0:
            raise ValueError("图像尺寸和焦距必须为正")

    def matrix(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.]])


def project(point_c, k: Intrinsics) -> np.ndarray:
    x, y, z = finite_array(point_c, (3,))
    if z <= 0:
        raise ValueError("投影点必须在相机前方")
    return np.array([k.fx * x / z + k.cx, k.fy * y / z + k.cy])


def deproject(uv, depth_z_m: float, k: Intrinsics) -> np.ndarray:
    u, v = finite_array(uv, (2,))
    if not math.isfinite(depth_z_m) or depth_z_m <= 0:
        raise ValueError("depth 是正的光轴 Z，单位 m；不是欧氏距离")
    return np.array([(u-k.cx)*depth_z_m/k.fx,
                     (v-k.cy)*depth_z_m/k.fy, depth_z_m])


def resize_intrinsics(k: Intrinsics, width: int, height: int) -> Intrinsics:
    """按 OpenCV resize 的像素中心约定；不涵盖额外裁剪/letterbox。"""
    sx, sy = width/k.width, height/k.height
    return Intrinsics(width, height, k.fx*sx, k.fy*sy,
                      (k.cx+0.5)*sx-0.5, (k.cy+0.5)*sy-0.5)


def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    finite_array([roll, pitch, yaw])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def validate_rotation(rotation) -> np.ndarray:
    r = finite_array(rotation, (3, 3))
    if not np.allclose(r.T @ r, np.eye(3), atol=1e-6):
        raise ValueError("R 不是正交旋转矩阵")
    if not np.isclose(np.linalg.det(r), 1., atol=1e-6):
        raise ValueError("det(R) 必须为 +1；镜像不是旋转")
    return r


def transform(point_src, r_dst_src, t_dst_src) -> np.ndarray:
    return validate_rotation(r_dst_src) @ finite_array(point_src, (3,)) + finite_array(t_dst_src, (3,))


def inverse_transform(point_dst, r_dst_src, t_dst_src) -> np.ndarray:
    r = validate_rotation(r_dst_src)
    return r.T @ (finite_array(point_dst, (3,)) - finite_array(t_dst_src, (3,)))


def ray_plane_intersection(origin_w, direction_w, normal_w, d: float,
                           min_abs_cos: float = 0.10) -> np.ndarray:
    """平面 n·p+d=0。仅接受向前且非近切向的射线交点。"""
    origin = finite_array(origin_w, (3,))
    direction = finite_array(direction_w, (3,))
    normal = finite_array(normal_w, (3,))
    if not math.isfinite(d) or not 0 < min_abs_cos <= 1:
        raise ValueError("平面/夹角参数无效")
    nn, nd = np.linalg.norm(normal), np.linalg.norm(direction)
    if min(nn, nd) < 1e-12:
        raise ValueError("法向和方向不能为零")
    denominator = float(normal @ direction)
    if abs(denominator)/(nn*nd) < min_abs_cos:
        raise ValueError("视线近乎平行于平面，拒绝不稳定交点")
    scale = -(float(normal @ origin)+d)/denominator
    if scale <= 0:
        raise ValueError("交点不在相机前方")
    return origin + scale*direction


def fit_plane_svd(points):
    """输入应先做语义/离群点筛选；这是最小二乘，不是 RANSAC。"""
    p = finite_array(points)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 3:
        raise ValueError("至少三个三维点")
    center = p.mean(axis=0)
    _, s, vt = np.linalg.svd(p-center, full_matrices=False)
    if s[1] < 1e-8:
        raise ValueError("点共线或重合，平面不可观")
    n = vt[-1]
    if n[2] < 0:
        n = -n
    d = -float(n @ center)
    rms = float(np.sqrt(np.mean((p@n+d)**2)))
    return n, d, rms


def fit_circle_3d(points):
    """教学用：平面上三维圆拟合。短圆弧/错误平面仍需外部质量门禁。"""
    p = finite_array(points)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 12:
        raise ValueError("至少 12 个三维轮廓点")
    center0 = p.mean(axis=0)
    _, s, vt = np.linalg.svd(p-center0, full_matrices=False)
    if s[1] < 1e-8:
        raise ValueError("轮廓几何退化")
    basis = vt[:2].T
    xy = (p-center0) @ basis
    a = np.column_stack([2*xy[:, 0], 2*xy[:, 1], np.ones(len(p))])
    b = np.sum(xy*xy, axis=1)
    solution, _, rank, _ = np.linalg.lstsq(a, b, rcond=None)
    if rank < 3:
        raise ValueError("圆拟合秩不足")
    c2 = solution[:2]
    r2 = solution[2]+float(c2@c2)
    if r2 <= 0:
        raise ValueError("无效半径")
    radius = math.sqrt(r2)
    residual = np.linalg.norm(xy-c2, axis=1)-radius
    return center0+basis@c2, radius, float(np.sqrt(np.mean(residual**2)))


def release_geometry(vehicle_w, r_wb, outlet_b, bucket_w, aircraft_z: float):
    """返回当前投放口及期望机体位置；Z 在本练习中固定为机体参考高度。"""
    vehicle, bucket = finite_array(vehicle_w, (3,)), finite_array(bucket_w, (3,))
    lever_w = validate_rotation(r_wb) @ finite_array(outlet_b, (3,))
    if not math.isfinite(aircraft_z):
        raise ValueError("高度无效")
    desired = np.array([bucket[0]-lever_w[0], bucket[1]-lever_w[1], aircraft_z])
    return vehicle+lever_w, desired


def deprojection_covariance(uv, depth_z_m, k, sigma_u, sigma_v, sigma_z):
    """只传播独立像素/Z误差；不包含外参、姿态、时延和系统偏差。"""
    deproject(uv, depth_z_m, k)
    u, v = finite_array(uv, (2,))
    sig = finite_array([sigma_u, sigma_v, sigma_z])
    if (sig < 0).any():
        raise ValueError("标准差不可为负")
    j = np.array([[depth_z_m/k.fx, 0, (u-k.cx)/k.fx],
                  [0, depth_z_m/k.fy, (v-k.cy)/k.fy], [0, 0, 1]])
    return j @ np.diag(sig**2) @ j.T
```

运行：

```bash
python -m tools.geometry_demo
python -m pytest -q tests/test_core.py
```

**完整示例：`tools/geometry_demo.py`。**

**完整文件：`tools/geometry_demo.py`。**

```python
import numpy as np
from cuadc_vision_lab.geometry import *


def main():
    k = Intrinsics(640, 480, 600, 600, 319.5, 239.5)
    p_c = deproject([379.5, 269.5], 2., k)
    print('pixel -> camera:', p_c)
    # 教学姿态：相机垂直向下、图像上方指向机头；不要当作实机外参。
    r_bc = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1.]])
    t_bc = [.08, .02, -.12]
    p_b = transform(p_c, r_bc, t_bc)
    r_wb = rpy_matrix(*np.deg2rad([5., -3., 30.]))
    p_w = transform(p_b, r_wb, [31., 0., 2.42])
    print('camera -> body:', p_b)
    print('body -> world:', p_w)
    outlet, desired = release_geometry([31., 0., 1.3], r_wb,
                                       [.026, -.065, -.32], p_w, 1.3)
    print('actual outlet:', outlet)
    print('desired vehicle (XY compensated, vehicle Z held):', desired)
    print('reproject:', project(p_c, k))


if __name__ == '__main__':
    main()
```

## 15.1 如何读输出

第一步 `(u,v,Z)` 得到相机坐标；第二步加入安装变换；第三步加入观测时刻飞机姿态和位置；最后把桶的 XY 与投放口外参组合，反求期望飞机参考点。

示例的安装姿态、位置和高度是合成数值。输出中的 `desired vehicle Z` 保持固定，表示“只反解水平位置”的练习假设；若要控制释放口到目标面的距离，必须再求解垂直几何。

## 15.2 练习与预期

把 roll 从 0° 改为 8°，只使用 `[0,0,−0.32]` 的力臂时，水平偏移幅度应约 `0.32 sin8°≈4.45 cm`。把旋转矩阵第三轴直接取反却不调整其他轴，应被 `det(R)=+1` 检查拒绝。

<a id="ch-16"></a>

# 16. 准星与人工对准标定：把可视化变成测量工具

## 16.1 画面中心不是必然的投放点

图像中心通常只是显示中心，主点可能不在此处；相机光心与投放口之间还有外参。即使两个点在某高度重合，换高度或姿态也可能分开。

一个有价值的调试画面应分别画：检测中心、几何估计桶口中心、A1 投放投影、A2 投放投影、图像主点，以及当前使用的高度/姿态/标定版本。

## 16.2 “把投放口点投到图像”与“画落点准星”不同

投放口本身相对相机的坐标是：

$$
p_C^{outlet}=R_{BC}^{T}(r_B^{outlet}-t_{BC})
$$

它可能在镜头后方、边缘甚至不在画面内。把这个点投影，不等于瓶子沿重力方向到目标平面的交点。

正确的静态落线准星，需要先求世界中的投放口，沿世界重力方向与选定目标平面相交，再用对应相机姿态投影回图像。还必须说明它没有模拟水瓶初速度与空气动力。[教学推导]

## 16.3 实际标定步骤

先固定飞机且卸下螺旋桨，用铅垂线/可靠测量方法给 A1、A2 的静态投影建立参考；在多个高度、多个姿态保存原图和测量表。分别拟合两个投放口或相机外参，不把两者混进一个不解释的像素偏移。

若只做单高度人工准星，可作为该工况的经验标定，但配置必须标注适用高度和姿态。不能把单点经验修正叫完整六自由度外参标定。

## 16.4 显示转向的检查

把板放到飞机左侧，画面提示应该与实际机体系方向一致。若显示镜像、旋转或云台模式变化，首先查显示与算法是否共享同一个变换，而不是反复试改控制符号。

**练习 5**：不用连接舵机，只做 A1/A2 静态准星叠加和位置误差 CSV。每个点记录至少三次独立放置，比较重复性与偏差。

<a id="ch-17"></a>

# 17. 深度的单位和含义：Z、斜距、相对高度不是同一个量

## 17.1 原始整数乘比例才得到米

RealSense 原始深度像素需要由实际 `depth_scale` 转成米；不要把 SDK 原始数据的 scale 与 ROS 消息编码默认单位混为一谈。[R08]

ROS REP 118 定义了常见深度表示语义：浮点米制表示与 OpenNI 兼容的 16 位毫米表示。具体驱动输出仍必须核对 encoding 与契约。[R30]

## 17.2 光轴 Z 与欧氏距离

对于点 `(X,Y,Z)`：

$$
r=\sqrt{X^2+Y^2+Z^2}
$$

反投影使用的深度通常是 Z，不是 r。画面边缘的 r 明显大于 Z；拿 r 替代 Z 会把 X、Y 一起放大。

## 17.3 飞控相对高度与相机目标距离

`home_z+1.3 m` 描述飞机参考点；相机光心高度还要加旋转后的相机 Z 偏置；目标表面可能在地面上方 0.3 m；沿光轴测量又与垂直距离不同。

因此：

```text
飞机参考点相对起飞高度
≠ 相机光心离地高度
≠ 投放口离地高度
≠ 相机到桶口的光轴 Z
≠ 相机到桶底的斜距
```

## 17.4 无效深度的表达

0、NaN、Inf、超量程、低有效比例都应被显式标记。不要用 `z=0`、默认高度或上一帧的值冒充当前真实测量。确实采用模型补偿时，要标注 `source=model` 和估计不确定性。

<a id="ch-18"></a>

# 18. 彩色与深度配准：同一个像素编号不一定看同一个地方

## 18.1 空间配准解决什么

颜色与深度有不同内参、视场和物理位置。`color[v,u]` 与未经配准的 `depth[v,u]` 通常不是同一视线。[R08]

概念过程为：深度像素反投影 → 深度相机坐标 → RGB 相机坐标 → RGB 像素。配准还需处理遮挡、冲突和空洞，不只是 resize。[R08]

## 18.2 配准后的内参要匹配输出

在 SDK 中从实际输出 profile 读取对应内参；在 ROS 中明确输出图像与 CameraInfo 的关系。不能把对齐到 RGB 的深度继续配原生深度 K。

同样，“尺寸一样”不是“配准成功”的证明。两幅图都为 640×480，仍然可能视角不同。

## 18.3 配准不消除遮挡

从一个传感器能看到的桶底区域，另一个传感器可能被桶沿遮挡。投影到颜色图后会出现空洞、错误边界或多对一覆盖。桶沿正是最应该认真检查的区域之一。[R08]

## 18.4 配准验收实验

拿一块有前后深度台阶的板，在近、中、远距离及画面边缘移动，叠加深度突变边缘与 RGB 边缘。只用一块大平墙检查不容易发现遮挡问题。

保存空间错位像素量与工况，进而估算它对目标中心和直径的影响。

<a id="ch-19"></a>

# 19. 深度鲁棒统计：先判有效，再谈平滑

## 19.1 为什么不只取中心一个像素

中心像素可能落在桶底、孔洞、反光、边缘混合或无效点。一个合法数值也不等于它属于你想测量的表面。

应先定义语义区域，再筛选有限、正、合理范围的深度，计算有效数量/比例、分位数跨度、中值和 MAD。

$$
MAD=\operatorname{median}(|z_i-\operatorname{median}(z)|)
$$

MAD 是鲁棒离散度，不自动等于标准差或置信区间。

## 19.2 两个表面的中值可能是错的表面

ROI 若一半是桶沿、一半是桶底，中值可能落在两者之间，或者偏向面积较大的那个。多加几层平滑不能恢复缺失的语义。

教学函数对过宽的 `p95−p05` 分布直接拒绝，避免把混合深度伪装成单一平面。真实倾斜平面天然有深度变化，所以真实场景应改为平面残差统计，而不是无限放宽这个教学阈值。

**完整文件：`cuadc_vision_lab/depth.py`。**

```python
"""深度有效性实验；有效值不足时抛出异常，绝不填成可投放目标。"""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class DepthEstimate:
    z_m: float
    mad_m: float
    valid_count: int
    valid_ratio: float
    spread_p90_m: float


def to_meters(raw: np.ndarray, scale: float) -> np.ndarray:
    if raw.ndim != 2 or not math.isfinite(scale) or scale <= 0:
        raise ValueError("输入需为二维原始深度，且 scale>0")
    return raw.astype(np.float64)*scale


def robust_depth(depth_m, mask, min_count=20, min_ratio=0.5,
                 near=0.2, far=6.0, max_spread_m=0.08) -> DepthEstimate:
    depth = np.asarray(depth_m, dtype=np.float64)
    mask = np.asarray(mask, dtype=bool)
    if depth.ndim != 2 or mask.shape != depth.shape:
        raise ValueError("深度与 mask 形状不匹配")
    if min_count < 1 or not 0 < min_ratio <= 1 or not 0 <= near < far or max_spread_m <= 0:
        raise ValueError("质量门限无效")
    selected = depth[mask]
    valid = selected[np.isfinite(selected) & (selected > near) & (selected < far)]
    ratio = len(valid)/max(1, len(selected))
    if len(valid) < min_count or ratio < min_ratio:
        raise ValueError("NO_VALID_DEPTH: 有效像素数量/比例不足")
    spread = float(np.percentile(valid, 95)-np.percentile(valid, 5))
    if spread > max_spread_m:
        raise ValueError("MIXED_SURFACES: 深度分布太宽，不能混合桶底/桶沿/地面")
    z = float(np.median(valid))
    mad = float(np.median(np.abs(valid-z)))
    return DepthEstimate(z, mad, len(valid), ratio, spread)
```

## 19.3 深度后处理的副作用

空间滤波可能混合边界，时间滤波可能引入拖尾，补洞可能填入邻近表面的历史估计，降采样则影响小桶沿可分辨性。RealSense 提供这些处理模块，但组合和参数必须按任务评估。[R54]

控制用深度与显示用彩色化深度要分开。`applyColorMap` 生成的彩色图不是米制深度输入。

<a id="ch-20"></a>

# 20. 空心圆筒：框中心、桶沿、桶底与三维筒心

## 20.1 四个看起来很像的“中心”

轴对齐包围框中心、白色像素重心、投影椭圆中心、真实圆心的投影，不应不加条件地当同一个点。

例如一侧桶壁更亮时，白色区域重心会偏向那一侧；桶身被遮挡时，包围框中心会偏；透视投影下椭圆中心也不必精确等于三维圆心的投影。

## 20.2 最危险的深度错配

假设检测到的是桶口边界中心，却取中心像素的桶底深度。画面斜视时，这条视线到达桶底的位置不是桶口三维圆心，只是同一条射线上的另一个点。

若平面高度用错 `Δh`、视线偏角为 `θ`，横向误差量级可近似为：

$$
\Delta x\approx\Delta h\tan\theta
$$

`Δh=0.3 m, θ=20°` 时约 `0.109 m`。这已经与细对准阈值同量级。[教学推导]

## 20.3 可选的几何方案

| 方案 | 所需证据 | 主要边界 |
|---|---|---|
| 桶沿三维点拟合 | 清晰轮廓、有效沿边深度、正确配准 | 边缘深度常最差，需要严格筛选 |
| 射线与桶口平面求交 | 像素几何中心、已知/估计平面 | 高度和平面法向必须可信 |
| 先拟合桶底再恢复中心轴 | 可见桶底及圆筒几何先验 | 不能把随便一个底部点当轴心 |
| 深度不足时拒绝定位 | 明确质量状态 | 可能增加重观测时间，但语义诚实 |

## 20.4 不要在视觉层偷偷扩大投放成功

视觉可以输出位置质量、候选身份和几何误差，不能仅凭“估计点在桶附近”宣称瓶子已落入有效区。判定投放后果需要额外观测或现场记录。

## 20.5 从桶外地面估计桶口平面

近距离向下观察时，框中心可能落在桶底、筒壁或载荷上。可以把框外一圈区域作为地面候选，排除其他目标与无效深度后，反投影为三维点，再鲁棒拟合地面平面；这给出一种独立于桶内内容的几何参考。

外环只是候选区域，不保证一定是地面。应检查有效比例、点的空间分布、平面内点比例与拟合残差；外环超出图像、混入另一桶或存在台阶时，要拒绝或显式更换方法，不能把中心深度静默当作地面。

令地面在同一世界坐标系中为 `n·p+d=0`，`n` 为向上的单位法向。假设圆筒沿该法向竖立、已知筒高为 `h`，桶口平面为 `n·p+d-h=0`。相机光心 `c` 与已变换到世界系的射线方向 `r` 给出：

$$
p_{rim}=c+\lambda r,\qquad
\lambda=\frac{h-d-n\cdot c}{n\cdot r}.
$$

分母接近零或 `lambda<=0` 时拒绝。只有相机光轴近似垂直地面、平面近似等深时，才可把光轴深度粗略写成 `Z_rim≈Z_ground-h`；相机倾斜时应使用完整平面交点。

这一步仅把一条射线落到桶口平面，不能证明框中心射线就是三维圆心。仍需考虑第24章的透视中心偏差，优先使用可靠桶沿点或标定几何。计算直径与画投放准星时应使用桶口平面，地面点应另存字段；混用两个高度会产生随距离变化的尺度与横向偏差。

练习：在合成场景中保持桶口不动，改变桶底深度、相机高度和倾角，比较“框中心深度”“外环地面减高度”与“平面交点”的结果。报告每种方法成立的假设和拒绝条件。

<a id="ch-21"></a>

# 21. 平面、三维圆与直径估计

## 21.1 快速估计什么时候成立

对于近似正视、等深、已校正图像，圆直径可近似：

$$
D\approx d_{px}Z/f
$$

前提是像素直径、Z 和 f 属于同一成像几何。用原图的 `d_px` 配网络输入的 `f`，或者用桶底 Z 配桶口直径，都会错。

## 21.2 倾斜时为什么不能只看外接框宽度

包围框宽度与目标姿态、轮廓遮挡、透视和镜头畸变相关。把外接框最长边乘深度，最多是某些工况的近似，不是通用三维测量。

更稳妥的教学路线是：对被确认属于同一物理圆周的点进行三维重建，拟合平面，再在平面基上拟合圆。正文中的 `fit_circle_3d()` 展示的是最小二乘骨架，并不包含真实边界质量判别。

## 21.3 RANSAC 与最小二乘如何分工

RANSAC 通过抽样构造假设、统计内点来抵抗离群点；最小二乘通常用于最终内点集精修。一次“拟合成功”仍需要检查内点比例、残差、圆弧覆盖、半径合理性及条件数。[教学说明]

只有一小段弧时，很多大半径圆都能贴合这段弧。残差很小也可能定位很不稳定；因此还要检查几何可观测性。

## 21.4 对 15/20/25 cm 的分类不能只设死阈值

规则给的是三档实体直径，实际估计还有误差。可以先估计连续直径，再用区间或概率做尺寸等级判断；若多个类别区间重叠，就保留不确定性。

连续直径估计与离散尺寸类别应分开保存。可以按可靠直径排序选择目标，也可以根据误差区间推断尺寸类别；当两个估计区间重叠时，应表达不确定，避免把排序结果当作绝对身份。

<a id="ch-22"></a>

# 22. 误差预算：厘米级偏差到底从哪里来

## 22.1 先用一阶敏感度量级判断

对于 `X=(u−cx)Z/fx`：

$$
\sigma_X^2\approx
(Z/f_x)^2\sigma_u^2+
((u-c_x)/f_x)^2\sigma_Z^2
$$

该式暂不包括内参不确定性和相关性。完整三维传播可写 `Σp=JΣqJᵀ`；正文代码给出了独立 `u,v,Z` 误差的教学实现。

再加入姿态小误差的量级 `e≈Lδθ`、时延误差 `vΔt`、外参平移偏差和测量平面偏差。不能把所有项都当独立零均值噪声简单平方和，固定偏差可能同向叠加。

## 22.2 一个教学误差预算表

| 项目 | 示例假设 | 量级 |
|---|---|---|
| 像素定位 | Z=2 m、f=600 px、误差 2 px | 约 6.7 mm |
| 姿态角 | 2 m 杠杆长度、误差 1° | 约 35 mm |
| 时间错位 | 1.4 m/s、100 ms | 约 140 mm |
| 安装平移 | 人工测量偏差 20 mm | 约 20 mm 固定偏差 |
| 桶口/桶底平面混淆 | 0.3 m 高差、20° 视线 | 约 109 mm |

这些是独立的示例量级，不是真实误差的测量结果。

## 22.3 门限不是精度证明

细对准的 `0.12 m` 表示估计几何误差需要进入这个范围；若桶坐标本身偏了 15 cm，控制器仍可能“稳定地对准错误位置”。所以必须分别测定位真值误差和控制跟踪误差。

## 22.4 如何诊断系统性偏差

误差几乎恒定：优先查平移外参/参考原点。随高度成比例：查角度、焦距、尺度。随速度改变方向：查时序。随画面边缘增加：查畸变、ROI 和视线。只在大倾角出现：查旋转顺序、桶口平面和刚体力臂。

**练习 6**：设计一次只改变高度、一次只改变横向速度的实验，其他条件不变。写出两种故障假设各自预测的误差趋势，再用数据区分。


---

<a id="ch-23"></a>

# 23. 第一套传统视觉算法：颜色候选、轮廓与几何筛选

## 23.1 为什么仍然学习传统视觉

传统方法不需要训练集就能帮助理解颜色、连通域、边缘和形状。它也是检查模型结果的工具：模型框出来后，能否找到合理桶沿？深度是否支持这个轮廓？

但传统方法不是默认比深度学习可靠。背景复杂、光照变化、低对比度和部分遮挡会直接挑战阈值方案。它应该先作为基线，再由任务数据决定是否保留。

## 23.2 最小流程

```text
读取 BGR
→ HSV/灰度
→ 生成白色候选掩膜
→ 小尺度形态学去噪
→ 提取轮廓及层级
→ 面积/形状/孔洞/边界检查
→ 输出候选，而不是马上投放
```

`inRange` 做范围阈值，`morphologyEx` 做形状运算，`findContours` 提取轮廓。每个 API 都应该能单独保存中间图检查。[R07][R19][R20]

## 23.3 阈值怎样调才可复现

先固定相机和数据集，在若干独立场景上统计误检/漏检。把阈值写进配置，记录配置哈希。不要一边飞一边随意拖滑块，然后只留下最终代码。

调试 GUI 可以帮助理解参数，但最终运行应有无窗口模式，不依赖远程桌面是否还在线。

## 23.4 多种候选的融合

白色候选、边缘圆/椭圆、蓝色地面区域、深度平面都可以作为线索。组合时要明确每项是在增加召回、减少误检，还是决定三维定位资格。

“白色+圆形”只说明外观合理；“蓝色”也可能来自桶外有效区域。不能把蓝色圆形区域误判成桶口。

<a id="ch-24"></a>

# 24. 轮廓层级、圆度、椭圆与透视偏差

## 24.1 一个环可能有两个轮廓

白色桶沿外边界和内边界会各产生轮廓。如果把每条轮廓都当目标，三个桶可能被错误计成六个。层级关系能帮助把外环和内孔组织成一个实体。[R17]

教学代码使用 `RETR_CCOMP`，只取有子孔洞的外轮廓。这个策略适合本章合成白环，但真实桶可能由于遮挡或阈值断裂失去完整孔洞，因此不能原样宣称实机可靠。

## 24.2 圆度和长短轴比

$$
C=4\pi A/P^2
$$

理想圆的圆度为 1。离散像素、粗糙边界、透视、遮挡会降低这个量。椭圆长短轴比有助于判别形状，但过于严格会拒绝斜视的真桶。

`fitEllipse` 至少需要足够点，输出的两轴顺序与角度约定要按 API 理解。直径用于测量前还要经过三维几何解释。[R17]

## 24.3 Hough 圆检测适合什么

HoughCircles 可在近圆形边缘上生成候选，但投影椭圆、遮挡弧、反光和纹理会影响稳定性。`minDist` 过大可能压掉相近真桶；过小可能为一个桶返回多个圆。[R21]

它不是在所有情况下都优于轮廓，更不是加入后就自动解决三桶身份问题。

## 24.4 图像中心估计的局限

椭圆中心是投影形状的一个描述量。对控制精度要求较高时，应通过三维圆、已知平面或标定实验验证它与真正筒心的关系。不要把“拟合残差很小”直接写成“中心定位误差很小”。

<a id="ch-25"></a>

# 25. 实践：生成教学 RGB-D 场景并找出三个环

## 25.1 这个实践能验证什么

它验证图像读写、颜色阈值、轮廓层级、深度筛选、简单尺度估计和 JSON 输出。场景是本教程生成的理想平面测试数据，**不用于证明真实 D435i、YOLO 或比赛效果**。

运行后得到：

```text
outputs/scene/
  color.png
  depth_m.npy
  intrinsics.json
  ground_truth.json
  detected.png
  detections.json
```

彩色图中的三个白环使用已知直径与焦距生成，内孔和背景深度不同，另有白矩形干扰。这样可以验证“不能把所有白色东西都当桶”。

## 25.2 完整候选检测器

**完整文件：`cuadc_vision_lab/detection.py`。**

```python
"""白色圆环基线：用于学习轮廓/掩膜，不是已验证的比赛模型。"""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class Candidate:
    u: float
    v: float
    diameter_px: float
    circularity: float
    axis_ratio: float
    rim_mask: np.ndarray


def detect_white_rings(bgr: np.ndarray, min_area=120.0) -> list[Candidate]:
    if bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.dtype != np.uint8:
        raise ValueError("需要 H×W×3 uint8 BGR 图像")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, np.array([0, 0, 160], np.uint8),
                        np.array([179, 70, 255], np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    clean = cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel)
    contours, hierarchy = cv2.findContours(clean, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        return []
    candidates = []
    for i, contour in enumerate(contours):
        # 只取拥有内部孔洞的外轮廓；避免把一个环检测成两个桶。
        if hierarchy[0, i, 3] != -1 or hierarchy[0, i, 2] == -1:
            continue
        area, perimeter = cv2.contourArea(contour), cv2.arcLength(contour, True)
        if area < min_area or perimeter <= 0 or len(contour) < 5:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if x <= 0 or y <= 0 or x+w >= bgr.shape[1] or y+h >= bgr.shape[0]:
            continue
        (u, v), axes, _ = cv2.fitEllipse(contour)
        circularity = 4*np.pi*area/(perimeter*perimeter)
        axis_ratio = min(axes)/max(axes)
        if circularity < 0.55 or axis_ratio < 0.55:
            continue
        filled = np.zeros(clean.shape, dtype=np.uint8)
        cv2.drawContours(filled, [contour], -1, 255, cv2.FILLED)
        rim_mask = (filled > 0) & (clean > 0)
        candidates.append(Candidate(float(u), float(v), float(max(axes)),
                                    float(circularity), float(axis_ratio), rim_mask))
    return sorted(candidates, key=lambda x: x.u)


def blur_score(bgr: np.ndarray) -> float:
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())
```

## 25.3 完整数据生成器与定位入口

**完整文件：`tools/make_scene.py`。**

```python
"""生成确定性的教学 RGB-D 测试数据；不是训练数据/真实相机噪声模型。"""
import argparse
import json
from dataclasses import asdict
from pathlib import Path
import cv2
import numpy as np
from cuadc_vision_lab.geometry import Intrinsics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='outputs/scene')
    args = parser.parse_args()
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    k = Intrinsics(960, 540, 800., 800., 479.5, 269.5)
    bgr = np.full((k.height, k.width, 3), [50, 70, 50], dtype=np.uint8)
    depth = np.full((k.height, k.width), 2.30, dtype=np.float64)
    gt = []
    yy, xx = np.indices(depth.shape)
    for (u, v), diameter in zip([(180, 210), (440, 330), (740, 220)], [.15, .20, .25]):
        radius = int(round(diameter*k.fx/2./2.))
        radial = np.sqrt((xx-u)**2+(yy-v)**2)
        rim = (radial <= radius) & (radial >= radius-6)
        inside = radial < radius-6
        bgr[rim] = [235, 235, 235]
        bgr[inside] = [160, 60, 30]
        depth[rim] = 2.0
        gt.append({'uv': [u, v], 'diameter_m': diameter, 'rim_depth_m': 2.0})
    # 干扰：实心白矩形；基线应借助层级与形状将其拒绝。
    bgr[420:465, 110:210] = 235
    rng = np.random.default_rng(26)
    depth += rng.normal(0., .002, depth.shape)
    depth[20:40, :] = 0.0
    if not cv2.imwrite(str(out/'color.png'), bgr):
        raise RuntimeError('图片写入失败')
    np.save(out/'depth_m.npy', depth)
    (out/'intrinsics.json').write_text(json.dumps(asdict(k), indent=2), encoding='utf-8')
    (out/'ground_truth.json').write_text(json.dumps(gt, indent=2), encoding='utf-8')
    print(out.resolve())


if __name__ == '__main__':
    main()
```

**完整文件：`tools/classical_detect.py`。**

```python
"""白色环检测 + 环上深度 + 针孔投影，适用于本章的正视平面合成数据。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from cuadc_vision_lab.geometry import Intrinsics, deproject
from cuadc_vision_lab.depth import robust_depth
from cuadc_vision_lab.detection import detect_white_rings


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scene', default='outputs/scene')
    args = p.parse_args()
    root = Path(args.scene)
    image = cv2.imread(str(root/'color.png'))
    if image is None:
        raise FileNotFoundError(root/'color.png')
    depth = np.load(root/'depth_m.npy', allow_pickle=False)
    k = Intrinsics(**json.loads((root/'intrinsics.json').read_text()))
    if depth.shape != image.shape[:2] or depth.shape != (k.height, k.width):
        raise ValueError('分辨率/内参不匹配')
    results = []
    for c in detect_white_rings(image):
        try:
            z = robust_depth(depth, c.rim_mask)
        except ValueError as exc:
            print(f'rejected: {exc}'); continue
        xyz = deproject([c.u, c.v], z.z_m, k)
        diameter = c.diameter_px*z.z_m/k.fx
        results.append({'frame': 'camera_optical', 'uv': [c.u, c.v],
                        'xyz_m': xyz.tolist(), 'diameter_m': diameter,
                        'depth_mad_m': z.mad_m, 'geometric_score': c.circularity})
        cv2.circle(image, (round(c.u), round(c.v)), round(c.diameter_px/2), (0, 255, 0), 2)
        cv2.putText(image, f'{diameter:.3f}m', (round(c.u)-40, round(c.v)-35),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 0), 1)
    (root/'detections.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    if not cv2.imwrite(str(root/'detected.png'), image):
        raise RuntimeError('写图失败')
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
```

## 25.4 怎么做更有价值的实验

依次降低白环亮度、加入阴影、挖掉一段桶沿、让两个环靠近、改变深度比例、交换内参。每次只改一项，记录哪些检测被拒绝、哪些错误仍悄悄通过。

预期现象应包括：无孔洞的矩形被拒绝；全零深度不能输出三维目标；分辨率不匹配应报错；深度单位错一千倍应被合理范围或直径门限拒绝。

<a id="ch-26"></a>

# 26. 深度学习入门：模型究竟在学习什么

## 26.1 检测、分类、实例分割和关键点

| 任务 | 典型输出 | 对 CUADC 的价值 |
|---|---|---|
| 图像分类 | 整张/裁剪图类别 | 判断一个筒内是什么标识 |
| 目标检测 | 框、类别、分数 | 快速找到桶和标识区域 |
| 实例分割 | 每个实体的掩膜/多边形 | 分离不同桶，辅助桶沿和可见区域 |
| 关键点 | 指定语义点坐标 | 可定义桶口点，但需要一致标注 |
| 语义分割 | 每像素类别 | 区分区域，但不天然给每个桶身份 |

Ultralytics 的 detect/segment API 提供相应任务接口；本文使用的是接口范式，不认定所选模型属于哪个版本。[R22][R23][R24]

## 26.2 从输入张量到损失

模型接收经过约定处理的图像张量，输出位置和类别等预测。训练用标注构造损失，通过梯度优化参数。你需要理解输入尺寸、批量、训练/验证区别、学习率、过拟合和模型权重，而不是先背全部网络层名。

一个模型能在训练图上画好框，只说明它拟合了这些样本；能否在另一块草地、另一种光照和实际机架遮挡下工作，需要独立数据检验。

## 26.3 单类桶还是三类大小桶

可以训练统一 `bucket_opening` 类，再通过几何估计连续直径；也可以按尺寸标成多个类别。但纯像素大小会同时受高度影响，模型可能错误把“近处的桶”当“大桶”。

本教程采用“统一检测 + 度量几何”的路线：模型找候选，几何估计连续直径，任务层再决定是否需要尺寸类别。这能把类别错误与尺度测量错误分开分析。

## 26.4 模型名称不是性能结论

更大的模型、更新的版本、更高的输入分辨率都可能增加计算成本。应在相同测试集、相同硬件、相同端到端指标下比较。不要把官网其他设备上的 FPS 当作当前运行设备 的实测速度。

<a id="ch-27"></a>

# 27. 数据集设计：怎样避免“训练很好，室外不行”

## 27.1 按任务工况采集，而不是按图片数量凑数

覆盖搜索高度、粗对准高度、细对准高度；中央与边缘；晴天、阴影、不同背景；水平与倾斜；满载水瓶遮挡和投后状态；完整桶、半桶与出视野；正常和失效深度。

数据量只是一个维度。大量相邻视频帧可能几乎没有新增场景信息。

## 27.2 按架次/session 划分

同一个视频的相邻帧不能随机分散到 train、val、test，否则测试集可能含几乎相同的画面。优先按采集架次、日期、场地或相机配置分组，整组分配。[R53]

建议 manifest：

```csv
image,label,split,session
images/train/flight01_0001.png,labels/train/flight01_0001.txt,train,flight01
images/val/flight02_0001.png,labels/val/flight02_0001.txt,val,flight02
images/test/flight03_0001.png,labels/test/flight03_0001.txt,test,flight03
```

这里是结构示例，不提供虚构的真实比赛图片。

## 27.3 难负样本必须覆盖

白袋、白纸、机架、水瓶、蓝色圆环、地面标记、其他场地的容器，都可能成为负样本。模型需要学会“不输出桶”，而不是所有白圆都给高分。

负样本图中没有目标时，可采用相应数据格式允许的空标签文件。数据审计器应区分“有意的负样本”与“漏了标签”。[R23][R24]

## 27.4 标识数据不要只用网络上的完美图标

侦察实际图像包含缩小、斜视、筒壁遮挡、压缩和曝光。训练图应包括最终显示链路中的真实外观。增强能扩展工况，但不能替代真实采集。

标识类别清单应来自明确的任务定义与标注手册。练习可以使用自选符号，实际比赛则应核对允许类别与填写方式，不能凭通用 GHS 知识自行替换。

<a id="ch-28"></a>

# 28. 标注规范：桶口、桶身和可见区域必须统一

## 28.1 先写标注手册

明确框住整个桶身，还是只框桶口；分割是可见白色环，还是填满整个开口区域；遮挡目标是否保留；图像边缘截断怎样标；严重模糊是否排除。

同一个数据集中混用“桶身”和“桶口”标注，会让模型中心和尺寸含义漂移。对投放测量而言，语义不一致比少几张图片更难修复。

## 28.2 检测标签格式

常见 YOLO 检测一行是：

```text
class_id center_x center_y width height
```

坐标相对图像宽高归一化到 `[0,1]`。类别编号是从 0 开始的数据集索引，不是状态机的桶 Track ID。[R23]

## 28.3 实例分割多边形格式

常见格式为：

```text
class_id x1 y1 x2 y2 x3 y3 ...
```

这是多边形点对，不是一个 bbox 后面再接点。至少需要足够的点构成有效区域。[R24]

## 28.4 多边形与空心环的拓扑问题

单条多边形一般不能自动表达“外环减内孔”的全部语义。导出工具可能把孔洞填满，也可能使用连接路径近似。`masks.xy` 的多边形表示不应被默认当作准确的桶沿测深掩膜。[R18][R24]

若模型 mask 标的是整个开口，可从另外的边缘/颜色/深度证据构造桶沿候选；若 mask 标的是白色环，必须检查训练格式是否保留了孔洞。绝不能在填满的桶口 mask 中随便取深度，再声称测的是桶沿。

**练习 7**：将一个含内孔的二值 mask 导出再导入，比较像素差异和孔洞是否保留。把结果写进数据格式规范。

<a id="ch-29"></a>

# 29. 实践：写一个数据集审计器

## 29.1 审计哪些问题

图片/标签缺失、非数字、NaN、类别越界、归一化坐标越界、零面积、多边形点数错误、bbox 越出图像、精确重复图片、同 session 跨数据集泄漏。

审计器不应自动“修正”它不理解的标签，例如把所有越界值裁到 0～1 后假装正常；这可能掩盖坐标系统错误。

**完整文件：`tools/dataset_audit.py`。**

```python
"""审计 YOLO 标签、精确重复图片和 session 泄漏，不自动修改标签。"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np


def validate_label(text: str, task: str, classes: int):
    if task not in {'detect', 'segment'} or classes < 1:
        raise ValueError('task/classes invalid')
    errors = []
    for lineno, line in enumerate(text.splitlines(), 1):
        if not line.strip():
            continue
        try:
            values = np.array([float(x) for x in line.split()])
        except ValueError:
            errors.append(f'{lineno}: not numeric'); continue
        if not np.isfinite(values).all() or len(values) < 1:
            errors.append(f'{lineno}: nonfinite/empty'); continue
        cls, coords = values[0], values[1:]
        if cls != int(cls) or not 0 <= cls < classes:
            errors.append(f'{lineno}: class out of range')
        if (coords < 0).any() or (coords > 1).any():
            errors.append(f'{lineno}: coordinates outside [0,1]')
        if task == 'detect':
            if len(coords) != 4:
                errors.append(f'{lineno}: detection requires cx cy w h'); continue
            cx, cy, w, h = coords
            if min(w, h) <= 0 or cx-w/2 < -1e-6 or cy-h/2 < -1e-6 or cx+w/2 > 1+1e-6 or cy+h/2 > 1+1e-6:
                errors.append(f'{lineno}: invalid/overflow box')
        else:
            if len(coords) < 6 or len(coords) % 2:
                errors.append(f'{lineno}: polygon needs >=3 point pairs'); continue
            poly = coords.reshape(-1, 2)
            area = abs(np.sum(poly[:, 0]*np.roll(poly[:, 1], -1)-np.roll(poly[:, 0], -1)*poly[:, 1]))/2
            if area <= 1e-8:
                errors.append(f'{lineno}: zero-area polygon')
    return errors


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--manifest', required=True, help='CSV: image,label,split,session')
    p.add_argument('--root', required=True)
    p.add_argument('--task', choices=['detect', 'segment'], default='segment')
    p.add_argument('--classes', type=int, default=1)
    args = p.parse_args()
    root = Path(args.root).resolve()
    errors, hashes, sessions = [], {}, {}
    with open(args.manifest, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError('empty manifest')
    for i, row in enumerate(rows, 2):
        if not {'image', 'label', 'split', 'session'} <= row.keys():
            raise ValueError('CSV fields must be image,label,split,session')
        if row['split'] not in {'train', 'val', 'test'} or not row['session']:
            errors.append(f'row {i}: missing session/invalid split')
        sessions.setdefault(row['session'], set()).add(row['split'])
        image, label = (root/row['image']).resolve(), (root/row['label']).resolve()
        if not image.is_relative_to(root) or not label.is_relative_to(root):
            errors.append(f'row {i}: path outside dataset'); continue
        if not image.is_file() or not label.is_file():
            errors.append(f'row {i}: missing image/label (empty negative label is allowed)'); continue
        pixels = cv2.imread(str(image))
        if pixels is None:
            errors.append(f'row {i}: unreadable image'); continue
        digest = hashlib.sha256(str(pixels.shape).encode()+pixels.tobytes()).hexdigest()
        if digest in hashes:
            errors.append(f'row {i}: duplicate image pixels, previous={hashes[digest]}')
        hashes[digest] = row['image']
        for e in validate_label(label.read_text(encoding='utf-8'), args.task, args.classes):
            errors.append(f"{row['label']}:{e}")
    for session, splits in sessions.items():
        if len(splits) > 1:
            errors.append(f'session leakage: {session} -> {sorted(splits)}')
    print(json.dumps({'images': len(rows), 'errors': errors}, ensure_ascii=False, indent=2))
    raise SystemExit(1 if errors else 0)


if __name__ == '__main__':
    main()
```

运行：

```bash
python -m tools.dataset_audit \
  --manifest datasets/buckets/manifest.csv \
  --root datasets/buckets --task segment --classes 1
```

## 29.2 它仍然不会发现什么

几乎重复但不完全相同的图片、语义标错、漏标真桶、多边形自交、边界偏差、隐蔽场地泄漏等仍需人工和更高级检查。测试代码通过不代表标签质量已经合格。

输出非零退出码意味着审计失败，适合接入持续集成。不要在训练脚本里忽略错误然后继续跑几小时。

<a id="ch-30"></a>

# 30. 实践：训练一份自己的分割基线

## 30.1 训练前准备

你需要自己的数据、通过审计的标签、可信本地基础分割权重和独立训练环境。训练代码使用学习者提供的权重路径，结果应通过独立测试集测量。

数据 YAML 示例：

```yaml
path: /absolute/path/to/datasets/buckets
train: images/train
val: images/val
test: images/test
names:
  0: bucket_opening
```

类别名称应与标注手册一致。绝对路径只在本机有效；发布时记录数据根目录约定和相对结构。[R24][R25]

## 30.2 一个可读的训练入口

下面代码采用常见 Ultralytics 训练接口，参数是起点。对不同版本支持项、模型任务和设备，应查固定版本官方说明，不在比赛部署环境盲目升级。[R25][R36]

**完整文件：`tools/train_segment.py`。**

```python
"""可选训练入口；需要自己的数据集和可信本地分割权重。"""
import argparse
import json
from pathlib import Path
import platform


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--weights', required=True)
    p.add_argument('--data', required=True)
    p.add_argument('--device', default='cpu')
    p.add_argument('--epochs', type=int, default=100)
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--batch', type=int, default=8)
    p.add_argument('--project', default='runs/cuadc')
    p.add_argument('--name', default='segment_baseline')
    args = p.parse_args()
    for name in ['weights', 'data']:
        if not Path(getattr(args, name)).is_file():
            raise FileNotFoundError(getattr(args, name))
    if min(args.epochs, args.imgsz, args.batch) <= 0:
        p.error('epochs/imgsz/batch must be positive')
    import ultralytics
    import torch
    from ultralytics import YOLO
    record = {'python': platform.python_version(), 'torch': torch.__version__,
              'ultralytics': ultralytics.__version__, 'arguments': vars(args),
              'note': 'Training parameters are starting points, not an accuracy guarantee.'}
    project = Path(args.project); project.mkdir(parents=True, exist_ok=True)
    (project/(args.name+'_environment.json')).write_text(json.dumps(record, indent=2), encoding='utf-8')
    model = YOLO(args.weights, task='segment')
    model.train(data=args.data, epochs=args.epochs, imgsz=args.imgsz,
                batch=args.batch, device=args.device, workers=2,
                project=args.project, name=args.name, seed=26,
                degrees=180., fliplr=0.5, flipud=0.5,
                mosaic=0.5, close_mosaic=10)


if __name__ == '__main__':
    main()
```

```bash
python -m tools.train_segment \
  --weights models/local_base_seg.pt \
  --data datasets/buckets/data.yaml \
  --device 0 --epochs 100 --imgsz 640 --batch 8
```

没有 GPU 时用 `--device cpu`，但训练时长可能明显增加。`--device 0` 不会自动替你配置 CUDA。

## 30.3 必须保存的产物

训练命令、库版本、数据版本、超参数、训练/验证曲线、best/last 权重、测试结果、模型哈希和失败样例。文件名叫 `best.pt` 只表示训练器选择了某个验证指标最好的权重，不表示部署一定最好。

不加载来源不明的权重文件；模型文件可能触发反序列化行为，应该从可信来源获得并核验。[工程要求]

<a id="ch-31"></a>

# 31. 超参数和增强：不要只靠“加 epochs”

## 31.1 每个常用参数改变什么

| 参数 | 影响 | 常见误用 |
|---|---|---|
| `imgsz` | 小目标采样密度、显存与时延 | 只提高尺寸，不测实时性 |
| `batch` | 显存、优化和吞吐 | 显存不足还强行增大 |
| `epochs` | 优化轮数 | 数据有错时越训越固化 |
| 学习率 | 更新幅度 | 凭损失波动随便调大 |
| `workers` | 数据加载并发 | 在资源有限机子开过多进程 |
| `conf/iou` | 推理筛选与抑制 | 把推理门限当训练质量改进 |

## 31.2 旋转、缩放、亮度与遮挡

桶无固定图像朝向时，旋转增强有价值；侦察标识的旋转范围应匹配真实可出现工况。亮度和色彩增强用于扩大曝光条件，不能把不现实颜色增强当鲁棒性。

Mosaic、MixUp、Copy-Paste 等会改变目标上下文与遮挡关系，应单独做消融。对桶沿的几何精度，要检查增强后边界是否仍具有物理合理性。[R36]

## 31.3 不要同时乱改五项

每次实验记录一个假设，例如“边缘目标漏检来自输入分辨率不足”，只改主要变量，再比较同一测试集上的召回、中心误差和延迟。没有可比基线，就无法知道提升来自哪里。

## 31.4 训练可复现的边界

固定随机种子能改善复现，但不保证跨 GPU、驱动、框架和算法配置逐位一致。记录环境，并把关键部署测试作为最终证据。[R52]

<a id="ch-32"></a>

# 32. 评价模型：mAP 之外还要看投放任务指标

## 32.1 基础检测指标

Precision 反映输出中多少正确，Recall 反映真目标中多少被找到；IoU 比较区域重叠，AP/mAP 综合不同阈值表现。分割任务还需要区分 box 指标与 mask 指标。[R26]

不能只报一个 mAP 数字。小桶、边缘、低空遮挡、强光、动态画面等子集应分别统计。

## 32.2 控制最关心什么

| 指标 | 推荐解释方式 |
|---|---|
| 桶口中心像素误差 | 与人工/测量真值比较 |
| 三维 XY 偏差与 RMSE | 分离固定偏差和随机噪声 |
| XY P95/P99 | 关注尾部错误，不只看平均 |
| 直径误差 | 是否容易混淆 15/20/25 cm |
| 假稳定目标率 | 误检是否会跨多帧通过门禁 |
| 第三桶发现时间 | 是否拖慢完整任务 |
| 错误重关联次数 | 是否换错已锁定目标 |
| 端到端观测年龄 | 状态机收到的观测有多旧 |

## 32.3 为什么较高 IoU 仍可能不适合投放

一个桶框整体偏一点仍可能有不错 IoU，但投放需要的圆心可能已经偏出目标精度范围。反过来，分割轮廓局部不完美但中心稳定，可能更适合定位任务。

模型选择应以真实任务损失为目标：漏一次需要多久重搜索？错锁一个桶后果是什么？错误数据通过门禁的代价通常远高于单帧漏检。

## 32.4 测试集不能无限调参

验证集用于开发，保留独立测试集只做阶段验收。反复看测试集失败样例并优化后，它实质上已经变成验证集，需要新的独立检验。

<a id="ch-33"></a>

# 33. 推理输出、letterbox 与坐标还原

## 33.1 用高层 API 先建立正确基线

Ultralytics 的结果对象包含框、分数、类别和 mask 等属性；这些属性属于其固定版本接口，不应与自行导出的原始张量布局混为一谈。[R18]

```text
result.boxes.xyxy：通常已还原到原图像素尺度
result.boxes.cls：数据集类别索引
result.boxes.conf：模型分数
result.masks.xy：原图尺度多边形表示
result.masks.data：分辨率可能与原图不同的掩膜张量
```

应实际打印 shape 和一个已知目标的位置确认，不靠变量名猜。

## 33.2 完整本地推理示例

**完整文件：`tools/yolo_predict.py`。**

```python
"""可选 Ultralytics 实例分割入口。只处理本地图像和本地权重，不自动下载。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--weights', required=True)
    p.add_argument('--image', required=True)
    p.add_argument('--out', default='outputs/yolo.json')
    p.add_argument('--device', default='cpu')
    p.add_argument('--conf', type=float, default=.35)
    args = p.parse_args()
    if not Path(args.weights).is_file():
        raise FileNotFoundError('请提供自己的本地分割模型；此示例不下载或训练模型')
    image = cv2.imread(args.image)
    if image is None:
        raise FileNotFoundError(args.image)
    from ultralytics import YOLO
    model = YOLO(args.weights, task='segment')
    result = model.predict(image, device=args.device, conf=args.conf, verbose=False)[0]
    rows = []
    if result.boxes is not None and len(result.boxes):
        if result.masks is None or len(result.masks.xy) != len(result.boxes):
            raise RuntimeError('非分割模型或框/掩膜数量不匹配')
        for box, polygon in zip(result.boxes, result.masks.xy):
            poly = np.asarray(polygon, dtype=float)
            if poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3 or not np.isfinite(poly).all():
                continue
            rows.append({'class_id': int(box.cls.item()), 'score': float(box.conf.item()),
                         'xyxy': box.xyxy[0].cpu().numpy().tolist(),
                         'polygon_original_pixels': poly.tolist()})
    out = Path(args.out); out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({'source_shape': list(image.shape), 'objects': rows}, indent=2), encoding='utf-8')
    print(out)


if __name__ == '__main__':
    main()
```

```bash
python -m tools.yolo_predict \
  --weights models/your_bucket_seg.pt \
  --image test_data/your_frame.png \
  --device cpu --out outputs/yolo.json
```

本例没有深度定位和 ROS 发布；它只是把模型输出变成可审计 JSON。没有真实权重时不运行，不把合成环算法冒充 YOLO。

## 33.3 导出后不要硬编码所有 YOLO 的张量格式

不同任务、模型代际、导出选项和是否带 NMS 都可能改变输出形状。应检查实际 ONNX 输入输出元数据，并使用固定模型的后处理契约。[R37]

`CHW/HWC`、RGB/BGR、浮点归一化、letterbox、置信度计算、mask 原型重建、NMS 及坐标还原任何一步出错，都可能得到“有框但位置错误”的结果。

## 33.4 原图坐标的金标准实验

准备一张目标处于已知像素位置的测试图，依次通过不同输入尺寸、长宽比、ROI 与导出后端，比较还原坐标是否一致。调试时同时保存网络输入图和最终原图叠加图。

<a id="ch-34"></a>

# 34. 从实例 mask 到可用于几何的测量区域

## 34.1 检测 mask 与测深 mask 可以不同

模型 mask 的任务是分辨实体；测深 mask 的任务是选中同一个物理表面。它们不一定相同。

例如模型输出整个桶口填充区域，测深则希望取桶沿。可以在轮廓附近构造窄带，但须验证该带位于实际桶沿，而不是背景或桶底。

## 34.2 腐蚀不是万能正确操作

对实心目标，腐蚀有时能远离边缘混合深度；对空心桶沿，过度腐蚀可能把真正需要的窄环全部去掉。形态学核大小应与像素尺度和物理目标匹配。[R19]

必须可视化最终测深像素集合，不能只看模型彩色 mask。

## 34.3 像素质量传播到三维质量

建议为每个实体保存：原 mask 面积、边界截断比例、可用深度比例、平面内点比例、轮廓覆盖角、三维拟合残差、估计中心与尺寸。

一个总分可以用于排序，但不能丢掉这些原因字段，否则失败时无法区分模型没看见、深度没测到、坐标没算对。

## 34.4 模型分数不是定位方差

`conf=0.9` 不能直接转换成“位置标准差 1 cm”。定位误差来自像素、深度、标定、时间与姿态，必须独立估计或通过数据标定。

<a id="ch-35"></a>

# 35. 把检测与几何组成一个可拒绝的观测流水线

## 35.1 一条推荐的接口链

```text
FrameBundle（图像、深度、内参、时戳、版本）
 → Candidate2D（框/轮廓、分数、来源）
 → MeasurementRegion（明确的物理表面）
 → Estimate3D（相机坐标、尺寸、质量）
 → BodyObservation（FLU、时间、质量）
 → LegacyAdapter 或自定义 ROS 消息
```

每一层都可以拒绝输入。拒绝原因不能都压成一句“没识别到”。

## 35.2 典型拒绝原因

```text
NO_COLOR_FRAME
DEPTH_NOT_REGISTERED
CALIBRATION_MISMATCH
MASK_TRUNCATED
NO_VALID_DEPTH
MIXED_SURFACES
PLANE_DEGENERATE
DIAMETER_UNCERTAIN
CLOCK_DOMAIN_MISMATCH
STALE_OBSERVATION
```

先为每一类拒绝原因编写可触发的输入，再确定诊断字段和计数方法。

## 35.3 不确定观测如何交给控制层

若使用新接口，可以保留检测但标记 `position_valid=false`。若必须兼容旧 `PoseArray`，无效三维目标不应放入数组；另外发诊断信息，避免状态机把坏坐标当真。

正常有图但无目标，仍可以发带原始时戳的空数组；相机失联、推理异常或重复旧帧，不应通过不断造新时间戳的空数组掩盖故障。

## 35.4 组合实验

用同一录制序列分别替换检测器、深度算法、外参和时序策略，其他模块保持不变。这样才能知道某次“更准”来自哪个环节，而不是把所有改动混在一次实飞里。


---

<a id="ch-36"></a>

# 36. 实时视觉程序：先设计数据流，再讨论多线程

## 36.1 最小可解释流水线

推荐先画出以下边界，再选择线程或进程。保持阶段输入输出可记录、可替换，便于逐层定位错误。

```text
采集：取得 frame + 原始时间信息 + 当前相机 profile
  ↓
输入验证：帧号、分辨率、编码、内参版本、时间域
  ↓
推理：输出当前帧的检测/实例掩膜
  ↓
几何：只在当前帧对应的深度与标定上计算
  ↓
质量门：可见性、深度、轮廓、三维拟合、帧龄
  ↓
观测消息：保持原始观测时刻；发布有效目标或健康空帧
  ↓
独立诊断：处理时间、拒绝原因、丢帧计数、相机状态
```

不能把“最近颜色帧”“最近深度帧”“最近检测结果”三个独立全局变量随意拼起来。即使每一个都是最新，也不代表它们来自同一时刻、同一图像坐标或同一标定版本。

## 36.2 一帧必须是一个不可拆散的数据包

```python
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class FramePacket:
    sequence: int
    observation_ns: int
    arrival_monotonic_ns: int
    calibration_id: str
    color_bgr: np.ndarray
    depth_m: np.ndarray
```

`frozen=True` 只禁止重新给字段赋值，并不自动让数组内容只读。跨线程传递之前应明确所有权；必要时使用 `.copy()` 和 `array.setflags(write=False)`。相机 SDK 的缓冲区生命周期也要检查，不能把已经被下一帧覆盖的内存交给推理线程。[R12]

最重要的不变量是：一次推理产生的 mask，只能用于这一个 `FramePacket` 的图像坐标和深度。完成推理之后，不能为了“更新位置”拿另一个包的深度重新拼装。

## 36.3 实时控制偏向处理新帧，不是补完所有旧帧

假设输入 30 fps、单帧推理 80 ms，则最大推理吞吐约 12.5 fps。若不丢弃排队帧，队列会持续增长，最终发布的是几秒前的世界。

这不是设置大队列就能解决的问题。通常需要降低输入处理频率、换更合适的模型/后端，或采用“最新帧槽位”。槽位满时替换尚未开始推理的旧帧，已经开始推理的帧仍要保持其完整数据包。

可使用 `queue.Queue(maxsize=1)` 实现最小有界队列，`put_nowait()`、`get_nowait()` 与 `Full`/`Empty` 显式处理拥塞。[R47] 若采用多个生产者或多个消费者，不要把“先检查 `empty()` 再 `get()`”当成原子操作；检查和取出之间状态可能改变。

**禁止行为**：为了维持 30 Hz 发布频率，重复发送同一帧结果并刷新 `header.stamp`。这会把旧证据伪装成新证据，并会让多帧确认计数失去意义。

## 36.4 三种部署形式

| 形式 | 适合什么阶段 | 要额外处理什么 |
|---|---|---|
| 单线程顺序处理 | 第一版离线和低负载实验 | 最容易解释，但处理慢时阻塞采集 |
| 采集线程 + 推理线程 | 需要保持新鲜度的相机节点 | 帧所有权、队列上限、退出与异常传播 |
| 采集/推理/记录独立进程 | 大数据记录或重模型部署 | IPC 延迟、共享内存、故障监控、版本契约 |

线程数量不是越多越好。OpenCV、推理后端、BLAS 本身也可能拥有线程池；多个节点争抢 CPU 会使 odom 回调延迟增大。在共享处理器上，视觉速度优化还应监测导航输入的回调延迟与稳定性。

<a id="ch-37"></a>

# 37. 让模型更快：优化顺序与测量方法

## 37.1 测量五段，不只看模型报告的 inference

记录 `capture→preprocess→inference→postprocess→geometry→publish` 各阶段耗时，以及消息发布时的总帧龄。另记输入帧率、真正处理帧率、有效三维观测帧率和最长连续中断。

用 `time.perf_counter_ns()` 测量同一进程的耗时；ROS `now()` 用于与消息时钟域比较，二者不能直接相减。[R28]

GPU 运算可能异步执行，CPU 函数返回不一定表示 GPU 已完成。基准测量应使用同步或 CUDA event；正常任务循环中不应为了每帧打印一个数而随意加入全局同步，造成新的阻塞。[R44]

## 37.2 优先级顺序

先去掉重复打开相机、重复初始化模型、逐帧重新加载标定和同步写大图等明显开销；再检查输入尺寸、算法区域和后处理；最后比较后端与精度模式。

| 优化 | 可能收益 | 必须复测的代价 |
|---|---|---|
| 降低网络输入尺寸 | 推理和显存下降 | 小桶沿、小图标细节丢失 |
| ROI 推理 | 无关背景减少 | 目标可能从 ROI 外进入 |
| 更小模型 | 延迟下降 | 逆光、小目标、遮挡召回变化 |
| FP16 / INT8 | 某些硬件上吞吐提高 | 数值、置信度与边界质量变化 |
| 导出 ONNX / OpenVINO | 部署与硬件适配 | 输出格式、NMS、动态形状、算子支持 |
| 减少记录量 | 降低写盘争用 | 排查故障时证据可能不足 |

ONNX 是模型交换格式，不是“自动更快”的保证。ONNX Runtime 的执行提供程序决定在哪种硬件执行；OpenVINO 也需要按实际目标设备测量。导出成功不等于掩膜坐标和数值与原模型一致。[R35][R37][R45]

## 37.3 导出一致性验收

固定一套包含正面、侧面、截断、逆光和无目标的图像，分别用原模型和导出模型处理。比较目标数量、类别、框坐标、掩膜重叠、几何中心、直径与拒绝原因。

不要只比较前十张“漂亮样例”。例如一个 INT8 模型平均 mAP 变化很小，却可能把白桶沿吞掉几个像素，引入厘米级几何偏差。

原生 PyTorch 推理通常需要 `model.eval()` 配合合适的推理上下文；`torch.inference_mode()` 关闭部分自动求导开销，但不会自动替代 `eval()` 的模型模式切换。使用封装推理库时，先查它是否已经处理这些步骤。[R43]

## 37.4 尾延迟与热稳定

至少记录中位数、P95、P99、最大处理时间。一次冷机运行 30 秒的结果不能证明装机后连续运行没有降频、USB 重连或内存增长。

教学验收可以设为“连续 10 分钟无无界队列增长，所有异常都有计数”，但这只是自己定义的工程测试，不是相机或算法的官方性能保证。

<a id="ch-38"></a>

# 38. 视觉系统至少有四种时间

## 38.1 时间字段表

| 时间 | 含义 | 典型来源 |
|---|---|---|
| 曝光/采样时刻 | 光线实际对应的物理时刻 | 相机硬件时间戳及其定义 |
| 主机收到帧时刻 | USB/驱动把帧交给程序的时刻 | 相机回调或 `wait_for_frames()` 返回后 |
| 推理完成时刻 | 算法得出结果的时刻 | 主机运行时钟 |
| 消息到达控制器时刻 | 下游处理此观测的时刻 | ROS 回调入口 |

帧 metadata 中的 sensor timestamp、backend timestamp、frame counter 和 timestamp domain 等信息，应逐项查设备支持情况，不能只因为字段叫 timestamp 就把它视为曝光中点。[R11]

例如，一帧在 `10.000 s` 曝光、`10.035 s` 到主机、`10.105 s` 推理完成、`10.112 s` 到状态机。准确表达这帧观测，需要保留原观测时刻及映射关系，而不是在发布时把时间改成 `10.112 s`。

## 38.2 同一时钟域不等于同一物理时刻

两个程序都调用主机 `now()`，可以消除“一个是开机秒数、另一个是 Unix 时间”这种明显错误；但 USB 传输、飞控通信、处理队列各自的延迟仍然存在。

若相机接收时间比曝光晚 35 ms，odom 接收时间比其代表的状态晚 15 ms，把两者都按接收时间对齐，仍可能有约 20 ms 的相对偏差。恒速 1.4 m/s 时，对应约 2.8 cm 的平移误差。这是说明机制的计算例子，不是设备的实测延迟。

## 38.3 时间映射的工程方法

硬件时钟与主机时钟可用 `t_host≈a*t_sensor+b` 建模，`a` 表示频率比例，`b` 表示偏置。拟合时需要处理主机接收抖动、时钟回跳和设备重启，并记录映射残差。

不能用一次 `host_now-sensor_now` 就永久锁定偏置，也不能在大幅时钟回跳后沿用旧目标坐标。更完整的同步需要确定相机、IMU、飞控状态各字段真正代表哪个时刻。

**建议实验**：让相机对准一个已知静止目标，做可控的小幅往复位移，离线扫描时间偏置，观察变换到世界后的目标散布最小时对应的偏置。该方法会与外参、姿态误差耦合，不能把最小残差当作硬件同步认证。

<a id="ch-39"></a>

# 39. 把图像与导航测量对齐到同一时刻

## 39.1 测量时间与接收时间分别保存

设图像曝光时刻为 `t_c`，导航位置样本来自里程计，姿态样本来自 IMU。图像中点的世界坐标应使用 `t_c` 时刻的位置和姿态，而不是推理结束时最新的一条状态。

```text
位置历史： p_before -------- t_c -------- p_after
姿态历史： q_before ---- t_c ---- q_after
                         ↓
                分别插值到同一测量时刻
                         ↓
                  相机 → 机体 → 世界
```

位置和姿态的采样频率、延迟可能不同，不必强行共用一条消息里的缓存姿态。为两条历史各检查时间单调、样本覆盖与最大间隙。若缺少后一个样本，可以有限等待；超过等待上限就丢弃该次观测，不用最近邻或无限外推掩盖缺口。

接收时刻用于链路活性和通信延迟，测量时刻用于几何对齐；两者应同时记录。所有测量时间必须已映射到同一时钟域，插值本身不会实现硬件同步。

## 39.2 插值公式

令视觉时间为 `t`，前后样本时刻分别为 `t0,t1`：

$$
\alpha=\frac{t-t_0}{t_1-t_0},\qquad
p(t)=(1-\alpha)p_0+\alpha p_1.
$$

角度插值不能直接对 `179°` 和 `−179°` 求算术平均得到 `0°`。下面用最短角差演示回绕处理；逐项欧拉角插值不等于完整三维姿态插值。实际通用实现可对单位四元数做 SLERP：先将内积为负的一端取反，再沿较短弧插值；接近重合时用归一化线性插值避免数值问题。

## 39.3 完整离线实践：时间历史

**完整文件：`cuadc_vision_lab/timing.py`。**

```python
"""有界时间历史练习：精确命中或合法夹逼；不做外推。"""
from dataclasses import dataclass
from bisect import bisect_left
import math
import numpy as np
from .geometry import finite_array


@dataclass(frozen=True)
class NavSample:
    stamp_ns: int
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float]


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class NavHistory:
    def __init__(self, max_gap_ns=50_000_000, history_ns=5_000_000_000):
        if max_gap_ns <= 0 or history_ns <= max_gap_ns:
            raise ValueError("历史范围/间隔错误")
        self.max_gap_ns = max_gap_ns
        self.history_ns = history_ns
        self.samples: list[NavSample] = []

    def append(self, sample: NavSample):
        finite_array(sample.xyz, (3,)); finite_array(sample.rpy, (3,))
        if not isinstance(sample.stamp_ns, int) or sample.stamp_ns <= 0:
            raise ValueError("测量时间必须为正整数纳秒")
        if self.samples and sample.stamp_ns <= self.samples[-1].stamp_ns:
            raise ValueError("非递增时戳：上层必须区分乱序与时钟重置")
        self.samples.append(sample)
        self.samples = [x for x in self.samples if sample.stamp_ns-x.stamp_ns <= self.history_ns]

    def at(self, stamp_ns: int):
        if stamp_ns <= 0 or not self.samples:
            return None
        i = bisect_left([s.stamp_ns for s in self.samples], stamp_ns)
        if i < len(self.samples) and self.samples[i].stamp_ns == stamp_ns:
            return self.samples[i]  # 精确命中不需要人为再找一对样本。
        if i == 0 or i == len(self.samples):
            return None
        before, after = self.samples[i-1], self.samples[i]
        gap = after.stamp_ns-before.stamp_ns
        if gap <= 0 or gap > self.max_gap_ns:
            return None
        fraction = (stamp_ns-before.stamp_ns)/gap
        xyz = np.array(before.xyz)+fraction*(np.array(after.xyz)-np.array(before.xyz))
        rpy = tuple(wrap(a+fraction*wrap(b-a)) for a, b in zip(before.rpy, after.rpy))
        return NavSample(stamp_ns, tuple(xyz), rpy)


def frame_age_ok(now_ns: int, stamp_ns: int, max_age_ns=400_000_000,
                 future_tolerance_ns=50_000_000) -> bool:
    return stamp_ns > 0 and -future_tolerance_ns <= now_ns-stamp_ns <= max_age_ns
```

**完整文件：`tools/timing_demo.py`。**

```python
import math
from cuadc_vision_lab.timing import NavSample, NavHistory


def main():
    history = NavHistory()
    epoch = 1_000_000_000
    for i in range(31):
        dt = round(i*1e9/30)
        history.append(NavSample(epoch+dt, (i/30*1.4, 0., 2.2), (0., 0., 0.)))
    observation = epoch+450_000_000
    nav = history.at(observation)
    print('capture-time X:', nav.xyz[0])
    print('120ms stale-pose position error at 1.4m/s:', 1.4*.12, 'm')
    print('no extrapolation:', history.at(epoch+1_050_000_000))
    gap = NavHistory()
    gap.append(NavSample(epoch, (0., 0., 0.), (0., 0., 0.)))
    gap.append(NavSample(epoch+80_000_000, (.1, 0., 0.), (0., 0., 0.)))
    print('80ms bracket rejected:', gap.at(epoch+40_000_000))


if __name__ == '__main__':
    main()
```

```bash
python -m tools.timing_demo
```

练习：将相邻 odom 间隔从 33 ms 改成 70 ms，程序应该拒绝；把查询时间移到最新样本之后，也应该拒绝。再测试 `179°→−179°`、重复时间戳、零时间戳和设备重启。

## 39.4 从曝光元数据理解时钟映射

以支持相应元数据的 RGB-D 相机为例，应先读取原始彩色与深度帧各自的时间信息，再做空间配准和推理。RealSense 定义了设备曝光中点、设备读出时间及映射到系统时钟的时间域；可用性需要逐帧检查。[R11][R55]

若已核实两个设备计数来自同一计数器、单位为微秒且按 32 位回绕，可以先求短差值，再将其加到已映射的帧时间上。下式只演示计数回绕，不能用于任意两个时钟的相减：

```python
def wrapped_delta_us(sensor_us, frame_us):
    return (int(sensor_us) - int(frame_us) + 2**31) % 2**32 - 2**31

assert wrapped_delta_us(12, 2**32 - 8) == 20
```

不得用 `now()` 静默替代缺失的曝光时间。确实只拿得到接收时间时，应标明时间源、估计延迟范围，并限定到允许这种误差的实验。RGB 与深度曝光偏差也应设独立门限，空间 `align` 不会消除偏差。

## 39.5 帧龄、心跳和历史各有边界

`capture_age = now - capture_stamp` 判断观测是否足够新；`arrival_age = monotonic_now - last_arrival` 判断链路是否仍活跃。刚收到一张旧图可能使第二项很小，第一项仍很大，两项必须分别检查。

主机系统时钟跳变可通过“系统时间增量减单调时间增量”发现。设备重启、时戳回退或时钟映射失效后，应清理待处理帧与插值历史并重新建立时间关系；不要把重启前后的数据拼接为连续轨迹。

练习：构造新到达但已过期的帧、重复帧号、时间回退和 RGB-D 差一帧；分别检查拒绝原因。过期观测不得靠修改时间戳进入新鲜度门。

<a id="ch-40"></a>

# 40. 从检测到跟踪：同一个桶为什么会有不同编号

## 40.1 三种编号不能混用

| 编号 | 例子 | 语义 |
|---|---|---|
| 模型类别 | class 0 = bucket | 某一类物体 |
| 跟踪 ID | track 7 | 连续观测中的一个实体假设 |
| 比赛桶号 | 1 号 = 15 cm 桶 | 规则定义的尺寸/目标语义 |

模型输出 `class_id=0`，不表示“零号桶”；Track 7 也不表示第七个实际桶。跟踪器维护实体假设，任务层再根据位置、尺寸和历史选择目标。

## 40.2 为什么在世界坐标里关联

飞机移动时，静止桶的 body 坐标会变化。把某一帧 `body.x=0.2` 与下一帧 `body.x=-0.2` 直接比较，可能误认为桶在移动。

时间对齐之后变换到 local，理想静止桶应保持位置不变。此时空间邻近才更适合做实体关联。前提仍是里程计、时间和外参没有严重系统误差。

## 40.3 先做门控，再做一对一关联

将本帧检测与历史轨迹组成候选配对。XY 距离和直径差都通过门限后，才计算代价，例如 `distance/position_gate + diameter_difference/diameter_gate`。按代价从小到大做贪心一对一匹配，未匹配检测建立新轨迹。

下面练习暂用 0.45 m 与 0.08 m 作为关联门，目的是观察算法行为。门限应由目标间距、定位误差和尺寸误差共同选择，不是通用默认参数。目标接近时，贪心结果可能次优，可比较全局分配方法。

一对一关联不等于帧内去重。两个重复轮廓仍可能创建两条轨迹，应在候选阶段处理重叠，并记录一帧内每条轨迹只更新一次。

## 40.4 完整离线实践：静态桶跟踪器

下面是独立的简化实现，演示空间/直径门、贪心一对一、平滑及三目标选二，不包含飞行和机构控制。

**完整文件：`cuadc_vision_lab/tracking.py`。**

```python
"""静态圆筒关联练习：观测、轨迹和任务选择分层。"""
from __future__ import annotations
from dataclasses import dataclass, field
from collections import deque
import numpy as np
from .geometry import finite_array


@dataclass(frozen=True)
class Observation:
    stamp_ns: int
    xyz: tuple[float, float, float]
    diameter_m: float
    score: float

    def __post_init__(self):
        finite_array(self.xyz, (3,))
        finite_array([self.diameter_m, self.score])
        if not 0.08 <= self.diameter_m <= 0.35 or not 0 < self.score <= 1:
            raise ValueError("观测尺寸/分数不合法；零分数不是默认满分")


@dataclass
class Track:
    id: int
    xyz: np.ndarray
    diameter_m: float
    score: float
    stamp_ns: int
    confirmations: int = 1
    position_deviation_m: float = 0.0
    diameter_samples: deque = field(default_factory=lambda: deque(maxlen=9))


class StaticBucketTracker:
    def __init__(self, position_gate_m=0.45, diameter_gate_m=0.08,
                 gap_reset_ns=600_000_000, memory_ns=120_000_000_000):
        finite_array([position_gate_m, diameter_gate_m, gap_reset_ns, memory_ns])
        if min(position_gate_m, diameter_gate_m, gap_reset_ns, memory_ns) <= 0:
            raise ValueError("关联门限和时间范围必须为正")
        self.position_gate_m = position_gate_m
        self.diameter_gate_m = diameter_gate_m
        self.gap_reset_ns = gap_reset_ns
        self.memory_ns = memory_ns
        self.tracks: list[Track] = []
        self.next_id = 1
        self.last_stamp_ns = -1

    def update(self, observations: list[Observation], stamp_ns: int):
        if stamp_ns <= self.last_stamp_ns:
            raise ValueError("同帧不可重复累计 confirmations")
        if any(o.stamp_ns != stamp_ns for o in observations):
            raise ValueError("同一次关联需来自同一视觉帧")
        self.last_stamp_ns = stamp_ns
        self.tracks = [t for t in self.tracks if stamp_ns-t.stamp_ns <= self.memory_ns]
        edges = []
        for ti, t in enumerate(self.tracks):
            for oi, o in enumerate(observations):
                dp = np.linalg.norm(t.xyz[:2]-np.asarray(o.xyz)[:2])
                dd = abs(t.diameter_m-o.diameter_m)
                if dp <= self.position_gate_m and dd <= self.diameter_gate_m:
                    edges.append((dp/self.position_gate_m+dd/self.diameter_gate_m, ti, oi))
        used_tracks, used_obs = set(), set()
        for _, ti, oi in sorted(edges):
            if ti in used_tracks or oi in used_obs:
                continue
            t, o = self.tracks[ti], observations[oi]
            if stamp_ns-t.stamp_ns > self.gap_reset_ns:
                t.xyz = np.array(o.xyz, dtype=float)
                t.diameter_m, t.score = o.diameter_m, o.score
                t.confirmations = 1
                t.position_deviation_m = 0.0
                t.diameter_samples.clear()
            else:
                residual = np.linalg.norm(t.xyz[:2]-np.asarray(o.xyz)[:2])
                t.position_deviation_m = 0.75*t.position_deviation_m+0.25*residual
                t.xyz = 0.75*t.xyz+0.25*np.asarray(o.xyz)
                t.confirmations += 1
                t.score = 0.7*t.score+0.3*o.score
            t.diameter_samples.append(o.diameter_m)
            t.diameter_m = 0.8*t.diameter_m+0.2*float(np.median(t.diameter_samples))
            t.stamp_ns = stamp_ns
            used_tracks.add(ti); used_obs.add(oi)
        for oi, o in enumerate(observations):
            if oi not in used_obs:
                t = Track(self.next_id, np.array(o.xyz, dtype=float), o.diameter_m,
                          o.score, stamp_ns)
                t.diameter_samples.append(o.diameter_m)
                self.tracks.append(t)
                self.next_id += 1
        return self.tracks

    def select_two(self, now_ns: int, route_end=False):
        candidates = [t for t in self.tracks if t.confirmations >= 3
                      and 0 <= now_ns-t.stamp_ns <= 10_000_000_000
                      and t.score >= 0.25 and t.position_deviation_m <= 0.35]
        distinct = []
        for t in sorted(candidates, key=lambda t: (-t.confirmations, -t.score)):
            if all(np.linalg.norm(t.xyz[:2]-u.xyz[:2]) >= 0.14 for u in distinct):
                distinct.append(t)
        if len(distinct) < (2 if route_end else 3):
            return []
        # 本函数只按尺寸选目标；执行顺序由任务层另行安排。
        return sorted(distinct, key=lambda t: t.diameter_m)[:2]
```

**完整文件：`tools/tracking_demo.py`。**

```python
import numpy as np
from cuadc_vision_lab.tracking import Observation, StaticBucketTracker


def main():
    tracker = StaticBucketTracker(); rng = np.random.default_rng(26)
    for frame in range(12):
        t = 1_000_000_000+frame*100_000_000
        observations = []
        for i, d in enumerate([.15, .20, .25]):
            pos = np.array([31.+i, -1.+i, .30])+rng.normal(0., .008, 3)
            observations.append(Observation(t, tuple(pos), d+rng.normal(0., .002), .85))
        tracker.update(observations, t)
        print(frame, [(x.id, x.confirmations, round(x.diameter_m, 3)) for x in tracker.tracks])
    print('select:', [x.id for x in tracker.select_two(t)])
    print('three distinct objects, not three detections of one object')


if __name__ == '__main__':
    main()
```

```bash
python -m tools.tracking_demo
```

验收：同一帧一个检测不能同时更新两个 Track；断流后的确认计数应按设计重置；只见到两个桶时默认不选，明确“航线结束”才允许选两个。

<a id="ch-41"></a>

# 41. EMA、中值、MAD：分别解决什么问题

## 41.1 指数平滑不是“越平越准”

$$
\hat x_k=(1-\alpha)\hat x_{k-1}+\alpha x_k.
$$

较小 `alpha` 更平滑，但响应更慢。对固定周期的一阶滤波，在缓慢变化条件下，等效滞后约为 `(1-alpha)/alpha` 个采样周期。`alpha=.25`、30 Hz 时约 0.1 s；这是低频近似，不是所有频率上的固定延迟。

静止桶的 **world 坐标**适合做稳健融合；随飞机运动而变化的 **body 坐标**直接平滑，会把机动混入估计并引入滞后。先把不同曝光时刻的观测统一到世界坐标，再融合；控制时再把世界估计转换到当前机体坐标。

## 41.2 直径为什么用中值

九个直径样本中若只有一两个被深度飞点污染，中值通常比均值更不受极端值影响。但九个样本全取错了桶底或壁面，中值仍然会稳定地错误。

可以先在短窗口求直径中值，再对中值做 EMA；同时用 `median(|d_i-median(d)|)` 衡量直径离散程度。报告窗口长度、有效样本数与跨越时长，避免把极短时段的重复帧当成充分观测。

## 41.3 MAD 与 position_deviation 不是一回事

MAD 是中位数绝对偏差；只有在特定分布假设下，才可以通过比例因子把它与标准差联系。使用 MAD 不会自动建立测量误差模型。

练习跟踪器的 `position_deviation` 是 XY 残差长度的指数平滑，不是方差矩阵或标准差。`confirmations>=3` 表示三个不同帧能关联到该轨迹，不代表三次独立同分布的精度实验。

## 41.4 平滑参数的实际调法

固定真实桶与真实相机姿态，先测静态散布；再做已知平移，观察世界点是否保持静止；最后改变滤波参数，分别比较偏差、抖动和延迟。

不要用滤波掩盖坐标轴相反、深度单位错误或时间偏置。系统偏差应先修正原因，否则滤波只会让错误曲线更好看。

<a id="ch-42"></a>

# 42. 三桶选二、冻结目标与受约束重捕获

## 42.1 搜索阶段先积累鲁棒位置

搜索需要覆盖范围与稳定实体假设，可以给每条轨迹维护一个有界窗口。练习取最近 15 个不同帧的世界坐标，以逐轴中位数作为位置快照，再用样本到快照的距离中位数描述散布：

```python
import numpy as np

def robust_snapshot(samples_xyz):
    points = np.asarray(samples_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('expected finite Nx3 points')
    if len(points) < 6:
        raise ValueError('need at least six distinct frame observations')
    center = np.median(points[-15:], axis=0)
    radial_spread = np.median(np.linalg.norm(points[-15:] - center, axis=1))
    return center, float(radial_spread)
```

函数只演示统计，调用者须先按帧去重并核对时间。15 与 6 是本练习选择的窗口和最少样本数。散布小只能说明一致性，不能证明绝对位置准确；还要限制最新测量年龄、窗口时长、直径离散度和位置范围。每个样本必须记录原始帧号与测量时间，不能重复累计。

## 42.2 选择目标与安排顺序分开

先筛可靠且空间独立的轨迹，再按任务准则选两个，例如估计直径较小的两个；之后再按距离或路径成本安排访问顺序。搜索未完成就只见到两个目标，与完整覆盖后仍只有两个，应采用不同的缺失处理策略。

候选质量可以综合确认帧数、检测分数、位置散布和直径散布，但门限不能由总分抵消：时间失效或身份歧义的候选，即使模型分数很高也不应进入计划。

## 42.3 冻结身份，按阶段更新位置

选择时保存任务 ID、世界坐标快照、尺寸范围与证据时间，形成任务记忆。接近目标时，用满足约束的新观测更新活动位置；接近动作边界时，再依据任务策略冻结本次使用的坐标。

冻结坐标不等于冻结飞机姿态，也不等于观测持续新鲜。任务记忆应保留最初的测量时间，并与 `last_live_observation_stamp` 分开记录。

## 42.4 受约束重捕获

重捕获需同时检查与原位置的距离、尺寸一致性、与另一已选目标或已处理区域的间距，并要求候选唯一且连续多帧一致。多个候选都通过时，应标记歧义，等待更多证据。

记录最佳和次佳匹配代价、候选数量、选择理由。仅凭“当前离飞机最近”可能切换实体；仅凭任务记忆也不能增加一帧新视觉确认。

练习：对三个固定点加入高斯抖动和一个大离群点，比较 EMA 与窗口中位数；再将其中两个点移近，验证歧义出现时系统保持待确认。

<a id="ch-43"></a>

# 43. ROS 视觉接口：先把语义写清楚

## 43.1 标准类型先守住标准语义

标准 `geometry_msgs/Pose` 由 `position` 与四元数 `orientation` 组成。`PoseArray` 表示同一参考系中的一组位姿；`orientation.x/y/z/w` 不能随意塞直径、置信度或另一坐标。[R56]

只测到中心位置时，优先考虑带 header 的点消息或自定义观测消息。若用 `PoseArray` 做本章的几何显示练习，可统一设置单位四元数 `(0,0,0,1)`，并声明它只是显示约定，未测量目标朝向。直径和质量应放入具名字段。

`header.frame_id` 写参考坐标系；它不会自动调用变换。接收端必须核对 frame、时间和单位，并在唯一明确的环节完成坐标变换。[R29]

## 43.2 一个可解释的自定义协议草案

```text
# BucketObservation.msg（教学设计）
geometry_msgs/Point center_body_m
float32 diameter_m
float32 detection_score
float32 position_quality
float32 depth_valid_ratio
float32 fit_residual_m
uint8 geometry_method
uint32 local_detection_id

# BucketObservationArray.msg（教学设计）
std_msgs/Header header
uint64 frame_sequence
string calibration_id
uint8 frame_status
BucketObservation[] observations
```

明确 `header.stamp` 是图像测量时刻，`frame_sequence` 对应哪个原始彩色帧，`center_body_m` 是哪个机体原点的米制坐标。地面投影点与桶口中心若都输出，应分设具名字段，不能互相替代。

检测分数与定位质量分开：一个桶被模型确信地识别，不代表深度可靠。`geometry_method` 区分三维桶沿、平面交点和单点深度；协方差只在有可解释误差模型时提供。

## 43.3 边界校验与适配

发布和接收两端都检查有限数、范围、时钟域、顺序与新鲜度。零分数、未知质量和无效坐标应有明确处理，不得将缺失质量静默提升成满分。

若需要与其他接口互通，先写字段到字段的映射表及拒绝案例；适配器只转换语义，不替任务层决定允许动作。第45章用标准 `PoseArray` 展示点，带同一测量时间的诊断保存直径与分数；这种诊断不替代原子化自定义观测消息。

<a id="ch-44"></a>

# 44. Image、CameraInfo 与深度订阅

## 44.1 不要直接把 ROS Image.data 当三维数组

消息还包含 `height`、`width`、`encoding`、`is_bigendian` 和 `step`。一行实际占用的字节数可能含 padding。用 `cv_bridge` 做明确编码转换，比假定 `width*height*3` 更容易避免错误。[R27][R31]

```python
color = bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
```

彩色图可以显式转 BGR；深度不能随手转成 `mono8`。符合 ROS 常见深度约定的 `32FC1` 以米表达，`16UC1` 常用毫米，但具体驱动必须核查，程序要把 scale 写入契约。[R30]

## 44.2 原始内参 K 与校正投影 P

`CameraInfo` 同时包含畸变 D、原始内参 K、校正旋转 R 和投影 P。不能对已经校正的图像再拿原始畸变重复校正，也不能把某一路相机的 K 用到另一路。[R32]

标准驱动可能在发布 rectified 图像时仍保留原始 D/K，并通过 P 表示校正后的投影。下章教学节点为了减少歧义，要求一种更严格的实验契约：输入图像已校正，附带的实验 CameraInfo 以 P 明确描述其针孔模型且 D 清零。这个约定是本实验的限制，不代表所有真实 ROS 驱动都这样发布。

## 44.3 近似同步的作用与限度

`ApproximateTimeSynchronizer` 把 header 时刻相近的多条消息配成一组；它不进行图像空间配准，也不修正传感器时钟偏差。`slop=0.02` 只是允许约 20 ms 的消息时间差，不是把 20 ms 差异消除了。[R34]

对静态内参，不一定需要每帧同步 CameraInfo，可以缓存同 profile 的标定；对会变焦、裁切、切换分辨率的相机，必须按配置版本更新。下章为了让新人看到完整配对关系，合成源每帧都发送同一套匹配 CameraInfo。

## 44.4 QoS 应与相机发布端兼容

视觉传感器流常采用 SensorDataQoS，即偏重新鲜数据而非保证每帧必达。订阅端使用与发布端兼容的配置；例如 best-effort 发布端不能满足要求 reliable 的订阅端。[R33]

```bash
ros2 topic list -t
ros2 topic info /lab/camera/color_rect --verbose
ros2 topic hz /lab/camera/color_rect
```

`hz` 测的是该观测进程收到消息的频率，不是相机真实曝光频率，也不证明有效深度或算法输出质量。

<a id="ch-45"></a>

# 45. 阶段实践：一个能隔离运行的 ROS 视觉小工程

## 45.1 这个练习能做什么、不能做什么

本章给出合成相机和基于白环的实验定位节点。输入是**已配准、已校正、正视等深平面**场景；输出 `/lab/vision/buckets_body`。节点不含 MAVROS 客户端，不解锁、不起飞、不操作舵机。

默认两个参数 `geometry_confirmed` 和 `frontoparallel_lab_scene` 为 false；未经核验不会发布目标。它不是把随便一个真实 D435i 话题 remap 进来就能飞的生产级定位器。

对真实侧视桶，本实验中的 `diameter_px*Z/fx` 和“中心像素配一个桶沿中值深度”可能有透视偏差；应该换成前文讲的三维桶沿/平面几何，再做实物验收。

## 45.2 工程结构

```text
ros2_ws/src/cuadc_vision_ros/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/cuadc_vision_ros
├── config/lab.yaml
├── cuadc_vision_lab/             # 离线核心模块副本
└── cuadc_vision_ros/
    ├── __init__.py
    ├── synthetic_camera_node.py
    └── plane_vision_node.py
```

将前文核心模块整理到图中的同名 Python 包，并保留空的 `__init__.py`。本练习为了展示最小工作空间使用副本；长期维护时应选择单一源码或可安装依赖。资源索引文件 `resource/cuadc_vision_ros` 是空文件，包目录也需要 `__init__.py`。

## 45.3 完整合成相机

**完整文件：`ros2_ws/src/cuadc_vision_ros/cuadc_vision_ros/synthetic_camera_node.py`。**

```python
"""独立合成相机；不是实际相机、不是实机定位数据，禁止接入飞控任务图。"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo


class SyntheticCamera(Node):
    def __init__(self):
        super().__init__('synthetic_camera_lab')
        self.bridge = CvBridge()
        self.rgb = self.create_publisher(Image, '/lab/camera/color_rect', qos_profile_sensor_data)
        self.dep = self.create_publisher(Image, '/lab/camera/depth_rect_aligned', qos_profile_sensor_data)
        self.info = self.create_publisher(CameraInfo, '/lab/camera/camera_info', qos_profile_sensor_data)
        self.image = np.full((540, 960, 3), 40, dtype=np.uint8)
        self.depth = np.full((540, 960), 2.30, dtype=np.float32)
        for u, v, diameter in [(180, 210, .15), (440, 330, .20), (740, 220, .25)]:
            radius = int(round(800*diameter/(2*2.0)))
            mask = np.zeros((540, 960), dtype=np.uint8)
            cv2.circle(mask, (u, v), radius, 255, -1)
            cv2.circle(mask, (u, v), radius-6, 0, -1)
            self.image[mask > 0] = 245
            self.depth[mask > 0] = 2.0
        self.timer = self.create_timer(.05, self.sample)
        self.get_logger().warning('SYNTHETIC LAB CAMERA ONLY; do not connect to aircraft')

    def sample(self):
        # 在仿真语义下对同一个静止场景重新采样。不是给旧实测录像伪造新采集时刻。
        stamp = self.get_clock().now().to_msg()
        color = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        depth = self.bridge.cv2_to_imgmsg(self.depth, encoding='32FC1')
        info = CameraInfo()
        info.width, info.height = 960, 540
        info.distortion_model = 'plumb_bob'
        info.d = [0., 0., 0., 0., 0.]
        info.k = [800., 0., 479.5, 0., 800., 269.5, 0., 0., 1.]
        info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        info.p = [800., 0., 479.5, 0., 0., 800., 269.5, 0., 0., 0., 1., 0.]
        for message in (info, color, depth):
            message.header.stamp = stamp
            message.header.frame_id = 'lab_color_optical_frame'
        self.info.publish(info)
        self.rgb.publish(color)
        self.dep.publish(depth)


def main():
    rclpy.init(); node = None
    try:
        node = SyntheticCamera(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

## 45.4 完整视觉节点

**完整文件：`ros2_ws/src/cuadc_vision_ros/cuadc_vision_ros/plane_vision_node.py`。**

```python
"""已配准、已校正、正视平面实验的 ROS 接口示例；不是通用真机定位器。

默认拒绝工作。仅在脱离飞控的实验环境核验输入语义后启用。
"""
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import String
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cuadc_vision_lab.geometry import Intrinsics, transform, validate_rotation, deproject
from cuadc_vision_lab.depth import robust_depth
from cuadc_vision_lab.detection import detect_white_rings


class PlaneVisionLab(Node):
    def __init__(self):
        super().__init__('plane_vision_lab')
        self.declare_parameter('geometry_confirmed', False)
        self.declare_parameter('frontoparallel_lab_scene', False)
        self.declare_parameter('depth_16uc1_scale_m', 0.001)
        self.declare_parameter('r_body_camera', [0., -1., 0., -1., 0., 0., 0., 0., -1.])
        self.declare_parameter('t_body_camera_m', [0., 0., 0.])
        self.enabled = bool(self.get_parameter('geometry_confirmed').value and
                            self.get_parameter('frontoparallel_lab_scene').value)
        self.r_bc = validate_rotation(np.array(self.get_parameter('r_body_camera').value).reshape(3, 3))
        self.t_bc = np.array(self.get_parameter('t_body_camera_m').value)
        if self.t_bc.shape != (3,) or not np.isfinite(self.t_bc).all():
            raise ValueError('t_body_camera_m invalid')
        self.scale = float(self.get_parameter('depth_16uc1_scale_m').value)
        if not np.isfinite(self.scale) or self.scale <= 0:
            raise ValueError('scale invalid')
        self.bridge = CvBridge()
        # 固定实验命名；不要 remap 到实机 /perception/drop_buckets_body。
        self.pub = self.create_publisher(PoseArray, '/lab/vision/buckets_body', qos_profile_sensor_data)
        self.diag = self.create_publisher(String, '/lab/vision/diagnostics', 10)
        self.color = Subscriber(self, Image, '/lab/camera/color_rect', qos_profile=qos_profile_sensor_data)
        self.depth = Subscriber(self, Image, '/lab/camera/depth_rect_aligned', qos_profile=qos_profile_sensor_data)
        self.info = Subscriber(self, CameraInfo, '/lab/camera/camera_info', qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([self.color, self.depth, self.info], 5, 0.02)
        self.sync.registerCallback(self.on_images)
        self.last_input_stamp_ns = -1
        self.last_arrival = time.monotonic()
        self.watchdog = self.create_timer(1., self.health)
        self.get_logger().warning('LAB ONLY: enabled=%s; no flight-control services' % self.enabled)

    def report(self, state, **kwargs):
        msg = String(); msg.data = json.dumps({'state': state, **kwargs}, ensure_ascii=False)
        self.diag.publish(msg)

    def health(self):
        if time.monotonic()-self.last_arrival > 1.:
            self.report('NO_SYNCHRONIZED_INPUT')

    def on_images(self, color_msg, depth_msg, info_msg):
        self.last_arrival = time.monotonic()
        if not self.enabled:
            self.report('DISABLED_UNVERIFIED_GEOMETRY'); return
        stamp = color_msg.header.stamp
        t_ns = stamp.sec*1_000_000_000+stamp.nanosec
        if t_ns <= 0 or t_ns <= self.last_input_stamp_ns:
            self.report('NONMONOTONIC_STAMP'); return
        self.last_input_stamp_ns = t_ns
        start = time.monotonic()
        try:
            if color_msg.header.frame_id != depth_msg.header.frame_id or color_msg.header.frame_id != info_msg.header.frame_id:
                raise ValueError('optical frame mismatch')
            if self.get_clock().now().nanoseconds-t_ns > 350_000_000 or self.get_clock().now().nanoseconds-t_ns < -50_000_000:
                raise ValueError('stale/future frame; clock domain must match')
            image = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            raw = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            if depth_msg.encoding == '16UC1':
                depth_m = raw.astype(np.float64)*self.scale
            elif depth_msg.encoding == '32FC1':
                depth_m = raw.astype(np.float64)
            else:
                raise ValueError('depth encoding must be documented 16UC1 or 32FC1')
            if image.shape[:2] != depth_m.shape or depth_m.shape != (info_msg.height, info_msg.width):
                raise ValueError('resolution mismatch')
            # 本例要求处理后的 CameraInfo 描述校正图像，D 已清零。
            if any(abs(x) > 1e-9 for x in info_msg.d):
                raise ValueError('this lab accepts explicitly rectified input only')
            projection = np.array(info_msg.p).reshape(3, 4)
            if np.linalg.norm(projection[:, 3]) > 1e-9:
                raise ValueError('nonzero projection translation unsupported by this lab')
            k = Intrinsics(info_msg.width, info_msg.height, projection[0, 0], projection[1, 1],
                           projection[0, 2], projection[1, 2])
            out = PoseArray()
            out.header.stamp = color_msg.header.stamp
            out.header.frame_id = 'lab_body_flu'
            rejected = 0
            measurements = []
            for candidate in detect_white_rings(image):
                try:
                    depth = robust_depth(depth_m, candidate.rim_mask)
                    # 仅正视等深平面成立。真实侧视桶应使用三维桶沿/平面算法。
                    diameter = candidate.diameter_px*depth.z_m/k.fx
                    if not .08 <= diameter <= .35:
                        raise ValueError('diameter outside this lab range')
                    body = transform(deproject([candidate.u, candidate.v], depth.z_m, k), self.r_bc, self.t_bc)
                    pose = Pose()
                    pose.position.x, pose.position.y, pose.position.z = map(float, body)
                    pose.orientation.w = 1.0  # 单位四元数仅用于显示，未测量朝向。
                    measurements.append({'index': len(out.poses), 'diameter_m': float(diameter),
                                         'circularity': float(candidate.circularity)})
                    out.poses.append(pose)
                except ValueError:
                    rejected += 1
            if self.get_clock().now().nanoseconds-t_ns > 350_000_000:
                self.report('STALE_AFTER_PROCESSING'); return
            self.pub.publish(out)  # 正常无目标帧可以发布空数组；相机异常不伪造空帧。
            self.report('OK', stamp_ns=t_ns, count=len(out.poses), rejected=rejected, measurements=measurements,
                        processing_ms=(time.monotonic()-start)*1000)
        except Exception as exc:
            self.report('REJECTED_INPUT', reason=str(exc))


def main():
    rclpy.init(); node = None
    try:
        node = PlaneVisionLab(); rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
```

## 45.5 构建文件

**完整文件：`ros2_ws/src/cuadc_vision_ros/package.xml`。**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>cuadc_vision_ros</name><version>0.1.0</version>
  <description>Plane vision lab; no flight control.</description>
  <maintainer email="example@example.com">CUADC teaching</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend><exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend><exec_depend>std_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend><exec_depend>message_filters</exec_depend>
  <exec_depend>python3-numpy</exec_depend><exec_depend>python3-opencv</exec_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

**完整文件：`ros2_ws/src/cuadc_vision_ros/setup.py`。**

```python
from setuptools import setup, find_packages
package_name = 'cuadc_vision_ros'
setup(name=package_name, version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/'+package_name]),
                  ('share/'+package_name, ['package.xml']),
                  ('share/'+package_name+'/config', ['config/lab.yaml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='CUADC teaching', maintainer_email='example@example.com',
      description='Isolated vision lab only', license='MIT',
      entry_points={'console_scripts': ['plane_vision_lab=cuadc_vision_ros.plane_vision_node:main',
                                          'synthetic_camera_lab=cuadc_vision_ros.synthetic_camera_node:main']})
```

**完整文件：`ros2_ws/src/cuadc_vision_ros/setup.cfg`。**

```ini
[develop]
script_dir=$base/lib/cuadc_vision_ros
[install]
install_scripts=$base/lib/cuadc_vision_ros
```

**完整文件：`ros2_ws/src/cuadc_vision_ros/config/lab.yaml`。**

```yaml
plane_vision_lab:
  ros__parameters:
    geometry_confirmed: false
    frontoparallel_lab_scene: false
    depth_16uc1_scale_m: 0.001
    # 教学假设，不是真实 D435i 外参。
    r_body_camera: [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0]
    t_body_camera_m: [0.0, 0.0, 0.0]
```

## 45.6 隔离构建和运行

在 Ubuntu 22.04 / ROS 2 Humble 机器上，退出 Conda，断开真实飞控、MAVROS 与 MAVLink 路由。安装发行版配套依赖；以下命令供学习者在该环境执行，构建和运行结果应填写到附录 F。

```bash
conda deactivate 2>/dev/null || true
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-numpy python3-opencv \
  ros-humble-cv-bridge ros-humble-message-filters

cd cuadc_vision_lab/ros2_ws
colcon build --symlink-install --packages-select cuadc_vision_ros
source install/setup.bash
export ROS_DOMAIN_ID=66
export ROS_LOCALHOST_ONLY=1
ros2 run cuadc_vision_ros synthetic_camera_lab
```

另开相同环境的终端：

```bash
source /opt/ros/humble/setup.bash
cd cuadc_vision_lab/ros2_ws
source install/setup.bash
export ROS_DOMAIN_ID=66
export ROS_LOCALHOST_ONLY=1
ros2 run cuadc_vision_ros plane_vision_lab --ros-args \
  -p geometry_confirmed:=true -p frontoparallel_lab_scene:=true
```

第三个终端同样 source 并设置 Domain 后检查：

```bash
ros2 topic echo /lab/vision/diagnostics
ros2 topic echo /lab/vision/buckets_body
```

应看到三个合成目标及诊断；停掉合成源后应出现输入中断诊断，不应该一直发送旧目标并刷新时间。

**不得把这些实验输出重映射到真机 `/perception/drop_buckets_body`。** 仅靠 Domain 不足以防止错误桥接；物理断开和独立启动图同样重要。

## 45.7 新人必须完成的修改

把图像编码故意改错，观察拒绝原因；把深度图尺寸改成与颜色不一致，必须拒绝；把白环全部去掉，应该得到健康空数组；冻结消息 stamp，应该报告非单调帧；切换掉一个门禁参数，应该停止目标输出。

这些行为比“看到三个漂亮框”更能证明你理解了接口。

<a id="ch-46"></a>

# 46. 视觉健康：没有目标与没有相机不是一回事

## 46.1 推荐状态划分

| 状态 | 解释 | 可否作为新目标证据 |
|---|---|---|
| `OK_TARGETS` | 新帧正常且有可信目标 | 可以 |
| `OK_EMPTY` | 新帧正常，没有目标通过检测/几何 | 不可以，但说明链路活着 |
| `DEPTH_INVALID` | 颜色可用，目标深度不可信 | 不可以 |
| `TIMESTAMP_INVALID` | 时间域、顺序或帧龄不合格 | 不可以 |
| `CAMERA_STALE` | 没有新的传感器输入 | 不可以 |
| `CALIBRATION_MISMATCH` | profile/外参/内参不一致 | 不可以 |
| `MODEL_ERROR` | 推理失败或模型尚未就绪 | 不可以 |

生产接口应能表达这些差异。标准位姿数组无法完整表示帧状态与几何质量，应使用具名观测消息与独立诊断。只有确实处理了新有效输入帧，才可能输出健康空观测；相机异常不能伪造为空场景。

## 46.2 心跳不应由显示线程伪造

画面窗口还在刷新，可能只是反复显示最后一张图；相机帧号是否推进、采样时刻是否推进和模型是否真正处理新输入，都要独立检查。

同理，ROS Timer 还能发状态文本，只证明进程部分逻辑还活着，不证明采集/推理/定位都健康。

## 46.3 断流恢复的边界

相机重启可能改变硬件时间原点、帧号和 profile。恢复后先重新验证标定与时间映射；清理待处理帧和不可用的短期轨迹；不要把恢复前后的数据硬拼成连续流。

若任务层已冻结目标，是否继续使用任务记忆由任务策略决定。视觉节点只能报告当前证据，不能把记忆坐标标记成重新看到了原目标。

<a id="ch-47"></a>

# 47. 把视觉接入任务节点：接收端检查清单

## 47.1 从消息到可用观测

接收端依次检查消息类型与帧语义、非零测量时间、时间顺序、未来与过期范围、数值有限性、尺寸和质量，再查测量时刻的导航历史。只有几何变换与区域检查也通过，才允许更新目标轨迹。

空观测可以证明一帧有效图像已处理，但不能替某条目标增加确认次数。相机停止、模型异常、时间无效和“场景中没有目标”应分开诊断。

## 47.2 顺序排查

```text
相机给了不同帧号的新输入吗？
 → stamp 是采样时刻还是取帧/发布时刻？属于哪个时钟域？
 → RGB 与深度曝光偏差是否合格？
 → 位置和姿态历史分别覆盖该测量时刻吗？
 → 坐标已经是约定的 body FLU 米制点吗？
 → 相机到机体外参是否只做了一次？
 → 目标属于约定的桶口平面还是地面？
 → 世界坐标、直径、质量和独立性是否通过？
```

逐级记录接受/拒绝计数、原因与帧号，能区分模型漏检与接口拒绝。

## 47.3 记录最终生效参数

参数可能依次经过默认值、配置文件、命令行、限幅和运行模式选择。启动时打印最终生效的相机配置、标定 ID、模型哈希、几何方法、质量门与时间门。

配置非法时给出字段名和原因；若程序会自动限幅或选择后备方法，应明确记录。不能仅保存一份配置文件，就假定运行时完全按它执行。

## 47.4 用非平凡姿态验证变换

若视觉节点已经输出世界坐标，接收端不能再按机体点旋转平移一次。话题名、字段名和 `frame_id` 应共同表达实际语义。

测试至少覆盖非零 yaw、相机平移与 roll/pitch，并将同一个固定世界点先投影到相机、再反向恢复。只在原点与水平姿态测试，错误变换容易碰巧得到相同结果。

<a id="ch-48"></a>

# 48. 粗对准、细对准：视觉与姿态补偿的分工

## 48.1 观测时刻与当前时刻分开

设图像测量时刻为 `t_c`，当前控制时刻为 `t`。先将目标恢复到世界坐标：

$$
p^W_T=p^W_B(t_c)+R^W_B(t_c)\left(R^B_Cp^C_T+t^B_C\right).
$$

再计算它相对当前机体的位置：

$$
p^B_T(t)=R^W_B(t)^T\left(p^W_T-p^W_B(t)\right).
$$

第二步消除的是“图像拍摄之后飞机移动了多少”的坐标影响，不会消除目标运动、地图漂移和测量误差。若直接使用旧帧的机体偏差，会把旧视角的误差当成当前误差。

## 48.2 搜索与接近阶段使用不同融合策略

搜索阶段更看重覆盖和实体稳定性，可以用短窗口世界位置快照；接近阶段更看重当前残余误差，应使用新鲜有效观测并补偿自身运动。

图像边缘筛选也应与阶段目标相符：搜索时一刀切掉边缘可能造成覆盖缺口；精细定位时可以依据边缘畸变、截断比例和测深质量收紧门限。每个门应有对应失效案例，不能只为提高通过率放宽。

## 48.3 从目标点反解机体位置

释放口在机体系的固定外参为 `r_release`，当前世界位置为：

$$
p^W_{release}(t)=p^W_B(t)+R^W_B(t)r^B_{release}.
$$

若只求水平对准，期望机体位置满足：

$$
p^{W,*}_{B,xy}=p^W_{T,xy}-(R^W_B(t)r^B_{release})_{xy}.
$$

这是根据实测姿态调整位置目标，并非直接给飞控姿态内环一个反向角度命令。多个释放口要分别标定，并与执行器编号对应。

## 48.4 失去视觉时明确停止更新

当观测超过使用年龄或身份不唯一，应撤销“新鲜视觉对准”状态，保留最后观测时间与任务记忆。实验接收器可以暂停推进并等待恢复；超时后报告失败，由任务策略处理。

等待不自动生成新确认，超时也不自动说明几何正确。将恢复条件、超时条件和失败路径分别验证，避免误把等待结束当作通过。

<a id="ch-49"></a>

# 49. RELEASE：目标冻结、姿态更新与最后一道门

## 49.1 冻结目标不等于冻结状态

可以为一次动作冻结目标世界坐标与任务身份，同时继续根据当前姿态计算释放口位置。被冻结的证据时间不能刷新，补偿后的机体目标还应限制变化速度，避免观测更新引入跳变。

## 49.2 动作前的四组检查

| 类别 | 需要回答的问题 |
|---|---|
| 身份与证据 | 是否仍对应选定实体？观测是否新鲜？是否已经处理过？ |
| 几何 | 参考点、实际释放口、目标平面及区域关系是否符合定义？ |
| 运动 | 水平/垂直速度、角速度是否处于经验证的范围？ |
| 持续性 | 条件是否连续成立，期间出现拒绝是否清零稳定计时？ |

门限应来自误差预算与台架实验。分别限制 roll 与 pitch，并不等于限制总倾角；若关心机体总倾角，应直接计算机体上轴与世界上轴夹角。

## 49.3 相关判据不算独立证据

当机体目标就是 `target_xy-(Rr)_xy`，且使用相同姿态时，“机体目标误差”与“释放口误差”在代数上可能相等。两项都通过不等于多了一份独立测量。

要独立验证几何，可使用测量尺、固定标记、另一视角视频或真实落点记录。不能通过把同一观测代入两种公式，声称交叉验证已经完成。

## 49.4 命令响应与机械结果分开

ACK 成功说明命令接口的响应，不直接证明载荷已脱离；ACK 超时也不能证明机构没有动作。练习中区分 `requested`、`accepted`、`physical_result_unknown` 与独立确认的结果。

机构动作开始后，视觉更新不一定能够撤回动作。发生响应不明时应保留证据并进入明确的处理路径，不能把重复发命令当作第二次可靠成功。

<a id="ch-50"></a>

# 50. 几何补偿的边界与误差来源

## 50.1 固定机体高度不等于固定释放口高度

若教学函数只反解 XY，并把机体 Z 固定为预设值，释放口到目标平面的距离仍会随姿态、杠杆臂和地形变化。

在单位法向 `n` 指向上的目标平面 `n·p+d=0` 中，释放口的有符号距离是 `n·p_release+d`。要求固定间距时，应把该约束写进完整位置求解；不要直接用起飞点相对高度代替。

## 50.2 机械延迟与脱离后运动分开

若测得命令到脱离的延迟为 `tau`，短时间近似下可以计算 `p_detach ≈ p_release + v_release*tau`。其中释放口速度还包含转动项 `omega × (R r_release)`，各向量必须在同一坐标系。

该近似只估计脱离时的位置，不是最终落点。延迟应通过多次同步视频或传感器记录测量，并报告分布；未知延迟不能用一个看似精确的常数代替。

## 50.3 用自由落体理解速度误差

若释放点到目标平面的垂直落差为 `h`、初始竖直速度为零、忽略空气阻力：

$$
t_f=\sqrt{2h/g},\qquad \Delta x=v_x t_f.
$$

`h` 是释放点与目标平面的落差，不能直接取机体起飞点相对高度。水瓶也不是质点，瓶身可能先碰桶沿；风阻、旋转和碰撞需要额外模型或实验。

## 50.4 用记录定位误差来源

保存目标估计、曝光时间、姿态、实际释放口位置、命令时间和独立视频。按相机视场位置、roll/pitch、释放口编号与风向分组比较落点偏差，先区分感知、几何、控制跟随与机构问题，再决定是否增加预测模型。

---

<a id="ch-51"></a>

# 51. 投放区搜索：视场、航线和多帧确认如何一起设计

## 51.1 视场覆盖不是识别覆盖

针孔近似、相机正视平面时，水平覆盖宽度为：

$$
W=2H\tan(\theta_h/2).
$$

`H` 是相机到所考察平面的距离，`theta_h` 必须是该输出模式有效的**水平**视场。不要拿对角 FOV 直接代入，更不能用飞机参考点相对起飞高度代替相机高度。[R08]

可检测覆盖还要扣除边缘畸变、裁切、遮挡、低像素密度区及模型漏检区。规划中应使用“实测有效检测足迹”，而不是整张图像的数学锥体。

## 51.2 从有效足迹计算航线间距

假设练习区域为 8 m × 5 m，在给定高度、姿态与曝光条件下，实测横向有效检测足迹宽度为 `W_valid`。选择重叠率 `rho` 后，初步线间距可取 `s <= (1-rho)*W_valid`。

外侧航线需结合足迹半宽、边界裕度和转弯空间布置；再用最不利目标位置检查是否仍有足够观察时长。航线条数由覆盖计算和实验决定，不能固定为某个工程采用过的条数。

练习：令 `W_valid=2 m`、`rho=0.3`，画出间距不超过 1.4 m 的路线；再让足迹因倾斜或裁切缩小，检查边界与角落是否出现空白。

## 51.3 从确认次数估计最低观测时间

假设有效检测频率为 `f_valid`，且目标必须得到 `n` 次确认，理想连续检测至少跨越约 `(n-1)/f_valid` 秒；还需考虑首次进入可检测区的相位、漏检、推理延迟、轨迹门和选择时机。

若有效覆盖沿飞行方向长度为 `L`，速度为 `v`，几何驻留时间约 `L/v`。必须让它明显大于确认所需时间，而不是只满足一次曝光。

例如 `f_valid=10 Hz,n=3` 时，三次连续采样跨越约0.2s；这不等于“停0.2s一定识别成功”。把相机30fps直接当作有效三维定位30Hz，会过于乐观。

## 51.4 优化搜索的正确实验

在训练场随机摆放三个桶，多次更换位置和方向。统计首次可靠发现时间、第三桶确认时间、每条航线漏检位置，以及未找到第三桶时两桶方案的成功率。

只有这些数据才支持减少航线、改变高度或提前接受两个目标。不要用一次桶集中在中央的幸运飞行证明整个区域都能快速覆盖。

正式任务前先核对随机目标信息的获取边界。训练场算法测试使用自建数据，进入比赛后遵守适用的信息获取与自主运行要求。

<a id="ch-52"></a>

# 52. O4 侦察：先画真实硬件链路

## 52.1 从设备清单确定可用接口

以 DJI O4 高清数字图传作为观察链路示例，首先记录确切子型号、镜头安装角、显示设备、实时视频模式、固件、有效裁切区域、地面录制/采集接口与端到端延迟。

因此不能默认它就是一台能被 `cv2.VideoCapture(0)` 打开的相机。`VideoCapture` 只能打开操作系统或所选后端实际支持的源；设备 USB 接口是否输出实时 UVC，需要按具体设备验证。[R03][R39][R46]

## 52.2 三种不同目标

| 目标 | 最小链路 | 额外证据 |
|---|---|---|
| 人员实时看图判读 | 相机→图传→显示 | 实时画质、可见时长、人员正确率 |
| 地面保存视频复核 | 上述链路+支持的记录方式 | 录像时刻与实际观察过程的关系 |
| 计算机实时自动识别 | 明确支持的视频输出+采集解码+模型 | 输入协议、延迟、颜色/裁切、丢帧和时钟 |

这三件事不能互相冒充。机载高分辨率录像清楚，不证明当时地面实时流同样清楚；地面显示存在，不证明 ROS 收到了原始画面。

## 52.3 两路相机各自独立标定

D435i 内参与外参不能沿用给 O4。即使固定在同一块支架上，也有不同光心、镜头投影、安装旋转、快门和视频处理链。

为 O4 建立独立配置，例如 `o4_live_profile_01`，把显示模式、是否开启电子防抖、画面旋转、裁切、输出尺寸一并记录。改变这些配置后重新确认视场和目标像素数。

<a id="ch-53"></a>

# 53. O4 高清图像如何变成可判读证据

## 53.1 四个限制必须同时满足

目标必须在视场内、没有被桶壁挡住、像素足够且不模糊、显示出来的时间足够人员或模型判读。任一项不成立，增加图传分辨率也不能自动解决。

官方 O4 系列规格给出了不同机型的 FOV、录像和传输模式，但这些只是选定模式下的设备规格，不代表实际安装与现场链路的实际有效观测性能。[R03]

## 53.2 三种像素数

传感器原始像素、机载录像像素、地面实时显示像素并不总相同。画面还可能经过电子防抖裁切、缩放、编码和解码。

假设地面实收图像横向1920像素，某平面的有效横向覆盖为8.6m，则12cm标识名义横向仅约 `1920*.12/8.6≈27` 像素。它内部图案、文字或细线占用的像素更少；这个算式也忽略了畸变、桶壁和斜视缩短，不能直接当作“足以识别”的结论。

## 53.3 运动模糊的定量约束

在近似正视、平面、纯横向运动条件下，模糊像素：

$$
b\approx \frac{f_x v t_{exp}}{H}.
$$

例如 `fx=800px,H=3m,v=6m/s`：曝光1/1000s约1.6像素，1/250s约6.4像素。对只有二十多像素的图标，6像素模糊可能很严重。这是示例模型，不是 O4 的实测焦距或曝光能力。

100fps并不意味着每帧曝光足够短。降低曝光时间通常需要更强光照或更高增益，增益又可能增加噪声。应检查实际画面和设备可用控制，而不是直接照抄一个快门数值。

## 53.4 固定航向与高速飞行的姿态

任务 yaw 固定不代表相机始终垂直地面。飞机为了加速和减速会俯仰，为抵抗侧风可能滚转，O4 视轴随之偏离。

因此高速过渡段产生的姿态变化可能使画面短暂离开侦察区。用固定矩形地面覆盖估计航线时，应考虑姿态包络，而非只用静止悬停照片。

## 53.5 建立自己的判读测试集

使用真实安装、真实地面显示或实际支持的采集链路，记录不同高度、横向偏距、曝光和速度下的标识。由不知道答案的人盲判，分别统计正确、错误、空白和判读耗时。

评估时随机化标识与桶位置，避免参与者靠记忆、场景布局或前一轮答案“识别”。这比单独测试相机静态清晰度更接近任务收益。

<a id="ch-54"></a>

# 54. 关键几何：筒壁会挡住标识，宽视场不能绕过遮挡

## 54.1 为什么这是侦察航线的核心约束

本练习取侦察筒高15cm、直径20cm，标识12×12cm。以下推导明确假设：标识平放在筒底附近的平面，筒壁不透明、垂直，桶内半径近似10cm，镜头可以看到桶口。这是建模假设；真实摆放位置、桶壁厚度和标识平面高度必须现场核验。

相机在桶中心水平距离很远时，即使白桶完整出现在图像里，从镜头到筒底标识的视线也可能先撞上近侧桶壁。图像处理不能恢复没有到达镜头的内容。

## 54.2 最简单的截面推导

令标识中心平面为 `z=0`，相机在 `(rho,H)`，桶沿在 `z=h`，内半径 `r`。从标识中心连到相机的直线，在桶沿高度的水平偏移为：

$$
\Delta_{rim}=\rho\frac{h}{H}.
$$

要使**中心点**不被桶壁挡住，至少满足：

$$
\rho\frac{h}{H}\le r.
$$

带入 `h=.15m,r=.10m,H=3m,rho=4m`：

$$
\Delta_{rim}=4\times\frac{.15}{3}=.20m>.10m.
$$

即相机在场地中心线上、目标处在横向边缘约4m处时，3m高度下连标识中心也可能被遮挡。**“3米高度画面能包住8米宽区域”不能证明单线侦察可行。**

## 54.3 看见中心也不代表看见整张标识

对标识平面中的任意一点 `q=(qx,qy)`，相机水平位置 `c=(cx,cy)`，视线在桶沿平面的交点为：

$$
q_{rim}=\left(1-\frac hH\right)q+\frac hHc.
$$

这个点要落在桶内开口圆内：

$$
\|q_{rim}\|\le r.
$$

整张标识需要其关键区域满足条件。12cm正方形半对角线约8.49cm，距离10cm内半径的余量并不大。用中心点条件设计航线，容易得到只能看到半张标识的结果。

当 rho=4m 时，中心可见的临界高度为 `H=rho*h/r=6m`。这只是中心视线恰好到边缘，不意味着6m能看清完整标签，也不应据此直接抬高飞行高度。

## 54.4 提高高度存在相反代价

升高可以减轻固定横向偏距下的桶壁遮挡，但标识像素会减少。降低高度提高像素，却要求更接近桶正上方。最佳高度不是一个由FOV单独决定的数，而是可见性、像素、姿态、清晰度和任务时间的约束交集。

## 54.5 完整数值练习

**完整文件：`cuadc_vision_lab/planning.py`。**

```python
"""几何/运动学可行性计算；输出不代表飞行参数建议或实机保证。"""
import math
import numpy as np
from .geometry import finite_array


def ground_span(height_m: float, horizontal_fov_deg: float) -> float:
    finite_array([height_m, horizontal_fov_deg])
    if height_m <= 0 or not 0 < horizontal_fov_deg < 179:
        raise ValueError("高度/FOV 参数错误")
    return 2*height_m*math.tan(math.radians(horizontal_fov_deg)/2)


def pixels_on_target(size_m: float, span_m: float, image_pixels: int) -> float:
    finite_array([size_m, span_m, image_pixels])
    if min(size_m, span_m, image_pixels) <= 0:
        raise ValueError("正参数")
    return size_m*image_pixels/span_m


def blur_pixels(focal_pixels: float, speed_m_s: float, exposure_s: float,
                distance_m: float) -> float:
    finite_array([focal_pixels, speed_m_s, exposure_s, distance_m])
    if min(focal_pixels, exposure_s, distance_m) <= 0 or speed_m_s < 0:
        raise ValueError("参数范围无效")
    return focal_pixels*speed_m_s*exposure_s/distance_m


def minimum_rest_to_rest_time(distance_m: float, vmax: float, amax: float) -> float:
    """理想一维对称加减速，不含 jerk、飞控延迟、转弯和风。"""
    finite_array([distance_m, vmax, amax])
    if distance_m < 0 or min(vmax, amax) <= 0:
        raise ValueError("参数范围无效")
    if distance_m <= vmax*vmax/amax:
        return 2*math.sqrt(distance_m/amax)
    return 2*vmax/amax+(distance_m-vmax*vmax/amax)/vmax


def label_visible_fraction(camera_xyz, rim_height_m=0.15, radius_m=0.10,
                           label_side_m=0.12, angle_deg=0., grid=61) -> dict:
    """圆筒内方形标签位于 z=0，针孔相机位置相对筒心；忽略壁厚。

    计算标签点到相机射线在桶沿平面的交点是否留在开口圆内。
    不是图像像素占比，不检查 FOV、分辨率或曝光；只检查遮挡。
    """
    c = finite_array(camera_xyz, (3,))
    finite_array([rim_height_m, radius_m, label_side_m, angle_deg])
    if rim_height_m < 0 or c[2] <= rim_height_m or min(radius_m, label_side_m) <= 0 or grid < 3:
        raise ValueError("相机必须高于桶沿，尺寸为正")
    q = np.linspace(-label_side_m/2, label_side_m/2, grid)
    xx, yy = np.meshgrid(q, q)
    pts = np.column_stack([xx.ravel(), yy.ravel()])
    theta = math.radians(angle_deg)
    r = np.array([[math.cos(theta), -math.sin(theta)],
                  [math.sin(theta), math.cos(theta)]])
    pts = pts @ r.T
    alpha = rim_height_m/c[2]
    at_rim = (1-alpha)*pts+alpha*c[:2]
    visible = np.linalg.norm(at_rim, axis=1) <= radius_m
    center_visible = np.linalg.norm(alpha*c[:2]) <= radius_m
    return {"surface_fraction_visible": float(visible.mean()),
            "center_visible": bool(center_visible),
            "rim_shift_m": float(np.linalg.norm(alpha*c[:2]))}
```

**完整文件：`tools/planning_demo.py`。**

```python
from cuadc_vision_lab.planning import *


def main():
    # 90° 为假设的、实测有效水平视场，不是声称 O4 的规格。
    width = ground_span(3., 90.)
    print('hypothetical ground span:', width)
    print('12cm target width in 1920px image:', pixels_on_target(.12, width, 1920))
    print('blur: fx=960,v=6,exposure=1/1000,H=3:', blur_pixels(960, 6, .001, 3))
    for offset in [0., .4, 1., 2., 4.]:
        print('camera offset=', offset, label_visible_fraction([offset, 0, 3.]))
    print('30m,12m/s,3m/s^2 ideal minimum seconds:', minimum_rest_to_rest_time(30., 12., 3.))


if __name__ == '__main__':
    main()
```

```bash
python -m tools.planning_demo
```

程序对标识方形内部采样，计算穿过桶沿时仍位于开口中的比例。这只是理想几何可见比例，不包含镜头视场、畸变、壁厚、标识弯曲和模型识别能力。

在该模型的3m高度、标识居中、桶壁无厚度假设下，水平偏距0～0.4m时全标签可见；偏距增大到1～2m时会逐渐遮挡；4m偏距可完全不可见。请以程序输出和真实筒测试核对，不把这个演示比例当作测量结果。

**练习**：把标识从桶底抬高、减小桶内半径、旋转标识45°，观察可见区域如何变化。再拿真实筒做同样的偏距试验，解释模型与实物差异。

<a id="ch-55"></a>

# 55. 怎样重新求侦察高度、航线与速度

## 55.1 不从“理论最快速度”开始

应先定义约束：边缘目标关键内容可见比例、最低有效像素数、最大模糊、最低可用帧数、姿态范围、场地边界和任务完成概率。再寻找满足约束的最快航线。

形式上可写为：

$$
\min T(\mathcal P,H,v)\quad\text{s.t.}\quad
\text{visibility, resolution, blur, coverage, safety constraints}.
$$

这里的约束阈值必须由人员判读或模型测试确定。不存在仅凭“O4高清”和“8×5m”就能给出的已验证最优速度。

## 55.2 三种路线候选

| 候选 | 优点 | 主要限制 |
|---|---|---|
| 单条中心线 | 路短、转弯少 | 最远侧桶的遮挡和低像素风险最大 |
| 多条平行扫线 | 能靠近不同横向位置 | 转场和拐弯增加，需要验证条带间遗漏 |
| 先广域发现桶，再经过其附近 | 直接针对实体获取近垂直视角 | 需要额外桶定位与任务规划接口 |

使用地面人员观察链路时，不可默认同时具备自动规划所需的计算机视频与目标定位接口。选择第三种路线前，先在隔离实验中验证图像输入、定位、路线生成和任务触发。

## 55.3 由可接受偏距推线间距

若真实测试表明在高度H时，标识可靠判读要求相机到桶中心的水平偏距不超过 `rho_max`，仅按横向扫描的理想几何，线间距不应大于 `2*rho_max`，还要留定位、姿态和可见时长余量。

这里的 `rho_max` 是“看清标识”的偏距，不是“能看见白筒”的偏距。两者混用，会把侦察覆盖大幅高估。

## 55.4 可用观测时间比横穿区域时间更重要

对一个固定目标，若满足全部约束的沿线有效长度为 `L_good`，速度为v，则可用时间近似 `L_good/v`。必须用有效区域长度，而不是整个FOV在地面投影的长度。

例如标签只在相机经过其近旁0.8m路径时可判读，6m/s只有约0.13s。即使实时流100fps，理想也只有约13帧；其中若有压缩、遮挡或姿态转动，真正可用帧还会减少。帧数和人员能否判读也不是一回事。

## 55.5 12m/s 不是任何短段都能达到

在起终速度均为零、最大加速度a、峰值速度上限v的理想模型中：

$$
D_{reach}=v^2/a.
$$

当D小于这个距离，速度呈三角形，峰值为 `sqrt(aD)`，时间为 `2sqrt(D/a)`；更长才有梯形匀速段。

例如教学假设 `D=20m,a=3m/s²,v=12m/s`，达到12m/s并刹停需要48m，而20m只允许峰值约7.75m/s、时间约5.16s。实际还需考虑jerk、倾角、风、定位和飞控限制。这里的3m/s²是演示值，不是真实飞机参数。

改变速度、高度或路线时，应重新验证曝光、观察时长、遮挡和制动距离。理想航段时间用于方案比较，完整任务时间还包括转弯、等待、重捕获与失败处置。

## 55.6 最小验证表

固定模型或固定判读人员，逐行实测：高度、速度、最大横向偏距、roll/pitch峰值、曝光、实时流尺寸、每个标识有效帧数、正确/错误/空白结果、总时长。

先找“稳定可读”的区域，再在该区域内提高速度。别把测试顺序倒过来：先高速飞一遍，再因为没看清就猜标识。

<a id="ch-56"></a>

# 56. 危险化学品标识识别：怎样确定类别与拒识

## 56.1 不凭常识补全官方类别表

若任务要求识别危险化学品标识，训练前应依据正式材料确认完整类别清单、标准图样、允许差异与评价口径。课堂可使用自选图形验证流程，评估结果不能直接推广到比赛图样。

通用GHS图形、某数据集危险品标签和比赛现场图样可能有差异。不能把网络搜集到的一套符号自动当作当届答案。

## 56.2 两阶段模型更易定位错误来源

一种教学结构是：先发现筒或标签区域，再对裁剪后的标签分类。这样能分别评估“没找到标识”和“找到了但分错类”。

也可以训练直接检测每种标识的模型，但远处小目标、桶沿遮挡和背景相似更容易混在一起。具体结构由数据和端到端测试决定，而不是模型名字决定。

## 56.3 拒识不是系统失败

在“正确 +100、错误 −100、拒识 0”的假设评价中，若某结果的**真实正确概率**为 p，则期望收益为 `100p-100(1-p)=200p-100`，大于零需要 p>0.5。这是说明拒识价值的计算例子，实际评价应代入任务定义的收益或代价。

但神经网络输出0.9分数不一定表示90%正确概率，且多个错误可能相关。因此不能直接用“置信度>.5”当比赛策略。应在独立验证集上校准分数、分析相似类别混淆，设更可靠的拒识条件。

## 56.4 不使用生成式补图作为事实证据

超分辨率、去模糊或补全可能生成原图不存在的细节。可把非生成的增强用于辅助观察，但评分答案必须能回溯到真实原始证据；不能用“增强后出现了一个骷髅图案”证明原图就是对应标识。

对被桶壁完全挡住的区域，任何模型都不能从当前帧直接测出其内容；需要改变观测几何或获得新视角，而不是提升预测自信。

<a id="ch-57"></a>

# 57. 标识透视校正、模板与时序融合

## 57.1 四点单应变换适用条件

标识近似平面、四角可靠、没有明显弯曲和遮挡时，可以通过单应矩阵把斜视矩形映射到正视小图。[R42]

```python
import cv2
import numpy as np

def rectify_label(image, corners, size=128):
    src = np.asarray(corners, dtype=np.float32)
    if src.shape != (4, 2) or not np.isfinite(src).all():
        raise ValueError('corners must be finite ordered 4x2 points')
    # 要求输入顺序：左上、右上、右下、左下；实际程序还需验证凸性和面积。
    dst = np.array([[0, 0], [size-1, 0], [size-1, size-1], [0, size-1]], np.float32)
    h = cv2.getPerspectiveTransform(src, dst)
    return cv2.warpPerspective(image, h, (size, size))
```

上述为局部教学片段，不是完整标签定位器。若角点来自被遮挡后的猜测，校正会拉伸错误；它不能创造被遮挡内容。

## 57.2 模板匹配与分类网络的取舍

模板法在图样固定、角度与光照受控时容易解释，但对旋转、裁切、压缩和透视敏感。分类网络能学习更大变化，但需要足够真实数据和严格独立测试。

不要只拿清晰官方图样训练，再直接评估飞行画面。训练与测试域至少应覆盖实际地面显示流中的压缩、噪声、桶影、倾角与尺寸。

## 57.3 时序融合先绑定实体

同一个筒连续帧中的分类可以融合；不同筒的结果不能只按类别累加，否则三个相似标识可能被当成一个，或同一个筒被反复记数。

建议为每个筒建立独立证据记录：观测时刻、来源帧号、候选类别、分数、可见比例、清晰度、有效像素数和最终判读状态。

可以使用质量加权投票或对数分数累积作为教学起点，但相邻视频帧高度相关，不能把100帧当作100次独立证据。增加重复帧不应把错误结果的置信度推到近乎100%。

## 57.4 五个筒中哪些为空

应区分“确认空筒”“有图样但无法判别”“图像没有覆盖到筒底”“完全未发现该筒”。这四类不能全部写成blank，也不能把没看到的标识自动判成空筒。

是否需要报告每个筒的空间对应和名称，应依据实际任务记录格式确定；练习表可先列目标编号、证据时间、判读与未知原因。

<a id="ch-58"></a>

# 58. 人员判读与任务证据接口

## 58.1 把人员观察组织成证据记录

图传判读练习可分为观察、记录和复核三项职责。每条答案绑定目标编号、来源帧或视频时间、候选类别与可见质量，避免多人报答案却无法追溯。

实际任务是否允许人工判读、如何提交及何时计时，应按适用规则确认。教材只讨论如何保留可复核证据。

## 58.2 四种完成状态分开

```text
route_completed        已按计划经过观察路线
stream_healthy         该时段图传链路有有效输入
evidence_available     对相应目标留下了可读证据
answer_recorded        判读结果已按要求记录
```

航线完成不证明视频正常；文件保存成功不证明标识可读；画面清楚也不自动证明答案正确。状态名称应对应可以观察的事实，不能以内部 ACK 冒充最终任务结果。

## 58.3 不同视觉链路独立管理

三维定位相机与图传相机分离时，应分别记录输入状态、时间源、标定和负载。根据任务阶段降低不需要的推理或录像负载，可以通过显式启停状态完成；恢复时重新验证帧号、时间和配置。

练习：断开图传输入，同时保持三维定位相机正常，验证两条健康状态不会相互覆盖；再恢复输入，检查旧画面没有被计为新证据。

## 58.4 最终结果需要逐项证明

软件打印完成、航线结束或录像存在，都只能证明相应内部步骤。建立“要求—可观察证据—记录位置”的对应表，遇到不可观察结果应标为未知，并设计补充观测。

<a id="ch-59"></a>

# 59. 记录什么数据，才能真正改进视觉

## 59.1 一次实验至少保存四层

原始或可复核图像/深度、相机profile与标定、算法中间结果、任务与导航上下文。只有最终框截图，通常无法区分时间、配准和外参问题。

建议每次运行建立独立目录：

```text
run_YYYYMMDD_HHMMSS/
├── manifest.json
├── calibration/
├── rgb/
├── depth/
├── frames.jsonl
├── observations.jsonl
├── rejects.jsonl
├── navigation/
└── evaluation/
```

`manifest` 记录环境版本、git提交、模型与配置hash、设备序列号、流模式、实际有效参数，以及操作者填写的场景条件。

## 59.2 深度别只存伪彩色JPEG

伪彩色图用于人看，不能还原原始度量深度。原始uint16可存无损PNG并同时保存scale；浮点深度可存NPY或其他保留浮点和无效值的格式。[R30]

彩色图用JPEG会引入压缩差异，测量像素级边缘偏差时应考虑无损帧或保存足够原始证据。大规模记录量受存储带宽限制，需设计抽样与故障前后窗口。

## 59.3 一条观测日志模板

```json
{
  "frame_sequence": 125,
  "observation_stamp_ns": 123456789000,
  "arrival_stamp_ns": 123456840000,
  "timestamp_domain": "documented_host_mapping",
  "calibration_id": "example_only",
  "geometry_method": "rim_points_circle_3d",
  "center_body_m": [0.03, -0.02, -1.95],
  "diameter_m": 0.151,
  "depth_valid_ratio": 0.91,
  "fit_residual_m": 0.004,
  "processing_ms": 28.4,
  "status": "VALID"
}
```

这是格式示例，不是实测记录。正式日志还应记录拒绝原因、单位和未知项，避免把无法测量的质量指标填成零。

## 59.4 写盘不能阻塞视觉或导航

记录线程要有有界队列、丢弃策略和磁盘空间监控。若磁盘写满，报告记录失败；不要让整个定位线程等待数秒，更不能把仍显示在窗口中的旧图当成连续输入。

<a id="ch-60"></a>

# 60. rosbag 与离线回放：让错误可重复

## 60.1 先只录视觉和必要状态

ROS bag 可以记录与回放话题。[R50] 对实验首先使用明确列表，不随手 `-a` 录制并回放整套真机图：

```bash
ros2 bag record -o vision_lab_bag \
  /lab/camera/color_rect \
  /lab/camera/depth_rect_aligned \
  /lab/camera/camera_info \
  /lab/vision/buckets_body \
  /lab/vision/diagnostics
```

如果记录实飞导航，应明确哪些字段用于测量、哪些可能触发状态机。不在连接真实飞控的ROS图中回放历史 setpoint、arm、servo 或能满足真实任务门禁的目标话题。

## 60.2 两种回放模式不能混用

**历史时间回放**：保留历史时间戳，所有计算节点使用与回放一致的模拟时钟；适合重现原时序。

**纯算法离线评估**：把帧按文件顺序送入纯函数，评价检测/几何，不运行实际飞行任务图。

把历史图像时间改成当前时间以绕过 stale 门，会破坏原始测量含义。合成相机在模拟世界中生成新的采样，与给旧实测图像伪造新采集时刻不是一回事。

## 60.3 回放时要检查什么

检查 CameraInfo 与 TF 是否完整、颜色与深度是否可配对、原始录制是否已丢帧，以及回放速率是否改变超时行为。ROS 测量时间与单调墙钟用途不同；使用 `/clock` 不会自动改变 `time.monotonic()` 或 C++ `steady_clock`，回放实验应明确选择一致的计时策略。

<a id="ch-61"></a>

# 61. 故障注入与自动测试

## 61.1 先让程序学会拒绝错误

| 注入 | 预期行为 |
|---|---|
| 深度全0或NaN | 不产生可信三维目标 |
| 桶底与桶沿深度混合 | 质量门拒绝或转三维拟合，不盲取均值 |
| 图像尺寸改变而内参不变 | 明确配置不匹配 |
| frame_id错 | 接口拒绝，或按明确TF转换，不静默忽略 |
| 时间戳重复/倒退 | 拒绝，必要时清空历史 |
| 消息延迟变大 | 增加stale计数，不刷新原始时间 |
| 两个桶轮廓合并 | 不凭单一外轮廓生成可信大桶 |
| 同桶重复检测 | 一对一关联与去重，不满足“三个实体” |
| 同一片断放进train和val | 数据审计报泄漏 |
| 模型文件丢失 | 启动失败，不自动下载未知替代模型 |

这些是本教程设计的验收行为；需要把对应测试落实到真实节点，而不只测试孤立数学函数。

## 61.2 核心自动测试

正文中的 `tests/test_core.py` 覆盖投影/反投影、单位转换、旋转与逆变换、平面/圆拟合退化、深度质量、时间夹逼、跨±π角度、关联与选择、视场/模糊/遮挡以及标签格式检查。

```bash
python -m pytest -q
```

将本机实际执行结果填写到附录 F。单元测试通过只说明这些构造输入上的程序行为符合预期，真实数据精度和实机行为需另行验证。

## 61.3 一个有意义的负例集

把过去所有失败分成可重现样例：逆光桶、强阴影、白纸、桶沿截断、双桶接触、桶内反光、深度孔洞、相机快速旋转、断流恢复。每修复一个问题，先把它加入回归集，再改代码。

否则下一次更换阈值可能让之前修好的问题再次出现。

<a id="ch-62"></a>

# 62. 不飞也能完成的大部分视觉验收

## 62.1 静态几何验收

相机固定，目标放在图像中央与四角、多种已知高度和距离处，比较估计坐标与独立测量。报告每轴bias、RMSE、P95误差及最大误差，不只报均值。

独立真值不能来自同一D435i深度再算一遍，也不能用状态机自己估计的桶坐标证明自己准。可使用测量板、已知几何点或适当的外部测量方法，并记录其精度限制。

## 62.2 旋转不变性验收

固定真实桶，缓慢改变相机/机体 roll、pitch、yaw，保持所有变换链使用对应姿态。世界坐标理应尽量不变；若目标随姿态在世界中画圆，优先检查外参和时间，而非立刻增加滤波。

## 62.3 高度扫描

至少覆盖搜索、粗对准、细对准对应的观察尺度，并额外测试高度误差和目标在边缘的情况。实际相机到桶口距离不等于状态机高度，应实测记录。

低空最容易出现机构、水瓶或脚架遮挡；试验要装上真实载荷形状和支架，但保持动力隔离。

## 62.4 O4 真实筒遮挡验收

用与比赛相同尺寸的侦察筒和标识，在不同水平偏距记录实际地面流。不要只拿一张平放地面的12cm图标模拟筒内图标；那样会漏掉第54章最关键的遮挡约束。

<a id="ch-63"></a>

# 63. 从台架到实飞：逐级放行

## 63.1 阶段与放行证据

| 阶段 | 允许做什么 | 通过证据 |
|---|---|---|
| 离线 | 图片/合成RGB-D | 数学、无效输入、数据审计通过 |
| 无动力台架 | 实际相机/机构几何 | 外参、尺度、遮挡、时间日志可复核 |
| 隔离ROS | 观测发布与状态替身 | 不会连接真实飞行命令，断流行为正确 |
| 仿真 | 任务状态转换 | 不靠伪造新时间戳通过门禁 |
| 受控低风险实飞 | 已批准的小范围验证 | 现场安全措施、飞手接管、日志完整 |
| 全任务 | 逐步提高速度与复杂度 | 多次独立摆放均满足验收指标 |

这不是“一次跑通就上12m/s”的清单。提高速度同时改变姿态、图像模糊、时间误差和制动距离，需要重新验证。

## 63.2 先影子评估，再控制闭环

可先让视觉只记录建议目标和估计误差，不向实际任务节点提供控制输入。观察不同姿态、速度和高度下输出是否合理。

影子评估通过后，才在团队已有安全流程中接入任务。实验代码中没有飞控客户端，不代表随便接上目标话题就没有风险。

## 63.3 正式比赛前不能做什么

不得以调试之名提前获取现场随机桶位置；不得在不允许的时机开启图传；不得把人员在比赛中的持续操纵包装成视觉辅助；不得以软件内部成功代替规则要求的实际结果。

比赛前核对当届规则、补充通知和现场适用要求，并将规则尺寸与教材练习参数区分保存。

<a id="ch-64"></a>

# 64. 常见故障：按症状定位到哪一层

| 症状 | 优先怀疑 | 最小验证 |
|---|---|---|
| 能看到桶但控制器known=0 | 时间/区域/接口拒绝 | 对照每级拒绝计数 |
| 整体固定偏一侧 | 外参平移/准星基准 | 静止多点测量，区别constant bias与yaw误差 |
| 前进时桶坐标向前飘 | 时间偏置/机体系滤波 | 改变速度，比较误差与速度相关性 |
| 左右方向相反 | optical→FLU轴映射 | 手工将目标向画面右移 |
| yaw改变后误差方向随之旋转 | 旋转方向/双重变换 | 非零yaw的已知点单测 |
| roll/pitch变化时目标大幅漂 | 姿态时间/深度平面/外参z | 固定桶做慢速倾斜 |
| 深度约差1000倍 | mm与m混用 | 打印raw、scale、depth_m |
| 桶越远估计直径越大 | 内参/resize/深度位置错误 | 固定真桶改变距离 |
| 框正常但三维点偏到桶壁 | mask语义/空心几何 | 显示采样点及各自深度 |
| 转换到世界后目标在场外 | home/heading/field轴 | 打印body、local、field三级坐标 |
| 经常出现HARD_SYNC_REJECT | odom间隔/时钟域/队列 | 记录真实相邻gap和挂起时间 |
| 很高fps却反应迟钝 | 无界队列/重复帧 | 记录帧号、capture age、排队长度 |
| 原地很好，飞起来就坏 | 模糊/时延/姿态/振动 | 动态台架或受控速度扫描 |
| 小桶被当成大桶 | 错深度、轮廓截断、桶壁 | 比较真实轮廓、米制直径与掩膜 |
| 三桶变成五个Track | 重复实例/关联断裂 | 按帧可视化一对一匹配 |
| 第二瓶一直等fresh visual | 记忆不是新帧/低空遮挡 | 检查真实更新arrival及重捕获条件 |
| YAML改了却门限没变 | profile强制覆盖 | 对照load_parameters与启动日志 |
| O4能看见五个白桶却判不出图案 | 筒壁遮挡/像素不足 | 用真实筒做横向偏距实验 |
| O4机载录像清楚，现场看不清 | 实时流与录像不同 | 保存或拍摄实际地面显示证据 |
| 一直打印拍照不完成 | O4尚未适配旧ACK语义 | 对照photo_mode/capture_done与实际相机进程 |
| 只在开录像时同步失败 | CPU/USB/磁盘争用 | 记录开关录像前后P95延迟 |
| 部署后模型框位置变了 | 预处理/NMS/导出输出契约 | 相同输入逐像素比对后处理 |
| ROS能列出话题但无回调 | QoS/Domain/同步条件 | topic info --verbose逐路确认 |
| 看似一直有心跳，相机已经断开 | Timer伪造健康 | 区分进程心跳和新帧计数 |

排查顺序尽量从可观测的输入事实开始，不先调高级模型。记录“哪个假设被实验排除”，避免团队多人同时改阈值造成无法归因。

<a id="ch-65"></a>

# 65. 部署时把模型、标定和运行环境绑定

## 65.1 一份可以复现的发布包

```text
release/
├── code_commit.txt
├── models/model_file
├── models/sha256.txt
├── config/runtime.yaml
├── calibration/camera.json
├── calibration/body_extrinsic.json
├── environment/versions.txt
├── evaluation/summary.json
└── README_run_and_rollback.md
```

不要只拷贝一个 `best.pt`。模型输入颜色、resize/letterbox、类别表、输出掩膜坐标、几何方法和标定必须匹配。

## 65.2 启动前检查

验证模型存在且hash一致，相机序列号与profile正确，内参分辨率一致，外参矩阵正交且det≈+1，时间域约定清楚，磁盘空间足够，诊断话题正常。

“det=+1”只证明它像一个旋转矩阵，不证明安装角写对；“相机在线”不证明镜头清洁；“模型可加载”不证明当前图像类别映射正确。形式检查与实物检查必须分开。

## 65.3 不在比赛现场自动更新依赖

固定经过验证的版本，保留回滚包。若新的硬件或模型要求更新环境，应先在独立环境重做回归，不直接覆盖可工作的ROS、SDK和NumPy组合。

训练与机载推理环境可以不同，但导出一致性测试要跨这两个环境执行。记录Python/框架/设备信息，才能解释数值或性能差异。[R35][R37][R52]

<a id="ch-66"></a>

# 66. 团队可以直接使用的三张记录表

## 66.1 标定记录

| 字段 | 填写内容 |
|---|---|
| 相机/序列号/镜头模式 | 实际值，不能用“D435i默认”代替 |
| 图像尺寸/编码/是否校正 | 明确K/P适用关系 |
| 标定日期/操作者/代码版本 | 可追溯 |
| 标定板实际尺寸 | 米，注明内角点数量 |
| 训练标定重投影误差 | 每幅与总体 |
| 独立图像误差 | 不参与拟合的视角 |
| R_BC/t_BC/参考原点 | 方向、单位、来源 |
| 各高度各姿态三维误差 | bias/RMSE/P95 |
| 本次机械改动 | 是否需要重标定 |

## 66.2 模型评估记录

| 字段 | 填写内容 |
|---|---|
| 权重hash/类别表 | 与发布包一致 |
| 数据session划分 | 证明无相邻帧泄漏 |
| 各类precision/recall | 包括最差场景 |
| 桶中心误差 | 像素与米分别报告 |
| 直径混淆 | 15/20/25cm的判别错误 |
| 拒识比例 | 不用只报成功帧精度 |
| 处理帧率与帧龄 | P50/P95/P99 |
| 实际硬件与温度状态 | 冷机/热机 |
| 已知失败样例 | 数量、路径和影响 |

## 66.3 O4侦察记录

| 字段 | 填写内容 |
|---|---|
| 设备型号/固件/显示端 | 必须准确 |
| 实时模式/录像模式 | 分开记录 |
| 安装视轴/EIS/裁切 | 固定配置编号 |
| 相机到标识平面高度 | 不直接用飞控相对高度代替 |
| 目标最远偏距 | 连同桶壁尺寸 |
| 每个目标的可读帧区间 | 时间与图像编号 |
| 速度、roll/pitch峰值 | 实际测量，不只参数值 |
| 正确/错误/空白 | 盲判结果 |
| 航线时间/返航时间 | 分开测量 |
| 失败归因 | 遮挡/模糊/低像素/传输/人员 |

<a id="ch-67"></a>

# 67. 综合实践：从空目录做到可解释视觉系统

## 67.1 十二个阶段性交付

| 阶段 | 交付物 | 通过标准 |
|---|---|---|
| 1 图像基础 | 读写、颜色、ROI小程序 | 能解释shape/dtype/BGR |
| 2 合成桶 | 场景生成与白环检测 | 正确识别合成环，拒绝明显干扰 |
| 3 几何基础 | 投影/反投影/旋转测试 | 非零姿态下往返一致 |
| 4 深度质量 | 无效值与混合表面拒绝 | 不把坏深度包装成高质量目标 |
| 5 实物标定 | 内参和外参报告 | 独立点验证而非仅训练误差 |
| 6 三维桶口 | 采样/拟合/质量可视化 | 解释桶沿、桶底和桶心区别 |
| 7 数据集 | 分组划分、标签审计 | 无已发现的session泄漏 |
| 8 模型 | 训练/导出/对比报告 | 检测、几何、延迟共同评估 |
| 9 时间 | 导航历史与夹逼实验 | 拒绝过大gap、回跳和外推 |
| 10 跟踪 | 静态桶实体与目标计划 | 类别/Track/任务ID分清 |
| 11 ROS | 隔离输入输出和健康状态 | 断流、不匹配、空帧行为可预测 |
| 12 O4与整机 | 实际筒可读覆盖/完整日志 | 路线执行不冒充成功判读 |

这些阶段不要求一开始就使用深度学习；也不要求为了“全面”把每个算法都塞进比赛版本。教学代码可以多，实飞路径应尽量明确、少分支、可回滚。

## 67.2 C++ 对照练习

把几何核心写成与ROS无关的C++17函数，通过同一组输入与Python输出对比。下面给出一个完整最小程序：

**完整文件：`cpp/geometry_test.cpp`。**

```cpp
// C++17 几何练习。无 ROS、相机或 MAVLink 依赖。
#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
using Point = std::array<double, 3>;
constexpr double pi = 3.14159265358979323846;

Point rotate(Point b, double roll, double pitch, double yaw) {
  for (double v : b) if (!std::isfinite(v)) throw std::invalid_argument("NaN point");
  if (!std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
    throw std::invalid_argument("NaN angle");
  const double cr=std::cos(roll), sr=std::sin(roll);
  const double cp=std::cos(pitch), sp=std::sin(pitch);
  const double cy=std::cos(yaw), sy=std::sin(yaw);
  const Point p{b[0], cr*b[1]-sr*b[2], sr*b[1]+cr*b[2]};
  const Point q{cp*p[0]+sp*p[2], p[1], -sp*p[0]+cp*p[2]};
  return {cy*q[0]-sy*q[1], sy*q[0]+cy*q[1], q[2]};
}

int main() {
  auto p=rotate({1,0,0},0,0,pi/2);
  assert(std::abs(p[0])<1e-12 && std::abs(p[1]-1)<1e-12);
  const auto outlet=rotate({0,0,-.32},8*pi/180,0,0);
  assert(std::abs(outlet[1]-.32*std::sin(8*pi/180))<1e-12);
  const Point target{31,2,.30};
  auto offset=rotate({.026,-.065,-.32},5*pi/180,-3*pi/180,.4);
  Point desired{target[0]-offset[0],target[1]-offset[1],1.3};
  assert(std::abs(desired[0]+offset[0]-target[0])<1e-12);
  assert(std::abs(desired[1]+offset[1]-target[1])<1e-12);
  std::cout<<"PASS: yaw, roll lever-arm, XY compensation\n";
  std::cout<<"vehicle Z remains "<<desired[2]<<", not outlet ground clearance\n";
}
```

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -O2 \
  cpp/geometry_test.cpp -o /tmp/cuadc_geometry_test
/tmp/cuadc_geometry_test
```

运行成功只覆盖该程序内的数学断言，不证明整个C++任务节点通过编译或实飞。

## 67.3 最终报告应该回答的六个问题

输入是什么、输出是什么、时间是什么、坐标是什么、何时拒绝、怎样证明。若新人能对这六个问题逐项给出代码位置和测试证据，就已经比“会调一个YOLO命令”更接近能维护比赛系统的水平。

<a id="ch-68"></a>

# 68. 学习验收题与参考答案

## 68.1 基础题

**题1：为什么 `image[v,u]` 不能写成 `image[u,v]`？**

数组先行后列，图像点惯例写u为横坐标、v为纵坐标。反过来会取错像素或越界。

**题2：D435i一个像素深度2m，是否表示它到镜头欧氏距离2m？**

通常是光轴Z深度，不是射线长度；应按具体SDK模型解释。[R08]

**题3：图像从1280缩到640，fx是否不变？**

不变会破坏投影。相应缩放焦距和主点，并考虑所用像素中心/裁切规则。

**题4：一个det=-1的矩阵能不能当作刚体旋转？**

它含反射，不是正常三维旋转；坐标转换中可能漏了轴符号或写错轴顺序。

## 68.2 几何与时间题

**题5：桶中心像素的深度来自桶底，能不能直接当作桶口中心的深度？**

不能无条件这样用。斜视时桶底、桶壁、桶沿和桶口几何中心不同；要明确所求平面和采样来源。

**题6：配准后的深度与RGB是否已经硬件同步？**

空间配准和时间同步是不同问题。配准把点投到同一图像几何，不能消除采样时刻差异。

**题7：把两个接收回调都打主机now，是否表示它们对应相同物理时刻？**

不是。只统一时钟域，各自传输和排队延迟仍可能不同。

**题8：视觉结果慢200ms，为什么不能把stamp改成发布时刻？**

那会把旧观测伪装成新观测，使导航补偿用错状态，并绕过帧龄检查。

**题9：逐项欧拉角插值是否等价于四元数 SLERP？**

不等价。短角差处理解决角度回绕，SLERP 沿单位四元数表示的旋转弧插值；一般三维姿态应避免把前者当作后者。

## 68.3 系统题

**题10：能把 PoseArray 的 orientation.x/y 当作直径和分数吗？**

不能保持标准位姿语义。标准字段是四元数，应把直径与质量放进具名观测字段。仅作点显示时可用单位四元数，并注明未测量朝向。[R56]

**题11：无效结果能只靠 confidence=0 表达吗？**

应先约定分数与无效状态的语义。发布端剔除不合法坐标，接收端独立校验；不能把缺失或非法分数自动改为满分。

**题12：第二个目标从任务记忆恢复，能不能增加一帧fresh视觉确认？**

不能。任务记忆不是新采样，应保留原测量时间，并与最新实时观测时间分开记录。

**题13：release_point_error<=20cm能否证明第一落点<=20cm？**

不能。它是模型/状态下的释放几何门，还存在测量、控制、机械与脱离后运动误差。

**题14：roll<=8°且pitch<=8°是否等价于总倾角<=8°？**

不等价。两个轴同时倾斜会产生更大的合成倾角。

## 68.4 O4 与评价题

**题15：O4看到整个8m宽区域，为什么可能看不见边缘筒内标识？**

筒壁遮挡。应检查从标识到相机的视线在桶沿平面是否落入开口，而不只检查相机FOV。

**题16：标签在机载录像里清楚，能否证明地面人员当时也看到相同细节？**

不能。实时流、传输、显示与录像链路可能不同，必须评估实际地面观察证据。

**题17：随机把同一视频帧拆到train和val，为什么验证分数可能虚高？**

相邻帧高度相似，场景、光照与目标位置泄漏；应按session等组划分，再审核近重复。

**题18：神经网络score=.9是否就是90%的正确概率？**

不是。需要独立数据上的校准与可靠性评估，不能直接按该数做收益最优决策。

**题19：12m/s参数设好了，20m转场是否一定1.67s？**

不是。加速、减速、姿态、轨迹约束和实际跟踪都会影响，D/v只是假设全程匀速的数值。

**题20：一个视觉工程何时才算可以交接？**

不仅要有模型和程序，还要有接口契约、标定来源、版本记录、拒绝策略、回归集、实际测量报告和已知限制。能够在陌生机器上复现并解释失败，才接近可维护的交付。


---

<a id="appendix-a"></a>

# 附录 A. 常用函数与数据结构速查

本附录是正文代码中常见语法/API的查阅入口。示例调用不等于所有版本都具有相同参数；原理固定的数学函数与版本相关的库接口分开理解。API来源参见文末对应官方资料。

## A.1 Python 基础

| 写法 | 含义 | 在视觉里的用途 |
|---|---|---|
| `def f(x):` | 定义函数 | 把投影、测深与ROS回调分开 |
| `return value` | 返回结果 | 让算法不依赖全局变量 |
| `class` | 定义对象类型 | 节点、跟踪器、配置 |
| `@dataclass` | 自动生成数据类基本方法 | 明确一帧和一个观测的字段 |
| `@dataclass(frozen=True)` | 禁止直接重设字段 | 减少帧包被意外修改，但不冻结数组内容 |
| `if/elif/else` | 条件分支 | 有效性和故障状态 |
| `for` | 迭代 | 每个目标、每个轮廓点 |
| `enumerate(items)` | 同时得到索引与元素 | 关联检测和mask |
| `zip(a,b)` | 配对迭代 | 点与深度、预测与真值 |
| `try/except/finally` | 异常与资源收尾 | 拒绝坏输入并关闭相机 |
| `raise ValueError(...)` | 显式拒绝不合要求的值 | NaN、错误shape、退化几何 |
| `with open(...)` | 自动关闭文件 | 读取manifest与标签 |
| `if __name__ == '__main__'` | 区分直接运行与导入 | 工具脚本不在import时启动设备 |
| `list` | 有序集合 | 每帧目标列表 |
| `dict` | 键值映射 | JSON字段、诊断信息 |
| `set` | 无重复集合 | 已配对检测索引 |
| `tuple` | 不可变序列 | 三维坐标输入 |
| `deque(maxlen=n)` | 有长度上限的队列 | 直径滑动窗口 |
| `Path` | 文件路径对象 | 路径存在性和目录管理 |
| `argparse` | 命令行参数 | 不把模型与数据路径写死 |
| `json.dumps` | 编码JSON字符串 | 可审计的观测日志 |
| `csv.DictReader` | 按列名读取CSV | 数据集session审计 |
| `hashlib.sha256` | 计算摘要 | 模型、配置、图像完整性与重复检查 |

类型注解不是自动运行时校验。`x: float` 不会自动拒绝字符串，`np.ndarray` 不会自动保证shape；关键入口仍需检查。

## A.2 NumPy：先记住维度，再记函数名

| API | 输入/输出要点 | 常见错误 |
|---|---|---|
| `np.asarray(x,dtype=...)` | 转数组，可共享底层内存 | 当成必然拷贝 |
| `.copy()` | 独立数组副本 | 跨帧共享可变SDK缓冲 |
| `.shape` | 各维长度 | HWC和CHW混淆 |
| `.dtype` | 元素类型 | uint16深度转uint8丢数据 |
| `.astype(np.float64)` | 类型转换 | 忘记乘深度scale |
| `.reshape(...)` | 改变形状解释 | 元素顺序并未按几何自动重排 |
| `.transpose(...)` | 调换维度 | 与旋转图像混淆 |
| `np.isfinite(x)` | 拒绝NaN/Inf | 只检查`x!=0` |
| `np.clip(x,a,b)` | 限幅 | 用限幅掩盖严重错误输入 |
| `np.mean/median` | 均值/中位数 | 混合表面也会有稳定中位数 |
| `np.percentile` | 分位数 | 样本少时把P99当可靠尾部估计 |
| `np.eye(3)` | 单位矩阵 | 用单位外参当真实标定 |
| `np.column_stack` | 按列拼接 | 列顺序与XYZ定义不一致 |
| `@` | 矩阵乘法 | 与逐元素`*`混淆 |
| `.T` | 转置 | 只有旋转矩阵才能用转置代替逆 |
| `np.linalg.norm` | 向量/矩阵范数 | XY距离与XYZ距离混淆 |
| `np.linalg.det` | 行列式 | det为1仍不保证矩阵正交 |
| `np.linalg.svd` | 奇异值分解 | 不检查秩与退化 |
| `np.linalg.lstsq` | 最小二乘 | 以为自动抗所有离群点 |
| `np.allclose` | 容差比较 | 使用无单位、不合理的容差 |
| `np.save/load` | 保存/读取NPY | 加载不可信对象数组 |
| `np.random.default_rng(seed)` | 局部随机生成器 | 把固定种子当跨环境完全复现保证 |

本教程的核心数学统一使用float64，便于分析；部署模型输入常用float32或其他精度，二者不应在不说明单位和用途的情况下互相覆盖。[R04]

## A.3 OpenCV 图像处理

| API | 主要用途 | 本任务注意点 |
|---|---|---|
| `cv2.imread` | 读图片 | 失败返回None，常见彩色为BGR |
| `cv2.imwrite` | 写图片 | 检查返回值和目录权限 |
| `cv2.cvtColor` | 颜色空间转换 | BGR→HSV与RGB→HSV不同 |
| `cv2.inRange` | 区间阈值 | HSV阈值依赖实际图像表示 |
| `cv2.threshold` | 全局阈值 | 强阴影下可能失效 |
| `cv2.adaptiveThreshold` | 局部阈值 | 参数不应跨尺度盲用 |
| `cv2.GaussianBlur` | 平滑 | 会改变细桶沿边缘 |
| `cv2.morphologyEx` | 开闭运算 | 不要填没桶沿内部几何 |
| `cv2.erode/dilate` | 腐蚀/膨胀 | 核大小对应像素尺度 |
| `cv2.Canny` | 边缘检测 | 边缘不是自动可靠的实体 |
| `cv2.findContours` | 轮廓与层级 | 外轮廓、孔洞语义不同 |
| `cv2.contourArea` | 轮廓面积 | 面积大不一定是桶口 |
| `cv2.arcLength` | 周长 | 受锯齿与采样影响 |
| `cv2.moments` | 图像矩 | 质心不必等于透视圆心 |
| `cv2.fitEllipse` | 椭圆拟合 | 最少点数、截断、短弧退化 |
| `cv2.minEnclosingCircle` | 最小包围圆 | 不是一般透视下真实桶圆 |
| `cv2.HoughCircles` | 圆候选 | 侧视椭圆与噪声会影响 |
| `cv2.fillPoly` | 栅格化多边形 | 填充桶口与桶沿不是同一mask |
| `cv2.polylines/circle/putText` | 调试叠加 | 不在叠加图上再做原始测量 |
| `cv2.resize` | 重采样 | 同步更新内参/坐标还原 |
| `cv2.Laplacian` | 二阶梯度 | 方差可辅助清晰度比较，不是通用判读保证 |

资料按功能分组参见阈值、颜色、形态学、轮廓与特征教程。[R07][R17][R19][R20][R21]

## A.4 OpenCV 标定与投影

| API | 输出代表什么 | 使用前提 |
|---|---|---|
| `findChessboardCorners` | 棋盘内角点像素 | pattern是内角点数量 |
| `cornerSubPix` | 亚像素角点细化 | 初值与图像质量合格 |
| `calibrateCamera` | 针孔模型K/D及各视角外参 | 准确板尺寸与丰富视角 |
| `projectPoints` | 物点投到图像 | rvec/tvec方向正确 |
| `Rodrigues` | 旋转向量与矩阵互转 | 不是三个普通欧拉角 |
| `solvePnP` | 常见语义为物体/世界→相机变换 | 正确3D-2D对应与内参 |
| `undistortPoints` | 校正点坐标 | 无P时通常返回归一化坐标 |
| `initUndistortRectifyMap` | 图像校正映射 | 输出投影模型要记录 |
| `remap` | 按映射重采样 | 深度插值不能忽略无效值和边缘 |
| `fisheye.calibrate` | 鱼眼模型标定 | 与普通针孔D模型不同 |
| `getPerspectiveTransform` | 四点单应矩阵 | 点序与平面假设正确 |
| `warpPerspective` | 单应重采样 | 无法恢复遮挡信息 |

`solvePnP` 返回的不是“飞机位置”的万能答案。它相对于所定义标定板/物体坐标系，必须经过坐标链解释才能得到机体外参。[R13][R14][R15][R16][R42]

## A.5 RealSense SDK 速查

| API | 作用 | 不能推断什么 |
|---|---|---|
| `rs.context()` | 枚举SDK设备 | 不代表流已启动 |
| `query_devices()` | 取得设备列表 | 多设备时不要默认第一个是目标 |
| `rs.pipeline()` | 流处理入口 | 不要每帧重新创建 |
| `rs.config()` | 指定流配置 | 配置不一定被设备支持 |
| `enable_device(serial)` | 固定设备 | 序列号要来自实际机器 |
| `enable_stream(...)` | 请求尺寸/格式/帧率 | 请求值不等于实际成功值 |
| `pipeline.start(config)` | 启动并返回profile | 核对实际profile |
| `wait_for_frames(timeout_ms)` | 等待帧组 | 返回时刻不是曝光时刻 |
| `get_color_frame/get_depth_frame` | 取得对应帧 | 先检查是否有效 |
| `rs.align(rs.stream.color)` | 深度空间对齐到颜色视图 | 不等于时间同步 |
| `align.process(frames)` | 产生对齐后的帧组 | 需确认输出内参及空洞 |
| `get_data()` | 取像素缓冲 | 检查内存生命周期 |
| `get_depth_scale()` | 原始深度到米的比例 | 不写死为任意常数 |
| `get_timestamp()` | 当前帧时间值 | 还要看时间域和具体字段定义 |
| `get_frame_timestamp_domain()` | 时间域标识 | 不自动等于ROS clock |
| `get_frame_number()` | 帧编号 | 重启可能归零 |
| `get_intrinsics()` | 对应视频profile内参 | 不拿另一流的内参代用 |
| `get_extrinsics_to(...)` | SDK定义的流间外参 | 不是相机到飞机的安装外参 |
| `rs2_deproject_pixel_to_point` | 像素/Z反投影 | 输入intrinsics模型必须匹配 |
| `pipeline.stop()` | 停止流并释放资源 | 放在finally里确保执行 |

实际支持项以设备profile、SDK版本与官方文档为准。[R06][R08][R11][R12]

## A.6 模型与ROS接口

| API/字段 | 教学用途 | 注意 |
|---|---|---|
| `YOLO(local_path, task='segment')` | 加载可信本地模型 | 不默认已有真实比赛权重 |
| `.train(...)` | 训练 | 在独立训练环境执行 |
| `.predict(...)` | 推理 | 核验原图坐标与预处理 |
| `.val(...)` | 评价 | 指标不能代替三维误差 |
| `.export(...)` | 后端导出 | 导出后重新比对输出 |
| `.boxes.xyxy/cls/conf` | 框/类别/分数 | 类别不是实体ID |
| `.masks.xy/data` | 多边形/掩膜张量 | 两者坐标分辨率不一定相同 |
| `Node.create_subscription` | ROS订阅 | 与发布端QoS兼容 |
| `Node.create_publisher` | ROS发布 | 话题命名不自动确保安全 |
| `CvBridge.imgmsg_to_cv2` | ROS→数组 | 指定编码，深度保持原表示 |
| `CvBridge.cv2_to_imgmsg` | 数组→ROS | 补齐真实stamp和frame |
| `ApproximateTimeSynchronizer` | 配对相近消息时间 | 不做物理同步/空间配准 |
| `get_clock().now()` | ROS当前时间 | 与monotonic不可直接相减 |
| `create_timer` | 周期诊断/处理 | 不用Timer伪造传感器新帧 |

参考模型模式与ROS对应消息/同步接口。[R18][R22][R25][R26][R27][R31][R32][R34][R37]

<a id="appendix-b"></a>

# 附录 B. 视觉接口与参数设计速查

## B.1 每份观测必须回答的八个问题

| 问题 | 建议记录 |
|---|---|
| 测量对象是什么？ | 桶口中心、地面点、轮廓或其他明确几何对象 |
| 坐标相对谁？ | frame、参考原点与轴向 |
| 单位是什么？ | 像素、米、秒与弧度分别标注 |
| 对应何时？ | 原始测量时间、时间源与时钟域 |
| 来自哪一帧？ | 原始帧号或单调帧序列 |
| 如何得到？ | 深度、平面投影、三维圆拟合等方法 |
| 有多可靠？ | 有效比例、拟合残差与可解释不确定性 |
| 失败如何表达？ | 帧状态、拒绝原因与诊断计数 |

## B.2 参数按用途分组

| 参数组 | 选择依据 | 验证方法 |
|---|---|---|
| 标定与外参 | 分辨率、镜头模型、物理安装 | 独立点与姿态验证 |
| 时间门 | 测量延迟分布、导航频率、动态误差预算 | 旧帧、未来帧、间隙与时钟跳变 |
| 深度门 | 有效比例、表面混合、传感器范围 | 缺深度、遮挡、双表面与反光样本 |
| 关联门 | 目标间距、定位与尺寸误差 | 接近目标、重复轮廓和位置突变 |
| 鲁棒窗口 | 有效帧频率、观测驻留时长 | 噪声、离群点、丢帧和机动 |
| 动作前条件 | 几何、运动、身份与证据年龄 | 台架和影子接收器，不直接驱动机构 |

正文中出现的数字只定义相应练习。选择参数时，先列需要拒绝的负例，再确认正常数据的通过率；不能把调宽门限当作提高定位精度。

## B.3 各层职责

| 层级 | 主要职责 |
|---|---|
| 采集 | 原始帧、profile、帧号、测量时间与时间域 |
| 几何 | 畸变、配准、相机到机体外参与几何点定义 |
| 导航对齐 | 位置/姿态独立历史与测量时刻插值 |
| 目标管理 | 一对一关联、质量统计、选择与受约束重捕获 |
| 任务接口 | 证据年龄、动作条件、超时与失败路径 |
| 物理验收 | 机构状态、真实落点、可读标识与独立记录 |

## B.4 一页接收器日志

建议每条记录包含：原始帧号、测量时间、接收时间、使用年龄、位置与姿态插值间隔、输入/输出 frame、目标 ID、几何方法、质量值、拒绝原因和最终生效配置 ID。保存中间层证据比只保存一个 `target_found=true` 更便于排错。

<a id="appendix-c"></a>

# 附录 C. 给零基础读者补齐数学与深度学习

## C.1 点、向量与坐标变换

点有位置，向量常表示位移。相机点转换到机体时有旋转和平移；一个方向向量只旋转、不加平移。

齐次变换：

$$
T^A_B=\begin{bmatrix}R^A_B&t^A_B\\0&1\end{bmatrix},\quad
\begin{bmatrix}p^A\\1\end{bmatrix}=T^A_B\begin{bmatrix}p^B\\1\end{bmatrix}.
$$

方向向量可写末位0，因此不会受到平移影响。链式变换的下标应能消去：`T_WB T_BC` 得到 `T_WC`，次序不能交换。

## C.2 为什么SVD能拟合平面

把点减去质心后组成矩阵A；寻找单位法向n，使 `||An||²` 最小，即所有点到该法向所定义平面的投影残差最小。SVD中最小奇异值对应的方向提供该最小二乘解。

若点几乎共线，法向并不唯一；若所有点来自桶壁和桶沿混合，最小二乘仍会给一个结果，但物理表面未必正确。因此要检查奇异值、语义区域、内点比例和残差。

## C.3 RANSAC解决离群点，不解决语义错误

RANSAC的基本过程：随机选最少样本拟合→计算符合残差门的内点→保留支持度较好的模型→用内点重新拟合。

平面最少需要3个不共线点；拟合时要拒绝退化样本。若每次采样均匀、内点比例为w、最少样本数s，N次都没抽到纯内点组合的理想概率为 `(1-w^s)^N`。这是简化概率模型，不能覆盖空间相关噪声和多平面歧义。

当桶底点比桶沿点多时，RANSAC可能很成功地拟合了**错误的目标表面**。因此它必须配合ROI语义、法向、高度与物理尺寸约束，不能放在任何深度点集上就叫鲁棒定位。

正文中的 `fit_plane_svd/fit_circle_3d` 是最小二乘教学函数，不包含完整RANSAC。真实部署应补充并验证离群点策略，而不是误读函数名。

## C.4 椭圆中心为什么不总是圆的投影中心

圆在一般透视下映射为二次曲线；投影不是保持所有中点与距离的欧氏变换。图像椭圆的代数中心不一定等于三维圆心投影。

近正视、目标较小、透视较弱时，把椭圆中心当作近似像素中心可能可用，但应测偏差。精细定位可考虑恢复对应平面，或直接对可靠桶沿点做三维拟合。

如果只能看到很短圆弧，多组不同圆都可能解释这些点，拟合残差很小也不保证圆心稳定。应检查可见角覆盖和参数稳定性，而不只看RMS。

## C.5 神经网络到底在学什么

给定图像x和标注y，网络 `f_theta(x)` 通过参数theta计算预测。训练优化一个损失L，让预测与标注更一致；推理只计算预测，不更新参数。

卷积使用局部权重在图像上滑动，形成特征图；非线性激活让多层组合不退化为单次线性变换；下采样扩大感受野但减少空间分辨率；多尺度特征有助于处理不同大小目标。这些是理解模型的基础，不意味着每一代YOLO内部结构完全相同。

检测器可能分别预测位置、类别与其他质量量；分割器还要产生像素/实例区域。不要把“网络输出一个分数”理解成“直接输出经过标定的物理尺寸”。[R18][R22]

## C.6 loss、梯度与反向传播

以两类分类为例，logits记为z，softmax：

$$
p_i=\frac{e^{z_i}}{\sum_j e^{z_j}},\qquad L=-\log p_y.
$$

实际计算先减去最大logit避免溢出。链式法则把损失对输出的导数逐层传回参数；梯度下降用 `theta←theta-lr*gradient` 更新。

训练损失下降可能只是记住训练集。检测/分割任务还涉及位置与区域损失，具体组合随架构与版本变化，应按固定实现确认，而不是背一套所有YOLO通用公式。

## C.7 一个完全不依赖PyTorch的训练实践

下面的两层网络只区分合成“+”与“X”，用NumPy手工写前向、交叉熵和反向传播。它不是危险品分类模型，没有实际比赛标签数据。

**完整文件：`tools/toy_classifier.py`。**

```python
"""NumPy 两层分类器教学：只区分合成 + / X，不是危险品模型。"""
import argparse
import numpy as np


def dataset(n: int, seed: int):
    if n < 2:
        raise ValueError('n must be >= 2')
    rng = np.random.default_rng(seed)
    labels = rng.integers(0, 2, n)
    images = np.zeros((n, 8, 8), dtype=np.float64)
    for i, label in enumerate(labels):
        if label == 0:
            images[i, 3:5, 1:7] = 1
            images[i, 1:7, 3:5] = 1
        else:
            for j in range(1, 7):
                images[i, j, j] = 1
                images[i, j, 7-j] = 1
        images[i] = np.roll(images[i], rng.integers(-1, 2), axis=0)
        images[i] = np.roll(images[i], rng.integers(-1, 2), axis=1)
    images += rng.normal(0, .10, images.shape)
    return np.clip(images, 0, 1).reshape(n, -1), labels


def fit(x, labels, epochs=120, learning_rate=.3, seed=26):
    if x.ndim != 2 or len(x) != len(labels) or len(x) == 0:
        raise ValueError('invalid dataset shape')
    if not np.isfinite(x).all() or not np.isin(labels, [0, 1]).all():
        raise ValueError('invalid input or labels')
    if epochs < 1 or not np.isfinite(learning_rate) or learning_rate <= 0:
        raise ValueError('invalid training parameters')
    rng = np.random.default_rng(seed)
    w1 = rng.normal(0, .12, (x.shape[1], 16)); b1 = np.zeros(16)
    w2 = rng.normal(0, .12, (16, 2)); b2 = np.zeros(2)
    onehot = np.eye(2)[labels]
    losses = []
    for _ in range(epochs):
        hidden_pre = x@w1+b1
        hidden = np.maximum(hidden_pre, 0)
        logits = hidden@w2+b2
        logits -= logits.max(axis=1, keepdims=True)
        exp = np.exp(logits)
        probability = exp/exp.sum(axis=1, keepdims=True)
        losses.append(float(-np.mean(np.log(np.maximum(probability[np.arange(len(x)), labels], 1e-12)))))
        # 交叉熵 + softmax 对 logits 的导数。
        grad_logits = (probability-onehot)/len(x)
        grad_w2 = hidden.T@grad_logits; grad_b2 = grad_logits.sum(axis=0)
        grad_hidden = (grad_logits@w2.T)*(hidden_pre > 0)
        grad_w1 = x.T@grad_hidden; grad_b1 = grad_hidden.sum(axis=0)
        # 全部梯度由本轮旧参数计算，之后统一更新。
        w1 -= learning_rate*grad_w1; b1 -= learning_rate*grad_b1
        w2 -= learning_rate*grad_w2; b2 -= learning_rate*grad_b2
    return (w1, b1, w2, b2), losses


def predict(x, weights):
    w1, b1, w2, b2 = weights
    return (np.maximum(x@w1+b1, 0)@w2+b2).argmax(axis=1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--epochs', type=int, default=120)
    args = parser.parse_args()
    x, y = dataset(256, 26)
    xt, yt = dataset(128, 27)
    weights, losses = fit(x, y, epochs=args.epochs)
    print('SYNTHETIC TOY ONLY: + versus X; not CUADC label accuracy')
    print('loss first/last:', round(losses[0], 6), round(losses[-1], 6))
    print('synthetic test accuracy:', float(np.mean(predict(xt, weights) == yt)))


if __name__ == '__main__':
    main()
```

```bash
python -m tools.toy_classifier
```

先读`dataset`了解输入，再读`fit`中的前向与梯度，最后看`predict`。把训练标签打乱，观察loss与测试表现；把测试噪声加大，观察分布变化；将隐藏层改为1个神经元，观察容量限制。

即使本合成任务达到100%测试正确率，也不能推断任何真实危险标识准确率。生成器只有很有限的形状、位移和噪声，远远没有比赛中遮挡、视角、光照与压缩的复杂性。

## C.8 过拟合、数据偏差与类别不平衡

若所有小桶图片都拍在蓝色地面，大桶都拍在水泥地，模型可能学习背景而非尺寸。若某类只有几十帧且来自同一个视频，帧数看起来多，独立变化仍然少。

对策是改采样设计、补独立场景、平衡工况，并用分组测试验证。提高epochs、调整confidence或更换大模型，不会自动消除这种偏差。

## C.9 统计报告中的三个陷阱

只报告成功帧误差而不报告拒绝率，会高估系统可用性；只报平均误差而不报长尾，会掩盖偶发错误；把相关连续帧当独立样本，会夸大统计把握。

应把“观测有效比例”和“有效观测精度”一起报，把“实体发现率”和“单帧检测率”分开报，把“不同摆放/不同飞行架次”作为重要独立分组。

<a id="appendix-d"></a>

# 附录 D. 正文练习代码的使用地图

## D.1 工具与输入条件

| 工具 | 是否需要真实设备/模型 | 产物 |
|---|---|---|
| `tools.make_scene` | 否 | 合成RGB/深度/内参/真值 |
| `tools.classical_detect` | 否，默认读合成场景 | 桶候选、米制近似、调试叠加 |
| `tools.geometry_demo` | 否 | 坐标与投放口计算 |
| `tools.timing_demo` | 否 | 合法插值与拒绝案例 |
| `tools.tracking_demo` | 否 | 多实体关联/三桶选二 |
| `tools.planning_demo` | 否 | FOV/模糊/速度/遮挡计算 |
| `tools.toy_classifier` | 否 | 合成分类训练loss与测试结果 |
| `tools.dataset_audit` | 需要自己的图片标签清单 | 格式、重复、session泄漏报告 |
| `tools.quality_report` | 图片目录 | 清晰度辅助指标 |
| `tools.calibrate_chessboard` | 需要真实标定板图片 | K/D、重投影误差报告 |
| `tools.capture_realsense` | 需要相机和SDK | 只读RGB-D与时间/profile信息 |
| `tools.yolo_predict` | 需要可信本地分割权重 | 原图尺度目标JSON |
| `tools.train_segment` | 需要训练环境、数据与初始权重 | 实际训练产物 |
| ROS合成相机/视觉节点 | 需要ROS2但不需要相机 | 隔离话题与健康诊断 |
| C++几何小程序 | 需要C++17编译器 | 数学断言测试 |

## D.2 从空环境开始的离线执行顺序

```bash
cd cuadc_vision_lab
python3 -m venv .venv
source .venv/bin/activate
python -m pip install numpy opencv-python pytest

python -m pytest -q
python -m tools.make_scene
python -m tools.classical_detect
python -m tools.geometry_demo
python -m tools.timing_demo
python -m tools.tracking_demo
python -m tools.planning_demo
python -m tools.toy_classifier
python -m tools.quality_report --images outputs/scene
```

按正文路径保存脚本、建立两个空 `__init__.py` 后，所有 `python -m tools...` 命令在自己的练习根目录执行。不要直接把某一个脚本拷到别处后忽略它对核心模块的依赖。

## D.3 清晰度辅助工具完整代码

**完整文件：`tools/quality_report.py`。**

```python
"""只输出图像质量代理指标；没有跨相机通用的“合格模糊阈值”。"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from cuadc_vision_lab.detection import blur_score


def main():
    p = argparse.ArgumentParser(); p.add_argument('--images', required=True)
    args = p.parse_args(); rows = []
    for path in sorted(Path(args.images).glob('*')):
        if path.suffix.lower() not in {'.png', '.jpg', '.jpeg'}:
            continue
        image = cv2.imread(str(path))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        rows.append({'file': str(path), 'width': image.shape[1], 'height': image.shape[0],
                     'laplacian_variance': blur_score(image),
                     'dark_ratio': float((gray <= 5).mean()),
                     'bright_ratio': float((gray >= 250).mean())})
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
```

Laplacian方差只是一个受尺寸、纹理和噪声影响的辅助量，不能用固定阈值跨所有场景证明标签可读。对同一个目标、相似尺度和图像处理链，它更适合相对比较。

## D.4 常见运行错误

`ModuleNotFoundError: cuadc_vision_lab`：确认在根目录运行，或正确安装包。

`No module named cv2/numpy`：确认当前Python解释器与安装依赖的解释器相同。

`No module named rclpy`：纯算法环境不是ROS环境；在ROS终端source发行版，不直接全局pip安装替代ROS二进制依赖。

`No module named pyrealsense2/ultralytics`：这两个是可选设备/模型步骤，不影响离线几何和合成练习。按固定环境官方说明安装，不把未装依赖视为算法错误。[R06][R18]

`找不到weights`：程序故意不自动下载替代模型；提供可信本地路径。

## D.5 文件授权与用途

整理课堂练习时，保存引用来源及第三方库、模型与数据的许可信息。公开发布自己制作的数据或模型前，应分别确认对应材料的使用条件。

<a id="appendix-e"></a>

# 附录 E. 中英文术语表

| 术语 | 中文含义 | 本教程中的关键区别 |
|---|---|---|
| Pixel | 像素 | 不等于物理厘米 |
| Intrinsics | 内参 | 描述相机成像，不描述安装位置 |
| Extrinsics | 外参 | 必须说明从哪个系到哪个系 |
| Optical frame | 光学坐标系 | 常见右、下、前，不是FLU |
| FLU | 前、左、上 | 机体坐标约定 |
| ENU | 东、北、上 | 本地世界方向约定 |
| Field frame | 比赛场地坐标系 | +X由任务航向锁定 |
| Distortion | 畸变 | 不能靠改姿态外参替代 |
| Rectification | 图像校正 | 与深度颜色配准不同 |
| Registration/Alignment | 空间配准 | 与时间同步不同 |
| Timestamp domain | 时间戳所属时钟域 | 相同单位不代表相同原点 |
| Exposure time | 曝光时长 | 与帧周期不同 |
| Latency | 延迟 | 与吞吐/FPS不同 |
| Observation age | 观测年龄 | 可以在高FPS下仍然很大 |
| ROI | 感兴趣区域 | 要区分检测区域与测量表面 |
| Mask | 掩膜 | 语义取决于标注，可能不保留孔洞 |
| Detection | 单帧检测 | 不是持久实体 |
| Track | 跟踪实体假设 | 不等于官方桶号 |
| Association | 数据关联 | 不是简单按数组索引对应 |
| Reacquisition | 重捕获 | 不应忽略原目标身份门 |
| EMA | 指数移动平均 | 平滑引入滞后 |
| MAD | 中位数绝对偏差 | 不自动等于标准差 |
| RANSAC | 随机采样一致性 | 抗离群，不自动识别正确表面 |
| SVD | 奇异值分解 | 可分析最小二乘与退化 |
| Reprojection error | 重投影误差 | 小不代表外部三维误差一定小 |
| Occlusion | 遮挡 | 信息可能根本未被相机观测 |
| Lever arm | 杠杆臂 | 相机/投放口相对机体的偏置 |
| Rigid-body compensation | 刚体几何补偿 | 不等于释放后弹道补偿 |
| Ground truth | 独立真值 | 不能来自同一估计链自证 |
| Domain shift | 数据分布变化 | 实验室与飞行场景不同 |
| Leakage | 数据泄漏 | 相邻帧跨训练验证集 |
| Abstention | 拒识/不作判断 | 不是把随机猜测写成低分结果 |
| Calibration of confidence | 概率校准 | 不等于相机标定 |
| ACK | 命令确认 | 不等于物理效果证明 |
| Safety gate | 安全门禁 | 不是传感器精度保证 |
| Profile | 流/运行配置 | 改配置可能使旧标定不适用 |
| Regression test | 回归测试 | 防止修新问题破坏旧功能 |

<a id="appendix-f"></a>

# 附录 F. 练习验证方法与记录

按下表记录自己实际执行的验证。教材给出测试输入和预期行为，不提供替代本机执行记录的历史通过数字。

| 层级 | 至少验证什么 | 本机结果与证据 |
|---|---|---|
| 环境 | Python、NumPy、OpenCV、pytest 和编译器版本 | 待填写 |
| 数学核心 | 往返投影、变换、拟合、退化与单位 | 待填写 |
| 时间 | 精确命中、合法夹逼、间隙、回退和过期 | 待填写 |
| 关联 | 一对一、不同帧计数、断流重置与实体独立性 | 待填写 |
| 合成场景 | 真值与估计差异、噪声和遮挡 | 待填写 |
| ROS | 构建、类型、frame、QoS、配对与断流 | 待填写 |
| 相机 | 元数据支持、RGB-D 偏差、深度和独立标定验证 | 待填写 |
| 模型 | 数据划分、实际训练、导出一致性与独立评估 | 待填写 |
| 真实场景 | 真实落点、图传可读性和人员/模型判读质量 | 待填写 |

合成数值检查只能验证在其假设下的算法行为。硬件、ROS 运行、模型与实景应分别验收，不把其中一项成功推广到其他层级。

## F.1 测试源码

将下面代码保存到自己的练习目录，执行 `python -m pytest -q`，并记录实际结果。出现失败时先核对输入语义和预期，不能为了得到绿色输出删除边界案例。

**完整文件：`tests/test_core.py`。**

```python
import math
import numpy as np
import pytest
from cuadc_vision_lab.geometry import *
from cuadc_vision_lab.depth import *
from cuadc_vision_lab.timing import *
from cuadc_vision_lab.tracking import *
from cuadc_vision_lab.planning import *
from cuadc_vision_lab.detection import *
from tools.dataset_audit import validate_label

K = Intrinsics(640, 480, 600, 600, 319.5, 239.5)


def test_projection_roundtrip():
    p = np.array([.2, -.1, 2.])
    assert np.allclose(deproject(project(p, K), p[2], K), p)


def test_depth_not_range():
    p = deproject([619.5, 239.5], 2., K)
    assert p[2] == 2 and np.linalg.norm(p) > 2


@pytest.mark.parametrize('z', [0., -1., math.nan, math.inf])
def test_bad_depth(z):
    with pytest.raises(ValueError):
        deproject([100, 100], z, K)


def test_resize_pixel_center():
    k = resize_intrinsics(K, 320, 240)
    assert k.fx == 300 and k.cx == 159.5 and k.cy == 119.5


def test_bad_intrinsics():
    with pytest.raises(ValueError):
        Intrinsics(640, 480, 0, 600, 320, 240)


def test_rotation_norm_and_inverse():
    r = rpy_matrix(.1, -.2, .3); p = [.2, .3, 2.]; t = [.1, 0, -.1]
    assert np.allclose(inverse_transform(transform(p, r, t), r, t), p)


def test_reflection_rejected():
    with pytest.raises(ValueError):
        validate_rotation(np.diag([1., 1., -1.]))


def test_plane_intersection():
    q = ray_plane_intersection([0, 0, 3], [.2, 0, -1], [0, 0, 1], -.3)
    assert np.allclose(q, [.54, 0, .3])


def test_parallel_ray_rejected():
    with pytest.raises(ValueError):
        ray_plane_intersection([0, 0, 3], [1, 0, 0], [0, 0, 1], 0)


def test_behind_ray_rejected():
    with pytest.raises(ValueError):
        ray_plane_intersection([0, 0, 3], [0, 0, 1], [0, 0, 1], 0)


def test_fit_plane():
    n, d, rms = fit_plane_svd([[0, 0, 2], [1, 0, 2], [0, 1, 2], [1, 1, 2]])
    assert np.allclose(n, [0, 0, 1]) and d == -2 and rms < 1e-12


def test_degenerate_plane():
    with pytest.raises(ValueError):
        fit_plane_svd([[0, 0, 0], [1, 0, 0], [2, 0, 0]])


def test_circle_3d():
    theta = np.linspace(0, 2*np.pi, 100, endpoint=False)
    p = np.c_[.1*np.cos(theta), .1*np.sin(theta), np.zeros(100)]
    r = rpy_matrix(.3, .1, .2)
    p = p@r.T + [1, 2, 3]
    c, radius, rms = fit_circle_3d(p)
    assert np.allclose(c, [1, 2, 3]) and abs(radius-.1)<1e-8 and rms<1e-8


def test_release_xy():
    r = rpy_matrix(.1, .2, .3); b = np.array([.026, -.065, -.32])
    _, desired = release_geometry([0, 0, 1.3], r, b, [31, 2, .3], 1.3)
    assert np.allclose((desired+r@b)[:2], [31, 2]) and desired[2] == 1.3


def test_covariance_psd():
    cov = deprojection_covariance([400, 280], 2., K, 1, 1, .02)
    assert (np.linalg.eigvalsh(cov) >= -1e-12).all()


def test_depth_scale():
    assert to_meters(np.ones((2, 2), np.uint16)*2000, .001)[0, 0] == 2.


def test_depth_robust():
    a = np.ones((10, 10))*2; a[0, 0]=0
    e = robust_depth(a, np.ones(a.shape, bool))
    assert e.z_m == 2 and e.valid_count == 99 and e.valid_ratio == .99


def test_depth_all_invalid():
    with pytest.raises(ValueError):
        robust_depth(np.zeros((10, 10)), np.ones((10, 10), bool))


def test_depth_mixed_surface():
    a = np.ones((10, 10))*2.; a[:5] = 2.3
    with pytest.raises(ValueError, match='MIXED'):
        robust_depth(a, np.ones(a.shape, bool))


def test_history_interpolation():
    h=NavHistory(); h.append(NavSample(1, (0,0,0),(0,0,0)))
    h.append(NavSample(40_000_001, (1,0,0),(0,0,0)))
    assert np.allclose(h.at(20_000_001).xyz, [.5,0,0])


def test_history_wrap_yaw():
    h=NavHistory(); h.append(NavSample(1,(0,0,0),(0,0,math.radians(179))))
    h.append(NavSample(40_000_001,(0,0,0),(0,0,math.radians(-179))))
    assert abs(abs(h.at(20_000_001).rpy[2])-math.pi)<1e-8


def test_exact_sample_and_no_extrapolation():
    h=NavHistory(); h.append(NavSample(1,(0,0,0),(0,0,0)))
    h.append(NavSample(40_000_001,(1,0,0),(0,0,0)))
    assert h.at(1).xyz == (0,0,0)
    assert h.at(40_000_001).xyz == (1,0,0)
    assert h.at(0) is None and h.at(50_000_001) is None


def test_gap_reject():
    h=NavHistory(); h.append(NavSample(1,(0,0,0),(0,0,0)))
    h.append(NavSample(80_000_001,(1,0,0),(0,0,0)))
    assert h.at(40_000_001) is None


def test_history_out_of_order():
    h=NavHistory(); sample=NavSample(1,(0,0,0),(0,0,0)); h.append(sample)
    with pytest.raises(ValueError): h.append(sample)


def test_frame_age():
    assert frame_age_ok(1_200_000_000, 1_000_000_000)
    assert not frame_age_ok(2_000_000_000, 1_000_000_000)
    assert not frame_age_ok(1_000_000_000, 1_100_000_000)


def test_tracking_select_three():
    t=StaticBucketTracker()
    for i in range(3):
        stamp=(i+1)*100_000_000
        obs=[Observation(stamp, (float(j),0.,0.), d, .8) for j,d in enumerate([.15,.20,.25])]
        t.update(obs,stamp)
    selected=t.select_two(stamp)
    assert len(t.tracks)==3 and [x.id for x in selected]==[1,2]


def test_tracking_two_requires_route_end():
    t=StaticBucketTracker()
    for i in range(3):
        stamp=(i+1)*100_000_000
        t.update([Observation(stamp,(0.,0.,0.),.15,.8), Observation(stamp,(1.,0.,0.),.2,.8)],stamp)
    assert len(t.select_two(stamp))==0 and len(t.select_two(stamp,True))==2


def test_tracking_duplicate_frame():
    t=StaticBucketTracker();t.update([],1)
    with pytest.raises(ValueError):t.update([],1)


def test_tracking_gap_resets():
    t=StaticBucketTracker()
    for stamp in [1,100_000_001,900_000_001]:
        t.update([Observation(stamp,(0.,0.,0.),.15,.8)],stamp)
    assert t.tracks[0].confirmations==1


def test_zero_confidence_rejected():
    with pytest.raises(ValueError):Observation(1,(0.,0.,0.),.15,0.)


def test_span_and_pixels():
    assert ground_span(3,90)==pytest.approx(6)
    assert pixels_on_target(.12,6,1920)==pytest.approx(38.4)


def test_motion_blur():
    assert blur_pixels(960,6,.001,3)==pytest.approx(1.92)


def test_triangular_trajectory():
    assert minimum_rest_to_rest_time(30,12,3)==pytest.approx(2*math.sqrt(10))


def test_cylinder_occlusion():
    assert label_visible_fraction([0,0,3])['surface_fraction_visible']==1.
    assert label_visible_fraction([4,0,3])['surface_fraction_visible']==0.
    assert not label_visible_fraction([4,0,3])['center_visible']


def test_yolo_valid_detect_label():
    assert validate_label('0 0.5 0.5 0.2 0.2', 'detect', 1)==[]


def test_yolo_invalid_label():
    assert validate_label('0 nan .2 .3 .4','detect',1)
    assert validate_label('2 .1 .1 .1 .1','detect',1)
    assert validate_label('0 .01 .01 .5 .5','detect',1)


def test_yolo_valid_segment_label():
    assert validate_label('0 .1 .1 .3 .1 .2 .3','segment',1)==[]


def test_yolo_bad_segment_label():
    assert validate_label('0 .1 .1 .2 .2','segment',1)
    assert validate_label('0 .1 .1 .2 .2 .3 .3','segment',1)


def test_negative_empty_label():
    assert validate_label('', 'segment', 1)==[]


def test_classical_empty():
    assert detect_white_rings(np.zeros((100,100,3),np.uint8))==[]

@pytest.mark.parametrize('call', [lambda: ground_span(math.nan, 100),
                                lambda: blur_pixels(600, math.inf, .001, 3),
                                lambda: minimum_rest_to_rest_time(20, 12, math.nan)])
def test_planning_nonfinite(call):
    with pytest.raises(ValueError):
        call()


def test_tracker_bad_gate():
    with pytest.raises(ValueError):
        StaticBucketTracker(position_gate_m=0)


def test_toy_classifier_learns():
    from tools.toy_classifier import dataset, fit, predict
    x, y = dataset(128, 26)
    weights, losses = fit(x, y, epochs=80)
    assert losses[-1] < losses[0]*.5
    assert np.mean(predict(x, weights) == y) > .9
```

## F.2 从练习结果走向真实验收

完成离线测试后，给每项新增依赖单独建立证据：真实相机需测量内外参和时钟，模型需评估真实数据，图传需检查实际显示链路，动作结果需独立观测。不要把合成矩阵、深度、速度或分类正确率复制成实机标定与任务成绩。

保留失败样本与修复前后的同条件比较，形成可重复实验，而不是仅记录“能运行”。

<a id="appendix-g"></a>

# 附录 G. 来源、版本与延伸阅读

## G.1 学习材料的使用边界

本教程的三维圆筒、两次投放和图传判读场景用于串联知识。示例代码、尺寸和性能数字均服务于对应练习；真实系统需要自己的相机标定、模型、配置与验收证据。

C++ 与 ROS 的基础可结合本目录其他教程学习。阅读顺序可以调整，但视觉观测的坐标、时间、单位、质量和失败语义应保持完整。

## G.2 外部资料

以下资料以公开项目或厂商官方文档为主。安装与运行时按所选发行版和 SDK 版本核对；链接是继续查阅接口的入口，实验结果使用附录 F 记录。

| 编号 | 官方资料 | 核对用途 |
|---|---|---|
| [R01] | RealSense D435i 官方产品资料 | 设备结构与规格；不是本机实测 |
| [R02] | RealSense 深度相机调优 | 成像与深度质量调试 |
| [R03] | DJI O4 Air Unit 官方规格 | 确切机型、FOV、实时/录像模式 |
| [R04] | NumPy 初学者指南 | 数组、形状、类型与基础运算 |
| [R05] | Python venv | 独立环境与环境重建 |
| [R06] | RealSense Python wrapper 官方仓库 | Python SDK入口和安装说明 |
| [R07] | OpenCV inRange 与颜色阈值 | HSV与阈值示例 |
| [R08] | RealSense SDK 投影几何 | 内参、畸变、反投影、外参与深度单位 |
| [R09] | RealSense 无人机深度图改进 | 运动与户外深度场景 |
| [R10] | RealSense ROS 2 wrapper | 相机ROS包装与配置入口 |
| [R11] | RealSense Frame Metadata | 帧元信息和时间字段 |
| [R12] | RealSense Frame Management | 帧持有、缓冲和队列 |
| [R13] | OpenCV calib3d API | 相机标定和几何接口 |
| [R14] | OpenCV fisheye API | 鱼眼模型，与普通D模型区分 |
| [R15] | OpenCV Python 相机标定教程 | 棋盘标定与重投影 |
| [R16] | OpenCV solvePnP | PnP变换方向和求解说明 |
| [R17] | OpenCV 轮廓与形状 API | 轮廓、拟合和形状测量 |
| [R18] | Ultralytics Predict | 高层推理和Results接口 |
| [R19] | OpenCV 形态学操作 | 腐蚀、膨胀、开闭运算 |
| [R20] | OpenCV 阈值操作 | 全局与自适应阈值 |
| [R21] | OpenCV 特征与Hough接口 | 边缘、线圆候选 |
| [R22] | Ultralytics 实例分割任务 | 分割任务与结果接口 |
| [R23] | Ultralytics 检测数据集格式 | 框标签归一化语义 |
| [R24] | Ultralytics 分割数据集格式 | 多边形标签与data YAML |
| [R25] | Ultralytics Train | 训练入口及参数 |
| [R26] | Ultralytics Val | 验证模式与指标 |
| [R27] | ROS cv_bridge 包索引 | ROS图像与OpenCV数组转换入口 |
| [R28] | Python time | monotonic和perf_counter等时钟 |
| [R29] | ROS REP 103 | 标准单位、坐标与轴约定 |
| [R30] | ROS REP 118 | 深度图表示约定 |
| [R31] | sensor_msgs/Image Humble | encoding、step和数据字段 |
| [R32] | sensor_msgs/CameraInfo Humble | K/D/R/P语义 |
| [R33] | ROS 2 Humble QoS 官方文档源码 | 使用官方Humble源码核对兼容性 |
| [R34] | ROS message_filters Humble Python 源码 | Subscriber和近似时间同步接口 |
| [R35] | ONNX Runtime Python 入门 | 运行会话和后端部署 |
| [R36] | Ultralytics 数据增强说明 | 增强项和训练参数 |
| [R37] | Ultralytics Export | 导出模式及格式限制 |
| [R39] | OpenCV VideoCapture | 读取实际支持的视频源 |
| [R42] | OpenCV 几何变换教程 | 缩放、仿射与透视变换 |
| [R43] | PyTorch inference_mode | 推理上下文；eval是另一件事 |
| [R44] | PyTorch CUDA semantics | 异步GPU执行与计时语义 |
| [R45] | OpenVINO Get Started | 推理部署入口；按目标环境核验 |
| [R46] | DJI O4 官方支持 | 设备兼容、模式与操作限制入口 |
| [R47] | Python queue | 有界队列、Full/Empty与线程通信 |
| [R50] | ROS 2 Humble rosbag 官方教程源码 | 录制回放；使用Humble官方源码核对 |
| [R52] | PyTorch Reproducibility | 随机种子与跨环境复现边界 |
| [R53] | scikit-learn GroupShuffleSplit | 按组切分数据，避免session交叉 |
| [R54] | RealSense 后处理滤波 | 滤波链、空间/时间处理与孔洞填充 |
| [R55] | librealsense 帧时间与元数据定义 | 时间域、曝光中点、读出时间与单位 |
| [R56] | ROS 2 Humble Pose 消息定义 | 位置与四元数姿态的标准语义 |

部分 ROS 网页可能要求访问校验。遇到这种情况可查阅官方仓库对应发行版的消息定义、源码与文档；不要仅依据搜索标题判断接口语义。

[R01]: https://realsenseai.com/products/depth-camera-d435i/ "RealSense D435i 官方产品资料"
[R02]: https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/ "RealSense 深度相机调优"
[R03]: https://www.dji.com/o4-air-unit/specs "DJI O4 Air Unit 官方规格"
[R04]: https://numpy.org/doc/stable/user/absolute_beginners.html "NumPy 初学者指南"
[R05]: https://docs.python.org/3/library/venv.html "Python venv"
[R06]: https://github.com/realsenseai/librealsense/tree/master/wrappers/python "RealSense Python wrapper 官方仓库"
[R07]: https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html "OpenCV inRange 与颜色阈值"
[R08]: https://github.com/realsenseai/librealsense/wiki/Projection-in-RealSense-SDK-2.0 "RealSense SDK 投影几何"
[R09]: https://dev.realsenseai.com/docs/depth-map-improvements-for-stereo-based-depth-cameras-on-drones "RealSense 无人机深度图改进"
[R10]: https://dev.realsenseai.com/docs/ros2-wrapper/ "RealSense ROS 2 wrapper"
[R11]: https://dev.realsenseai.com/docs/frame-metadata/ "RealSense Frame Metadata"
[R12]: https://dev.realsenseai.com/docs/frame-management/ "RealSense Frame Management"
[R13]: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html "OpenCV calib3d API"
[R14]: https://docs.opencv.org/4.x/db/d58/group__calib3d__fisheye.html "OpenCV fisheye API"
[R15]: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html "OpenCV Python 相机标定教程"
[R16]: https://docs.opencv.org/4.13.0/d5/d1f/calib3d_solvePnP.html "OpenCV solvePnP"
[R17]: https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html "OpenCV 轮廓与形状 API"
[R18]: https://docs.ultralytics.com/modes/predict/ "Ultralytics Predict"
[R19]: https://docs.opencv.org/4.13.0/d9/d61/tutorial_py_morphological_ops.html "OpenCV 形态学操作"
[R20]: https://docs.opencv.org/4.x/d7/d4d/tutorial_py_thresholding.html "OpenCV 阈值操作"
[R21]: https://docs.opencv.org/4.x/dd/d1a/group__imgproc__feature.html "OpenCV 特征与Hough接口"
[R22]: https://docs.ultralytics.com/tasks/segment/ "Ultralytics 实例分割任务"
[R23]: https://docs.ultralytics.com/datasets/detect/ "Ultralytics 检测数据集格式"
[R24]: https://docs.ultralytics.com/datasets/segment/ "Ultralytics 分割数据集格式"
[R25]: https://docs.ultralytics.com/modes/train/ "Ultralytics Train"
[R26]: https://docs.ultralytics.com/modes/val/ "Ultralytics Val"
[R27]: https://index.ros.org/p/cv_bridge/ "ROS cv_bridge 包索引"
[R28]: https://docs.python.org/3/library/time.html "Python time"
[R29]: https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst "ROS REP 103"
[R30]: https://www.ros.org/reps/rep-0118.html "ROS REP 118"
[R31]: https://docs.ros.org/en/humble/p/sensor_msgs/msg/Image.html "sensor_msgs/Image Humble"
[R32]: https://docs.ros.org/en/humble/p/sensor_msgs/msg/CameraInfo.html "sensor_msgs/CameraInfo Humble"
[R33]: https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst "ROS 2 Humble QoS 官方文档源码"
[R34]: https://raw.githubusercontent.com/ros2/message_filters/humble/src/message_filters/__init__.py "ROS message_filters Humble Python 源码"
[R35]: https://onnxruntime.ai/docs/get-started/with-python.html "ONNX Runtime Python 入门"
[R36]: https://docs.ultralytics.com/guides/yolo-data-augmentation/ "Ultralytics 数据增强说明"
[R37]: https://docs.ultralytics.com/modes/export/ "Ultralytics Export"
[R39]: https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html "OpenCV VideoCapture"
[R42]: https://docs.opencv.org/4.x/da/d6e/tutorial_py_geometric_transformations.html "OpenCV 几何变换教程"
[R43]: https://docs.pytorch.org/docs/2.14/generated/torch.autograd.grad_mode.inference_mode.html "PyTorch inference_mode"
[R44]: https://docs.pytorch.org/docs/2.14/notes/cuda.html "PyTorch CUDA semantics"
[R45]: https://docs.openvino.ai/2025/get-started.html "OpenVINO Get Started"
[R46]: https://www.dji.com/support/product/o4-air-unit "DJI O4 官方支持"
[R47]: https://docs.python.org/3/library/queue.html "Python queue"
[R50]: https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.rst "ROS 2 Humble rosbag 官方教程源码"
[R52]: https://docs.pytorch.org/docs/2.14/notes/randomness.html "PyTorch Reproducibility"
[R53]: https://scikit-learn.org/stable/modules/generated/sklearn.model_selection.GroupShuffleSplit.html "scikit-learn GroupShuffleSplit"
[R54]: https://dev.realsenseai.com/docs/post-processing-filters/ "RealSense 后处理滤波"


[R55]: https://raw.githubusercontent.com/realsenseai/librealsense/master/include/librealsense2/h/rs_frame.h "librealsense 帧时间与元数据定义"
[R56]: https://raw.githubusercontent.com/ros2/common_interfaces/humble/geometry_msgs/msg/Pose.msg "ROS 2 Humble Pose 消息定义"
