# 从 0 搭建完成 CUADC 任务的 ROS 2 系统

> **主线**：Ubuntu 22.04、ROS 2 Humble、C++17 / Python、MAVROS、ArduPilot、多旋翼侦察与救援。  
> **阅读目标**：不仅会启动一个节点，还能设计接口、组织工作空间、接入视觉和飞控、验证时间与坐标、构建状态机、复现故障，并把系统交给下一届队员。  
> **基准代码**：`cuadc_full_mission_node_3.cpp`，源码版本字符串为 `full-mission-v11-geometric-fine-alignment-2026-09-03`。[P1]  
> **配套练习**：`CUADC_ROS2_阶段实践工作空间.zip`。本文包含主要完整文件，压缩包用于免去逐段复制。  
> **编写日期**：2026-09-05。教程以 Humble 为教学基线，不把其他发行版的新功能默认为 Humble 可用。

## 先区分三种内容

**【源码事实】**指直接从上传的 v11 文件可以确认的行为。它不代表实飞时 YAML、飞控参数、相机外参和外围 Python 节点一定与默认值一致。

**【ROS 机制】**指 ROS 2、MAVROS、MAVLink 等公开接口的语义。文末给出官方文档；部分文档网页受访问限制，另列同一官方项目的原始文档入口。

**【教学设计】**指本文新写的练习、工程拆分、故障注入和改进建议。它们不是已经合并到你们实际工作空间的功能。

你们已经说明使用 DJI O4 进行侦察；但上传的 v11 仍是 D435i 六点拍照流程。前面对话中的“3 m 单趟侦察、非任务区 12 m/s”和时间估算属于讨论方案，**不是本教程确认过的已实施配置或实飞性能**。

同样，本文不会把“严格时间戳插值”写成“硬件采样绝对同步”，也不会把“收到舵机 ACK”写成“传感器确认水瓶已脱离”。这些区别直接关系到系统能否被正确调试。

## 如何使用本文

第一次按第 1～28 章建立 ROS 基础；第二遍结合第 29～44 章理解真实任务；第三遍完成第 45～60 章的联调、回放和故障实验。查问题时可直接使用文末命令表、API 表、源码接口表。

每段代码均属于以下一种：

| 标签 | 含义 |
|---|---|
| 完整文件 | 按标注路径保存，可以纳入配套工程构建 |
| 局部示例 | 用于解释 API，需要放进已有类或函数 |
| 伪代码 | 表达工程逻辑，不应直接编译 |
| 只读诊断 | 不发起飞、解锁或投放命令，但可能连接真实设备 |
| 仅 SITL | 只准在确认隔离的仿真环境执行 |

**所有配套练习默认运行在 `/lab`，不含 MAVROS 飞行客户端，也不打开飞控串口。不要把教学输出 remap 到真实 `/mavros/setpoint_*`。**

安全隔离必须同时考虑物理连接、MAVLink 路由、ROS Domain 和控制权限；仅改话题名字不构成可靠安全隔离。

## 目录

- [1. ROS 在 CUADC 系统中到底负责什么](#c01)
- [2. 学习路线与安全测试分级](#c02)
- [3. 为 ROS 准备的 Linux 与终端基础](#c03)
- [4. 从空 Ubuntu 建立 ROS 2 Humble 环境](#c04)
- [5. 环境变量、overlay 与“改了代码却没生效”](#c05)
- [6. ROS Graph：Node、Topic、Service、Action、Parameter](#c06)
- [7. 先学会看系统：ROS 2 命令行工具](#c07)
- [8. 工作空间、包和构建产物](#c08)
- [9. 一个 C++ ROS 包是怎样组成的](#c09)
- [10. Python ROS 包、入口与跨语言通信](#c10)
- [11. Node 的生命周期、`spin()` 与对象存活](#c11)
- [12. Topic、发布订阅和接口契约](#c12)
- [13. 读懂 ROS 消息：Header、Pose、Twist、Odometry](#c13)
- [14. 旧 PoseArray 协议与新的自定义接口](#c14)
- [15. QoS：有 Topic 却收不到消息的核心原因](#c15)
- [16. 实践 4：30 Hz 模拟 odom、C++ 观察器与 QoS 故障](#c16)
- [17. Service：请求、响应与非阻塞客户端](#c17)
- [18. 实践 5：C++ 异步 Future、轮询与超时清理](#c18)
- [19. 执行机构协议：幂等、去重与物理状态不确定](#c19)
- [20. Action：适合带进度和取消的长任务](#c20)
- [21. Parameter：默认值、YAML、启动覆盖与有效配置](#c21)
- [22. Launch：从七个终端到一个可审计入口](#c22)
- [23. 日志、诊断与任务事件](#c23)
- [24. Executor 与回调调度：为什么有数据也来不及处理](#c24)
- [25. 并发、回调组和有界队列](#c25)
- [26. 三种时间：ROS 时间、系统时间、单调时间](#c26)
- [27. v11 的视觉—导航同步：实际做了什么，没有做什么](#c27)
- [28. 实践：把时间与几何函数写成不依赖 ROS 的单元测试](#c28)
- [29. 坐标系：ROS、飞控、场地、机体和相机不是同一个原点](#c29)
- [30. tf2：让坐标关系成为可查询的系统接口](#c30)
- [31. 标定接口：相机、机体、投放口应怎样交接](#c31)
- [32. 图像、D435i 与 ROS 数据流水线](#c32)
- [33. 实践：写一个视觉替身和健康检查节点](#c33)
- [34. 目标跟踪如何与 ROS 数据有效性结合](#c34)
- [35. MAVROS：ROS 图与飞控之间的协议桥](#c35)
- [36. 串口、UDP、TCP 和 MAVLink 路由](#c36)
- [37. 飞控状态、解锁与起降命令的 ROS 语义](#c37)
- [38. 飞控参数、ROS 参数与 NAV30 频率约束](#c38)
- [39. setpoint、轨迹和真实飞行速度](#c39)
- [40. 把 ROS 接口组装成完整任务状态机](#c40)
- [41. 起飞前门禁：配置正确、数据正确、控制权正确](#c41)
- [42. 投放对准：ROS 层必须保证几何与动作时序可解释](#c42)
- [43. DJI O4 侦察：视频链路与 ROS 任务完成条件](#c43)
- [44. 非任务区 12 m/s：ROS 能做什么，不能证明什么](#c44)
- [45. 综合实践：不接飞控的闭环任务系统](#c45)
- [46. 故障注入：故意把系统弄坏，再证明它解释得清楚](#c46)
- [47. 从软件替身走到 ArduPilot SITL](#c47)
- [48. Gazebo Harmonic 与 Humble：建立可复现的仿真链](#c48)
- [49. RViz2、rqt_graph 和可视化调试](#c49)
- [50. rosbag2：把任务变成可分析的数据](#c50)
- [51. 安全回放：历史命令不能流回真实飞机](#c51)
- [52. 性能分析：从 CPU 占用到端到端延迟](#c52)
- [53. 多机通信、DDS、RMW 与安全隔离](#c53)
- [54. 一键启动、Shell 与 systemd 的工程边界](#c54)
- [55. 控制权与安全：ROS 不是最后一道保护](#c55)
- [56. 测试与持续集成：怎样证明修改没有破坏原来的能力](#c56)
- [57. 工作空间发布、版本管理与团队传承](#c57)
- [58. 任务耗时、成绩条件和软件成功之间的关系](#c58)
- [59. 故障排查手册：从症状找到证据](#c59)
- [60. 学习计划、阶段验收和自测答案](#c60)
- [附录 A. 日常命令速查表](#appendix-a)
- [附录 B. C++ / Python ROS API 查阅手册](#appendix-b)
- [附录 C. v11 源码 ROS 接口与函数定位](#appendix-c)
- [附录 D. 配套工作空间完整结构与剩余文件](#appendix-d)
- [附录 E. 资料来源、版本边界与交付验证](#appendix-e)

---

<a id="c01"></a>
# 1. ROS 在 CUADC 系统中到底负责什么

## 1.1 先看完整的数据链

```text
相机传感器 → 图像采集 → 桶检测 / 深度 / 外参变换
                                  │
                            ROS 检测话题
                                  ↓
飞控 → MAVLink → MAVROS → 任务状态机 → MAVROS → MAVLink → 飞控
                        │             │
                        │             └→ 舵机命令与回执
                        ├→ 任务状态、诊断信息
                        └→ rosbag / 文本日志 / 照片索引

O4 相机 → O4 图传 → 地面显示 / 人员判读
                 （不天然等于 ROS 图像话题）
```

ROS 的主要价值是：让这些独立模块用统一的通信、命名、时间、参数和启动方式协作。它不是图像识别算法，不是飞控固件，也不是一个自动保证安全的“总控制器”。

在当前源码中，任务节点主要处理 odom、目标检测、状态和服务回复，再发布位置目标或发送离散命令。它没有直接控制电机转速，也没有实现 ArduPilot 的底层 EKF 和姿态控制器。[P1]

## 1.2 三个容易混淆的层次

| 层次 | 解决的问题 | 项目中的例子 |
|---|---|---|
| 算法层 | 给定输入，应该算出什么 | 桶关联、坐标变换、轨迹采样 |
| ROS 层 | 输入输出怎样连接、何时到达 | Topic、QoS、Timer、Executor、Service |
| 飞控 / 物理层 | 指令如何作用到飞机 | GUIDED、位置控制、PWM、机构动作 |

例如：桶定位偏了 20 cm，可能是算法滤波，也可能是 ROS 排队延迟，还可能是外参原点不一致。单看识别框通常无法区分。

## 1.3 新人必须会回答的六个问题

一个节点的职责是什么？输入是什么类型？输入在哪个坐标系？时间戳代表什么？多久不来算故障？输出会不会直接影响真实执行机构？

这六个问题能写清楚，才有资格把两个节点接起来。

**实践 0**：不用写代码，画出你们实际系统图，给每条箭头补上“消息类型、坐标系、时间戳含义”。未知项写“待测”，不要猜。

<a id="c02"></a>
# 2. 学习路线与安全测试分级

## 2.1 为什么不能第一天就跑完整状态机

完整任务同时涉及解锁、模式切换、空间运动和投放。如果一个新人的环境变量、消息单位或坐标符号还没弄清，直接运行只能增加问题之间的耦合。

本文采用下面的学习顺序：

| 阶段 | 实践对象 | 必须证明什么 |
|---|---|---|
| A | 普通消息与命令行 | 会创建节点、观察消息、解释名字 |
| B | 软件模拟 odom / 视觉 | 会检查 QoS、频率、时间戳、有效性 |
| C | 模拟服务与状态机 | 不阻塞、会超时、会区分 ACK 和动作 |
| D | rosbag / RViz | 能记录、回放、解释坐标与事件 |
| E | ArduPilot SITL | 飞控状态与任务状态能正确衔接 |
| F | 拆桨台架 | 串口、通道、机构和接管路径正确 |
| G | 受控低速实飞 | 数据、轨迹和故障行为都有证据 |
| H | 逐级提速和比赛流程 | 性能提升不破坏任务门禁 |

## 2.2 练习环境统一约定

所有教学终端使用：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
```

`88` 是本文选择的实验域号，不是 ROS 推荐的通用固定值。双机实验时再显式调整 `ROS_LOCALHOST_ONLY`，不要在其他章节悄悄关闭。

真实飞机、SITL、回放、教学至少应有可辨认的配置文件和启动入口。Domain ID 是通信分组，不是密码；知道相同 Domain 的进程仍可能加入。[R02][R03]

## 2.3 每次实验的记录模板

```text
实验编号：
目标：
代码版本 / commit：
ROS 发行版和软件包版本：
输入与输出话题：
参数文件：
是否接真实飞控：否 / 是，说明：
预期结果：
实际结果：
证据：日志路径、bag 路径、截图、视频时间段
异常解释：
下一次只改变哪一个变量：
```

“这次飞得很好”不是可复现结论；“同一参数下 10 次任务全部在某门禁正常退出，附事件日志”才接近工程结论。

<a id="c03"></a>
# 3. 为 ROS 准备的 Linux 与终端基础

## 3.1 路径、当前目录和用户目录

```bash
pwd                   # 当前目录
ls -lah               # 包含隐藏文件的目录列表
cd ~                  # 当前用户的 home
mkdir -p ~/cuadc_ros2_lab_ws/src
cd ~/cuadc_ros2_lab_ws
```

`~` 是当前用户的家目录，不等于固定的 `/home/cuadc`。`src/` 是相对路径；`/opt/ros/humble/` 是绝对路径。

在不同电脑上，用户名、串口 by-id 和相机序列号通常不同；把它们写死在算法源文件里，会让移植变得困难。

## 3.2 `source` 与执行脚本不是一回事

```bash
source /opt/ros/humble/setup.bash
bash scripts/run_lab.sh
```

第一条把脚本设置的环境变量留在当前 Shell。第二条启动一个子 Shell；子进程修改环境通常不会反向修改父 Shell。

因此，在一个终端 `source install/setup.bash`，另一个终端不会自动拥有相同环境。

## 3.3 重定向与管道

```bash
ros2 node list > nodes.txt
ros2 topic list -t | tee topics.txt
ros2 launch cuadc_tutorial_bringup lab.launch.py 2>&1 | tee run.log
```

`>` 覆盖文件，`>>` 追加；`2>&1` 把标准错误合并到标准输出。管道末端成功不代表前端成功，因此可靠的启动脚本要理解 `set -o pipefail`。

```bash
set -o pipefail
```

不要把 `| grep ERROR` 当作完整日志保存：它会丢掉错误前的模式变化、时间戳和成功 ACK。

## 3.4 权限与进程

```bash
id
groups
ps -ef | grep -E 'mavros|mission|realsense'
pgrep -af 'mavros|mission|realsense'
```

不要用 `sudo ros2 ...` 解决所有权限问题。它可能使用不同 Python、不同 ROS 环境，并留下 root 所有的日志。串口访问问题应检查用户组、设备归属及占用者。

`Ctrl+C` 通常向前台进程组发送中断信号；这不等于真实飞机已降落，也不等于飞控一定刹停。停止进程和停止物理运动必须分开理解。

## 3.5 文本文件格式

Shell 脚本用 LF 换行；从 Windows 复制来的 CRLF 可能导致 `bad interpreter`。YAML 用空格缩进，不用 Tab。命令里的反斜杠 `\` 表示续行时，后面不要再加空格或注释。

**验收**：能解释 `pwd`、`source`、`export`、`2>&1`、`tee`、`Ctrl+C` 的作用，而不是只会复制整串命令。

<a id="c04"></a>
# 4. 从空 Ubuntu 建立 ROS 2 Humble 环境

## 4.1 固定学习基线，而不是追逐最新发行版

本文选择 Ubuntu 22.04 + Humble，是因为你们的现有资料和项目使用这套组合。官方 Humble 二进制安装文档对应 Ubuntu Jammy 22.04。[R01]

先确认系统：

```bash
cat /etc/os-release
uname -m
locale
which python3
python3 --version
```

不要往其他 Ubuntu 版本强行添加 Jammy 的软件源。不要把 ROS 1 的 `catkin_make`、`roscore` 混入这份 ROS 2 教程。

## 4.2 一次性的源配置

以下针对**新建教学机**。比赛机已稳定安装时，不要在赛前临时执行系统全量升级。

```bash
sudo apt update
sudo apt install -y locales software-properties-common curl
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo add-apt-repository universe
```

官方文档现使用 `ros2-apt-source` 管理源与密钥。下面按官方方法增加失败检查；安装前应查看下载的来源和发行版是否匹配。[R01A]

```bash
ROS_APT_SOURCE_VERSION="$(curl -fsSL \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
test -n "$ROS_APT_SOURCE_VERSION" || exit 1
. /etc/os-release
CODENAME="${UBUNTU_CODENAME:-$VERSION_CODENAME}"
test "$CODENAME" = jammy || { echo '本教程要求 Ubuntu 22.04'; exit 1; }
curl -fL -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

安装 ROS 前，应按照官方说明让新系统的关键系统包完成必要更新；先审阅 APT 的变更列表，异常卸载必须停止处理。

## 4.3 安装基础工具

```bash
sudo apt install -y ros-humble-desktop ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep python3-pytest \
  python3-yaml build-essential cmake git
source /opt/ros/humble/setup.bash
```

后续按实际需要安装：

```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras \
  ros-humble-tf2-tools ros-humble-rqt-graph \
  ros-humble-rqt-image-view ros-humble-cv-bridge
```

相机驱动、Gazebo Harmonic 及其与 Humble 的非默认组合单独处理，不在这里把所有依赖一股脑装上。

## 4.4 `rosdep` 是依赖解析，不是编译器

初次使用且系统尚未初始化：

```bash
sudo rosdep init
rosdep update
```

已经初始化时不要反复运行 `sudo rosdep init`，更不要随手删除系统中的源列表。以后在工作空间根目录使用：

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

`--ignore-src` 表示工作空间内已有源包不再重复作为系统包安装。

## 4.5 最小验收

终端 A：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

终端 B：

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

两个终端必须处在相同 Domain 和可通信配置中。先证明 C++ 与 Python 的基础通信正常，再开始项目工作空间。[R01]

<a id="c05"></a>
# 5. 环境变量、overlay 与“改了代码却没生效”

## 5.1 underlay / overlay

```text
/opt/ros/humble             系统 underlay
        ↑
~/cuadc_ros2_lab_ws/install 当前 overlay
```

惯用顺序：

```bash
source /opt/ros/humble/setup.bash
source ~/cuadc_ros2_lab_ws/install/setup.bash
```

`install/setup.bash` 可能包含构建时记录的父工作空间链。`local_setup.bash` 主要设置当前工作空间，适用于已经正确 source underlay 的情况。不要通过反复叠加十个旧工作空间来“碰运气”。[R04]

## 5.2 用证据确认运行的是谁

```bash
which ros2
which python3
printenv ROS_DISTRO
printenv AMENT_PREFIX_PATH
printenv CMAKE_PREFIX_PATH
printenv PYTHONPATH
ros2 pkg prefix cuadc_tutorial_py
ros2 pkg executables cuadc_tutorial_py
```

当程序行为与新源码不同，先查可执行文件来自哪个 `install/`，再考虑算法是否有问题。

## 5.3 Conda 的问题是环境混用，不是数学上不能共存

你们已有环境教程强调运行 ROS 时退出 Conda。本文延续这个项目约定，减少 Python ABI、`PATH`、动态库路径混用。[P3]

这不意味着所有 ROS 系统永远不能使用虚拟环境。为推理节点建立专用环境是可行的架构选择，但必须验证 Python ABI、消息类型支持、`cv_bridge` 和动态库来源。新人先用系统 Python，再学习隔离部署。

```bash
echo "${CONDA_PREFIX:-}"
echo "${VIRTUAL_ENV:-}"
python3 -c 'import sys; print(sys.executable); print(sys.version)'
python3 -c 'import rclpy; print(rclpy.__file__)'
```

## 5.4 daemon 缓存不等于机器人核心

```bash
ros2 daemon stop
ros2 daemon start
```

这通常用于刷新 CLI 发现缓存，尤其是切换 Domain / RMW 后。ROS 2 并不是靠这个 daemon 充当 ROS 1 的 master；停止 daemon 不应被解释成停止所有机器人节点。[R02][R03]

**实践 1**：在两个不同终端 source 不同工作空间，比较 `ros2 pkg prefix`。记录为什么“文件名相同”不代表“运行同一个二进制”。

<a id="c06"></a>
# 6. ROS Graph：Node、Topic、Service、Action、Parameter

## 6.1 一个可以独立解释的小系统

```text
sim_plant ── odom ──→ telemetry_observer
     │
     └──── odom ──→ vision_stub ── buckets ──→ perception_guard
                                                  │
                                             vision_ready
                                                  ↓
                                            mission_dry_run
```

Node 是计算单元，Topic 是数据流，Message 是数据结构。一个进程可包含多个节点；一个包也可安装多个可执行程序。它们不是一一对应关系。[R05]

## 6.2 三类通信怎么选

| 需求 | 首选形式 | CUADC 例子 |
|---|---|---|
| 持续更新、最新数据重要 | Topic | odom、检测结果、诊断状态 |
| 短请求 / 短回复 | Service | 查询状态、设置配置、发命令 |
| 长任务、进度、取消 | Action | 通用扫描任务接口 |

Action 不会自动让长任务变安全，也不会自动提供飞控模式接管。v11 采用 Timer 状态机加 Service，而不是 ROS Action。[P1]

## 6.3 Parameter 是节点配置，不是全局变量仓库

`search_speed_m_s` 属于任务节点；相机曝光属于相机节点；飞控的速度上限属于 ArduPilot。三者名字即使相似，也不是同一套参数机制。

## 6.4 包、进程、节点名、可执行文件名

```text
包：cuadc_tutorial_py
可执行入口：vision_stub
节点名：vision_stub
完整节点名：/lab/vision_stub
Python 模块：cuadc_tutorial_py.vision_stub
```

它们可以相同，也可以不同。诊断时要使用正确层次的命令。

## 6.5 ROS 1 与 ROS 2 的最小区别表

| 常见 ROS 1 说法 | 本教程 ROS 2 对应 |
|---|---|
| roscore / master | DDS 发现，不运行 roscore |
| catkin_make | colcon + ament |
| rostopic | ros2 topic |
| roslaunch | ros2 launch |
| 全局参数服务器 | 节点参数服务 |
| 默认只关心 queue_size | 还必须考虑完整 QoS |

不要因为旧文件名叫 `offb_node.cpp` 就认为项目使用 PX4 OFFBOARD。你们实际主线是 ArduPilot GUIDED。[P1]

<a id="c07"></a>
# 7. 先学会看系统：ROS 2 命令行工具

## 7.1 排查链从上到下

```bash
ros2 node list
ros2 node info /lab/vision_stub
ros2 topic list -t
ros2 topic info /lab/buckets -v
ros2 interface show cuadc_tutorial_interfaces/msg/BucketArray
ros2 topic echo /lab/buckets --once
```

`list` 确认名字存在；`info -v` 查看端点与 QoS；`interface show` 查结构；`echo` 查内容。看到 Topic 名不代表已经收到了有效数据。[R06]

## 7.2 类型名的三种写法

```text
命令行：nav_msgs/msg/Odometry
C++：   nav_msgs::msg::Odometry
Python：from nav_msgs.msg import Odometry
```

生成头文件常为小写下划线：

```cpp
#include "geometry_msgs/msg/pose_stamped.hpp"
```

名字转换由 ROS 接口生成工具完成，不是手工维护三套数据类。

## 7.3 频率与带宽

```bash
ros2 topic hz /lab/odom
ros2 topic bw /lab/buckets
```

这里测到的是工具所接收到的消息，不一定等于传感器物理采样频率。CLI 本身受 CPU、QoS、终端输出和网络影响。平均 30 Hz 也可能夹杂 200 ms 的长空档。

CLI 可用选项会随软件包更新变化，先看：

```bash
ros2 topic hz --help
ros2 topic echo --help
```

不要把某发行版里的 `hz` 选项假定为所有 Humble 安装都具备。

## 7.4 只在教学命名空间发消息

```bash
ros2 topic pub --once /lab/manual_note std_msgs/msg/String \
  '{data: "hello CUADC"}'
```

这只是消息发送实验。不要照着练习对真实解锁服务或执行机构话题试错。

## 7.5 每个阶段的输出必须可观察

对于一个任务节点，至少能看到：当前状态、当前目标、目标身份、输入新鲜度、门禁不满足原因、最近一次请求与回复。

一个只有“正在运行”的进程，即使算法正确，也很难在赛场调试。

<a id="c08"></a>
# 8. 工作空间、包和构建产物

## 8.1 目录分工

```text
cuadc_ros2_lab_ws/
├── src/        自己维护的源码
├── build/      CMake / setuptools 中间产物
├── install/    ros2 run / launch 查找的安装结果
├── log/        colcon 构建日志
└── scripts/    自己维护的入口脚本
```

`src` 里的 `src/` 可能再次出现：外层是工作空间源包目录，内层是某个 C++ 包的源码目录，并不是多写了一层。

## 8.2 配套工程包划分

```text
src/
├── cuadc_tutorial_interfaces   自定义 msg / srv / action
├── cuadc_tutorial_py           模拟输入、校验、软件任务
├── cuadc_tutorial_cpp          C++ 观察器和异步请求练习
└── cuadc_tutorial_bringup      launch 和 YAML
```

这是教学拆分。真实项目是否需要拆成四个包，要看接口稳定性、团队分工与测试方式，不是包越多越专业。

## 8.3 构建完整教学工程

把压缩包中的 `cuadc_ros2_lab_ws` 放到用户目录后：

```bash
cd ~/cuadc_ros2_lab_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon list
colcon build --symlink-install --packages-up-to cuadc_tutorial_bringup
source install/setup.bash
```

`--packages-select` 只选择指定包；`--packages-up-to` 连同工作空间中所需依赖一起构建。

`--symlink-install` 方便某些源码或资源变动映射到安装目录，但不意味着 C++ 修改后无需重新编译；新增 Python 入口、修改安装规则和接口文件通常也需要重建。[R04]

## 8.4 首次不要把编译错误当运行错误

```text
找不到头文件       → 依赖 / include / find_package
链接 undefined     → target 链接依赖
package not found  → 构建结果 / source / 包名
executable not found → 安装规则 / 入口 / 构建失败
运行后没有回调     → 话题 / QoS / executor / 数据源
```

**验收**：能够指出一个改动需要修改 `package.xml`、`CMakeLists.txt`、`setup.py` 中的哪一个；不能回答“都改一下试试”。


<a id="c09"></a>
# 9. 一个 C++ ROS 包是怎样组成的

## 9.1 `package.xml` 描述“这个包依赖谁”

下面是配套 C++ 包的完整声明。它没有 `mavros_msgs`，因为练习只读取模拟 odom，不创建飞控服务客户端。

```xml
<?xml version="1.0"?>
<package format="3">
  <name>cuadc_tutorial_cpp</name>
  <version>0.1.0</version>
  <description>Read-only C++ observer exercises</description>
  <maintainer email="training@example.com">CUADC Training</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>nav_msgs</depend>

  <export><build_type>ament_cmake</build_type></export>
</package>
```

`<depend>` 适用于构建和运行都需要的依赖；`<buildtool_depend>` 指定构建系统；`<test_depend>` 声明测试依赖。声明了依赖不代表系统已经安装，`rosdep` 才负责解析安装。[R07]

## 9.2 `CMakeLists.txt` 描述“具体编译安装什么”

```cmake
cmake_minimum_required(VERSION 3.8)
project(cuadc_tutorial_cpp)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
add_executable(telemetry_observer src/telemetry_observer.cpp)
target_compile_features(telemetry_observer PUBLIC cxx_std_17)
target_compile_options(telemetry_observer PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(telemetry_observer rclcpp nav_msgs std_msgs)
add_executable(async_probe src/async_probe.cpp)
target_compile_features(async_probe PUBLIC cxx_std_17)
ament_target_dependencies(async_probe rclcpp std_srvs)
install(TARGETS telemetry_observer async_probe DESTINATION lib/${PROJECT_NAME})
ament_package()
```

逐项理解：

| 指令 | 在本例中的作用 |
|---|---|
| `project()` | 定义工程名，通常与包名一致 |
| `find_package()` | 查找依赖提供的构建配置 |
| `add_executable()` | 把指定 `.cpp` 编成可执行文件 |
| `target_compile_features()` | 明确使用 C++17 |
| `ament_target_dependencies()` | 设置 ROS 依赖所需编译和链接信息 |
| `install(TARGETS ...)` | 让 `ros2 run` 找到程序 |
| `ament_package()` | 导出包元数据，通常放在末尾 |

这里不是用 `g++ xxx.cpp` 单独手工拼所有 ROS 动态库。

## 9.3 创建自己的最小包

```bash
cd ~/cuadc_ros2_lab_ws/src
ros2 pkg create my_first_observer --build-type ament_cmake \
  --dependencies rclcpp std_msgs
```

自动生成的是骨架，不会自动知道你的 `.cpp` 文件和可执行文件名。新增源文件后仍要补 `add_executable` 和安装规则。

## 9.4 编译选项与调试

```bash
cd ~/cuadc_ros2_lab_ws
colcon build --packages-select cuadc_tutorial_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`RelWithDebInfo` 兼顾优化与调试符号。实际性能测试应记录构建模式，不要拿 Debug 的延迟与 Release 混比。

**实践 2**：给 `telemetry_observer` 增加一条启动日志，再编译。确认日志变化；然后用 `ros2 pkg prefix` 确认运行包来自当前工作空间。

<a id="c10"></a>
# 10. Python ROS 包、入口与跨语言通信

## 10.1 为什么一个项目里可以同时有 C++ 和 Python

Topic 的双方约定的是 ROS 消息类型和通信契约，不是相同编程语言。C++ 可以发布 odom，Python 可以订阅；Python 可以发检测数组，C++ 可以执行状态机。[R08]

这正适合 CUADC：控制节拍和任务结构常用 C++，视觉推理和设备接口可用 Python。不能把“混合语言”误认为“两个程序共享同一块 Python/C++ 内存”。

## 10.2 包文件结构

```text
cuadc_tutorial_py/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/cuadc_tutorial_py
├── cuadc_tutorial_py/
│   ├── __init__.py
│   ├── hello.py
│   └── ...
└── test/
```

目录名重复是正常的：外层是 ROS 包，内层是 Python 模块包。

### 完整 `setup.py`

```python
from setuptools import find_packages, setup

package_name = 'cuadc_tutorial_py'
modules = [
    'hello', 'sim_plant', 'vision_stub', 'perception_guard',
    'payload_server', 'payload_client', 'mission_dry_run',
    'scan_action_server', 'scan_action_client', 'tf_demo',
]
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CUADC Training',
    maintainer_email='training@example.com',
    description='Software-only exercises; no MAVROS or hardware control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        f'{name} = {package_name}.{name}:main' for name in modules
    ]},
)
```

### 完整 `setup.cfg`

```ini
[develop]
script_dir=$base/lib/cuadc_tutorial_py
[install]
install_scripts=$base/lib/cuadc_tutorial_py
```

`console_scripts` 的一项：

```text
hello = cuadc_tutorial_py.hello:main
```

意思是创建名为 `hello` 的可执行入口，导入 `cuadc_tutorial_py.hello` 模块并调用它的 `main()`。不是直接寻找一个叫 `hello` 的文件。

## 10.3 第一个完整 Python 节点

本例使用共享辅助模块 `common.py`，完整内容放在配套工程和附录 D；辅助模块负责初始化、关闭和实验命名空间检查。

```python
from rclpy.node import Node
from std_msgs.msg import String
from .common import require_lab_namespace, spin_main


class Hello(Node):
    def __init__(self):
        super().__init__('hello')
        require_lab_namespace(self)
        self.count = 0
        self.pub = self.create_publisher(String, 'hello', 10)
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        msg = String()
        msg.data = f'CUADC ROS 2 practice, sequence={self.count}'
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        self.count += 1


def main(args=None):
    spin_main(Hello, args)
```

运行：

```bash
source ~/cuadc_ros2_lab_ws/install/setup.bash
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
ros2 run cuadc_tutorial_py hello --ros-args -r __ns:=/lab
```

另一个终端观察：

```bash
ros2 topic echo /lab/hello
ros2 topic hz /lab/hello
```

应看到不断递增的序号，设计频率是 2 Hz；实际观测可能有调度抖动。

## 10.4 改代码后什么时候要重新构建

已有 Python 模块内部逻辑，在 symlink-install 下通常可直接更新后重启；新增入口、增加依赖、更改安装文件列表，必须重新构建并重新 source。

Python 导入失败时先确认解释器与模块搜索路径，不要立即 `sudo pip install` 覆盖系统依赖。

<a id="c11"></a>
# 11. Node 的生命周期、`spin()` 与对象存活

## 11.1 典型 C++ 执行顺序

```cpp
// 局部教学骨架，完整可执行示例见第 16、18 章。
rclcpp::init(argc, argv);
auto node = std::make_shared<MyNode>();
rclcpp::spin(node);
rclcpp::shutdown();
```

`init` 初始化 ROS 上下文，构造节点时创建通信实体，`spin` 使执行器调度回调，`shutdown` 结束上下文。[R07][R09]

如果创建完订阅就直接从 `main()` 返回，程序不会继续帮你收消息。

## 11.2 句柄为什么要保存成成员变量

```cpp
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
rclcpp::TimerBase::SharedPtr timer_;
```

这些对象管理订阅、发布器和定时器的生命周期。构造函数中创建一个临时对象、离开作用域立即销毁，可能导致节点看似启动但实体不再工作。

## 11.3 不在构造函数里等“任务完成”

构造函数负责建立对象和基础配置，而不是写一个无限循环等 GPS、相机或服务。这样的等待可能发生在执行器开始工作之前，导致健康数据永远得不到处理。

推荐启动状态：

```text
构造：注册通信、读参数、准备状态
spin：开始调度
WAIT_DATA：异步等待各项就绪
```

## 11.4 普通 Node 与 LifecycleNode

LifecycleNode 提供 configure、activate、deactivate、cleanup 等受控生命周期，适合设备资源管理。它不同于任务中的 SEARCH、ALIGN、RELEASE；前者管理节点能否工作，后者管理飞行任务正在做什么。[R10]

v11 是普通 `rclcpp::Node`，并未使用 ROS managed lifecycle。不要把已有的 `State` 枚举当成 LifecycleNode API。

## 11.5 两种退出都要讲清楚

正常退出：程序保存结果、释放资源、关闭 ROS。

异常退出：系统崩溃、断电、SIGKILL 时，析构函数和退出日志都不一定执行。因此，飞行安全不能依赖“C++ 析构函数里发送 LAND”。

<a id="c12"></a>
# 12. Topic、发布订阅和接口契约

## 12.1 发布一次消息发生了什么

```cpp
// 局部示例。
std_msgs::msg::String msg;
msg.data = "SEARCH";
state_pub_->publish(msg);
```

这表示把消息交给通信系统。它不证明某个指定订阅者已处理完成，更不证明外部设备已经执行动作。

发布端、传输队列、订阅端、执行器、业务处理是不同环节。

## 12.2 订阅回调的职责

```cpp
// 局部示例。
odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
  "odom", rclcpp::SensorDataQoS(),
  [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    // 校验、更新快照、记录到达时刻；不要在这里睡眠。
    latest_odom_ = *msg;
  });
```

`ConstSharedPtr` 表示通过这个指针只读消息。保存数据副本和保存消息指针各有成本；多线程时都要设计共享状态的同步。

## 12.3 相对名、绝对名和私有名

| 写法 | 例子 | 典型解析结果 |
|---|---|---|
| 相对名 | `odom` | 在 `/lab` 节点下成为 `/lab/odom` |
| 绝对名 | `/mavros/state` | 不因普通 namespace 前缀而改变 |
| 私有名 | `~/status` | 归属完整节点名，例如 `/lab/guard/status` |

v11 大量使用绝对 `/mavros/...` 和 `/cuadc/...` 名字。仅给它加 namespace，不能自动隔离所有控制端点。[P1][R11]

## 12.4 remap 只改名字，不改数据

```bash
ros2 run cuadc_tutorial_py hello --ros-args \
  -r __ns:=/lab -r hello:=text_stream
```

这个实验把发布输出改成 `/lab/text_stream`。Remap 不会把 ENU 转成 NED，也不会把 `PoseArray` 转成自定义消息。

## 12.5 接口契约模板

```yaml
name: /perception/drop_buckets_body
message_type: geometry_msgs/msg/PoseArray
producer: 视觉节点，实际可执行名待查
consumer: /visual_drop_mission_node
units: meters
frame: 机体 FLU，原点必须与状态机一致
time_stamp: 上游视觉帧的观测时刻；具体物理含义需验证
rate: 以实测为准
empty_semantics: 正常未检测到目标
invalid_semantics: 原消息没有显式 valid，需要另定协议
stale_policy: 按状态区别处理
side_effects: 影响选桶和对准，不直接驱动舵机
```

这份文档比一张“Topic 名称表”多了时间、单位和失效含义，才足以供别人接入自己的视觉节点。

<a id="c13"></a>
# 13. 读懂 ROS 消息：Header、Pose、Twist、Odometry

## 13.1 `Header` 不是附属装饰

```text
stamp.sec / stamp.nanosec
frame_id
```

时间戳回答“这份数据对应哪个时刻”；frame_id 回答“数字在哪个参考系表达”。没有明确这两项，坐标数值本身不完整。

frame_id 只是一个标识。给相机坐标贴上 `base_link` 标签，并不会完成相机外参变换。

## 13.2 常见标准消息

| 消息 | 数据含义 | 常见错误 |
|---|---|---|
| `Point` | 三维点，无时间和参考系 | 当成带 frame 的目标 |
| `PointStamped` | 点 + Header | 忽略时间 |
| `Pose` | 位置 + 旋转 | 默认四元数全零 |
| `PoseStamped` | 位姿 + Header | 把目标姿态与实测姿态混用 |
| `Twist` | 线速度 + 角速度 | 忘记表达坐标系 |
| `Odometry` | 位姿、速度及协方差 | 用一种 frame 解释全部字段 |
| `Image` | 编码、尺寸、字节缓冲 | RGB/BGR、步长、位深弄错 |
| `CameraInfo` | 内参、畸变、图像尺寸 | 与当前分辨率不匹配 |

## 13.3 Odometry 的两个 frame

标准消息定义规定：pose 的参考系为 `header.frame_id`，twist 的表达系为 `child_frame_id`。[R12]

例如：

```text
header.frame_id = map
child_frame_id  = base_link
```

不能未经确认就把 `twist.twist.linear.z` 等同世界竖直速度。飞机倾斜时，机体系 z 分量与世界竖直分量不是同一个量。

**源码阅读点**：v11 直接用 odom twist 的 x/y 模长做水平速度、z 做垂直速度。实际是否符合预期，应核对所用 MAVROS 插件版本、运行消息的 frame 字段与实测方向；单凭变量名无法证明。[P1]

## 13.4 四元数必须合法

水平、零偏航的单位四元数：

```python
msg.pose.orientation.x = 0.0
msg.pose.orientation.y = 0.0
msg.pose.orientation.z = 0.0
msg.pose.orientation.w = 1.0
```

四个分量全零不是“没有旋转”，而是无效旋转表示。有效性校验至少检查 finite、模长，以及是否需要归一化。

## 13.5 协方差不等于错误码

不要简单把所有零协方差解释成“绝对精准”；具体消息和上游实现可能对未知值有不同约定。任务门禁应明确自己依赖哪些质量指标，不能只看 odom 消息是否存在。

**实践 3**：使用 `ros2 interface show nav_msgs/msg/Odometry`，从 x/y/z 一直追到所属 frame，并画出消息嵌套结构。

<a id="c14"></a>
# 14. 旧 PoseArray 协议与新的自定义接口

## 14.1 v11 的视觉协议不是标准姿态语义

当前 `bucket_callback()` 读取：

```text
pose.position.x/y/z = 桶的机体系位置
pose.orientation.x  = 桶直径，单位 m
pose.orientation.y  = 置信度
```

这是项目自定义编码，不是合法四元数意义。[P1: bucket_callback]

因此不能直接把这种 `PoseArray` 当正常姿态数组送给 tf2 做通用位姿变换；不能让 RViz 的箭头方向代表桶朝向；不能把 `orientation.y` 当 pitch。

兼容旧项目时必须保留该契约。新项目更推荐明确命名的接口，但**修改消息类型需要生产者和消费者一起改**，不是只替换某一侧。

## 14.2 教学版自定义消息

### `Bucket.msg`

```text
# 本帧检测序号，不是跨帧 Track ID，也不是比赛的 1/2/3 号桶。
uint32 detection_id
# 坐标系由外层 BucketArray.header.frame_id 定义，单位 m。
geometry_msgs/Point center
float64 diameter_m
float32 confidence
```

### `BucketArray.msg`

```text
# stamp 的契约：该帧模拟采集的 ROS 时间，不是发布完成时间。
std_msgs/Header header
uint64 frame_seq
# valid=false 是算法/传感器无效；valid=true 且数组为空是正常未发现目标。
bool valid
string detail
cuadc_tutorial_interfaces/Bucket[] buckets
```

这里故意区分三类身份：本帧 `detection_id`、状态机跨帧 `track_id`、比赛按直径定义的桶号。它们不应共享一个含糊的 `id`。

`valid=true` 且数组为空表示节点工作正常但未发现目标；`valid=false` 表示测量不可信。不能靠“有没有消息”同时表达这两种情况。

## 14.3 接口生成包

```cmake
cmake_minimum_required(VERSION 3.8)
project(cuadc_tutorial_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bucket.msg"
  "msg/BucketArray.msg"
  "msg/MissionEvent.msg"
  "srv/PayloadOp.srv"
  "action/ScanArea.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

```xml
<?xml version="1.0"?>
<package format="3">
  <name>cuadc_tutorial_interfaces</name>
  <version>0.1.0</version>
  <description>Teaching-only CUADC interfaces</description>
  <maintainer email="training@example.com">CUADC Training</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>action_msgs</depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
```

`rosidl_generate_interfaces` 生成 C++ / Python 类型支持，`DEPENDENCIES` 声明字段引用的外部接口包。即使消费者全是 Python，这种教程路径中的接口生成包仍采用 `ament_cmake`。[R13]

编译后：

```bash
ros2 interface show cuadc_tutorial_interfaces/msg/BucketArray
ros2 interface show cuadc_tutorial_interfaces/srv/PayloadOp
ros2 interface show cuadc_tutorial_interfaces/action/ScanArea
```

## 14.4 怎样做旧接口适配器

```text
你自己的视觉结果
       ↓
校验 frame / 单位 / 时间戳
       ↓
适配成旧 PoseArray 契约
       ↓
未修改的 v11
```

另一条迁移路径：

```text
你自己的视觉结果 → BucketArray → 修改后的任务订阅器
```

适配器要保留原始观测时间，不能因“转换了一遍消息”就把时间戳刷新为当前时刻。

## 14.5 接口变化属于协议变化

新增字段通常会改变消息类型支持，双方及 rosbag 分析环境要一致。交接时保存 `.msg/.srv/.action`、版本号和构建依赖，不要只交一份 Python 节点。

<a id="c15"></a>
# 15. QoS：有 Topic 却收不到消息的核心原因

## 15.1 先抓住两项匹配关系

QoS 是通信端点的策略组合，不是一个“速度越高越好”的参数。订阅者提出要求，发布者提供能力，必须兼容才能通信。[R14]

### Reliability

| 发布者 | 订阅者 | 是否可匹配 |
|---|---|---|
| best effort | best effort | 是 |
| best effort | reliable | 否 |
| reliable | best effort | 是 |
| reliable | reliable | 是 |

### Durability

| 发布者 | 订阅者 | 结果 |
|---|---|---|
| volatile | volatile | 新消息 |
| volatile | transient local | 不匹配 |
| transient local | volatile | 新消息，不依赖历史补发 |
| transient local | transient local | 可接收保留历史 |

所有影响兼容性的策略都要同时满足，不能只看 reliability。

## 15.2 为什么传感器常用 SensorDataQoS

连续传感器中，及时获得较新观测往往比补收陈旧观测更重要。`SensorDataQoS()` 采用 best effort 等传感器取向策略。[R14]

v11 对 odom、罗盘、视觉检测使用 `SensorDataQoS()`；FCU 状态和 NAV30 布尔状态使用 reliable；位置 setpoint 发布使用深度 10 的默认 QoS。[P1]

## 15.3 reliable 不能替代业务 ACK

`publish()` 成功不代表目标业务已经处理。即便可靠传输已经送达订阅端，也不能证明舵机转到了释放角度。因此关键命令仍要有服务回复、命令身份和物理证据。

“Reliable = TCP、best effort = UDP”也不是严格对应。QoS 是语义策略，实际传输由中间件实现，不能用这个类比替代网络分析。

## 15.4 depth 并非越大越好

假设消息每 33 ms 到一帧，消费者每帧处理 100 ms。不断增大队列只会让消费者越来越落后；看似“没有丢帧”，实际上正用旧画面控制飞机。

`depth=1` 也不等于整个视觉链路零延迟。相机内部、驱动、推理线程、DDS 和回调队列都可能排队。

## 15.5 其他策略应怎么理解

Deadline 是期望的更新期限，不会替你保证发布器真的按时运行。Lifespan 限制消息寿命，不自动知道图像曝光时刻。Liveliness 表示实体活性，不等于数据质量。

**工程推导**：CUADC 仍需应用层的“最后有效观测到达时间”“采集时间单调性”“目标可信度”等检查；只配置 DDS QoS 不够。

## 15.6 不要把瞬时执行命令持久化

“当前任务状态”可考虑 retained/transient-local 风格供晚加入的监视器读取；“立即释放第一瓶”不应作为持久历史消息在执行节点重启后重新触发。

状态和事件分开设计，避免启动一个订阅者就把旧动作又执行一遍。

<a id="c16"></a>
# 16. 实践 4：30 Hz 模拟 odom、C++ 观察器与 QoS 故障

## 16.1 实验目的

不接飞控，观察数据频率、最大间隔、消息年龄，并制造“Topic 存在但没有回调”的典型错误。

### 完整观察器

```cpp
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class TelemetryObserver final : public rclcpp::Node
{
public:
  TelemetryObserver() : Node("telemetry_observer")
  {
    const std::string ns = get_namespace();
    if (ns != "/lab" && ns.rfind("/lab/", 0) != 0) {
      throw std::runtime_error("Teaching node requires namespace /lab");
    }
    declare_parameter<bool>("force_reliable", false);
    rclcpp::QoS qos = rclcpp::SensorDataQoS();
    if (get_parameter("force_reliable").as_bool()) {
      qos.reliable();  // 故障练习：源为 best effort 时将无法匹配。
    }
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto now_steady = Clock::now();
        if (have_) {
          const double dt = std::chrono::duration<double>(now_steady - last_).count();
          intervals_.push_back(dt);
          while (intervals_.size() > 150U) { intervals_.pop_front(); }
        }
        last_ = now_steady;
        have_ = true;
        const auto & p = msg->pose.pose.position;
        finite_ = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
        // 原样保留上游时间戳，用于诊断，不冒充采集时刻校准。
        const rclcpp::Time stamp(msg->header.stamp, get_clock()->get_clock_type());
        age_s_ = (now() - stamp).seconds();
      });
    pub_ = create_publisher<std_msgs::msg::String>("navigation_summary", 10);
    timer_ = create_wall_timer(1s, [this]() { report(); });
  }

private:
  void report()
  {
    double sum = 0.0;
    double max_gap = 0.0;
    for (double dt : intervals_) { sum += dt; max_gap = std::max(max_gap, dt); }
    const double hz = sum > 0 ? static_cast<double>(intervals_.size()) / sum : 0.0;
    const double heartbeat_age = have_ ?
      std::chrono::duration<double>(Clock::now() - last_).count() :
      std::numeric_limits<double>::infinity();
    const bool fresh = have_ && finite_ && heartbeat_age <= 0.5;
    std_msgs::msg::String msg;
    msg.data = std::string("fresh=") + (fresh ? "true" : "false") +
      " hz=" + std::to_string(hz) + " max_gap_s=" + std::to_string(max_gap) +
      " header_age_s=" + std::to_string(age_s_);
    pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", msg.data.c_str());
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<double> intervals_;
  Clock::time_point last_{};
  bool have_ = false;
  bool finite_ = false;
  double age_s_ = 0.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<TelemetryObserver>()); }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("telemetry_observer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
```

理解代码的四个层次：订阅回调记录数据；`steady_clock` 记录本机到达间隔；ROS `now()` 与消息 stamp 比较年龄；1 Hz 诊断 Timer 汇总输出。

观察器只报告，不发布 `/cuadc/nav30_ready`，不替代你们原项目的 NAV30 节点。

## 16.2 启动数据源

```bash
bash ~/cuadc_ros2_lab_ws/scripts/run_lab.sh
```

默认会启动模拟输入和观察器，但不会启动 `mission_dry_run`。

```bash
ros2 topic info /lab/odom -v
ros2 topic echo /lab/navigation_summary
```

模拟器请求的更新频率为 30 Hz。若实际明显偏低，先查 CPU、定时器和输出阻塞，不要认为设了 30 就已经测得 30。

## 16.3 故意制造不兼容

在另一终端启动一个名字不同的观察器：

```bash
ros2 run cuadc_tutorial_cpp telemetry_observer --ros-args \
  -r __ns:=/lab -r __node:=reliable_observer \
  -p force_reliable:=true
```

因为模拟源使用 best effort，此观察器要求 reliable，应该收不到 odom。用 `topic info -v` 比较两个订阅端 QoS。

关闭该节点，改成：

```bash
ros2 run cuadc_tutorial_cpp telemetry_observer --ros-args \
  -r __ns:=/lab -r __node:=best_effort_observer \
  -p force_reliable:=false
```

## 16.4 必须能解释的现象

平均频率接近 30 Hz，但最大间隔偶尔超过 50 ms，说明严格插值仍可能失败。

`header_age_s` 大，不一定代表通信丢包，也可能是时间基准或上游排队错误。

`fresh=false` 是该观察器接收视角的判断，不是飞控硬件诊断，也不证明 GPS 或 EKF 一定异常。

**验收产物**：保存两种 QoS 的端点信息、实际输出和解释。


<a id="c17"></a>
# 17. Service：请求、响应与非阻塞客户端

## 17.1 服务不是“带返回值的普通本地函数”

服务请求会经过 ROS 通信和执行器调度。服务端可能未启动，可能拒绝，也可能已经执行但响应迟到。调用者不能假定下一行就已经知道结果。

```text
客户端发请求 → 服务端收到 → 服务回调执行 → 回复 → 客户端处理结果
       └──────────── 期间其他回调仍要运行 ────────────┘
```

ROS 2 官方明确区分同步与异步服务；在回调内同步等待尤其容易形成死锁。[R15]

## 17.2 消息定义与服务定义

消息 `.msg` 是一段数据结构。服务 `.srv` 用 `---` 分隔请求与响应。

### 教学载荷服务完整定义

```text
# 只供软件模拟；request_id 用于当前服务进程生命周期内的去重。
string request_id
uint8 payload_index
bool release
---
bool accepted
bool already_applied
string detail
```

`request_id` 是业务去重键，不是 MAVLink 命令号；`payload_index` 是 0、1，不是飞控 SERVO9、10 通道。

## 17.3 客户端的最小状态

```text
WAIT_SERVICE
   ↓ 服务可见
SEND_ONCE
   ↓ 保存 future + 发送时刻
WAIT_RESPONSE
   ├→ accepted
   ├→ rejected
   └→ timeout / unknown
```

不要在每一个 50 ms Tick 里不加条件地发送一次请求。那不是“保证执行”，而是在制造并发命令和回执歧义。

## 17.4 Python 异步完整练习

```python
"""One asynchronous mock request, with timeout and no blind retry."""
import time
import uuid
from rclpy.node import Node
from cuadc_tutorial_interfaces.srv import PayloadOp
from .common import require_lab_namespace, spin_main


class PayloadClient(Node):
    def __init__(self):
        super().__init__('payload_client')
        require_lab_namespace(self)
        self.client = self.create_client(PayloadOp, 'payload/operate')
        self.future = None
        self.sent_at = None
        self.started_at = time.monotonic()
        self.finished = False
        self.timer = self.create_timer(0.05, self.tick)

    def tick(self):
        if self.finished:
            return
        if self.future is None:
            if not self.client.service_is_ready():
                if time.monotonic() - self.started_at > 5.0:
                    self.get_logger().error('service discovery timeout; no request was sent')
                    self.finished = True
                return
            req = PayloadOp.Request()
            req.request_id = str(uuid.uuid4())
            req.payload_index = 0
            req.release = True
            self.future = self.client.call_async(req)
            self.sent_at = time.monotonic()
            self.get_logger().info('one mock request sent')
        elif self.future.done():
            self.finished = True
            try:
                result = self.future.result()
                self.get_logger().info(f'accepted={result.accepted}, {result.detail}')
            except Exception as exc:
                self.get_logger().error(f'request error: {exc}')
        elif time.monotonic() - self.sent_at > 2.0:
            # 只清理客户端等待，不代表取消服务器行为。
            self.client.remove_pending_request(self.future)
            self.future.cancel()
            self.finished = True
            self.get_logger().error('ACK unknown; do not infer not applied, no automatic retry')


def main(args=None):
    spin_main(PayloadClient, args)
```

`call_async` 返回 Future；`done()` 只检查完成状态。超时后清理本地 pending 请求并不等于取消服务端行为。

本例只发送一次模拟请求，没有自动重试。真实可重复命令和一次性物理动作需要不同重试策略。

## 17.5 诊断命令

```bash
ros2 service list -t
ros2 service type /lab/payload/operate
ros2 interface show cuadc_tutorial_interfaces/srv/PayloadOp
```

只在模拟服务上手工调用：

```bash
ros2 service call /lab/payload/operate \
  cuadc_tutorial_interfaces/srv/PayloadOp \
  '{request_id: "practice-001", payload_index: 0, release: true}'
```

这不是 ArduPilot 舵机命令，不会接触真实硬件。

<a id="c18"></a>
# 18. 实践 5：C++ 异步 Future、轮询与超时清理

## 18.1 最容易写错的一行

```cpp
// 反例：不要放在单线程控制回调中。
auto result = client_->async_send_request(request).get();
```

`.get()` 可能阻塞等待，而响应也需要同一个执行器处理。结果是“正在等自己释放线程”。

改用在多个 Tick 之间保存 Future，检查 `wait_for(0s)`。响应尚未就绪时立即返回，让其他回调继续执行。[R15][R16]

## 18.2 完整 C++ 自请求练习

下面的节点创建一个无副作用 Trigger 服务，同时作为它的客户端。它的意义是证明：即使单线程，只要正确异步，也能正常获得回复。

```cpp
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class AsyncProbe final : public rclcpp::Node
{
public:
  AsyncProbe() : Node("async_probe")
  {
    const std::string ns = get_namespace();
    if (ns != "/lab" && ns.rfind("/lab/", 0) != 0) {
      throw std::runtime_error("requires /lab namespace");
    }
    // 自建无副作用服务；演示单线程回调下正确的非阻塞写法。
    server_ = create_service<std_srvs::srv::Trigger>(
      "probe", [](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = true;
        response->message = "mock probe only";
      });
    client_ = create_client<std_srvs::srv::Trigger>("probe");
    timer_ = create_wall_timer(50ms, [this]() { tick(); });
  }
private:
  void tick()
  {
    if (done_) { return; }
    if (!future_.valid()) {
      if (!client_->service_is_ready()) { return; }
      auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
      auto pending = client_->async_send_request(request);
      request_id_ = pending.request_id;
      future_ = pending.future.share();
      sent_ = Clock::now();
      return; // 释放执行器，使响应回调有机会执行。
    }
    if (future_.wait_for(0s) == std::future_status::ready) {
      done_ = true;
      const auto result = future_.get();
      RCLCPP_INFO(get_logger(), "success=%s, %s",
        result->success ? "true" : "false", result->message.c_str());
    } else if (std::chrono::duration<double>(Clock::now() - sent_).count() > 2.0) {
      if (request_id_) { client_->remove_pending_request(*request_id_); }
      done_ = true;
      RCLCPP_ERROR(get_logger(), "timeout; cleanup is not remote cancellation");
    }
  }
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_;
  std::optional<int64_t> request_id_;
  rclcpp::TimerBase::SharedPtr timer_;
  Clock::time_point sent_{};
  bool done_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<AsyncProbe>()); }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("async_probe"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
```

运行：

```bash
ros2 run cuadc_tutorial_cpp async_probe --ros-args -r __ns:=/lab
```

应获得 `success=true` 和 `mock probe only`。

## 18.3 为什么保留 request_id

客户端超时后，除了清掉自己保存的 Future，还要考虑 `rclcpp::Client` 内部未完成请求的清理。Humble 的 Client 提供 `remove_pending_request`；长期不断积累未完成请求可能增加资源占用。[R16]

**源码事实**：v11 保存 SharedFuture 并在超时后重置它，示例中的显式客户端 pending 清理是教学补充，不应说原实现已经做了这一点。[P1]

## 18.4 `future.valid()` 与 `future ready` 不同

`valid()` 代表 Future 与某个异步状态关联；`ready` 代表结果已经可取。有效并不等于请求成功，ready 也不等于业务 accepted。

正确层次：

```text
future 存在
 → future 完成
 → 没有异常
 → response 表示接受
 → 外部状态达到预期
 → 物理证据成立
```

越往后证据越强，不要跳层。

## 18.5 作业

把 mock 服务改为固定返回 `success=false`，检查客户端能否区分通信成功与业务拒绝。

再把客户端服务名改成不存在的名字，检查它是在等服务发现，还是已经发出请求等 ACK。二者日志应不同。

<a id="c19"></a>
# 19. 执行机构协议：幂等、去重与物理状态不确定

## 19.1 一次投放至少有三种状态

```text
未发送
已发送但执行结果未知
已收到接受回复
```

如果还有传感器，才能进一步得到“机构到位”“水瓶已脱离”。MAVLink `COMMAND_ACK` 的 accepted 表示飞控接受并尝试执行命令，不代表长过程或机械动作已经完成。[R17]

## 19.2 完整模拟服务端

```python
"""A memory-only service. accepted means mock state updated, not physical detach."""
from rclpy.node import Node
from std_msgs.msg import String
from cuadc_tutorial_interfaces.srv import PayloadOp
from .common import require_lab_namespace, spin_main


class PayloadServer(Node):
    def __init__(self):
        super().__init__('payload_server')
        require_lab_namespace(self)
        self.declare_parameter('reject_release', False)
        self.cache = {}
        self.released = [False, False]
        self.pub = self.create_publisher(String, 'payload/mock_state', 10)
        self.srv = self.create_service(PayloadOp, 'payload/operate', self.on_request)

    def on_request(self, req, res):
        signature = (req.payload_index, req.release)
        if not req.request_id or len(req.request_id) > 128 or req.payload_index >= 2:
            res.detail = 'invalid request'
            return res
        if req.request_id in self.cache:
            old_signature, old_accepted, old_detail = self.cache[req.request_id]
            if signature != old_signature:
                res.detail = 'same request_id with different content rejected'
                return res
            res.accepted = old_accepted
            res.already_applied = old_accepted
            res.detail = 'deduplicated: ' + old_detail
            return res
        if len(self.cache) >= 256:
            res.detail = 'mock cache full; stop and reset the exercise'
            return res
        if req.release and self.get_parameter('reject_release').value:
            res.accepted = False
            res.detail = 'release intentionally rejected for exercise'
        else:
            self.released[req.payload_index] = req.release
            res.accepted = True
            res.detail = 'memory flag changed only; no hardware accessed'
        self.cache[req.request_id] = (signature, res.accepted, res.detail)
        msg = String()
        msg.data = f'request={req.request_id}, flags={self.released}, accepted={res.accepted}'
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        return res


def main(args=None):
    spin_main(PayloadServer, args)
```

它只改内存布尔值。重复相同 request_id、相同内容，不再次改变动作语义；相同 ID 搭配不同内容会被拒绝。

去重缓存只在本进程生命周期有效。服务进程重启后缓存丢失；生产系统如需跨重启去重，需要持久化、会话号和恢复策略。

## 19.3 实践 6：重复请求

连续执行两次：

```bash
ros2 service call /lab/payload/operate \
  cuadc_tutorial_interfaces/srv/PayloadOp \
  '{request_id: "dedup-demo", payload_index: 0, release: true}'
```

第二次应标记 `already_applied=true`。再使用相同 ID、把 `release` 改成 false，应该拒绝，而不是把它当作收回命令。

新的收回操作必须有新的 ID，例如 `dedup-demo-stow`。

## 19.4 为什么不能对投放“无脑重发”

如果第一次已脱瓶但 ACK 丢失，重发可能重复驱动机构；如果第一次没执行，不重发又会保留载荷。应用层必须把这种不确定性作为真实状态，而不是拿一个 bool 掩盖。

MAVLink 协议层重传与任务层“再投一次”不是同一件事。多层同时重试，还可能把一次请求放大成多条命令。

## 19.5 两种证据应分开记日志

```text
command_accepted = true
physical_detach_confirmed = unknown
```

当前 v11 根据命令 ACK、0.7 s 保持及收回 ACK推进载荷计数，没有提供水瓶脱离传感器输入。[P1]

这套策略可作为现有软件事实解释，但不能据此宣称已经验证了真实投放成功或比赛有效区命中。

<a id="c20"></a>
# 20. Action：适合带进度和取消的长任务

## 20.1 Action 的三个部分

```text
Goal：要完成什么
Result：最终结果
Feedback：执行中的进度
```

`.action` 用两个 `---` 分隔三部分。Goal 被接受和最终成功不是同一件事；取消请求被接受也不代表物理系统瞬间停住。[R18]

### 完整教学 Action

```text
# 教学 Action：只计时报告进度，不控制飞机。
geometry_msgs/Point[] waypoints
float32 speed_m_s
---
bool success
uint32 completed
string detail
---
uint32 current
float32 progress
```

本文只用这个接口模拟进度，没有让 Action 控制真实飞机。

## 20.2 完整模拟服务端

```python
"""Cancelable teaching action: time progress only, no setpoint output."""
import math
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from cuadc_tutorial_interfaces.action import ScanArea
from .common import require_lab_namespace, spin_main


class ScanServer(Node):
    def __init__(self):
        super().__init__('scan_action_server')
        require_lab_namespace(self)
        self.lock = threading.Lock()
        self.busy = False
        self.group = ReentrantCallbackGroup()
        self.server = ActionServer(
            self, ScanArea, 'scan_mock', execute_callback=self.execute,
            goal_callback=self.on_goal, cancel_callback=self.on_cancel,
            callback_group=self.group)

    def on_goal(self, req):
        valid = (1 <= len(req.waypoints) <= 20 and
                 math.isfinite(req.speed_m_s) and 0 < req.speed_m_s <= 2.0)
        valid = valid and all(math.isfinite(v) and abs(v) <= 10
                              for p in req.waypoints for v in (p.x, p.y, p.z))
        with self.lock:
            if self.busy or not valid:
                return GoalResponse.REJECT
            self.busy = True
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute(self, handle):
        result = ScanArea.Result()
        try:
            n = len(handle.request.waypoints)
            for i in range(n):
                # 每点模拟 1 秒工作，和真正航程/速度没有物理对应。
                for _ in range(20):
                    if not rclpy.ok():
                        return result
                    if handle.is_cancel_requested:
                        handle.canceled()
                        result.completed = i
                        result.detail = 'mock canceled, no aircraft control'
                        return result
                    time.sleep(0.05)
                feedback = ScanArea.Feedback()
                feedback.current = i + 1
                feedback.progress = float(i + 1) / n
                handle.publish_feedback(feedback)
                result.completed = i + 1
            handle.succeed()
            result.success = True
            result.detail = 'mock progress finished, not reconnaissance evidence'
            return result
        finally:
            with self.lock:
                self.busy = False


def main(args=None):
    spin_main(ScanServer, args, MultiThreadedExecutor)
```

这里 `execute` 工作循环有短暂 `sleep`，但它处于明确设计的多线程 Action 教学执行路径，不是 20 Hz 飞行控制 Tick。取消回调需要独立获得调度机会，因此设置了 Reentrant callback group 和多线程执行器。

busy 状态用锁保护；仅切换为多线程而不保护共享状态，会引入新的竞争条件。

## 20.3 完整客户端

```python
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from cuadc_tutorial_interfaces.action import ScanArea
from .common import require_lab_namespace, spin_main


class ScanClient(Node):
    def __init__(self):
        super().__init__('scan_action_client')
        require_lab_namespace(self)
        self.client = ActionClient(self, ScanArea, 'scan_mock')
        self.sent = False
        self.started = time.monotonic()
        self.timer = self.create_timer(0.2, self.tick)

    def tick(self):
        if self.sent:
            return
        if not self.client.server_is_ready():
            if time.monotonic() - self.started > 5:
                self.sent = True
                self.get_logger().error('action discovery timeout')
            return
        goal = ScanArea.Goal()
        goal.speed_m_s = 1.0
        for x in (1.0, 2.0, 3.0):
            p = Point()
            p.x, p.z = x, 1.2
            goal.waypoints.append(p)
        self.sent = True
        future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        future.add_done_callback(self.goal_response)

    def feedback(self, msg):
        self.get_logger().info(f'progress={msg.feedback.progress:.0%}')

    def goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warning('goal rejected')
                return
            # 需要主动取消时保存 handle，再调用 handle.cancel_goal_async()。
            self.handle = handle
            handle.get_result_async().add_done_callback(self.result)
        except Exception as exc:
            self.get_logger().error(str(exc))

    def result(self, future):
        try:
            wrapped = future.result()
            self.get_logger().info(f'status={wrapped.status}; {wrapped.result.detail}')
        except Exception as exc:
            self.get_logger().error(str(exc))


def main(args=None):
    spin_main(ScanClient, args)
```

运行时分两个终端：

```bash
ros2 run cuadc_tutorial_py scan_action_server --ros-args -r __ns:=/lab
```

```bash
ros2 run cuadc_tutorial_py scan_action_client --ros-args -r __ns:=/lab
```

观察：

```bash
ros2 action list -t
ros2 action info /lab/scan_mock
```

应先接受 Goal，再出现进度，最后输出 Result。这只是约 3 秒的软件工作过程，不是实际飞过三个航点。

## 20.4 Action 是否必须用于完整 CUADC

不必须。v11 自己维护状态机，结构上完全可以完成任务。Action 的优势在于向上层暴露一致的长任务 API；代价是更多生命周期、取消和并发状态。

新队员应先学会明确的 Timer 状态机，再判断是否需要包装成 Action。

## 20.5 生产级 Action 还缺什么

配套客户端只演示标准回调链。生产使用还需要结果超时、服务端消失、会话隔离、重复 Goal、取消结果检查和控制权仲裁。不能把这个练习换个 Topic 名就用于实飞。

<a id="c21"></a>
# 21. Parameter：默认值、YAML、启动覆盖与有效配置

## 21.1 参数的完整生命周期

```text
declare 默认值
 → 启动参数 / YAML 覆盖
 → get_parameter 读取
 → 校验 / clamp / profile 强制覆盖
 → 算法成员变量实际使用
```

`ros2 param get` 读取的是 ROS 参数服务器中的值。如果程序启动时把它拷贝进成员变量，又在代码里覆盖成员变量，显示值可能不同于实际有效值。

当前 v11 的 `rescue_reliability_profile` 会强制设置细对准高度、门限和部分搜索参数，这个区别尤其重要。[P1: load_parameters]

## 21.2 常用命令

```bash
ros2 param list /lab/sim_plant
ros2 param get /lab/sim_plant rate_hz
ros2 param describe /lab/sim_plant rate_hz
ros2 param dump /lab/sim_plant > sim_plant_params.yaml
```

修改参数只用于教学：

```bash
ros2 param set /lab/vision_stub empty_frames true
ros2 param set /lab/vision_stub empty_frames false
```

本例 `empty_frames` 在每次回调时读取，因此修改后会被消费。模拟器 `rate_hz` 启动时用于创建 Timer，运行中仅改参数不会自动重建 Timer。

## 21.3 YAML 节点名必须匹配

```yaml
/lab/vision_stub:
  ros__parameters:
    pipeline_delay_s: 0.10
    empty_frames: false
```

`ros__parameters` 有两个下划线。`0.10` 是数值，不是字符串；`false` 是布尔值，不是 `'false'` 字符串。

参数文件中写了错误节点名，节点可能继续用默认值，且程序照样运行。

## 21.4 C++ 动态参数校验示例

```cpp
// 局部示例：须 #include "rcl_interfaces/msg/set_parameters_result.hpp"
// 回调句柄必须保存为成员，实际使用处也要读取或安全更新参数。
parameter_callback_ = add_on_set_parameters_callback(
  [](const std::vector<rclcpp::Parameter> & changes) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : changes) {
      if (p.get_name() == "preview_speed_m_s") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            !std::isfinite(p.as_double()) || p.as_double() <= 0.0) {
          result.successful = false;
          result.reason = "preview speed must be positive and finite";
          return result;
        }
      }
    }
    return result;
  });
```

不要在校验回调中先改变硬件再决定拒绝；校验和提交应保持清晰。关联参数要整体验证，避免一个更新生效、另一个未生效导致不一致。[R19]

## 21.5 比赛配置建议

把配置分为几何标定、任务策略、飞控连接、测试模式。飞行中不允许任意改外参、投放通道或场地坐标。

启动时输出一份 **effective configuration**，保存真正参与计算的值，而不仅是 YAML 原文。记录覆盖顺序和配置文件哈希。

**实践 7**：修改 `rate_hz` 但不重启，观察频率是否变化；再重启比较。用实验解释“参数存在”和“程序支持动态调整”的区别。

<a id="c22"></a>
# 22. Launch：从七个终端到一个可审计入口

## 22.1 Launch 管的是进程组织，不自动保证就绪

写成先启动相机、再启动任务节点，不代表相机已经能提供有效画面。`TimerAction` 延迟几秒，也只是等待，不是健康验证。

正确组合是：Launch 负责启动；节点状态机负责基于消息、服务和期限判断 readiness。[R20]

## 22.2 完整教学启动文件

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare('cuadc_tutorial_bringup'),
                                   'config', 'lab.yaml'])
    mission = LaunchConfiguration('mission')
    nodes = []
    for executable in ['sim_plant', 'vision_stub', 'perception_guard',
                       'payload_server', 'tf_demo']:
        nodes.append(Node(package='cuadc_tutorial_py', executable=executable,
                          name=executable, namespace='lab',
                          parameters=[params], output='screen', respawn=False))
    nodes.append(Node(package='cuadc_tutorial_cpp', executable='telemetry_observer',
                      name='telemetry_observer', namespace='lab',
                      parameters=[params], output='screen', respawn=False))
    nodes.append(Node(package='cuadc_tutorial_py', executable='mission_dry_run',
                      name='mission_dry_run', namespace='lab',
                      parameters=[params], output='screen', respawn=False,
                      condition=IfCondition(mission)))
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='false',
                              description='Start software-only mission preview'),
        *nodes,
    ])
```

`LaunchConfiguration` 是运行时替换对象，不是普通 Python bool。因此不能写 `if LaunchConfiguration('mission'):` 作为条件；这里用 `IfCondition`。

`FindPackageShare` 让配置路径相对于安装包定位，不依赖当前工作目录，也不硬编码 `/home/某人`。

## 22.3 完整教学参数文件

```yaml
/lab/sim_plant:
  ros__parameters:
    rate_hz: 30.0
    max_speed_m_s: 1.0
    pause_after_s: -1.0
    pause_duration_s: 0.0
/lab/vision_stub:
  ros__parameters:
    pipeline_delay_s: 0.10
    empty_frames: false
    invalid_frames: false
    freeze_stamp: false
    future_stamp_s: 0.0
    pause_after_s: -1.0
    pause_duration_s: 0.0
/lab/mission_dry_run:
  ros__parameters:
    flight_enable: false
/lab/telemetry_observer:
  ros__parameters:
    force_reliable: false
```

启动：

```bash
ros2 launch cuadc_tutorial_bringup lab.launch.py
ros2 launch cuadc_tutorial_bringup lab.launch.py mission:=true
ros2 launch cuadc_tutorial_bringup lab.launch.py --show-args
```

第二条只是软件任务预览，仍然不连接飞控。

## 22.4 Launch 参数与节点参数

`mission:=true` 是 Launch 参数，由启动脚本决定是否启动某节点。

`flight_enable: false` 是节点参数，由节点代码读取。名字不同、处理层次也不同。不是所有 `name:=value` 都会自动送入节点参数服务器。

## 22.5 为什么默认不 respawn 任务节点

空中任务节点崩溃后从 WAIT_FCU 重启，可能失去已投载荷、冻结目标和任务阶段。如果自动重启逻辑没有会话恢复与执行权限门禁，反而更危险。

相机等设备进程是否允许重启可以单独设计；任务执行器与载荷节点的自动重启必须保守。

**本工程所有节点 `respawn=False` 是教学设计，不等于现实系统所有进程永远不允许自恢复。**

## 22.6 Launch 安装规则

```cmake
cmake_minimum_required(VERSION 3.8)
project(cuadc_tutorial_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
```

文件放在源码目录但没有安装，可能在开发机偶然能运行，换机就找不到。配置和 launch 必须进入安装树。

<a id="c23"></a>
# 23. 日志、诊断与任务事件

## 23.1 日志要回答“为什么”，不只回答“发生了什么”

不好的日志：

```text
release failed
```

更有用的日志：

```text
state=RELEASE reason=motion_gate
horizontal_speed=0.23 limit=0.15
outlet_error=0.09 limit=0.20
vision_mode=frozen_target
request_sent=false
```

这样才能区分未对准、运动不稳、身份错误或已经发命令但没有 ACK。

## 23.2 日志等级

DEBUG 用于高频细节；INFO 用于正常阶段变化；WARN 表示降级但可能继续；ERROR 表示当前过程失败或需要保护；FATAL 表示不可继续运行的严重问题。[R21]

把每帧信息都打印成 ERROR，会让真正的故障淹没。

## 23.3 节流日志

```cpp
// 局部示例，时间单位为毫秒。
RCLCPP_WARN_THROTTLE(
  get_logger(), *get_clock(), 1000,
  "Waiting for valid odometry");
```

节流减少重复打印，但若节流时钟使用 ROS time，暂停仿真或回放跳时会影响日志节拍。诊断信息缺失不能直接解释成回调没运行。

## 23.4 结构化事件

```text
std_msgs/Header header
string session_id
uint64 sequence
string previous_state
string state
string reason
```

配套软件任务每次状态变化发布该事件，便于 bag 回放后统计各阶段时长。会话号避免两次运行的事件被混在一起。

事件流与“当前状态”同时存在：事件用于追溯，当前状态供监视器晚加入时快速了解现况。

## 23.5 诊断 Topic 与 `/rosout`

`/rosout` 适合收集日志，但复杂指标最好使用结构化消息。生产工程可使用 `diagnostic_msgs` / `diagnostic_updater` 组织健康状态。[R21A]

建议暴露 odom 最大间隔、视觉年龄、对齐成功比例、当前门禁、请求等待时间、队列长度及有效参数快照。

**实践 8**：启动软件任务后记录 `/lab/mission/events`，输出每个状态进入时刻。不要靠人工看终端估计耗时。

<a id="c24"></a>
# 24. Executor 与回调调度：为什么有数据也来不及处理

## 24.1 单线程并不意味着系统只有一个线程

SingleThreadedExecutor 表示被它管理的回调通常由一个执行线程调度；底层 DDS、设备 SDK 或进程外节点仍可能有自己的线程。

一个耗时 300 ms 的回调，会影响同执行器下的 odom 回调、20 Hz Timer 和 Service 响应处理。[R09]

## 24.2 20 Hz 控制周期的预算

周期 50 ms 不代表你可以在 Tick 内放心用满 50 ms。还要留出 odom、状态、服务回复等任务和系统调度余量。

```text
50 ms 周期
├── 必要计算
├── 输入回调与消息分发
├── 操作系统调度抖动
└── 安全余量
```

把 YOLO 推理、JPEG 写盘或阻塞相机采集放进同一控制回调，会把设备延迟传给任务时序。

## 24.3 v11 的调度结构

`create_wall_timer(50ms, tick)` 建立名义 20 Hz Tick；`main()` 使用普通 `rclcpp::spin`；输入回调更新导航、视觉及服务相关状态。[P1]

Tick 中先处理部分健康和 ACK，再按 `publish_setpoint_` 发布 setpoint，然后执行状态分支更新目标。也就是说，目标通常在更新后的下一个发布周期送出；这是一项源码时序事实，不应把整条链路当零延迟。

## 24.4 频率、延迟和抖动

频率：单位时间处理多少次。

延迟：一个观测从采集到被使用用了多久。

抖动：间隔或延迟的变化程度。

一个 30 Hz、但总延迟 500 ms 的视觉链路，不能等价于一个 30 Hz、延迟 50 ms 的链路。

**验收**：给出一次最慢回调的耗时，而不仅是平均 CPU 使用率。


<a id="c25"></a>
# 25. 并发、回调组和有界队列

## 25.1 多线程不是给 `spin()` 换个名字

把 SingleThreadedExecutor 换成 MultiThreadedExecutor 后，是否真正并行还取决于 callback group。默认回调组通常是 MutuallyExclusive，同一组中的回调不能同时执行。Reentrant 允许并发，但也把共享数据竞争问题交给程序员。[R22]

【局部示例：C++ 中分开输入组和控制组】

```cpp
input_group_ = create_callback_group(
  rclcpp::CallbackGroupType::MutuallyExclusive);
control_group_ = create_callback_group(
  rclcpp::CallbackGroupType::MutuallyExclusive);

rclcpp::SubscriptionOptions options;
options.callback_group = input_group_;
odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
  "odom", rclcpp::SensorDataQoS(),
  [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    last_odom_ = *msg;
  }, options);

timer_ = create_wall_timer(
  std::chrono::milliseconds(50),
  [this]() { tick(); }, control_group_);
```

这里需要类成员 `std::mutex mutex_`、两个回调组、订阅器和 Timer 的 SharedPtr。若 Tick 读取 `last_odom_`，也必须用同一把锁；只锁写入一侧没有意义。

## 25.2 更推荐“短锁复制，锁外计算”

```cpp
nav_msgs::msg::Odometry snapshot;
{
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot = last_odom_;
}
// 下面的几何计算不持锁。
```

不要拿着共享锁等待 Service 回复、写文件或执行推理。否则一个资源锁就可能把整个多线程程序重新串行化。

对于当前 v11，不应该仅为“提速”直接改成多线程：`known_buckets_`、导航历史、活动目标和状态变量之间没有因此自动获得线程安全。[P1]

## 25.3 图像处理采用有界队列

实时任务常见策略不是“所有帧都处理”，而是“处理足够新的帧”。一种教学架构是：

```text
采集线程 → 容量 1～2 的待处理队列 → 推理线程 → ROS 发布
```

队列满时可以丢弃旧帧，但必须在接口文档中写明策略。采集时间戳必须随对应图像一起传递，不能在推理完成后重新盖上当前时间。

投放命令不能照搬这种“旧的随便丢”策略。传感器流、可覆盖的 setpoint、不可逆动作是三类不同的数据。

## 25.4 进程隔离与组件组合

ROS 2 支持把多个组件装进同一进程，从而提供进程内通信等优化机会。但组件组合也会扩大崩溃和调度互相影响的范围。[R09][R39]

对新人团队，优先把“相机 / 推理”“任务状态机”“记录与显示”按职责分开，再通过测量决定是否组合。不能为了少开几个终端，把所有耗时工作放进一个 Node。

**实验**：仅在教学工程中让一个订阅回调延迟 100 ms，观察 odom 最大间隔与 Timer 输出。先解释现象，再考虑增加线程。

<a id="c26"></a>
# 26. 三种时间：ROS 时间、系统时间、单调时间

## 26.1 时间戳不是一个没有单位的数字

一个时间值至少要说明：时钟来源、纪元、单位、对应事件。

例如“相机 15230 ms”可能是设备开机后的硬件时间；“ROS now”可能是系统 Unix 时间；二者不能直接相减，即使都能转换成 `double`。

ROS 2 的时钟设计区分 SystemTime、SteadyTime 和 ROSTime。启用 `use_sim_time` 后，ROS 时间可以由 `/clock` 驱动，也可能暂停或跳变。[R23]

## 26.2 不同用途选不同时间

| 用途 | 常用时间基础 | 需要注意 |
|---|---|---|
| 比较视觉和 odom 的观测时间 | 已统一时钟域的 ROS 时间戳 | “统一”必须有可说明的方法 |
| 记录航班的日历时间 | 系统时间 | 可能被校时改变 |
| 判断回调多久没收到 | 单调时间 | 不因系统改时间而倒退 |
| rosbag 仿真回放 | `/clock` 驱动的 ROS 时间 | 不能自动改变 `time.monotonic()` |
| 控制周期实际执行间隔 | 单调时间 | 应测量实际 dt，而非假定总是 0.05 s |

C++ 中，`std::chrono::steady_clock` 适合测持续时间；ROS 消息的 `Header.stamp` 使用 ROS 定义的时间字段。Python 中，对应的持续时间测量可用 `time.monotonic()`。

## 26.3 代码中的两种 now

```python
capture_stamp = self.get_clock().now().to_msg()
arrival_monotonic = time.monotonic()
```

第一项适合放进消息；第二项适合做本进程的 watchdog。单调时钟的数值不要作为多台计算机可直接比较的公共时间戳。

```cpp
const auto ros_time = now();
const auto steady_time = std::chrono::steady_clock::now();
```

二者都叫“现在”，却不是相同的数据类型，也不是相同的物理契约。

## 26.4 `/clock` 暂停时会发生什么

若 ROS 时间停止，ROS 时间驱动的逻辑可能不再推进；而 wall timer、steady timeout 或设备线程仍可能继续执行。于是“回放暂停一分钟”可能让实际 watchdog 判定断流。

这是混合时钟的自然结果，不应仅凭“回放报超时”就认定实飞也会报同样故障。

## 26.5 时间回跳必须有状态重置方案

回放循环、仿真重置、设备重启都可能让时间变小。目标关联、轨迹采样、历史缓存和请求去重不能无条件沿用旧状态。

【教学设计】为每次任务建立 `session_id`，为每个传感器启动周期建立 `stream_epoch`；遇到时钟回跳时清理对应历史，明确是否允许任务继续，而不是仅把负 dt 改成 0。

<a id="c27"></a>
# 27. v11 的视觉—导航同步：实际做了什么，没有做什么

## 27.1 源码的真实时间链

v11 的 `odom_callback()` 不用收到的 odom `header.stamp` 作为历史时间，而是在回调内调用 `now()`，把**机载电脑接收 / 执行回调的时间**存入 `navigation_history_`。[P1]

源码注释约定：相机侧在 `wait_for_frames()` 返回后采样机载电脑的 ROS 时间，作为视觉消息时间戳。此次仅凭 C++ 文件，不能验证上游 Python 节点确实如此实现。

```text
相机真实曝光 t_camera
   └─ USB / SDK 缓冲 → wait_for_frames 返回 → 标记 t_vision

飞控真实状态采样 t_fcu
   └─ MAVLink / USB / MAVROS / Executor → odom 回调 → 标记 t_odom
```

统一电脑时钟解决的是“时间数字处于同一时钟域”；它不自动消除相机缓冲和飞控传输延迟。

## 27.2 严格的是“插值条件”

对每一帧视觉，`navigation_sample_at()` 找到前后两个 odom 样本：

\[
t_0\le t_v\le t_1
\]

并要求：

\[
0<t_1-t_0\le 0.050\text{ s}
\]

没有前后夹住就不能用最近邻代替，也不向未来外推。位置线性插值，roll / pitch / yaw 按角度最短方向插值。[P1]

严格拒绝坏区间能防止明显陈旧的姿态进入几何计算，但不能证明 `t_v` 与 `t_odom` 对应的**物理采样事件**完全同步。

## 27.3 为什么要待处理队列

视觉消息到达时，晚于其时间戳的那个 odom 可能还没有执行回调。源码允许帧进入 `pending_vision_frames_`，默认再等 0.15 s；期间新 odom、视觉回调和 Tick 都可能推进处理。

等待的是“后一个真实样本出现”，不是人为把视觉延迟补成某个固定值。

## 27.4 这些量不要混在一起

| 参数或状态 | v11 默认 / 实现 | 含义 |
|---|---:|---|
| odom 历史 | 5 s | 可查询的接收时间历史范围 |
| 插值最大间隔 | 50 ms | 相邻导航样本允许的最大空洞 |
| pending 等待 | 150 ms | 暂时缺后样本时等待的最长时间 |
| 视觉流水线延迟上限 | 1.5 s | 消息收到时与其时间戳的差 |
| 视觉心跳超时 | 1.5 s | 消息接收是否断流 |
| 起飞前有效同步帧数 | 至少 3 帧 | 不是要求看到三个桶 |
| NAV30 ready 新鲜度 | 3 s | 外部 ready 信号是否仍有效 |

以上是源码默认或代码约束，不是已测系统精度。[P1]

## 27.5 需要进一步验证的三个实验

静止飞机时持续看同一目标，测世界坐标抖动；水平匀速往返，比较方向改变时的系统偏差；缓慢改变 roll / pitch，检查补偿后目标是否固定。

若目标误差随运动方向反号，优先调查时间偏差；若误差随距离放大，优先调查角度或标定；若始终固定偏一侧，优先调查平移外参。这里是诊断假设，不是单凭现象就能唯一确定原因。

<a id="c28"></a>
# 28. 实践：把时间与几何函数写成不依赖 ROS 的单元测试

## 28.1 为什么把纯计算从 Node 中拿出来

插值、角度归一化和连续稳定门禁不需要启动 MAVROS。把它们写成普通函数，就能在没有 ROS 的计算机上验证边界情况。

下面的 Python 实验演示与 v11 相近的“严格夹取”原则，但不是替换其导航模块的补丁。

**完整文件：`src/cuadc_tutorial_py/cuadc_tutorial_py/math_core.py`**

```python
"""Pure functions: usable by pytest without importing ROS."""
from bisect import bisect_left
from dataclasses import dataclass
import math


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def rotate_rpy(v, roll, pitch, yaw):
    x, y, z = v
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    yr, zr = cr * y - sr * z, sr * y + cr * z
    xp, zp = cp * x + sp * zr, -sp * x + cp * zr
    return (cy * xp - sy * yr, sy * xp + cy * yr, zp)


@dataclass(frozen=True)
class Sample:
    t_ns: int
    x: float
    yaw: float


def interpolate_bracket(samples, t_ns, max_gap_ns=50_000_000):
    """演示 v11 的严格前后夹逼；只插值 x 和 yaw 以便理解。

    samples 必须严格递增；等于第一个点时也缺少 before，因此拒绝。
    不做最近邻替代，不做外推。
    """
    if len(samples) < 2:
        return None
    if any(a.t_ns >= b.t_ns for a, b in zip(samples, samples[1:])):
        return None
    times = [s.t_ns for s in samples]
    i = bisect_left(times, t_ns)
    if i == 0 or i == len(samples):
        return None
    a, b = samples[i - 1], samples[i]
    gap = b.t_ns - a.t_ns
    if gap <= 0 or gap > max_gap_ns:
        return None
    r = (t_ns - a.t_ns) / gap
    return Sample(t_ns, a.x + r * (b.x - a.x),
                  wrap_angle(a.yaw + r * wrap_angle(b.yaw - a.yaw)))


class StableGate:
    def __init__(self, duration_s):
        if not math.isfinite(duration_s) or duration_s < 0:
            raise ValueError('invalid duration')
        self.duration_s = duration_s
        self.since = None
        self.last_time = None

    def update(self, ok, now_s):
        if not math.isfinite(now_s):
            self.since = self.last_time = None
            return False
        backwards = self.last_time is not None and now_s < self.last_time
        self.last_time = now_s
        if not ok or backwards:
            self.since = None
            return False
        if self.since is None:
            self.since = now_s
        return now_s - self.since >= self.duration_s
```

## 28.2 测试比示例数据更重要

**完整文件：`src/cuadc_tutorial_py/test/test_math_core.py`**

```python
import math
import pytest
from cuadc_tutorial_py.math_core import Sample, StableGate, interpolate_bracket, rotate_rpy


def test_identity():
    assert rotate_rpy((1, 2, 3), 0, 0, 0) == (1, 2, 3)


def test_yaw_quarter_turn():
    assert rotate_rpy((1, 0, 0), 0, 0, math.pi / 2) == pytest.approx((0, 1, 0))


def test_roll_outlet():
    y = rotate_rpy((0, 0, -0.32), math.radians(8), 0, 0)[1]
    assert y == pytest.approx(0.32 * math.sin(math.radians(8)))


def test_interpolation_and_wrapped_yaw():
    s = [Sample(0, 0, math.radians(179)), Sample(40_000_000, 4, math.radians(-179))]
    out = interpolate_bracket(s, 20_000_000)
    assert out.x == pytest.approx(2)
    assert abs(out.yaw) == pytest.approx(math.pi)


def test_reject_large_gap():
    assert interpolate_bracket([Sample(0, 0, 0), Sample(60_000_000, 1, 0)],
                               20_000_000) is None


def test_reject_outside():
    s = [Sample(0, 0, 0), Sample(40_000_000, 1, 0)]
    assert interpolate_bracket(s, -1) is None
    assert interpolate_bracket(s, 50_000_000) is None
    assert interpolate_bracket(s, 0) is None


def test_reject_duplicate_times():
    assert interpolate_bracket([Sample(0, 0, 0), Sample(0, 1, 0)], 0) is None


def test_stable_gate_and_reset():
    gate = StableGate(0.5)
    assert not gate.update(True, 0)
    assert not gate.update(True, 0.4)
    assert gate.update(True, 0.5)
    assert not gate.update(False, 0.6)
    assert not gate.update(True, 0.7)
    assert gate.update(True, 1.3)


def test_clock_backwards():
    gate = StableGate(0.5)
    gate.update(True, 10)
    assert not gate.update(True, 9)
```

在工作空间根目录运行：

```bash
PYTHONPATH="$PWD/src/cuadc_tutorial_py${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m pytest -q src/cuadc_tutorial_py/test/test_math_core.py
```

这组测试不需要 `rclpy`，只需要 Python 与 pytest。不要把纯函数通过测试理解为整个 ROS 控制系统已经通过测试。

## 28.3 每个边界对应实际故障

179° 到 -179° 应经过很短的角差，而不是转过 358°；一个恰好位于最早历史样本处的查询，在此教学实现中按 v11 的方式拒绝，因为没有更早的夹取样本；50 ms 以上的空洞不接受；稳定 0.49 s 后条件失效，必须重新计时。

这些测试能阻止“修改数学函数后，正常样本看似没问题，极端情况下却错”的退化。

**扩展作业**：加入 roll / pitch 的插值测试、非有限数值测试和 1000 条随机旋转逆变换测试。随机测试需固定随机种子，并记录失败输入。

<a id="c29"></a>
# 29. 坐标系：ROS、飞控、场地、机体和相机不是同一个原点

## 29.1 先把约定写成表

| 名称 | 常见方向约定 | 本项目要核对的原点 |
|---|---|---|
| ROS local / ENU | X 东、Y 北、Z 上 | 飞控导航本地参考原点 |
| 飞控 NED | X 北、Y 东、Z 下 | 飞控使用的导航原点 |
| 机体 FLU | X 前、Y 左、Z 上 | 视觉和投放外参共同采用的机体参考点 |
| 机体 FRD | X 前、Y 右、Z 下 | 飞控 / MAVLink 具体消息约定 |
| 相机 optical | X 右、Y 下、Z 前 | 对应相机成像坐标系原点 |
| 比赛 field | X 锁定航向前方、Y 左、Z 上 | v11 锁定的 mission home |

ROS 标准约定见 REP-103；`map`、`odom`、`base_link` 的通常职责见 REP-105。MAVROS会进行相应的坐标约定转换，但不能替你猜测自定义视觉的物理安装外参。[R24][R25][R28B]

## 29.2 `frame_id="map"` 不会自动旋转数字

```python
msg.header.frame_id = 'map'
msg.point.x = 1.0
```

这只是声称“这个 1.0 在 map 中”。如果数字实际来自相机坐标，改字符串就是给错误数据贴上正确标签，错误会更隐蔽。

同样，把节点 namespace 改为 `/uav1` 不会自动把 `base_link` 变成 `uav1/base_link`。

## 29.3 v11 如何锁定场地

`home_ = position_`；`mission_yaw_ = mean_heading()`；field 到 local 的平移和旋转为：

\[
\begin{bmatrix}x_L\\y_L\end{bmatrix}
=
\begin{bmatrix}x_H\\y_H\end{bmatrix}
+R_z(\psi_0)
\begin{bmatrix}x_F\\y_F+\delta_y\end{bmatrix}
\]

`field_lateral_offset_m` 就是 \(\delta_y\)。正值整体向初始机头左侧平移，负值向右。返回 home 的原始坐标不应因此被当作另一个起飞点。[P1]

## 29.4 罗盘角和 ENU yaw

源码使用：

\[
\psi_{ENU}=\operatorname{wrap}((90^\circ-heading)\pi/180)
\]

因为罗盘角通常从北方向顺时针，而 ENU yaw 从东向逆时针。实际接口约定必须按数据源确认，不能把所有“heading”字段都套进这个公式。

## 29.5 原点到底是飞控、RTK，还是重心

仅凭一个 `/mavros/local_position/odom` 话题，不能可靠断言其物理参考点就是 RTK 天线中心、机架中心或可见飞控外壳中心。要核对飞控估计输出的定义、安装参数、传感器位置补偿和你们外参测量的参考点。

对代码最重要的契约是：`p_vehicle` 与 `r_release` 必须以同一个机体参考点相接。把“相对重心测的外参”加到“相对另一参考点的位姿”上，会留下固定误差。

## 29.6 四元数不等于四个角

ROS 消息通常按 x、y、z、w 存储四元数。单位姿态是 `(0,0,0,1)`，不是四个零。使用姿态前应检查有限性和范数，必要时归一化；严重无效数据应该拒绝，而不是悄悄沿用成“新鲜姿态”。[R26]

v11 对正常四元数归一化后提取 RPY；范数过小时沿用之前的角度，这是当前实现行为，不是本文推荐的通用安全策略。[P1]

<a id="c30"></a>
# 30. tf2：让坐标关系成为可查询的系统接口

## 30.1 tf2 与手写矩阵的关系

tf2 管理随时间变化的坐标变换关系；它不会替你完成外参标定，也不会判断一个传感器时间戳是否对应真实曝光时刻。

v11 主要自己维护导航历史和刚体变换，并没有因为消息里出现 `frame_id` 就自动调用 tf2。以下是独立教学模块。[P1][R26]

## 30.2 教学 TF 树

```text
lab_map
  └── lab_base_link        动态：来自软件 odom
        ├── lab_release_1  静态：A1 机构外参
        └── lab_release_2  静态：A2 机构外参
```

树中每个 child 应有清晰的唯一父节点，不能同时让两个发布者给出互相冲突的同一变换。

## 30.3 完整发布程序

**完整文件：`src/cuadc_tutorial_py/cuadc_tutorial_py/tf_demo.py`**

```python
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from .common import require_lab_namespace, spin_main


class TfDemo(Node):
    def __init__(self):
        super().__init__('tf_demo')
        require_lab_namespace(self)
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, 'odom', self.on_odom,
                                            qos_profile_sensor_data)
        transforms = []
        for child, xyz in [('lab_release_1', (0.026, -0.065, -0.32)),
                           ('lab_release_2', (-0.026, 0.065, -0.32))]:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'lab_base_link'
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            t.transform.rotation.w = 1.0
            transforms.append(t)
        self.static.sendTransform(transforms)

    def on_odom(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        p = msg.pose.pose.position
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation = msg.pose.pose.orientation
        self.dynamic.sendTransform(t)


def main(args=None):
    spin_main(TfDemo, args)
```

静态变换在 `/tf_static` 使用适合晚加入订阅者的持久性；动态姿态通常在 `/tf`。这并不意味着它们具有无限的历史缓存。[R26]

## 30.4 查询和检查

启动教学 Launch 后：

```bash
ros2 run tf2_ros tf2_echo lab_map lab_base_link
ros2 run tf2_ros tf2_echo lab_map lab_release_1
ros2 run tf2_tools view_frames
```

`view_frames` 可能生成本地图文件，输出路径以工具打印为准。`tf2_echo` 只能说明 TF 链可查询，不能证明变换与物理安装一致。

【独立小实验】不运行上述 TF 节点时，可以用命令发布同一静态关系：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.026 --y -0.065 --z -0.32 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id lab_base_link --child-frame-id lab_release_1
```

不要同时运行两个同名 child 的发布方式。

## 30.5 查询目标时间而非一律“最新”

要转换一帧旧图像，应查询其观测时刻的变换；一律使用最新 TF，会重新引入运动时差。查询失败通常包括树未连接、请求时间早于缓存或晚于最新数据、时钟域不一致。

【局部示例】

```python
from rclpy.time import Time
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException

# 构造时保留成员 self.tf_buffer / self.tf_listener。
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# 非实时教学调用；在控制回调内优先采用不阻塞策略。
try:
    tr = self.tf_buffer.lookup_transform(
        'lab_map', 'lab_base_link', Time.from_msg(msg.header.stamp),
        timeout=Duration(seconds=0.0))
except TransformException as exc:
    self.get_logger().warning(f'该观测缺少变换，暂不使用: {exc}')
```

TransformListener 自己也需要 Executor 执行回调。单线程里一边长时间阻塞等 TF，一边希望同线程收到 TF，可能产生不必要的等待或死锁。

<a id="c31"></a>
# 31. 标定接口：相机、机体、投放口应怎样交接

## 31.1 两条变换链必须闭合

视觉目标：

\[
p_{bucket}^{W}=p_B^{W}+R_B^{W}\left(t_C^B+R_C^B p_{bucket}^{C}\right)
\]

机构位置：

\[
p_{release}^{W}=p_B^{W}+R_B^{W}r_{release}^{B}
\]

相机外参和机构外参都必须相对于同一个 B。前一条链的 camera optical 方向也必须与相机反投影输出一致。

## 31.2 本源码明确给出的机构外参

默认参数为：

```text
A1 / SERVO9:  (+0.026, -0.065, -0.32) m
A2 / SERVO10: (-0.026, +0.065, -0.32) m
```

这些数值是上传源码的默认值；它们是否仍匹配最新机体，必须靠实际装配和标定确认。**本文没有从这份 C++ 中获得 D435i 的完整相机外参。**[P1]

## 31.3 接口适配的责任边界

视觉适配器负责“相机坐标 → 正确 body 坐标”；状态机负责“采集时 body → local”；机构模型负责“当前 body → 出瓶口 local”。

如果视觉适配器已经做过某个旋转，状态机不得再次做同一旋转。最常见的错误之一就是“为了保险，两边都补偿了一次”。

## 31.4 地面标定实验

拆桨、机构安全隔离后，把明确可测的标记放在正前、正左、正右、正下方，核对 body 数字符号。然后只旋转飞机，不移动标记，检查 world 目标是否近似不动。

记录相机序列号、分辨率、内参版本、外参参考点、机体朝向、两个机构的独立偏置。更换相机支架后，原 YAML 不应无条件沿用。

## 31.5 与人工准星标定的关系

人工准星帮助建立“什么画面位置对应真实出瓶口”的经验关系；ROS 接口则需要把这个关系变成明确的坐标、单位和参数版本。本文未读取前面对话中的标定 ZIP，因此不声称知道其节点名或输出格式。

<a id="c32"></a>
# 32. 图像、D435i 与 ROS 数据流水线

## 32.1 `sensor_msgs/Image` 不是一个 JPEG 文件名

图像消息包含高度、宽度、编码、每行步长和像素数据；CameraInfo 携带标定相关信息。不能只看数组长度就假定是 RGB，也不能把 BGR 当 RGB 直接用于颜色识别。[R27]

常见编码包括 `rgb8`、`bgr8`、`mono8`；深度数据还要区分整数原始单位和浮点米制。不同驱动的具体话题名与深度单位要查实际消息和驱动配置，不在本文猜成固定值。

## 32.2 三种架构都可以，但不要混用责任

```text
A. 相机 ROS 驱动 → Image + CameraInfo → 独立视觉节点 → 检测
B. Python 直接调用设备 SDK → 内部推理 → 检测 ROS 话题
C. 相机 ROS 驱动 → 共享内存 / 组件处理 → 检测
```

上传 C++ 只消费 `/perception/drop_buckets_body`，它不要求一定采用 A，也不能证明相机上游是哪一种具体实现。[P1]

## 32.3 带宽算一遍就明白为什么不要全录原图

一张 1280×720、三通道、每通道 1 字节的未压缩图像约为：

\[
1280\times720\times3=2{,}764{,}800\text{ byte}
\]

30 fps 时仅像素数据约 82.9 MB/s，还未计入其他开销。这是给定尺寸下的算术估算，不是你们 D435i 当前实际带宽。

把所有 RGB、深度、对齐深度、点云同时跨无线网传输，会挤占真正重要的状态和检测数据。任务节点通常只需要小型检测结果，不需要持有整个图像缓冲区。

## 32.4 Pipeline 的测量点

```text
设备采集 → SDK 返回 → 推理开始 → 推理结束 → publish
                                      ↓
                                   接收回调 → 时间对齐成功 → 控制使用
```

每一步分别记录时间，才知道瓶颈在哪里。只测“模型推理用了 20 ms”，不能说明整条视觉链只有 20 ms。

## 32.5 不阻塞的图像查看

使用 rqt 图像查看时，先看实际话题，再选择工具支持的 transport。图像压缩有 CPU 代价，不能认为“压缩后一定更实时”。先记录 CPU、端到端年龄和掉帧，再决定传输策略。[R27A][R27B]

**实践 9**：为一条真实视觉输入写一页接口契约。必须包括空帧、设备断线、时间回跳、深度无效和模型未加载时的行为。


<a id="c33"></a>
# 33. 实践：写一个视觉替身和健康检查节点

## 33.1 先接“假视觉”，再接模型

如果状态机一接 YOLO 就不工作，你需要区分：图像有问题、模型没检测到、ROS 消息不匹配，还是控制门禁没通过。

视觉替身把图像算法先拿掉，用确定的世界目标与软件 odom 生成检测。它的用途是测试接口，不是证明真实识别能力。

## 33.2 完整视觉替身

**完整文件：`src/cuadc_tutorial_py/cuadc_tutorial_py/vision_stub.py`**

```python
"""Synthetic detections with capture stamps and delayed publication."""
from collections import deque
import math
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from cuadc_tutorial_interfaces.msg import Bucket, BucketArray
from .common import require_lab_namespace, spin_main


class VisionStub(Node):
    def __init__(self):
        super().__init__('vision_stub')
        require_lab_namespace(self)
        self.declare_parameter('pipeline_delay_s', 0.10)
        self.declare_parameter('empty_frames', False)
        self.declare_parameter('invalid_frames', False)
        self.declare_parameter('freeze_stamp', False)
        self.declare_parameter('future_stamp_s', 0.0)
        self.declare_parameter('pause_after_s', -1.0)
        self.declare_parameter('pause_duration_s', 0.0)
        self.delay = float(self.get_parameter('pipeline_delay_s').value)
        if not math.isfinite(self.delay) or not 0 <= self.delay <= 5:
            raise ValueError('pipeline_delay_s must be in [0, 5]')
        self.start = time.monotonic()
        self.seq = 0
        self.counter = 0
        self.first_stamp_ns = None
        self.pending = deque(maxlen=200)
        self.pub = self.create_publisher(BucketArray, 'buckets', qos_profile_sensor_data)
        self.sub = self.create_subscription(Odometry, 'odom', self.on_odom, qos_profile_sensor_data)
        self.timer = self.create_timer(0.01, self.flush)
        self.world_buckets = [(3.2, -0.6, 0.0, 0.15),
                              (3.5, 0.6, 0.0, 0.20),
                              (4.0, 0.0, 0.0, 0.25)]

    def on_odom(self, odom):
        self.counter += 1
        if self.counter % 3 != 0:
            return  # 导航 30 Hz 时，生成约 10 Hz 视觉。
        elapsed = time.monotonic() - self.start
        after = float(self.get_parameter('pause_after_s').value)
        duration = float(self.get_parameter('pause_duration_s').value)
        if after >= 0 and after <= elapsed < after + duration:
            return
        msg = BucketArray()
        stamp = odom.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if self.first_stamp_ns is None:
            self.first_stamp_ns = ns
        if self.get_parameter('freeze_stamp').value:
            ns = self.first_stamp_ns
        ns += int(float(self.get_parameter('future_stamp_s').value) * 1e9)
        msg.header.stamp.sec = ns // 1_000_000_000
        msg.header.stamp.nanosec = ns % 1_000_000_000
        msg.header.frame_id = 'lab_base_link'
        msg.frame_seq = self.seq
        self.seq += 1
        msg.valid = not self.get_parameter('invalid_frames').value
        msg.detail = 'synthetic; unit attitude; not a real camera'
        p = odom.pose.pose.position
        if msg.valid and not self.get_parameter('empty_frames').value:
            for i, (x, y, z, diameter) in enumerate(self.world_buckets):
                if math.hypot(x - p.x, y - p.y) > 2.2:
                    continue  # 仅圆形软件视域，不是 D435i 光学视场模型。
                b = Bucket()
                b.detection_id = i + 1
                b.center.x, b.center.y, b.center.z = x - p.x, y - p.y, z - p.z
                b.diameter_m = diameter
                b.confidence = 0.95
                msg.buckets.append(b)
        self.pending.append((time.monotonic() + self.delay, msg))

    def flush(self):
        now = time.monotonic()
        while self.pending and self.pending[0][0] <= now:
            _, msg = self.pending.popleft()
            self.pub.publish(msg)  # 这里不能把 header.stamp 更新为 now。


def main(args=None):
    spin_main(VisionStub, args)
```

这里有三个值得学的地方。

第一，消息的时间戳在“合成观测”时确定，延迟发布时不覆盖。第二，队列有长度上限，避免处理阻塞时无限吃内存。第三，正常无目标用有效空数组表达，设备或推理无效用 `valid=false` 表达。

替身直接复用 odom 时间戳、假设单位姿态，并用圆形距离筛选代替真实视场；它不是完整相机仿真。`world_buckets` 中的目标只是教学地图上的合成点，与比赛实际未知桶位置无关。

## 33.3 完整健康检查节点

**完整文件：`src/cuadc_tutorial_py/cuadc_tutorial_py/perception_guard.py`**

```python
"""Application health contract; not an odometry/image interpolation node."""
import math
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from cuadc_tutorial_interfaces.msg import BucketArray
from .common import require_lab_namespace, spin_main


class PerceptionGuard(Node):
    def __init__(self):
        super().__init__('perception_guard')
        require_lab_namespace(self)
        self.last_good_arrival = None
        self.last_stamp_ns = None
        self.last_seq = None
        self.good_count = 0
        self.bad_count = 0
        self.health_pub = self.create_publisher(Bool, 'vision_ready', 10)
        self.sub = self.create_subscription(BucketArray, 'buckets', self.on_frame,
                                            qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.tick)

    def on_frame(self, msg):
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        okay = msg.valid and msg.header.frame_id == 'lab_base_link'
        okay = okay and stamp_ns > 0 and -0.05 <= age <= 0.5
        okay = okay and (self.last_stamp_ns is None or stamp_ns > self.last_stamp_ns)
        okay = okay and (self.last_seq is None or msg.frame_seq > self.last_seq)
        for bucket in msg.buckets:
            vals = (bucket.center.x, bucket.center.y, bucket.center.z,
                    bucket.diameter_m, bucket.confidence)
            okay = okay and all(math.isfinite(v) for v in vals)
            okay = okay and 0.08 <= bucket.diameter_m <= 0.35 and 0 <= bucket.confidence <= 1
        if okay:
            self.last_stamp_ns = stamp_ns
            self.last_seq = msg.frame_seq
            self.last_good_arrival = time.monotonic()
            self.good_count += 1
        else:
            self.bad_count += 1
            self.good_count = 0
            if self.bad_count % 10 == 1:
                self.get_logger().warning(f'frame rejected: age={age:.3f}, seq={msg.frame_seq}')
        # 一个正常空数组可以刷新健康度，但不能增加目标数。

    def tick(self):
        fresh = (self.last_good_arrival is not None and
                 time.monotonic() - self.last_good_arrival <= 0.8)
        msg = Bool()
        msg.data = bool(fresh and self.good_count >= 3)
        self.health_pub.publish(msg)


def main(args=None):
    spin_main(PerceptionGuard, args)
```

检查包括：有效位、frame_id、时间戳非零、年龄、时间和序号单调、数值有限、尺寸与置信度范围。

这个节点**没有做 odom 夹取插值**，因此 `/lab/vision_ready` 不可冒充生产系统的“时间对齐已成功”，更不能 remap 成 `/cuadc/nav30_ready`。

## 33.4 三种“没桶”

| 情况 | 视觉链路是否健康 | 任务能否立刻选择目标 |
|---|---|---|
| 有效空数组持续到达 | 可以健康 | 不能 |
| `valid=false` 持续到达 | 数据到达，但不可用 | 不能 |
| 完全不再发布 | 心跳断流 | 不能 |

健康检查回答“传感器链是否可用”；目标选择回答“是否有足够可信的桶”。把二者合并为一个 Bool，会让起飞或搜索逻辑很难解释。

## 33.5 序号重置的后果

本教学 guard 要求 `frame_seq` 单调。如果单独重启视觉替身而保留 guard，新序号可能小于之前值，后续会被拒绝。这是故意保留的简化边界：正式系统应增加 `stream_epoch` 并设计受控重置流程，而不是让任意重启自动恢复任务。

**实践 10**：先设置 `empty_frames=true`，再设置 `invalid_frames=true`。解释为什么两者对健康输出的影响不同。

<a id="c34"></a>
# 34. 目标跟踪如何与 ROS 数据有效性结合

## 34.1 三层 ID 不应混用

一次检测框的序号、跨帧跟踪器的 Track ID、任务已经选定的目标身份，是不同概念。

v11 的 `BucketTrack.id` 由状态机内部建立；它并不是视觉节点直接提供的“1 号、2 号、3 号筒类别”。源码根据尺寸排序，选出两个较小目标，再按离飞机的距离决定投放顺序。[P1]

## 34.2 数据关联前先保证坐标可比较

两帧 body 坐标在飞机运动后会改变。要判断“是不是同一只静止桶”，应先基于对应观测时间变换到共同 local 坐标，再比较空间距离。

v11 的关联门默认是位置差不大于 0.45 m、直径差不大于 0.08 m。符合门限的候选按归一化代价排序，进行一对一贪心关联；它不是全局最优分配算法。[P1]

## 34.3 为什么位置和 body 使用不同滤波

源码对 local 位置做 EMA，默认 alpha=0.25；body 默认 alpha=1，保留最新观测。直径先用窗口中值，再平滑，并计算 MAD 判断尺寸稳定性。

这不是 ROS 自带的滤波能力，而是应用层算法。ROS 只负责把带时间和语义的数据送到它。[P1]

## 34.4 三桶选二、第二目标冻结

搜索途中优先获得至少三个稳定、空间独立的目标；完整航线结束时可放宽为两个。两目标计划记录 ID、位置、直径。对准时可以有限度更新同一目标，进入 RELEASE 后不再接收目标更新。

第一瓶完成后，第二瓶使用已选目标记忆，不回到完整 SEARCH。记忆对象的 `arrival` 刻意保持很旧，不能因为复制对象时调用了 `now()`，就把旧目标说成刚看见。[P1]

## 34.5 通信健康与目标置信度是两个状态

一个新鲜的错检仍是错检；一个延迟很大的高置信度检测也可能不适合闭环。任务门禁至少要分别考虑：消息到达、采集年龄、几何有效、跟踪一致、空间合法和是否已经投过。

【教学设计】不要用一个 `bool detected` 承担所有含义。可以定义结构体保存判定原因，并发布诊断结果供日志解释。

<a id="c35"></a>
# 35. MAVROS：ROS 图与飞控之间的协议桥

## 35.1 数据经过哪些层

```text
任务 C++ / Python
  ↓ ROS 消息或 Service
MAVROS 插件
  ↓ MAVLink 消息 / Command
串口、UDP 或 TCP
  ↓
ArduPilot
```

MAVROS 不是 ArduPilot，也不只是一个“打开串口的脚本”。它包含多类插件，将飞控状态、位置、命令和参数接口映射到 ROS。[R28]

## 35.2 先区分 ArduPilot GUIDED 与 PX4 OFFBOARD

你们源码等待 `mode == "GUIDED"`，并调用 ArduPilot 使用的起飞 / 降落命令路径。不要因为项目最早文件名叫 `offb_node.cpp`，就把 PX4 OFFBOARD 的全部准入条件、模式名和失联行为直接套过来。[P1][R29]

PRESTREAM 在此代码中是预发布当前位置目标的应用策略；不应讲成“所有 ArduPilot 系统都必须严格先发 1.5 秒才允许 GUIDED”。

## 35.3 安装与版本确认

在已配置 ROS apt 源的开发机上：

```bash
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

GeographicLib 数据是 MAVROS 常见运行依赖。若脚本路径不存在，先检查实际安装包，而不是随便下载第三方同名脚本。[R28]

```bash
ros2 pkg prefix mavros
ros2 pkg executables mavros
dpkg-query -W ros-humble-mavros ros-humble-mavros-msgs
ros2 interface show mavros_msgs/msg/State
ros2 interface show mavros_msgs/srv/CommandTOL
```

当前网上 MAVROS 文档可能比你安装的软件更新。所有具体启动参数与插件名称，以安装版本的 Launch、YAML 和 `ros2 ... --help` 为准。

## 35.4 不存在的话题先查插件

FCU 已连接却没有某类话题，可能是插件未启用、相关 MAVLink 消息没有到达，或话题名称与配置不同。

先 `ros2 node list`，再对实际节点 `ros2 node info`、`ros2 param list`，最后检查 MAVROS 安装目录和日志。不要默认每个版本的插件参数结构完全相同。

<a id="c36"></a>
# 36. 串口、UDP、TCP 和 MAVLink 路由

## 36.1 串口 URL 是设备路径与链路配置

MAVROS 支持的连接 URL 形式包含 serial、udp、tcp 等，详细语法应按其官方连接文档与安装版本核对。[R28A]

```text
serial:///dev/serial/by-id/<实际设备标识>:<波特率>
udp://<绑定地址>:<本地端口>@<远端地址>:<远端端口>
```

不要把示例中的设备标识直接复制到另一架飞机，也不要认为换成 115200 一定会让所有 USB 或 UART 链路更快。

## 36.2 为什么优先使用 `/dev/serial/by-id`

`/dev/ttyACM0` 或 `ttyUSB0` 的编号可能随插拔顺序改变；稳定设备标识更容易避免误连设备。实际系统是否提供这些链接，必须现场查看。

【只读诊断】

```bash
ls -l /dev/serial/by-id/
id -nG
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
sudo lsof /dev/ttyACM0
```

若用户缺少串口组权限，可由管理员加入适当组并重新登录；不要用长期 `chmod 777` 代替权限管理。不要在真实飞行中拔线、重启 USB 服务或关闭占用进程。

## 36.3 UDP 的本地端口不等于飞控端口

UDP URL 中的绑定端表示“本进程在哪收包”，远端表示“向谁发包”。端口冲突、错误绑定地址、NAT、防火墙和桥接网络都可能造成单向可见。

【只读诊断】

```bash
ip -br addr
ip route
ss -lunp
ss -ltnp
```

不要同时让两个进程抢同一个串口。需要 MAVROS、地面站和记录工具同时接入时，应设计明确的路由或转发关系，而不是把每个程序都指向同一个物理端口。[R28A][R28C]

## 36.4 ROS Domain 与 MAVLink system ID 不是同一层隔离

ROS Domain 控制 ROS 发现范围；MAVLink 的系统 / 组件标识和路由决定飞控消息的对象。即使 ROS 在独立 Domain，错误的串口或 UDP 转发仍可能把命令送到真机。

## 36.5 一个连接故障的检查顺序

```text
设备是否存在
  → 权限是否正确
  → 是否被别的进程独占
  → URL 与端口是否正确
  → 是否收到 MAVLink heartbeat
  → MAVROS State.connected 是否为 true
  → 所需插件和遥测是否可用
```

`connected=true` 只解决链路的一层问题，不说明 EKF、GPS、罗盘或任务视觉均已合格。

<a id="c37"></a>
# 37. 飞控状态、解锁与起降命令的 ROS 语义

## 37.1 四个状态不要混成一个“ready”

连接成功、模式为 GUIDED、飞控已 armed、任务安全门禁通过，是四个独立判断。解锁 ACK 成功后，仍应观察实际 `State.armed`；起飞命令 ACK 成功后，仍应观察实际高度。[P1][R17]

## 37.2 本任务用到的服务

| 服务 | 类型 | 请求的含义 |
|---|---|---|
| `/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` | 请求解锁或上锁 |
| `/mavros/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` | 请求起飞 |
| `/mavros/cmd/land` | `mavros_msgs/srv/CommandTOL` | 请求降落 |
| `/mavros/cmd/command` | `mavros_msgs/srv/CommandLong` | 请求通用 MAVLink 命令 |

它们不是“函数返回后飞机已经做完动作”的同步控制接口。[P1][R17]

## 37.3 服务返回值要结合飞控遥测

```text
发 arm 请求 → 回复 success → State.armed=true → 任务可考虑下一步
发 takeoff → 回复 success → 高度达到门限 → 进入后续导航
发 land → 回复 success → 落地持续确认 → 请求 disarm
```

请求拒绝要记录 reason / result；请求超时要保留“是否可能已执行”的不确定性。调用 `Future.cancel()` 不能撤销已经送到飞控的动作。

## 37.4 起飞阶段为什么停发位置目标

v11 在 TAKEOFF 中禁用位置 setpoint，避免应用位置目标与起飞命令互相干扰；达到目标高度的 90% 后才进入 SEARCH 并恢复位置目标流。[P1]

这是你们具体实现的控制权切换方式。新人应先在 SITL 验证其时序，而不是在别的模式下随意混发速度、位置和起飞命令。

## 37.5 本文不提供一键真机解锁示例

学会 Service API 通过第 17～19 章模拟服务完成。进入真实硬件测试时，需要独立的设备身份核验、拆桨条件、飞控预解锁检查和现场操作流程。

命令行会调用服务不等于有资格绕过飞控安全检查；更不应该通过删除检查来“解决 GPS / EKF 不允许解锁”。

<a id="c38"></a>
# 38. 飞控参数、ROS 参数与 NAV30 频率约束

## 38.1 两类参数有不同存储位置

`fine_align_alt_m` 属于任务 ROS 节点；飞控位置控制、传感器和输出映射参数由 ArduPilot 管理。`ros2 param set` 改任务节点，不能天然等价于修改飞控参数。[R19][R31B]

MAVROS 可能提供飞控参数相关接口，但名称和实现随版本配置变化，应先 `ros2 service list -t` 再查具体类型。

## 38.2 “请求 30 Hz”和“收到 30 Hz”不同

飞控支持按 MAVLink 流或单消息间隔请求数据频率；实际频率仍受链路带宽、调度、其他客户端和消息支持情况影响。[R31]

一个 nominal 30 Hz 流有约 33.3 ms 间隔，不代表所有间隔都小于 50 ms。平均频率合格可能同时含有偶发 200 ms 空洞。

## 38.3 v11 的 NAV30 信号来自外部

源码订阅 `/cuadc/nav30_ready` Bool，并记录接收年龄。注释将其归于 `mavros_nav30_lock_node.py`。但此次附件没有提供该脚本完整内容，不能确认其统计窗口、设置了哪些消息频率、如何处理掉帧。[P1]

不要给这个话题手动发布 `true` 来跳过起飞门禁。

## 38.4 教学频率诊断应输出哪些量

建议至少记录窗口内平均频率、最大间隔、P95/P99 间隔、最后到达年龄、消息时间戳年龄、连续合格时长和样本数量。

第 16 章 observer 提供部分读数；它没有声称达到生产 NAV30 readiness 标准。完整统计规则应成为单独、可测试的接口契约。

## 38.5 一个链路预算例子

串口每秒可传输的数据不只包含 odom，还包含心跳、姿态、状态、参数响应和命令 ACK。提高所有遥测频率可能让高优先级命令等待更久。

【教学设计】按任务实际需要配置最小遥测集合，分别测试“空闲”“解锁 / 起飞瞬间”“舵机动作”“地面站参数下载”时的最大间隔，而不是只在静止时执行一次 `ros2 topic hz`。

<a id="c39"></a>
# 39. setpoint、轨迹和真实飞行速度

## 39.1 setpoint 是参考，不是实际状态

`/mavros/setpoint_position/local` 接收位置目标；飞控根据自己的控制器和限制跟踪它。轨迹生成器把参考移动得很快，飞机不一定能跟上。[P1][R29]

ROS 发布频率、setpoint 轨迹峰值速度、飞机实测速度和飞控速度上限，至少是四个独立量。

## 39.2 v11 的梯形时间参数

源码使用三维航段距离 \(d\)，名义峰值速度 \(v\)，最小航段时间 \(T_{min}\)：

\[
T=\max(T_{min},d/(0.8v))
\]

在归一化航段时间内，20% 加速、60% 匀速、20% 减速。每个独立航段参考速度都从零开始、到零结束。[P1]

## 39.3 这个轨迹没有显式最大加速度约束

当最小航段时间没有主导时，加速段时长为 \(0.2T\)，因此：

\[
a_{ref}=v/(0.2T)=5v/T
\]

例如在约 30 m 航段设 12 m/s，\(T\approx3.125s\)，则名义参考加速度约为 19.2 m/s²。**这是该数学轨迹要求的量，不是你们飞机已经能实现的量。**

因此过去仅用 `距离/(0.8×12)` 得到的转场时间，只能当参考轨迹时长，不能当真实比赛用时保证。

## 39.4 到点不只看时间

v11 的 `segment_complete(radius)` 同时要求：航段时间已结束，且飞机实际三维位置进入终点半径。跟踪落后时，状态机会继续等飞机，而不是时间到就算完成。[P1]

## 39.5 对准使用另一种速度限制

`slew_target_toward()` 用实际控制 dt 限制 setpoint 每次移动的三维距离：

\[
\|\Delta p_{sp}\|\le v_{max}\Delta t
\]

它能限制位置参考的变化率，但不能直接保证姿态峰值、加速度、闭环无振荡或者物理投放精度。

## 39.6 实践：把期望轨迹和实际轨迹放在一起

同时记录 setpoint 与 odom，离线画 XY 路径、速度、位置误差和每阶段耗时。只画航线规划，不画实际轨迹，会掩盖高速跟踪问题。

不要通过增加 ROS Timer 到 200 Hz 来补偿飞控物理能力不足；先确认通信、轨迹与控制约束一致。

<a id="c40"></a>
# 40. 把 ROS 接口组装成完整任务状态机

## 40.1 当前源码的状态链

```text
WAIT_FCU → WAIT_NAV_STABLE → PRESTREAM → WAIT_GUIDED → WAIT_ARM
→ TAKEOFF → SEARCH
→ ALIGN_COARSE → ALIGN_FINE → RELEASE（A1）
→ ALIGN_COARSE → ALIGN_FINE → RELEASE（A2）
→ RECON_TRANSIT → RECON_DESCEND → RECON_SCAN
→ RECON_RETURN_CLIMB → RECON_RETURN
→ LAND → DISARM → DONE

异常：RETURN_HOME / PILOT_OVERRIDE / ABORT
```

这份状态机是普通 C++ 应用状态机，不是 ROS Lifecycle 状态图；`enter(State::SEARCH)` 不会触发 ROS 自带的 lifecycle transition。[P1]

## 40.2 每个状态至少写五件事

进入时初始化什么、持续输出什么、由什么条件成功退出、多久超时、失败后去哪里。不能只写“这个状态飞向桶”。

| 状态 | 输入依赖 | 主要输出 | 成功依据 |
|---|---|---|---|
| WAIT_NAV_STABLE | 配置、导航、航向、视觉、NAV30 | 等待 / 日志 | 门禁连续满足 |
| TAKEOFF | State、odom | CommandTOL | 实际高度门限 |
| SEARCH | odom、时间对齐视觉 | 搜索位置目标 | 两目标计划锁定 |
| ALIGN | odom、目标记忆、可用视觉 | 补偿后位置目标 | 几何与稳定条件 |
| RELEASE | odom、冻结目标、服务回复 | 舵机命令、位置目标 | 动作协议流程完成 |
| RECON_SCAN | odom | 航点、拍照请求 | 路线触发完成 |
| LAND / DISARM | odom、State | 起降 / 上锁服务 | 落地持续确认与未 armed |

## 40.3 输入回调和状态决策分开

回调尽量负责校验、缓存和时间记录；Tick 负责决定动作和状态迁移。不要让 odom 回调、视觉回调、计时器各自随意发送不同的控制目标。

当前源码有些健康和重关联逻辑位于回调中，阅读时必须把这些入口一起追踪，不可只看 `switch(state_)`。[P1]

## 40.4 任务核心与 ROS 包装的推荐分层

【教学设计】

```text
mission_core：输入快照 → 决策 / 输出意图（纯 C++，易测）
ros_adapter：订阅、时间、QoS、参数、日志
fcu_gateway：唯一控制出口、请求关联、权限与门禁
perception_adapter：相机 / 模型结果标准化
bringup：配置与启动
```

不要求立即重构你们已稳定代码。先写清边界、加入测试，再拆包。模块数量不是先进程度，能够单独测试和说明责任才有价值。

**实践 11**：选 `ALIGN_FINE`，画出所有可能退出路径，包括视觉正常、视觉失效 fallback、超时 fallback、重试和最终返航。


<a id="c41"></a>
# 41. 起飞前门禁：配置正确、数据正确、控制权正确

## 41.1 默认 `flight_enable=false` 不是 v11 的仿真模式

v11 把 `flight_enable=true` 纳入 `config_valid_`。保持默认 false 会使任务进入 ABORT，而不是完整跑一遍但不输出。[P1]

本文教学 `mission_dry_run` 则完全相反：只允许 false，根本没有真实飞控接口。这是两个不同程序，不应混淆。

## 41.2 v11 起飞前明确校验的配置

两个载荷、SERVO9 / SERVO10、1200→1500、0.7 s、初始化与回收开启、固定初始航向等构成硬配置契约。更换舵机行程后不能只改某一处 YAML，还要检查这个契约是否匹配新的、已经验证的机构。[P1]

这些是你们具体机构约束，不是 CUADC 或 MAVROS 的通用要求。

## 41.3 导航锁定不等于舵机门禁已通过

`navigation_ready_to_lock()` 检查未 armed、导航和罗盘新鲜、NAV30、航向稳定、视觉准备、静止持续时间等；实际代码没有把 `servos_initialized_` 写进此函数。

舵机初始化 ACK 会在 WAIT_ARM 阶段阻止自动解锁 / 起飞。因此应分别诊断“为什么不能锁定场地”和“为什么不能解锁”，不要把两个状态的条件混在一起。[P1]

## 41.4 起飞前 ON_GROUND 检查在本版被旁路

源码注释和实现都表明，本版不依赖可靠的 ExtendedState ON_GROUND 做起飞锁定。它仍检查未解锁和静止，但这不是对所有环境都充分的“已在地面”证明。[P1]

教学中保留这一事实，同时要求实飞团队单独验证地面准备状态。不要把某次现场兼容处理推广成所有无人机的安全原则。

## 41.5 ready 不能永久缓存

ready=true 后，如果视觉、NAV30 或 odom 失效，门禁必须重新评估。只有当前状态与当前数据均支持下一步，才执行下一步。

【教学设计】把门禁结果写成带原因的结构：

```text
navigation_fresh: true
heading_stable: true
vision_aligned: false
servo_initialized: true
start_authorized: false
blocking_reason: VISION_ALIGNMENT_NOT_READY
```

这样新人看到“没起飞”时，不需要猜是哪个 `if` 阻止了执行。

<a id="c42"></a>
# 42. 投放对准：ROS 层必须保证几何与动作时序可解释

## 42.1 两次姿态使用的时间不同

视觉目标转换使用观测对应的历史位置和 RPY；投放口补偿使用当前飞机位置和姿态。前者回答“桶在哪里”，后者回答“机构现在在哪里”。[P1]

\[
p_{bucket}^{W}=p_B^{W}(t_v)+R_B^{W}(t_v)p_{bucket}^{B}(t_v)
\]

\[
p_{release}^{W}(t)=p_B^{W}(t)+R_B^{W}(t)r_{release}^{B}
\]

不能把两处姿态都偷换成当前姿态，也不能把当前投放口用旧姿态计算。

## 42.2 两级对准与最终门禁

| 阶段 | 飞机参考点相对高度 | 主要判据 |
|---|---:|---|
| COARSE | 1.7 m | 中心 / 出瓶口误差 ≤0.35 m，高度差 ≤0.15 m，连续0.5 s |
| FINE 正常视觉 | 1.3 m | 中心 ≤0.25 m，出瓶口 ≤0.12 m，高度差 ≤0.08 m，连续0.45 s |
| RELEASE | 1.3 m | 中心 ≤0.35 m，当前 / 预测出瓶口 ≤0.20 m，高度差 ≤0.15 m，另加运动与身份门禁 |

这是默认 rescue profile 下的主要数值；源码开启该 profile 时会覆盖很多 ROS 参数内部使用值。[P1]

## 42.3 姿态补偿与姿态限制不是一回事

补偿使用完整 Rz·Ry·Rx，把投放口三维外参旋转到 local，并反解飞机中心 XY。

限制要求 **分别**满足 `abs(roll)≤8°` 和 `abs(pitch)≤8°`，以及角速度向量模长≤10°/s。它不是把“合成总倾角”限制在8°；两轴都接近8°时，机体总倾斜会大于8°。[P1]

水平速度≤0.15 m/s、垂直速度绝对值≤0.10 m/s是另外的门禁。它们同时连续满足0.5 s后才能发释放命令。

## 42.4 高度控制的实际含义

目标 z 仍是 `home_z + fine_align_alt_m`。完整三维旋转主要用来补偿 XY，并没有反解让出瓶口离地高度恒定，也没有根据桶顶高度或水瓶落点建自由落体模型。[P1]

## 42.5 RELEASE 不再被视觉 ID 跳变干扰

进入 RELEASE 后桶的 local 坐标冻结，但实时姿态变化仍会改变期望飞机中心。未发送释放请求时，代码以约0.30 m/s限制修正目标变化；请求已发出或开始动作后，则直接更新为计算出的 `direct_release_pose_`。因此不能把整个释放动作阶段都描述成始终经过同一 slew 限速。[P1]

## 42.6 延迟预测只有默认零提前量

代码估算出瓶口速度，再计算 `p + v × detach_delay`；默认 `release_command_to_detach_delay_s=0`。这仅是脱离时刻的短时线性位置估计，不是脱离后落点预测。

## 42.7 ACK 不是水瓶传感器

源码的成功流程：释放命令被接受，保持动作时间，再回收舵机并收到 ACK，才推进载荷计数。没有附加传感器证据时，这证明的是软件与命令协议流程，不是“水瓶一定已离机、一定落在有效区”。[P1][R17][R30]

教程里的模拟服务更弱，只改变内存标志；不能用它验证供电、舵机堵转或水瓶卡滞。

## 42.8 应记录什么帮助赛后解释偏差

冻结目标、当前 odom、当前 RPY、投放口坐标、期望中心、所有误差、速度与角速度、请求 / ACK / 回收时刻、目标 ID、外参版本。没有这些数据，很难区分识别错误、补偿错误和机械落点偏差。

<a id="c43"></a>
# 43. DJI O4 侦察：视频链路与 ROS 任务完成条件

## 43.1 当前已知和未知

已知：你们明确说使用 DJI O4 高清数字图传进行侦察。

未知：本轮没有提供 O4 相机安装角、实际显示链路、图像是否进入机载电脑、视频采集接口、有效画幅和人工判读测试结果。因此本章不把 O4 自动写成某个 `/camera/image_raw` 节点，也不把某个高度的视场覆盖当作已验证事实。

## 43.2 O4 可以位于 ROS 图之外

```text
ROS：飞机到侦察区 → 执行观察航线 → 返航
视频：O4 连续采集 → 地面观看 → 人员判断标识 → 记录结果
```

没有可用视频采集接口时，ROS 仍可以完成导航；但它无法从“航点走完”推出“地面人员已经看清三个标识”。

## 43.3 原版 v11 仍绑定六张照片

原代码发布 PHOTO_ONLY 请求、六个 capture_request，并接收 `capture_done` 的编号 ACK；DONE 的完整成功判定要求照片数量达标。[P1]

若换 O4 而没有运行原拍照节点，路线飞完后可能因缺少照片 ACK 被判未完整成功。这是接口迁移问题，不是 O4 的画质问题。

## 43.4 推荐把成功拆成不同层

【教学设计】

```text
recon_route_completed：导航观察路线走完
video_evidence_available：有约定的视频或照片证据
recon_result_available：有判读结果及其来源
flight_safely_completed：返航、落地和上锁完成
competition_completed：另按裁判规则与实际成绩认定
```

不要把 `recon_completed=true` 偷换成“危险品识别正确”。也不要为了通过旧 DONE 条件伪造六个照片 ACK。

## 43.5 对航线改版的 ROS 工作清单

删除或保留拍照接口要明确；不再使用的照片 ACK 不应阻塞正常返航；侦察模式切换应有可验证响应；记录进入 / 离开观察区域的事件，便于和视频对时。

“3 m、中心线、6 m/s”仅是前面对话的规划设想。真正定案需要用实际镜头和安装角测视场覆盖、桶内遮挡、运动模糊、曝光、地面判读时间与盲区。

## 43.6 图传断流怎么办

没有 ROS 可读的图传健康反馈时，任务节点不能可靠自动判断屏幕是否黑屏。必须把这种不可观测性写进系统约束，而不是在代码中放一个始终 true 的 video_ready。

**实践 12**：写一份 O4 侦察接口迁移表，列出旧照片链路每个话题“保留 / 删除 / 替换 / 仅日志”，并定义新的成功条件。

<a id="c44"></a>
# 44. 非任务区 12 m/s：ROS 能做什么，不能证明什么

## 44.1 配置值不是理论极限

设定 `transit_speed_m_s=12.0` 只能改变任务参考生成；还需核对飞控速度 / 加速度 / 倾角约束、推进能力、制动距离和场地余量。本文不继承此前未经实飞验证的“60秒一定完成”结论。[P1][R29]

## 44.2 先看制动距离的量级

若用恒定减速度 \(a\) 从速度 \(v\) 降到0，则理想制动距离：

\[
d_{stop}=\frac{v^2}{2a}
\]

仅作物理算例：12 m/s、假设可用减速度3 m/s²，则需要24 m，尚未计入延迟、风和轨迹误差。此处3 m/s²不是你们飞机的实测值。

因此投放区与侦察区之间的短距离转场，可能根本不存在足够长的“12 m/s 稳态平台”。在任务区外提速，并不代表可以到边界才开始减速。

## 44.3 两段观察线不等于无停车连续飞越

v11 每条独立 Segment 的参考都以零末速度结束。把六点改成两点，再建立返航 Segment，也不会自动获得连续曲率、非零边界速度、无停车掉头的轨迹。[P1]

需要真正连续航线时，应显式设计段间速度与加速度衔接；这属于轨迹算法改动，而不仅是 ROS Launch 改参数。

## 44.4 控制链延迟对高速位置影响更大

若相对时间误差为 \(\Delta t\)，匀速近似下额外位移误差约为 \(v\Delta t\)。12 m/s 下50 ms对应0.6 m；这解释了为什么仅有高频通信还不够，还需要低延迟和准确时间语义。

## 44.5 一个可审计的提速过程

低速基线 → 单段提速 → 实际速度 / 跟踪误差测量 → 减速和接管验证 → 带传感器负载测试 → 整体任务测试。

每次只改变一类变量，保存飞控参数和 ROS 有效参数。不要把提高搜索速度、改变高度、替换视觉和升级固件同时进行，否则失败原因很难定位。

## 44.6 故障返航单独配置

正常竞速速度与故障返航速度应该有明确的策略边界。导航不确定、传感器异常或载荷状态不明时，不能仅因为“非任务区”就机械套用最大速度。

本章讨论工程设计，不声称已把用户之前的12 m/s要求修改进上传源码。

<a id="c45"></a>
# 45. 综合实践：不接飞控的闭环任务系统

## 45.1 实验要解决什么

你将连接软件飞机、视觉替身、数据健康检查、模拟舵机服务和任务状态机，观察一套完整 ROS 事件链。

它不包含 ArduPilot 动力学、真实桶选择、刚体闭环和飞行安全认证；目标是证明 ROS 接口与任务推进机制工作方式可理解。

## 45.2 软件飞机完整代码

**完整文件：`src/cuadc_tutorial_py/cuadc_tutorial_py/sim_plant.py`**

```python
"""Kinematic point simulator, NOT a dynamics simulator and NOT an autopilot."""
import math
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from .common import finite_positive, require_lab_namespace, spin_main


class SimPlant(Node):
    def __init__(self):
        super().__init__('sim_plant')
        require_lab_namespace(self)
        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('max_speed_m_s', 1.0)
        self.declare_parameter('pause_after_s', -1.0)
        self.declare_parameter('pause_duration_s', 0.0)
        self.rate = finite_positive(self.get_parameter('rate_hz').value, 'rate_hz')
        self.speed = finite_positive(self.get_parameter('max_speed_m_s').value, 'max_speed')
        self.pause_after = float(self.get_parameter('pause_after_s').value)
        self.pause_duration = float(self.get_parameter('pause_duration_s').value)
        self.p = [0.0, 0.0, 0.0]
        self.target = list(self.p)
        self.start = self.last = time.monotonic()
        self.pub = self.create_publisher(Odometry, 'odom', qos_profile_sensor_data)
        self.sub = self.create_subscription(PoseStamped, 'setpoint_preview', self.on_target, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    def on_target(self, msg):
        if msg.header.frame_id != 'lab_map':
            self.get_logger().error('拒绝 frame_id 不是 lab_map 的预览点')
            return
        p = msg.pose.position
        candidate = [p.x, p.y, p.z]
        if (all(math.isfinite(x) for x in candidate) and
                abs(p.x) <= 10 and abs(p.y) <= 10 and 0 <= p.z <= 5):
            self.target = candidate

    def tick(self):
        now_mono = time.monotonic()
        dt = max(1e-4, min(0.1, now_mono - self.last))
        self.last = now_mono
        elapsed = now_mono - self.start
        if (self.pause_after >= 0 and
                self.pause_after <= elapsed < self.pause_after + self.pause_duration):
            return  # 模拟导航数据中断；不是让真实飞机停车。
        delta = [b - a for a, b in zip(self.p, self.target)]
        distance = math.sqrt(sum(d * d for d in delta))
        step = min(distance, self.speed * dt)
        velocity = [0.0, 0.0, 0.0]
        if distance > 1e-9:
            velocity = [d / distance * step / dt for d in delta]
            self.p = [a + v * dt for a, v in zip(self.p, velocity)]
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lab_map'
        msg.child_frame_id = 'lab_base_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = self.p
        msg.pose.pose.orientation.w = 1.0  # 示例全程姿态单位旋转。
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = velocity
        # 因姿态为单位旋转，示例的 child/world 速度分量恰好相同。
        self.pub.publish(msg)


def main(args=None):
    spin_main(SimPlant, args)
```

模型只让一个点按速度上限朝预览目标移动，姿态恒为单位旋转。它没有质量、电机、风、惯性和制动限制，不能拿其用时预测真机。

## 45.3 教学状态机完整代码

**完整文件：`src/cuadc_tutorial_py/cuadc_tutorial_py/mission_dry_run.py`**

```python
"""A ROS wiring exercise, deliberately NOT a flight-ready CUADC controller.

It uses a kinematic point plant, validates fresh data, publishes previews,
and calls a memory-only payload service. Real tracking, flight modes,
rigid release-point gates, exact timestamp transforms and failsafes are not implemented.
"""
import math
import time
import uuid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from cuadc_tutorial_interfaces.msg import MissionEvent
from cuadc_tutorial_interfaces.srv import PayloadOp
from .common import require_lab_namespace, spin_main


class MissionDryRun(Node):
    def __init__(self):
        super().__init__('mission_dry_run')
        require_lab_namespace(self)
        self.declare_parameter('flight_enable', False)
        if self.get_parameter('flight_enable').value:
            raise RuntimeError('此练习不支持 flight_enable=true；没有飞控接口')
        self.session = str(uuid.uuid4())
        self.sequence = 0
        self.state = 'WAIT_DATA'
        self.entered = time.monotonic()
        self.odom = None
        self.odom_at = None
        self.vision_ready = False
        self.vision_at = None
        self.goal = None
        self.arrived_since = None
        self.future = None
        self.command_at = None
        self.payload = 0
        self.index = -1
        self.plan = [
            ('TAKEOFF', (0.0, 0.0, 2.2)),
            ('SEARCH_ENTRY', (3.0, 0.0, 2.2)),
            ('COARSE_1', (3.2, -0.6, 1.7)),
            ('FINE_1', (3.2, -0.6, 1.3)),
            ('DROP_1', None),
            ('COARSE_2', (3.5, 0.6, 1.7)),
            ('FINE_2', (3.5, 0.6, 1.3)),
            ('DROP_2', None),
            ('RECON_ENTRY', (5.5, 0.0, 2.0)),
            ('RECON_EXIT', (6.0, 0.0, 2.0)),
            ('RETURN_HOME', (0.0, 0.0, 2.0)),
            ('LAND', (0.0, 0.0, 0.0)),
        ]
        self.sp_pub = self.create_publisher(PoseStamped, 'setpoint_preview', 10)
        self.state_pub = self.create_publisher(String, 'mission/state', 10)
        self.event_pub = self.create_publisher(MissionEvent, 'mission/events', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.on_odom,
                                                 qos_profile_sensor_data)
        self.health_sub = self.create_subscription(Bool, 'vision_ready', self.on_health, 10)
        self.client = self.create_client(PayloadOp, 'payload/operate')
        self.timer = self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        p = msg.pose.pose.position
        if msg.header.frame_id == 'lab_map' and all(math.isfinite(v) for v in (p.x, p.y, p.z)):
            self.odom = msg
            self.odom_at = time.monotonic()

    def on_health(self, msg):
        self.vision_ready = msg.data
        self.vision_at = time.monotonic()

    def enter(self, state, reason):
        event = MissionEvent()
        event.header.stamp = self.get_clock().now().to_msg()
        event.session_id = self.session
        event.sequence = self.sequence
        self.sequence += 1
        event.previous_state, event.state, event.reason = self.state, state, reason
        self.event_pub.publish(event)
        self.get_logger().info(f'{self.state} -> {state}: {reason}')
        self.state = state
        self.entered = time.monotonic()
        self.arrived_since = None

    def next_step(self):
        self.index += 1
        if self.index >= len(self.plan):
            self.goal = None
            self.enter('DONE', 'software exercise only')
            return
        state, self.goal = self.plan[self.index]
        self.enter(state, 'next demo step; fixed teaching route')

    def fault(self, reason):
        if self.future is not None and not self.future.done():
            self.client.remove_pending_request(self.future)
            self.future.cancel()
        self.future = None
        self.goal = None
        self.enter('FAULT', reason + '; no real aircraft is connected')

    def tick(self):
        now = time.monotonic()
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        if self.state in ('DONE', 'FAULT'):
            return
        odom_ok = self.odom_at is not None and now - self.odom_at < 0.5
        health_ok = self.vision_at is not None and now - self.vision_at < 0.5 and self.vision_ready
        if self.state == 'WAIT_DATA':
            if odom_ok and health_ok and self.client.service_is_ready():
                self.next_step()
            elif now - self.entered > 10.0:
                self.fault('startup readiness timeout')
            return
        if not odom_ok or not health_ok:
            self.fault('stale odometry or vision health')
            return
        if now - self.entered > 20.0:
            self.fault('step timeout')
            return
        if self.state in ('DROP_1', 'DROP_2'):
            self.payload = 0 if self.state == 'DROP_1' else 1
            self.send_operation(release=True)
            return
        if self.state in ('WAIT_RELEASE', 'WAIT_STOW'):
            self.check_operation(now)
            return
        if self.goal is None:
            self.fault('missing preview goal')
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lab_map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = self.goal
        msg.pose.orientation.w = 1.0
        self.sp_pub.publish(msg)
        p = self.odom.pose.pose.position
        error = math.sqrt(sum((a - b) ** 2 for a, b in zip((p.x, p.y, p.z), self.goal)))
        if error > 0.08:
            self.arrived_since = None
        elif self.arrived_since is None:
            self.arrived_since = now
        elif now - self.arrived_since > 0.45:
            self.next_step()

    def send_operation(self, release):
        if not self.client.service_is_ready():
            self.fault('mock payload server not ready')
            return
        req = PayloadOp.Request()
        req.request_id = f'{self.session}:{self.payload}:{"release" if release else "stow"}'
        req.payload_index, req.release = self.payload, release
        self.future = self.client.call_async(req)
        self.command_at = time.monotonic()
        self.enter('WAIT_RELEASE' if release else 'WAIT_STOW', 'mock request sent')

    def check_operation(self, now):
        if self.future is None:
            self.fault('missing future')
            return
        if self.future.done():
            try:
                response = self.future.result()
            except Exception as exc:
                self.fault(f'mock request failed: {exc}')
                return
            if not response.accepted:
                self.fault('mock command rejected')
                return
            if self.state == 'WAIT_RELEASE':
                if now - self.command_at >= 0.7:
                    self.send_operation(release=False)
            else:
                self.future = None
                self.next_step()
        elif now - self.command_at > 2.0:
            self.fault('mock ACK timeout; operation outcome unknown')


def main(args=None):
    spin_main(MissionDryRun, args)
```

这份代码同时演示：订阅与发布、默认安全参数、异步服务、连续到点确认、状态事件、会话 ID、超时和故障退出。

## 45.4 与真实 v11 的区别必须读完

教学路线中的桶点是固定合成点，**没有用视觉替身的检测进行真实目标选择**；视觉只参与健康门禁。全过程要求视觉健康，而 v11 只有 SEARCH 强制持续视觉。教学释放0.7 s从发请求开始计时，v11则从释放 ACK 后开始。教学 FAULT 不会发返航或降落服务，因为根本没有飞控。

软件飞机继续记住最后一个预览目标；停止发布不等于立即停止它运动。这是刻意暴露的接口边界：执行端是否会超时悬停，必须另行设计并验证，不能由 ROS 发布者退出自动推导。

因此这套工程是“完整 ROS 联调练习”，不是可替代 v11 的实飞控制器。

## 45.5 构建与启动

解压工作空间，进入其根目录，在没有 Conda / venv 污染的新终端执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to cuadc_tutorial_bringup
source install/setup.bash
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
ros2 launch cuadc_tutorial_bringup lab.launch.py mission:=true
```

另一终端使用同样环境后：

```bash
ros2 topic echo /lab/mission/events
ros2 topic echo /lab/mission/state
ros2 topic echo /lab/payload/mock_state
ros2 topic hz /lab/odom
```

## 45.6 预期现象与验收

WAIT_DATA通过后进入软件起飞、接近两个合成桶、两次模拟释放 / 回收、简化侦察和返回，最终为 DONE。检查两次模拟 release 的 request_id 不同，重试同一请求时不重复改变内存动作。

不接受仅“终端没有报错”作为验收。至少保存状态事件、实际软件位置和两次请求记录。

<a id="c46"></a>
# 46. 故障注入：故意把系统弄坏，再证明它解释得清楚

## 46.1 实验规则

每次只注入一类故障；先跑通基线，再记录注入时刻。进入 FAULT 后本教学任务不会自动恢复。下一次实验需停止相关教学进程并重新启动，不要复用部分旧状态。

## 46.2 动态故障参数

以下命令仅对 `/lab` 教学节点执行：

```bash
# 正常无目标，不等于链路故障。
ros2 param set /lab/vision_stub empty_frames true

# 帧存在但内容被声明无效。
ros2 param set /lab/vision_stub invalid_frames true

# 模拟持续重复旧时间戳。
ros2 param set /lab/vision_stub freeze_stamp true

# 模拟时间戳比本机现在超前。
ros2 param set /lab/vision_stub future_stamp_s 2.0

# 模拟执行机构服务明确拒绝释放。
ros2 param set /lab/payload_server reject_release true
```

这些参数在回调中读取，能动态改变行为。每条单独测试，结束后重启恢复基线。

## 46.3 启动时缓存的参数

`sim_plant` 的频率、速度与暂停配置，以及 `vision_stub` 的 pipeline_delay，在构造时读取。`ros2 param set` 改数据库不代表成员变量同步更新；要改 YAML 后重启，或者显式实现参数回调。[本教学代码]

例如把 `lab.yaml` 中软件飞机设置为：

```yaml
/lab/sim_plant:
  ros__parameters:
    rate_hz: 30.0
    max_speed_m_s: 1.0
    pause_after_s: 6.0
    pause_duration_s: 2.0
```

这会模拟 odom 输出暂停，不是安全停车算法。

## 46.4 预期结果表

| 注入 | 预期可观察现象 | 错误理解 |
|---|---|---|
| 有效空帧 | 健康仍可能 true | 空帧说明相机死机 |
| 无效帧 | guard 拒绝，健康下降，任务 FAULT | 消息在发就一定健康 |
| 时间戳冻结 | 单调 / 年龄检查失败 | 给它重新填 now 就解决了 |
| pipeline_delay=1.0 s | 超过教学 guard 的0.5 s年龄限制 | 提高队列深度就解决 |
| 20 Hz odom | 平均与最大间隔改变 | 只要有 odom 都能满足 v11 的50 ms严格夹取 |
| 停止模拟服务 | 发现失败或请求超时 | 没 ACK 就证明没动作 |
| 重复 request_id | 同参数幂等返回 | 每次重试都要重复动作 |
| 错误 frame_id | 拒绝几何输入 | 改掉标签即可 |

## 46.5 更进一步的实验

给软件服务添加可配置回复延迟，验证等待不会阻塞 odom；让视觉只重启自身，观察 seq 重置；在回放中降低倍率，观察 steady watchdog 与 ROS 时间年龄不同。

每个实验都应写“预计谁先发现、多久发现、输出什么事件、是否继续发布控制”。

<a id="c47"></a>
# 47. 从软件替身走到 ArduPilot SITL

## 47.1 SITL 能多验证什么

SITL 在电脑上运行飞控软件，能够验证飞控模式、命令响应、导航和更多飞控行为。它比简单点模型更接近系统，但仍不能代替真实机构、射频链路和现场视觉。[R32]

## 47.2 先不接 Gazebo

可以先用 ArduPilot 自带仿真模型启动 SITL，再接 MAVROS。这样减少 Gazebo、图形驱动和桥接软件同时出错的概率。

上传的旧教程提供了一条 ArduPilot 构建与 `sim_vehicle.py` 的流程；版本、依赖与启动参数应按当前选定分支及官方文档核对，不把旧命令不加说明地当成永久接口。[P3][R32]

## 47.3 仅 SITL 的示意启动流程

在已经按官方文档准备并构建的独立 ArduPilot 工作目录中：

```bash
# 只运行电脑上的 SITL；不要连接真实飞控。
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14550
```

另一终端在独立 ROS 实验域启动 MAVROS 的安装版本入口。常见入口形式如下，具体可执行文件和参数先检查：

```bash
ros2 pkg executables mavros
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@
```

若当前安装要求从 Launch 加载插件配置，就按该安装包的 Launch 运行；不要在不知道错误原因时反复换 PX4 / ArduPilot 的启动文件。

## 47.4 隔离清单

真实飞控不接入；没有到真机串口的路由；地面站确认连接的是 SITL；UDP端口单独规划；测试域独立；教学预览程序不 remap 成飞控控制输出。

在这之后才按官方 SITL 流程测试 GUIDED、解锁、起飞与降落。完整任务也必须先以低速、可视化、可记录方式运行。

## 47.5 保存仿真状态

SITL 参数与场景应记录版本。风、传感器失效、仿真倍率等属于仿真配置；只能在明确的仿真环境修改，不能把 `SIM_*` 调试习惯带到实机参数管理。[R32]

<a id="c48"></a>
# 48. Gazebo Harmonic 与 Humble：建立可复现的仿真链

## 48.1 Gazebo 不是 MAVROS 的必需部分

ArduPilot Gazebo 插件负责模拟器与飞控之间的仿真数据交换；ROS 与 MAVROS 可以作为另一条任务接口链。不要把所有链接统称“ROS连Gazebo”。[R33]

```text
Gazebo 物理 / 传感器 ←→ ArduPilot 仿真插件 ←→ SITL
                                                   ↕ MAVLink
                                               MAVROS
                                                   ↕ ROS
                                               任务程序
```

## 48.2 版本组合必须显式固定

你们旧资料采用 Ubuntu22.04、Humble、Gazebo Harmonic。Harmonic与Humble不是默认发行版配对；桥接依赖、包来源和冲突应按官方组合说明核对。不能把安装 Fortress 的桥接教程与 Harmonic 插件编译教程随意拼接。[P3][R33A]

## 48.3 先验证四个最小单元

Gazebo能独立加载简单世界；ArduPilot插件能加载且无符号错误；SITL与仿真模型交换数据；MAVROS看到SITL状态。每项分别留日志，再加入相机和任务。

```bash
# 只读检查，帮助确定实际安装。
gz --versions
printenv | grep '^GZ_'
ros2 pkg list | grep -E 'ros_gz|mavros'
```

具体 world 文件路径、模型名和插件路径取决于实际克隆目录与分支，不要把他人的 `$HOME/path/to/...` 原样保留。

## 48.4 `/clock` 必须有唯一合理来源

使用ROS仿真传感器时，要明确是否桥接仿真时间，以及哪些节点开启use_sim_time。系统中多个不同仿真器同时发布 `/clock` 会破坏时间一致性。

v11混用ROS时间戳与steady计时，回放和加速仿真时行为不自动保持实时时间比例；应按第26、51章验证，不要只把仿真倍率调高就用来估计比赛耗时。[P1][R23]

## 48.5 仿真视觉也需要标定

渲染出来的图像应有对应CameraInfo、安装位姿和时间戳。不能因为是仿真就使用全零相机外参，或把世界坐标直接伪装成body检测而不注明。

**实践 13**：在SITL / Gazebo中验证固定机头航向下的前、后、左、右平移。通过实际轨迹证明坐标变换正确，再测任务路线。


<a id="c49"></a>
# 49. RViz2、rqt_graph 和可视化调试

## 49.1 可视化要显示事实，而不只是漂亮航线

RViz2 的 Fixed Frame 决定各显示项的共同参考。教学工程使用 `lab_map`；若写成不存在的 `map`，TF 报错并不奇怪。[R34]

```bash
rviz2
ros2 run rqt_graph rqt_graph
```

在RViz中添加Grid、TF、Odometry，并把Odometry话题设为 `/lab/odom`；必要时把其 Reliability设为Best Effort，以匹配发布端。

## 49.2 四条线比一条线更有价值

【教学设计】建议分别显示计划路线、实际 odom 路线、当前 setpoint 轨迹、估计目标位置。再显示投放口而非只显示机体中心。

`nav_msgs/Path` 用一系列 PoseStamped 表达轨迹；`visualization_msgs/Marker` 适合显示桶、边界、箭头与文字。它们是可视化输出，不应成为唯一飞控指令源。[R34]

## 49.3 Marker 常见错误

`ns`与`id`决定标记的身份；反复生成新ID而不删除，会造成画面残留。scale必须合理，透明度不能为0，header.frame_id必须存在，姿态四元数应有效。

不要通过在RViz中手动调整显示偏移来“修复”真实坐标错误；那只会让错误不再被看见。

## 49.4 rqt_graph 不能证明数据正确

图上有连线，只说明存在通信关联。QoS、消息年龄、数值有限性、帧定义和业务有效性还要通过消息内容与诊断验证。

## 49.5 现场可视化也会增加负载

地面机显示大图、远程桌面和多路录像可能占用网络与CPU。最终验收应在实际比赛的显示与记录负载下进行，而不是把所有诊断关掉后才宣布稳定。

<a id="c50"></a>
# 50. rosbag2：把任务变成可分析的数据

## 50.1 文本日志与 bag 的分工

日志解释“为什么进入这个分支”；bag保存“当时输入和输出到底是什么”。两者都需要，单独任何一种都不够。

Humble的rosbag2支持选择话题、分包、压缩及QoS覆盖；具体选项以安装版本帮助为准。[R35][R36]

## 50.2 第一次只录小数据

在教学工作空间根目录，另开终端并source后执行：

```bash
mkdir -p bags
ros2 bag record \
  -o "bags/lab_$(date +%Y%m%d_%H%M%S)" \
  --qos-profile-overrides-path \
  src/cuadc_tutorial_bringup/config/bag_qos.yaml \
  /lab/odom /lab/buckets /lab/vision_ready \
  /lab/setpoint_preview /lab/mission/state /lab/mission/events \
  /lab/payload/mock_state /tf /tf_static
```

先开始记录，再启动 `mission:=true`，可以减少漏掉初始事件的机会。按Ctrl+C正常结束记录，检查输出目录，不要直接断电。

**完整文件：`src/cuadc_tutorial_bringup/config/bag_qos.yaml`**

```yaml
/lab/odom:
  history: keep_last
  depth: 100
  reliability: best_effort
  durability: volatile
/lab/buckets:
  history: keep_last
  depth: 30
  reliability: best_effort
  durability: volatile
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local
```

## 50.3 先 `info` 再分析

```bash
ros2 bag info bags/<本次目录名>
du -sh bags/<本次目录名>
df -h .
```

检查每个话题的类型、条数、持续时间和存储格式。命令退出成功但某个关键话题计数为0，也不能算录制成功。

## 50.4 为什么 rosbag 没录到 Service 动作

不要假定在Humble上 `record -a` 就能完整记录所有Service请求、响应及其物理后果。对关键动作，主动发布自己的MissionEvent或CommandEvent，包括请求ID、目标通道、请求值、发送时间、回复结果和结果不确定性。[R35][R17]

当前v11主要使用文本日志，教学工程增加了MissionEvent。这是本文的新设计，不是原代码已存在的话题。[P1]

## 50.5 存储策略

优先记录导航、目标、任务状态、命令事件和关键参数；高带宽图像按需要分辨率、频率和压缩。分包可以限制单文件体积，但不能保证突然断电时最后文件必然完整。

图像压缩和写盘都应测对20Hz任务周期与30Hzodom的影响。不要默认把全部数据 `-a` 都录下来就是最可靠方案。

## 50.6 单次实验的目录建议

```text
flight_YYYYMMDD_HHMMSS/
  manifest.txt
  effective_ros_params.yaml
  fcu_params_backup.txt
  launch_console.log
  bag/
  photos_or_video_index/
  notes.md
```

视频本体可以很大；至少保存其时间基准、文件位置和与ROS事件对时的方法。O4的视频时间轴未必与ROS时间直接相同。

<a id="c51"></a>
# 51. 安全回放：历史命令不能流回真实飞机

## 51.1 先关闭控制出口

回放是重新发布消息，不只是“打开一个文件看一看”。若历史setpoint与真实MAVROS处于同一个可通信系统，可能产生真实控制风险。

回放时断开真实飞控路由，采用独立Domain，仅启动分析 / 显示节点，并按白名单选择传感器与事件话题。Domain隔离之外仍要检查MAVLink物理路由。[R35]

## 51.2 教学 bag 的回放方式

停止原软件Launch，防止新旧odom混在一起。设置新实验域，再进行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=89
export ROS_LOCALHOST_ONLY=1

ros2 bag play bags/<本次目录名> \
  --clock 30 \
  --topics /lab/odom /lab/buckets /lab/vision_ready \
           /lab/mission/events /tf /tf_static
```

此命令刻意不播放 `setpoint_preview`，也不启动任务决策节点。需要RViz使用历史时间时，另开同域终端：

```bash
rviz2 --ros-args -p use_sim_time:=true
```

`--clock 30`、`--topics`、`--remap`属于Humble播放接口；具体已安装版本仍可用 `ros2 bag play --help`确认。[R35A]

## 51.3 旧数据接收时重打 now，会改变重现实验

v11在odom回调内使用接收时间重建导航历史。因此即使播放原bag，它得到的历史时间也可能是回放时刻，而不是原记录中的采样时间。再叠加steady计时，原节点不会天然确定性重现所有实飞分支。[P1]

正确做法是区分：

```text
观察型回放：查看原始输入与输出
算法型回放：用明确的仿真时间和输入驱动重算
系统型仿真：重新运行飞控与环境
```

三者回答不同问题。

## 51.4 倍速回放不等于等比例任务计时

`--rate 0.5` 放慢消息，但Python monotonic和C++ steady clock仍按真实秒推进。固定0.5 s watchdog可能更容易触发，这是设计与时钟选择共同作用的结果。

正式的算法回放架构应把“时间来源”作为依赖注入，并记录原始采样时间、接收时间与任务使用时间，避免靠覆盖时间戳掩盖问题。

## 51.5 离线导出状态耗时

以下脚本只读bag，不发布ROS消息。它按session分组，计算每个状态事件到下一个事件之间的ROS时间差；最后状态没有下一个事件，持续时间留空。

**完整文件：`scripts/export_events.py`**

```python
#!/usr/bin/env python3
"""Read a local rosbag and export MissionEvent messages; never publishes ROS data."""
import argparse
import csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, default=Path('mission_events.csv'))
    parser.add_argument('--topic', default='/lab/mission/events')
    parser.add_argument('--storage', default='sqlite3', help='Match ros2 bag info Storage id')
    args = parser.parse_args()
    if not args.bag.exists():
        parser.error('bag path does not exist')
    if args.output.exists():
        parser.error('output already exists; select a new path to avoid overwriting')
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        parser.error(f'source ROS and the custom interface workspace first: {exc}')
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=args.storage),
                rosbag2_py.ConverterOptions('', ''))
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    required = 'cuadc_tutorial_interfaces/msg/MissionEvent'
    if types.get(args.topic) != required:
        parser.error(f'{args.topic} must have type {required}; found {types.get(args.topic)}')
    cls = get_message(required)
    rows = []
    previous_by_session = {}
    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if topic != args.topic:
            continue
        msg = deserialize_message(raw, cls)
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        row = dict(session_id=msg.session_id, sequence=msg.sequence,
                   ros_stamp_ns=stamp_ns, bag_time_ns=bag_ns,
                   previous_state=msg.previous_state, state=msg.state,
                   reason=msg.reason, seconds_until_next_event='')
        prev = previous_by_session.get(msg.session_id)
        if prev is not None:
            delta_ns = stamp_ns - rows[prev]['ros_stamp_ns']
            rows[prev]['seconds_until_next_event'] = (
                delta_ns / 1e9 if delta_ns >= 0 else 'CLOCK_BACKWARD')
        previous_by_session[msg.session_id] = len(rows)
        rows.append(row)
    fields = ['session_id', 'sequence', 'ros_stamp_ns', 'bag_time_ns',
              'previous_state', 'state', 'reason', 'seconds_until_next_event']
    with args.output.open('x', encoding='utf-8-sig', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f'Exported {len(rows)} events to {args.output}; terminal event duration is unknown.')


if __name__ == '__main__':
    main()
```

运行前source ROS和包含自定义消息的工作空间：

```bash
python3 scripts/export_events.py bags/<本次目录名> \
  --output mission_events.csv
```

它只支持本文的MissionEvent类型，不会自动理解v11文本日志。其他bag存储格式可按 `ros2 bag info` 的Storage id传入 `--storage`。

<a id="c52"></a>
# 52. 性能分析：从 CPU 占用到端到端延迟

## 52.1 为什么 CPU 50% 仍可能掉控制周期

多核总占用不高时，某个关键线程仍可能被阻塞、等待锁或集中运行在一个核心。CPU平均值不能代替回调耗时、调度延迟和尾部抖动。

【只读诊断】

```bash
top -H
ps -eo pid,ppid,pcpu,pmem,cmd --sort=-pcpu | head -n 20
free -h
df -h
```

有条件再使用系统性能分析或ROS tracing；先记录可重复的症状，不要盲目设实时优先级。[R40]

## 52.2 给 Tick 加时间预算

【局部示例】

```cpp
const auto begin = std::chrono::steady_clock::now();
// tick 的核心工作
const double elapsed_ms = std::chrono::duration<double, std::milli>(
  std::chrono::steady_clock::now() - begin).count();
if (elapsed_ms > 10.0) {
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
    "tick calculation slow: %.2f ms", elapsed_ms);
}
```

10ms是教学报警示例，不是适合所有飞机的统一阈值。循环间隔与循环内部计算耗时还应分别测量：计算只用1ms，也可能因调度停顿200ms后才被调用。

## 52.3 使用P95 / P99和最大值

平均延迟掩盖偶发长尾；最大值又可能被极少数异常支配。把分位数、最大值、样本数和故障发生阶段一起保存，才能进行合理比较。

## 52.4 高频日志本身也可能是负载

每帧打印完整图像、目标列表或浮点矩阵会产生大量格式化、终端和文件写入。正常运行用节流状态；状态迁移、请求发送与拒绝原因保留完整事件；故障附近的详细输入通过bag分析。

## 52.5 优化顺序

先删除无用数据与阻塞，再调整队列和执行器，然后检查网络、存储和推理并行，最后考虑组件化或底层传输优化。不要先把所有QoS改成Reliable或把队列放到10000。

**实践 14**：分别在“只运行导航”“加视觉”“加图像显示”“加bag”“加O4相关地面流程”下测同一组时延指标，形成对照表。

<a id="c53"></a>
# 53. 多机通信、DDS、RMW 与安全隔离

## 53.1 从单机扩展到机载电脑和地面电脑

两台电脑至少需要网络可达、匹配的Domain、兼容的消息接口与QoS，以及允许必要的发现 / 数据通信。运行在同一Wi-Fi并不自动满足这些条件。[R03][R37]

```bash
printenv ROS_DOMAIN_ID
printenv ROS_LOCALHOST_ONLY
printenv RMW_IMPLEMENTATION
ip -br addr
```

双机实验需显式取消localhost限制；不要在不理解网络边界时把所有接口开放到公共网络。

## 53.2 RMW 是可替换的中间层实现

ROS2可以使用不同RMW实现。更改 `RMW_IMPLEMENTATION` 后，CLI后台daemon与节点环境可能不同，造成“命令行看不见节点”的混淆；按官方说明重启daemon并统一环境。[R37]

```bash
ros2 daemon stop
ros2 daemon start
```

不要在比赛前为了尝鲜更换全队RMW。不同实现的发现、共享内存和网络参数应在独立环境测试。

## 53.3 namespace、frame前缀、Domain各管一件事

namespace区分ROS资源名字；frame前缀区分TF坐标名称；Domain区分通信发现域；MAVLinksystem ID区分飞控协议对象。多机系统要分别规划，不可只给Node加 `/uav1` 就认为全隔离了。

## 53.4 无线连接优先传什么

机载任务闭环应尽量在机载电脑与飞控之间完成，不依赖地面网络持续可用。地面优先接收必要遥测、结果与诊断，再按能力传图像。

O4视频链路和ROS网络可以相互独立，但仍需对供电、电磁环境、天线和现场负载做实际验证；本文没有相关硬件测试数据。

## 53.5 认证与权限

Domain ID和话题名不是访问控制。需要更强通信安全时，研究ROS2 / DDS安全机制与部署策略，同时保留物理控制权限制。[R41]

对于执行机构，应用层还应有唯一授权控制出口。普通显示节点不应因为加入同一网络就能发送投放请求。

<a id="c54"></a>
# 54. 一键启动、Shell 与 systemd 的工程边界

## 54.1 好的启动脚本不仅是 source 三行

它应该明确工作目录、环境、设备身份、配置文件、日志目录和退出码；防止同时启动两个任务控制器；并把预检和真正授权任务分开。

**完整文件：`scripts/run_lab.sh`，仅教学系统**

```bash
#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${CONDA_PREFIX:-}" || -n "${VIRTUAL_ENV:-}" ]]; then
  echo '请在未激活 Conda/venv 的干净 ROS 终端运行本练习。' >&2
  exit 2
fi
[[ -f /opt/ros/humble/setup.bash ]] || { echo '缺少 ROS 2 Humble' >&2; exit 2; }
[[ -f "$ROOT/install/setup.bash" ]] || { echo '先执行 colcon build' >&2; exit 2; }
# ROS 的环境脚本可能读取未定义变量，source 期间暂时关闭 nounset。
set +u
source /opt/ros/humble/setup.bash
source "$ROOT/install/setup.bash"
set -u
export ROS_DOMAIN_ID=88
export ROS_LOCALHOST_ONLY=1
exec ros2 launch cuadc_tutorial_bringup lab.launch.py "$@"
```

`exec` 让主进程接收信号的关系更清楚。`set -euo pipefail` 帮助暴露脚本错误，但ROS环境脚本可能读取未定义变量，所以示例在source附近暂时关闭nounset，而不是假定所有外部脚本都兼容。

## 54.2 systemd 自动启动不应该等于自动起飞

可以让只读诊断、日志或设备健康检查在开机时运行；任务启动授权必须单独设计。相机或日志守护进程的自动重启也不能自动复位任务或重新初始化机构。

【教学配置示意，不直接启用】

```ini
[Unit]
Description=CUADC read-only monitoring
After=network-online.target

[Service]
Type=simple
User=cuadc
WorkingDirectory=/home/cuadc/approved_release
ExecStart=/home/cuadc/approved_release/scripts/run_monitor_only.sh
Restart=no
TimeoutStopSec=10

[Install]
WantedBy=multi-user.target
```

这里的用户、路径和脚本都要由实际部署提供。本文没有交付 `run_monitor_only.sh`，此片段只说明服务边界，不可原样启用。

## 54.3 不要把依赖顺序当成业务ready

`After=network-online.target` 不说明飞控已连接，更不说明视觉、导航与机构就绪。Launch中的启动事件同样不说明传感器已通过稳定门禁。

## 54.4 停止与清理

只结束本次启动的进程组，保存日志和退出原因；不要用无差别 `pkill python` 或 `killall` 清理现场。对真实飞行中的进程，停止软件不等于安全降落，应先依照接管与飞控失联策略处置。

## 54.5 为每个实验记录生效配置

启动日志应包含版本、设备标识、参数文件路径、必要文件哈希、ROS发行版和飞控固件版本。避免“同名文件夹其实内容不同”的交付问题。

<a id="c55"></a>
# 55. 控制权与安全：ROS 不是最后一道保护

## 55.1 一条真实控制输出应该只有一个明确所有者

两个节点同时发布 `/mavros/setpoint_position/local`，并不会自动协商谁更重要。飞控可能交替收到互相冲突的参考。

同样，地面站、脚本和状态机同时发送模式或舵机命令，也可能产生竞争。需要明确控制权移交，而不是“谁最后发包谁说了算”。

## 55.2 v11 飞手接管逻辑

自动飞行状态中离开GUIDED时，代码停止发布自动目标，进入PILOT_OVERRIDE；若涉及释放动作，尝试回收机构并等待相关收尾。它不会主动把模式切回GUIDED。[P1]

这和飞控自身安全开关、遥控失联策略、独立人工断动力手段是不同层次。软件停止不能替代硬件安全设计。

## 55.3 进程崩溃后不能保证执行 C++ 析构函数

断电、内核问题、强制终止或硬件故障可能绕过正常退出逻辑。不能把“析构时发land”当作可靠失效保护。

执行端应有经过验证的模式 / 失联行为和独立接管路径，且在仿真及受控测试中证明其实际效果。[R29]

## 55.4 当前落地判据的局限

v11通过相对高度与速度连续稳定确认落地，不依赖ExtendedState。默认实际上是|相对高度|≤0.12m、水平速度≤0.10m/s、|垂直速度|≤0.08m/s，持续至少2s。[P1]

这仍是估计状态推断，不是触地开关。地面高程变化、EKF高度漂移或错误参考原点都可能影响判断，不能称为普遍保证。

## 55.5 故障策略要与可观测性匹配

“视觉失效但导航可信”和“导航失效”不是同一个返航条件；“释放ACK丢失”和“机构传感器确认未动作”也不是同一个状态。

【教学设计】使用明确分类：可继续、可重试、保持等待、请求返航、请求降落、人工接管、动作结果不确定。每个分类都要记录依据，避免一个 `catch (...)` 把所有错误都当成可以重试。

<a id="c56"></a>
# 56. 测试与持续集成：怎样证明修改没有破坏原来的能力

## 56.1 四类测试逐层推进

纯函数测试检查几何与时序边界；ROS接口测试检查类型、QoS与调用；系统测试检查状态迁移和故障；SITL / 实机测试检查飞控与物理执行。

较低层测试通过不代替较高层。配套9个纯Python测试属于第一层。[R38]

## 56.2 标准测试命令

```bash
colcon test --packages-select cuadc_tutorial_py
colcon test-result --verbose
```

先确认构建完成并source，检查测试结果而不只看命令退出码。需要详细输出可按安装版本使用console事件处理选项。[R38]

## 56.3 只读静态检查

```bash
python3 -m compileall -q src/cuadc_tutorial_py scripts
bash -n scripts/run_lab.sh
```

语法检查不验证消息类型支持是否正确安装，更不验证节点实际交互。C++必须在目标ROS环境通过编译和链接；本文交付时的实际验证范围见文末。

## 56.4 一个任务回归用例应怎样写

```text
前置：固定版本，独立实验域，模拟服务和导航正常。
输入：第6秒开始停止视觉2秒。
预期：健康检查变为false；任务进入FAULT；停止新动作请求；
      保留已发送请求结果不确定的事件；不自动恢复原任务。
证据：bag、事件CSV、进程日志、参数快照。
```

真实v11的用例应按其SEARCH视觉重捕获宽限和后续目标冻结逻辑重新写，不直接拿教学工程FAULT策略作为标准。[P1]

## 56.5 CI 不接硬件

CI适合编译、格式、单元测试和隔离的mock / SITL；不要把真实飞控凭据、串口或真实图传路由暴露给普通代码提交任务。

固定依赖版本与基础镜像，记录编译器和ROS包版本；若使用硬件在环测试，必须是额外受控流程，而不是任何分支提交都自动解锁。

## 56.6 修改后的最小回归集合

坐标正负方向、角度跨±π、视觉空帧与断流、时间回跳、50ms夹取边界、第二目标记忆非新鲜视觉、投放请求幂等、ACK超时、接管后不抢回控制权、落地确认丢失时不上锁。

这些是针对你们任务的高价值测试，比只检查“程序能启动”更能防止比赛退化。


<a id="c57"></a>
# 57. 工作空间发布、版本管理与团队传承

## 57.1 应该交付什么

对新人，推荐交付源码、构建清单、参数、Launch、启动脚本、测试和README。不要只交一个自己电脑上能运行的 `install/`，却不说明依赖、CPU架构和构建环境。

对于比赛用固定发布件，可以额外保存经验证的构建结果，但必须绑定源码版本、参数快照、系统环境和设备适配记录。

## 57.2 工作空间目录职责

| 目录 | 作用 | 发布考虑 |
|---|---|---|
| `src/` | 源码与接口 | 必须保留 |
| `build/` | 编译中间结果 | 通常不作为新人源码交付主体 |
| `install/` | 可被source的安装空间 | 可重建；预编译交付需说明环境 |
| `log/` | 构建日志 | 调试有用，避免误当飞行数据 |
| `config/` | 参数与运行配置 | 绑定硬件版本 |
| `bags/` | 实验数据 | 单独管理大小与隐私 |
| `scripts/` | 启动和分析入口 | 写清工作目录与前置条件 |

源码中不能暗藏只在一个人的home目录存在的绝对模型路径。必要资源应通过包share目录或明确参数查找。

## 57.3 实验版本与比赛版本分开

```text
development：允许频繁修改
candidate：固定依赖，进行回归
approved_release：只使用已通过验收的配置
```

每次从candidate变为approved，都记录“哪些测试通过，哪些未覆盖”。不能仅因为文件名加了 `final` 或 `latest` 就视为可信版本。

## 57.4 回滚要同时回滚相关契约

回滚C++但保留新YAML、回滚视觉但保留新消息定义，可能制造比原故障更隐蔽的问题。接口类型、参数、相机标定与机构行程应成套管理。

ROS参数数据库与程序内部有效参数可能不同，尤其v11存在rescue profile覆盖，发布记录要保存内部实际使用值。[P1]

## 57.5 保留证据而非只保留经验

“上次就是这样飞成功的”不如一份manifest：源码commit、参数sha256、相机和飞控身份、实测odom间隔、任务事件、投放结果、是否人工接管。

**实践 15**：把配套工作空间复制到另一台干净的Humble开发机，只按README构建。记录所有README没有覆盖的依赖，回补后再试一次。

<a id="c58"></a>
# 58. 任务耗时、成绩条件和软件成功之间的关系

## 58.1 程序计时不等于裁判计时

v11的 `mission_start_time_` 在检测已解锁并进入TAKEOFF时更新；总超时默认240s，而且RELEASE、返航及落地等状态存在不同的排除和独立处理逻辑。[P1]

你们上传的2026规则Ver20260330，对比赛开始、机组计时、填写记录单、侦察、着陆和完赛有专门定义。软件的 `DONE` 不能直接替代这些判定。本章只解释已上传版本，不宣称后续公告没有调整。[P4]

## 58.2 三个时间要分别报告

参考轨迹理论时长、飞行器实际执行时长、比赛计时器最终用时。

参考时长由规划器给出；实际执行还包含跟踪误差、对准等待、ACK与重试；比赛计时还取决于规则规定的人工作业和成绩记录。用一张表混写，会让优化目标偏离真正影响成绩的部分。

## 58.3 任务时间表应该来自状态事件

| 阶段 | 计划时间 | 实际时间 | 超差原因 |
|---|---:|---:|---|
| 起飞准备 / 上升 | 估算 | 从日志提取 | NAV30、解锁、爬升 |
| 到投放区 | 轨迹值 | odom与事件 | 跟踪落后、减速 |
| 搜索 | 路线 / 终止策略 | SEARCH时长 | 凑齐三目标时刻 |
| A1 / A2对准投放 | 门限与转场估算 | 状态时长 | 视觉、姿态、ACK |
| 侦察 | 路线与观察需求 | 路线和视频事件 | 视场、判读、等待 |
| 返航与落地 | 估算 | 事件与未armed | 减速、落地确认 |

不要把缺少日志的阶段填成看似精确的小数。未知就写未知，并安排测量。

## 58.4 规则要求的结果比路线完成更强

上传规则要求载荷投放与有效区结果满足后续任务条件，并依据侦察记录与着陆状态认定得分。没有落点和判读结果的传感器时，程序只能维护“命令已执行到某一步”，不能自动证明比赛条件都成立。[P4]

## 58.5 时间分的取舍要有数量级意识

按上传规则的时间分公式，完赛条件成立时，每节约1秒约对应200/300分，即约0.67分。这个计算依赖该规则版本，并不意味着每种加速都划算。[P4]

增加一次错误判读或丢失投放有效性，可能比节省若干秒影响更大。因此ROS系统优化应优先消除无意义等待、重复搜索和阻塞，而不是牺牲数据可信度。

<a id="c59"></a>
# 59. 故障排查手册：从症状找到证据

## 59.1 `ros2: command not found`

先检查ROS是否安装和当前终端是否source；再检查PATH。不要立即重装系统。

```bash
ls /opt/ros
command -v ros2
printenv ROS_DISTRO
```

## 59.2 `Package not found`

检查包是否在src、是否被colcon发现、是否构建成功、是否source正确overlay。用 `ros2 pkg prefix 包名` 看当前究竟解析到哪个目录。

如果source了多个同名包工作空间，后面的overlay可能覆盖前面的。清理终端环境后再确认。

## 59.3 找到包但 `No executable found`

C++检查install(TARGETS)、可执行目标名和编译结果；Python检查console_scripts、setup.cfg、安装后入口和模块路径。文件名不一定等于可执行名。

```bash
ros2 pkg executables cuadc_tutorial_cpp
ros2 pkg executables cuadc_tutorial_py
```

## 59.4 有话题但订阅无数据

依次检查类型、发布者实际数量、QoS、Domain、RMW、网络及回调是否被阻塞。

```bash
ros2 topic info /lab/odom --verbose
ros2 topic echo /lab/odom --qos-reliability best_effort --once
```

发布者存在不表示正在发消息；同名但不同类型也不表示能正确连接。

## 59.5 参数设置成功但行为不变

检查节点全名、YAML匹配、参数读取时机、是否缓存到成员变量、是否有动态回调、是否被profile覆盖。v11默认rescue profile会强制若干数值，单看 `ros2 param get` 不足以证明内部有效值。[P1]

## 59.6 时间戳总是“来自未来”

检查设备时间与ROS时间是否同纪元；是否开启use_sim_time却没接clock；两台电脑是否有时钟偏差；是否在发布时错误叠加了时间；bag是否用了当前系统时间解析历史数据。

不要第一反应把future tolerance改到几秒，应该先解释偏差来源。

## 59.7 `HARD_SYNC_REJECT_AFTER_WAIT`

在v11中，这表示等待窗口结束仍找不到满足最大间隔的前后导航样本。检查odom实际回调间隔、视觉时间戳范围、时钟回跳、数据是否被重排序和执行器是否堵塞。[P1]

平均30Hz并不能排除一个关键帧落在100ms空洞里。

## 59.8 `VISUAL_FAILSAFE`

先查是哪种原因：心跳断流、时间对齐失效还是时钟回跳。v11的SEARCH会先保持位置尝试重捕获，再按条件返航或降落；不是所有状态都要求持续视觉。[P1]

不要把下一步改成“无论什么错都继续投”，那会破坏目标确定的前提。

## 59.9 相机框稳定，但桶世界坐标随飞机移动

优先检查body / optical轴、相机外参、观测时间和姿态、是否重复旋转，以及p_vehicle的物理参考点。可以使用静止目标、只转机体的实验分离问题。

仅增加EMA平滑会让错误变慢，不会把错误坐标系变正确。

## 59.10 整条航线向左偏

区分固定横向平移、航向角误差导致远处发散、ENU / field混用、路径跟踪超调。v11的field_lateral_offset只能处理其定义下的整体横向标定，不能替代航向和姿态校验。[P1]

测多个不同前向距离处的偏差，比在一个点上调offset更有辨识力。

## 59.11 setpoint很快，飞机实际很慢

检查飞控限制、轨迹加速度要求、闭环误差、setpoint实际到达频率及控制模式。不要只看C++中12.0就断言飞控执行有问题。

## 59.12 Service一直pending

检查服务是否存在、类型正确、服务端回调能否执行、客户端是否阻塞自身Executor、回调组是否互斥导致死锁、请求是否超时后仍未清理。[R15][R16][R22]

真实MAVLink命令还要检查路由、命令支持和FCU回复，不能只看ROS服务发现。

## 59.13 舵机ACK成功但瓶子没掉

检查机械挂载、行程、供电、通道映射、servo功能占用，以及是否有真实物理反馈。ACK成功不等于该机构完成预期动作。[R17][R30]

不要在未搞清物理状态时重复对同一机构发送不可逆动作。

## 59.14 第一瓶后直接返航

需要定位准确状态和reason：第二目标记忆缺失、对准超时、release门禁、身份限制、服务异常或总任务超时，均可能导致不同分支。仅说“只投了一瓶”不能唯一判断视觉出了问题。

## 59.15 O4侦察已经看完，最后仍报未完整成功

检查是否还在使用v11的六照片ACK成功条件。没有迁移成功契约时，O4导航完成不会替旧D435i拍照服务生成ACK。[P1]

## 59.16 rosbag有odom但没有TF

检查是否录制/tf与/tf_static、静态发布端与记录器QoS、录制是否晚于静态发布且持久性不匹配。也要确认系统本来是否发布TF；v11手算坐标并不代表自动产生一棵TF树。

## 59.17 RViz一回放就提示变换外推

检查Fixed Frame、use_sim_time、/clock来源和历史缓存；确认没有实时TF发布者与旧bag同时运行。不要通过把所有图像时间戳改成当前时间来掩盖问题。

## 59.18 单机好用，双机无消息

检查localhost限制、相同Domain、网络接口、发现组播 / 防火墙、RMW配置和消息版本。先传一个小String再传图像，避免把带宽问题和发现问题混在一起。[R03][R37]

## 59.19 断开终端后任务不按预期结束

检查shell与子进程关系、是否后台运行、Launch停止路径和飞控失联策略。SSH断开或kill一个Node并不天然等于飞机返航；实际行为由整条控制链和FCU配置决定。

## 59.20 一次排障报告的合格格式

```text
症状：
最早出现的时间：
受影响的状态：
已确认的输入事实：
尚未确认的假设：
最小复现步骤：
修改了什么：
验证结果：
是否完成相关回归：
```

把事实与猜测分开，是团队长期进步的关键。

<a id="c60"></a>
# 60. 学习计划、阶段验收和自测答案

## 60.1 建议按能力阶段推进，不按“读完页数”推进

| 阶段 | 章节 | 必须交出的成果 |
|---|---|---|
| 入门 | 1～8 | 一张系统图、可构建工作空间、CLI观察记录 |
| 节点与通信 | 9～16 | C++与Python节点、消息定义、QoS故障复现 |
| 任务接口 | 17～24 | 异步服务、幂等请求、Action、Launch与日志 |
| 时间与坐标 | 25～34 | 纯函数测试、TF树、时间戳契约、视觉替身 |
| 飞控理解 | 35～44 | 真实接口表、门禁图、O4迁移表、高速约束分析 |
| 综合验证 | 45～56 | mock闭环、故障注入、bag、SITL与测试报告 |
| 团队交付 | 57～60 | 一份能被另一人复现的发布件与验收单 |

## 60.2 16项实践的最终检查表

| 编号 | 任务 | 通过标准 |
|---|---|---|
| 0 | 写系统接口图 | 每条箭头有类型、frame、时间、失效语义 |
| 1 | 基础环境与CLI | 能解释当前source与包解析路径 |
| 2 | 创建并构建包 | 新终端可运行，不依赖隐藏环境 |
| 3 | Python / C++消息交流 | 类型与namespace正确 |
| 4 | 自定义接口 | interface show与程序一致 |
| 5 | QoS故障 | 能复现并解释Reliable订阅BE发布失败 |
| 6 | 异步模拟服务 | 等待期间odom仍持续处理 |
| 7 | Action与参数Launch | 区分接收目标、反馈与最终结果 |
| 8 | 结构化日志 | 可提取每次状态迁移原因 |
| 9 | 时间 / 几何与视觉契约 | 边界测试通过，未知标定明确标出 |
| 10 | 视觉替身故障 | 区分空帧、无效帧、断流 |
| 11 | 真实状态图 | ALIGN_FINE各分支完整 |
| 12 | O4迁移设计 | 不伪造照片ACK，成功条件分层 |
| 13 | SITL坐标验证 | 四方向与固定航向可证明 |
| 14 | 性能对照 | 有均值、尾部间隔、负载与样本数 |
| 15 | 跨机交付 | 另一人只靠README可完成软件构建与实验 |

上表是集中验收编号；章内小练习可以继续细分，不要求所有学习者同一天完成。

## 60.3 自测题

**题1：为什么topic list看得到话题，订阅回调却不执行？**

可能发布端没有实际数据、QoS不兼容、类型 / 域 / 网络不匹配，或执行器被堵塞。先 `topic info --verbose` 和独立echo，不要先重装ROS。

**题2：把图像header.stamp改成推理结束的now，能降低延迟吗？**

不能。它只让旧观测看起来新鲜，破坏历史位姿对齐。应保留观测时间并测量真实流水线延迟。

**题3：Reliable保证水瓶释放一次且只一次吗？**

不保证。DDS可靠传输、ROS服务回复、MAVLink ACK和物理机构动作是不同层。要设计应用ID、幂等性、超时不确定性与必要物理反馈。

**题4：30Hz导航是否必然满足v11同步？**

不必然。最大样本间隔、消息年龄、前后夹取和时钟域都要满足。

**题5：frame_id正确，坐标一定正确吗？**

不一定。字符串不会旋转坐标；相机安装、单位和物理参考点仍需标定验证。

**题6：ros2 param set返回成功，代码一定用了新值吗？**

不一定。节点可能只在构造时读取，或者被内部profile覆盖。要检查参数应用逻辑与有效值输出。

**题7：MultiThreadedExecutor一定解决回调阻塞吗？**

不一定。默认互斥回调组、共享锁和同步等待都可能继续阻塞，甚至引入数据竞争。

**题8：O4图传显示正常，ROS能知道识别成功吗？**

没有设计相应反馈接口时不知道。路线完成、视频可用和判读正确应区分。

**题9：位置目标速度设置12m/s，参考时长就是实际耗时吗？**

不是。实际受加速度、制动、飞控限制、跟踪误差和阶段门禁影响。

**题10：关闭状态机进程可以保证飞机安全降落吗？**

不能。进程退出不自动产生可靠LAND动作，飞控失联与人工接管必须独立设计和验证。

**题11：为什么第二目标记忆不能打成新鲜视觉？**

复制旧目标不会创造新观测。伪造新arrival会绕过细对准对当前视觉的判断。

**题12：为什么完整任务成功不能只看DONE？**

DONE的含义由代码定义；载荷实际落点、侦察正确性、照片 / 视频证据和比赛记录仍需额外验证。

## 60.4 最后一个综合作业

在不改真实v11的情况下，用配套工作空间完成mock任务，记录bag，导出状态CSV，注入一次视觉时间冻结故障，然后提交两份对照报告。

报告必须包含：系统图、构建和启动命令、有效参数、时钟与坐标契约、状态事件、故障原因、恢复边界，以及哪些结论仍不能外推到真机。

完成这项作业后，再进入SITL和受控台架；这比直接背诵几十个ROS命令更接近“能为CUADC任务独立负责”的能力。

---


<a id="appendix-a"></a>
# 附录 A. 日常命令速查表

所有示例先确认工作目录、ROS环境和实验隔离。标为只读的命令不主动发飞行指令，但仍可能增加系统负载。

## A.1 环境与包

| 目的 | 命令 | 解释 |
|---|---|---|
| 加载ROS | `source /opt/ros/humble/setup.bash` | 只影响当前shell及子进程 |
| 加载工作空间 | `source install/setup.bash` | 必须在对应根目录或使用绝对路径 |
| 查看发行版 | `printenv ROS_DISTRO` | 检查当前环境，不等于全系统版本审计 |
| 查包目录 | `ros2 pkg prefix 包名` | 定位当前overlay生效位置 |
| 查执行入口 | `ros2 pkg executables 包名` | 包名与可执行名可能不同 |
| 查构建发现 | `colcon list` | 与运行时包解析不同 |
| 按依赖构建 | `colcon build --packages-up-to 包名` | 同时构建其工作空间依赖 |
| 只构建一个包 | `colcon build --packages-select 包名` | 依赖须已可用 |
| 运行测试 | `colcon test` | 随后查test-result |
| 汇总测试 | `colcon test-result --verbose` | 不忽略失败项 |

## A.2 图、消息与QoS

| 目的 | 命令 |
|---|---|
| 列节点 | `ros2 node list` |
| 节点接口 | `ros2 node info /lab/sim_plant` |
| 列话题与类型 | `ros2 topic list -t` |
| 话题端点 / QoS | `ros2 topic info /lab/odom --verbose` |
| 读取一条 | `ros2 topic echo /lab/odom --qos-reliability best_effort --once` |
| 频率 | `ros2 topic hz /lab/odom` |
| 带宽 | `ros2 topic bw /lab/odom` |
| 消息定义 | `ros2 interface show nav_msgs/msg/Odometry` |
| 服务列表 | `ros2 service list -t` |
| 服务类型 | `ros2 service type /lab/payload/operate` |
| Action列表 | `ros2 action list -t` |
| Action连接 | `ros2 action info /lab/scan_mock` |

不同CLI版本可用选项以 `--help`为准。命令行统计是工具接收端的观测，并不直接等于源传感器硬件采样率。

## A.3 参数与启动

```bash
ros2 param list /lab/sim_plant
ros2 param describe /lab/sim_plant max_speed_m_s
ros2 param get /lab/sim_plant max_speed_m_s
ros2 param dump /lab/sim_plant
ros2 launch cuadc_tutorial_bringup lab.launch.py --show-args
ros2 launch cuadc_tutorial_bringup lab.launch.py mission:=true
```

`param dump`写出的仍是节点参数系统值，不能反映未回写参数数据库的内部覆盖。

## A.4 TF与bag

```bash
ros2 run tf2_ros tf2_echo lab_map lab_release_1
ros2 run tf2_tools view_frames
ros2 bag info <bag目录>
ros2 bag record --help
ros2 bag play --help
```

不要在真实控制系统中试验bag播放；第51章给出隔离和白名单方法。

## A.5 本教学工程的可执行入口

```text
cuadc_tutorial_py:
  hello
  sim_plant
  vision_stub
  perception_guard
  payload_server
  payload_client
  mission_dry_run
  scan_action_server
  scan_action_client
  tf_demo

cuadc_tutorial_cpp:
  telemetry_observer
  async_probe

cuadc_tutorial_bringup:
  lab.launch.py
```

独立运行节点时要加 `--ros-args -r __ns:=/lab`。Launch已统一设置namespace，不必重复添加。

<a id="appendix-b"></a>
# 附录 B. C++ / Python ROS API 查阅手册

## B.1 生命周期与节点对象

| API | 作用 | 使用时要问什么 |
|---|---|---|
| `rclcpp::init / rclpy.init` | 初始化ROS上下文 | 是否在同一上下文重复初始化？ |
| `rclcpp::Node / rclpy.node.Node` | 节点对象 | 名字与namespace最终是什么？ |
| `rclcpp::spin / rclpy.spin` | 执行回调 | 有哪些回调可能阻塞？ |
| `spin_some` | 执行一部分就绪工作 | 外层循环是否忙等，是否饥饿？ |
| `rclcpp::shutdown / rclpy.shutdown` | 关闭上下文 | 如何先收尾线程与资源？ |
| `destroy_node` | Python释放节点资源 | 是否仍有线程访问该对象？ |
| `get_name / get_namespace` | 读取节点名和命名空间 | 检查重映射后的实际值 |
| `get_logger` | 获取节点日志器 | 日志是否包含阶段与原因？ |
| `get_clock / now` | 获取ROS时间 | use_sim_time是否开启？ |

## B.2 Pub / Sub / Timer

| API | 作用 | 常见错误 |
|---|---|---|
| `create_publisher<T>` | C++创建发布者 | 类型、QoS或话题名不匹配 |
| `create_publisher(Type, name, qos)` | Python创建发布者 | 把参数位置记反或类型没生成 |
| `publish(msg)` | 发布消息 | 认为返回即代表业务完成 |
| `create_subscription` | 注册订阅回调 | 丢失对象引用、回调阻塞、QoS不兼容 |
| `create_wall_timer` | C++按墙钟周期调度 | 认为周期是硬实时保证 |
| `create_timer` | Python创建Timer | 与所用Clock / Executor行为混淆 |
| `timer.cancel()` | 停止继续触发Timer | 认为会取消已发出的远端动作 |
| `rclcpp::SensorDataQoS` | 传感器常用QoS配置 | 没确认对端请求的兼容性 |
| `QoS(depth).reliable()` | 指定可靠性 | 把可靠传输当成低延迟保证 |
| `transient_local()` | 保留发布端历史供匹配订阅者 | 当成磁盘持久化或无限缓存 |

## B.3 Service、Future与Action

| API / 对象 | 作用 | 关键语义 |
|---|---|---|
| `create_service` | 创建服务端 | 回调返回只代表该服务定义的结果 |
| `create_client` | 创建客户端 | 与create_subscription不同 |
| `service_is_ready` | 当前服务是否可用 | 可用性之后仍可能变化 |
| `wait_for_service` | 等服务发现 | 不能无限阻塞关键控制路径 |
| `async_send_request` | C++异步请求 | 保存future及请求ID |
| `call_async` | Python异步请求 | 不阻塞并继续spin |
| `future.done / wait_for(0)` | 非阻塞检查完成 | 完成也可能是异常或拒绝 |
| `future.result / get` | 读取回复 | 处理异常与返回结果 |
| `remove_pending_request` | 清理本地未完成请求登记 | 不撤销远端动作 |
| `Future.cancel` | 取消本地future等待状态 | 不等于MAVLink取消 |
| `ActionClient.send_goal_async` | 提交Action目标 | 先返回目标是否接受 |
| `get_result_async` | 等最终结果 | 不等于目标接受Future |
| `publish_feedback` | 服务端发送进度 | 进度与成功仍不同 |
| `cancel_goal_async` | 请求取消目标 | 是否接受、何时停止需由服务端实现 |
| `goal_handle.succeed / abort / canceled` | 设置结束状态 | execute还需返回Result |

## B.4 参数与Launch

| 名称 | 用途 |
|---|---|
| `declare_parameter` | 声明类型和默认值 |
| `get_parameter` | 读取参数系统值 |
| `add_on_set_parameters_callback` | 验证 / 处理参数更新；注意回调生命周期 |
| `SetParametersResult` | 返回是否接受修改及原因 |
| `LaunchDescription` | 描述启动动作集合 |
| `DeclareLaunchArgument` | 声明启动参数 |
| `LaunchConfiguration` | 在运行时解析启动参数 |
| `Node`（launch_ros） | 启动ROS节点进程或入口 |
| `IfCondition` | 条件启动，不是业务ready |
| `FindPackageShare` | 查安装后的包资源目录 |
| `PathJoinSubstitution` | 构造可替换路径 |
| `ExecuteProcess` | 启动一般程序，如bag记录器 |
| `RegisterEventHandler` | 对进程等事件作反应 |

## B.5 tf2和常用消息

`TransformBroadcaster`发布动态变换；`StaticTransformBroadcaster`发布静态关系；`Buffer`保存可查询变换；`TransformListener`订阅并更新Buffer；`lookup_transform(target, source, time)`请求指定时间下的关系。

| 消息 | 适用内容 | 不应误解为 |
|---|---|---|
| `PoseStamped` | 某frame中的位姿与时间 | 自动包含速度或必然为飞行指令 |
| `PointStamped` | 带frame和时间的点 | 具有方向的完整位姿 |
| `Odometry` | pose、twist、协方差与frame语义 | 原始GPS或RTK天线坐标 |
| `PoseArray` | 同一header下多个位姿 | 标准桶检测消息 |
| `Image` | 像素、编码与布局 | 必然是JPEG或RGB |
| `CameraInfo` | 相机内参 / 成像模型信息 | 完整机体安装外参 |
| `Path` | 多个带时间位姿的路径 | 自动被飞控执行的任务 |
| `Marker` | 可视化几何 / 标签 | 真实世界物体存在证明 |
| `DiagnosticArray` | 诊断状态集合 | 通用自动起飞授权 |

<a id="appendix-c"></a>
# 附录 C. v11 源码 ROS 接口与函数定位

## C.1 订阅接口

| 实际名字 | 类型 | 源码使用的QoS | 用途 |
|---|---|---|---|
| `/mavros/state` | `mavros_msgs/msg/State` | Reliable，depth10 | connected、armed、GUIDED状态 |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | SensorDataQoS | 位姿、速度、导航历史 |
| `/mavros/extended_state` | `mavros_msgs/msg/ExtendedState` | SensorDataQoS | 保留诊断，本版落地不依赖它 |
| `/mavros/global_position/compass_hdg` | `std_msgs/msg/Float64` | SensorDataQoS | 初始罗盘锁定 |
| `/cuadc/nav30_ready` | `std_msgs/msg/Bool` | Reliable，depth5 | 外部导航流ready |
| 默认 `/perception/drop_buckets_body` | `geometry_msgs/msg/PoseArray` | SensorDataQoS | 桶检测，可由参数改话题 |
| `/cuadc/recon/capture_done` | `std_msgs/msg/UInt32` | depth10默认QoS | 照片编号ACK |

## C.2 发布与服务客户端

| 接口 | 类型 | 是否可能影响真实执行 |
|---|---|---|
| `/mavros/setpoint_position/local` | PoseStamped | 是，飞行位置参考 |
| `/cuadc/recon/photo_mode` | Bool | 改变外部相机进程模式 |
| `/cuadc/recon/capture_request` | PointStamped | 触发外部拍照处理 |
| `/mavros/cmd/takeoff` | CommandTOL | 是，起飞请求 |
| `/mavros/cmd/land` | CommandTOL | 是，降落请求 |
| `/mavros/cmd/arming` | CommandBool | 是，解锁 / 上锁 |
| `/mavros/cmd/command` | CommandLong | 是，含舵机控制 |

正确前缀是 `/cuadc`，不是 `/cudac`。`capture_request.header.frame_id`在此源码中用来编码`recon_wp_N`，不是已经建立的标准TF坐标系；这是项目自定义语义。[P1]

## C.3 关键函数定位

以下行号针对本次上传文件自动提取；后续添加注释后行号可能变化，应以函数名搜索为主。

| 函数 | 上传文件起始行 | 主要责任 |
|---|---:|---|
| `VisualDropMissionNode()` | 230 | 构造节点、创建所有通信实体和Timer |
| `declare_parameters()` | 233 | 声明参数默认值 |
| `load_parameters()` | 234 | 读取、裁剪、profile覆盖与安全配置检查 |
| `state_callback()` | 846 | FCU连接、GUIDED、armed与人工接管 |
| `odom_callback()` | 885 | 更新位姿速度与接收时间导航历史 |
| `compass_callback()` | 980 | 罗盘角转ENU、维护航向样本 |
| `nav30_ready_callback()` | 816 | 外部30Hz就绪信号及到达时间 |
| `navigation_sample_at()` | 1001 | 严格前后夹取与历史姿态插值 |
| `bucket_callback()` | 1130 | 视觉时间戳和PoseArray协议解码 |
| `process_pending_vision_frames()` | 966 | 等待可对齐导航样本 |
| `process_time_aligned_vision_frame()` | 1231 | body→local与目标跟踪入口 |
| `try_reacquire_active_target_from_frame()` | 1313 | 按位置与直径约束重新绑定任务目标 |
| `geometry_yaw_from_odom()` | 1387 | 锁定绝对航向加相对odom yaw变化 |
| `body_to_local()` | 1427 | 观测时刻的完整刚体目标变换 |
| `release_point_local_current()` | 1465 | 实时投放口世界坐标 |
| `desired_vehicle_pose_for_release_target()` | 1477 | 按姿态与机构外参反解中心XY |
| `field_to_local()` | 1502 | 比赛field→local |
| `local_to_field()` | 1520 | local→比赛field |
| `merge_frame_detections()` | 1295 | 跨帧目标一对一关联 |
| `try_lock_target_plan()` | 1729 | 优先三桶选二、路线末端可接受两桶 |
| `state_requires_vision()` | 1896 | 当前仅SEARCH强制持续视觉 |
| `landing_candidate()` | 1903 | odom相对高度与速度着陆候选 |
| `navigation_ready_to_lock()` | 2076 | 锁场地前导航、时间和静止门禁 |
| `lock_frame()` | 2089 | 锁home、mission yaw并构建路线 |
| `build_search_route()` | 2102 | 投放区七线蛇形 |
| `build_recon_route()` | 2103 | 旧版六点RGB侦察路线 |
| `request_recon_photo()` | 2194 | 发送带航点编号的拍照请求 |
| `recon_capture_ack_callback()` | 823 | 照片编号去重与确认计数 |
| `tick()` | 2372 | 20Hz调度、ACK、健康与状态机 |
| `flight_gate_ok()` | 2717 | 自动飞行通用门禁 |
| `update_search()` | 2561 | 搜索推进、选桶与重试 |
| `update_alignment()` | 2567 | 1.7m粗对准 |
| `update_fine_alignment()` | 2573 | 1.3m细对准与last-trusted fallback |
| `update_release()` | 2579 | 最终几何与运动门禁、机构协议推进 |
| `finish_payload_release()` | 3463 | 载荷计数、第二目标或侦察阶段 |
| `start_segment()` | 2004 | 参考航段时长与梯形规划 |
| `slew_target_toward()` | 3154 | 基于dt的目标位置变化率限制 |
| `sample_segment()` | 3603 | 采样梯形位置参考 |
| `segment_complete()` | 3644 | 时间结束且实际位置进入终点半径 |
| `initialize_servos_if_ready()` | 2379 | 地面条件下机构初始化请求 |
| `send_servo()` | 874 | MAVLink命令183、通道与PWM |
| `check_servo_results()` | 2377 | 舵机ACK及超时处理 |
| `check_service_results()` | 2376 | 起降、解锁等服务返回处理 |
| `publish_setpoint()` | 2382 | 发布固定任务航向的位置参考 |
| `enter()` | 877 | 状态进入时间与迁移记录 |

## C.4 不能从这一份 C++ 确认的内容

相机内参、相机完整安装外参、视觉模型版本、NAV30监测脚本具体规则、拍照进程的保存目录和落盘可靠性、实际加载YAML、飞控所有参数、当前O4的安装与显示接口、实测最大速度与制动能力。

对这些内容，必须读取对应文件或进行测量；不要用源码注释、过往对话设想或默认参数补成已确认事实。

<a id="appendix-d"></a>
# 附录 D. 配套工作空间完整结构与剩余文件

## D.1 文件树

```text
cuadc_ros2_lab_ws/
├── scripts/
│   ├── export_events.py
│   └── run_lab.sh
├── src/
│   ├── cuadc_tutorial_bringup/
│   │   ├── config/
│   │   │   ├── bag_qos.yaml
│   │   │   └── lab.yaml
│   │   ├── launch/
│   │   │   └── lab.launch.py
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── cuadc_tutorial_cpp/
│   │   ├── src/
│   │   │   ├── async_probe.cpp
│   │   │   └── telemetry_observer.cpp
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   ├── cuadc_tutorial_interfaces/
│   │   ├── action/
│   │   │   └── ScanArea.action
│   │   ├── msg/
│   │   │   ├── Bucket.msg
│   │   │   ├── BucketArray.msg
│   │   │   └── MissionEvent.msg
│   │   ├── srv/
│   │   │   └── PayloadOp.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── cuadc_tutorial_py/
│       ├── cuadc_tutorial_py/
│       │   ├── __init__.py
│       │   ├── common.py
│       │   ├── hello.py
│       │   ├── math_core.py
│       │   ├── mission_dry_run.py
│       │   ├── payload_client.py
│       │   ├── payload_server.py
│       │   ├── perception_guard.py
│       │   ├── scan_action_client.py
│       │   ├── scan_action_server.py
│       │   ├── sim_plant.py
│       │   ├── tf_demo.py
│       │   └── vision_stub.py
│       ├── resource/
│       │   └── cuadc_tutorial_py
│       ├── test/
│       │   └── test_math_core.py
│       ├── package.xml
│       ├── setup.cfg
│       └── setup.py
├── .gitignore
├── LICENSE
├── PROVENANCE.json
├── README.md
└── VALIDATION.md
```

## D.2 Python公共安全与运行封装

**完整文件：`src/cuadc_tutorial_py/cuadc_tutorial_py/common.py`**

```python
"""Shared helpers. This module does not open any device or FCU connection."""
import math
import rclpy
from rclpy.executors import ExternalShutdownException


def require_lab_namespace(node):
    """Guard against accidentally using the root or an aircraft namespace."""
    ns = node.get_namespace()
    if ns != '/lab' and not ns.startswith('/lab/'):
        raise RuntimeError('教学节点只允许 --ros-args -r __ns:=/lab')


def finite_positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} 必须为正有限数')
    return value


def spin_main(node_type, args=None, executor_type=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = node_type()
        if executor_type is None:
            rclpy.spin(node)
        else:
            executor = executor_type(num_threads=4)
            executor.add_node(node)
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Action 工作者先结束，再销毁节点。
        if rclpy.ok():
            rclpy.shutdown()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
```

namespace检查用于避免误运行，不是防恶意操作的安全机制；用户仍然可以通过重映射破坏隔离。工程没有创建MAVROS客户端，才是默认不发飞控动作的重要边界。

## D.3 Python依赖清单

**完整文件：`src/cuadc_tutorial_py/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>cuadc_tutorial_py</name>
  <version>0.1.0</version>
  <description>Safe software-only ROS 2 exercises</description>
  <maintainer email="training@example.com">CUADC Training</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros_py</depend>
  <depend>cuadc_tutorial_interfaces</depend>
  <test_depend>python3-pytest</test_depend>
  <export><build_type>ament_python</build_type></export>
</package>
```

Python包的 `resource/cuadc_tutorial_py` 和模块目录中的 `__init__.py` 需要存在；它们可以是空文件。压缩包已经包含。

## D.4 Bringup构建清单

**完整文件：`src/cuadc_tutorial_bringup/CMakeLists.txt`**

```cmake
cmake_minimum_required(VERSION 3.8)
project(cuadc_tutorial_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
```

**完整文件：`src/cuadc_tutorial_bringup/package.xml`**

```xml
<?xml version="1.0"?>
<package format="3">
  <name>cuadc_tutorial_bringup</name>
  <version>0.1.0</version>
  <description>Launch and configuration for software-only labs</description>
  <maintainer email="training@example.com">CUADC Training</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>ros2launch</exec_depend>
  <depend>launch</depend>
  <depend>launch_ros</depend>
  <depend>ament_index_python</depend>
  <depend>cuadc_tutorial_py</depend>
  <depend>cuadc_tutorial_cpp</depend>

  <export><build_type>ament_cmake</build_type></export>
</package>
```

## D.5 按章节练习还是直接运行整套

逐章学习时，可以只运行一个发送器和一个观察器；到第45章再运行Launch。直接运行整套只能看到结果，不能代替理解每条接口。

配套工程不需要模型文件、D435i或飞控。它不会完整实现比赛成绩判定，也不会通过配置开关突然变成实飞程序。

<a id="appendix-e"></a>
# 附录 E. 资料来源、版本边界与交付验证

## E.1 项目内资料

<a id="p1"></a>
**[P1] 上传源码**：`cuadc_full_mission_node_3_v11(1).cpp`，版本字符串`full-mission-v11-geometric-fine-alignment-2026-09-03`。本教程通过接口、函数体和默认参数说明其实际行为，没有修改该文件。

<a id="p2"></a>
**[P2] 前一份教程**：`CUADC_Cpp_ROS2_MAVROS_从零到完整状态机教程.md`。用于衔接学习主线；ROS机制以本教程给出的官方资料和实际代码为准。

<a id="p3"></a>
**[P3] 项目环境资料**：`基于ardupilot的gazebo仿真环境配置教程.docx`、`安装mavros以及运行状态机代码.docx`。它们说明项目沿用的Ubuntu22.04 / Humble与早期MAVROS流程；旧安装命令和版本假设不自动等价于当前最佳组合。

<a id="p4"></a>
**[P4] 上传规则**：`2026中国大学生飞行器设计创新大赛竞赛规则 ★公开★.pdf`，Ver20260330，多旋翼无人机侦察与救援章节，印刷页36～41。只在第58章等处解释这份已上传规则，不宣称它覆盖之后所有补充公告。

## E.2 官方资料入口

下列引用在正文中按编号链接。查询基线为2026-09-05。ROS官方网页部分受访问保护，本次查阅同时使用官方仓库Humble分支原始文档；MAVROS `ros2`分支文档可能更新，运行时仍须核对安装版本。

| 编号 | 官方资料 | 原始文档备用入口 |
|---|---|---|
| [R01] | [ROS 2 Humble Ubuntu deb安装](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Installation/Ubuntu-Install-Debs.rst) |
| [R01A] | [官方apt源配置原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Installation/_Apt-Repositories.rst) | — |
| [R02] | [ROS 2环境配置](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) | — |
| [R03] | [ROS 2发现机制](https://docs.ros.org/en/humble/Concepts/Basic/About-Discovery.html) | — |
| [R04] | [colcon、工作空间与overlay](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.rst) |
| [R05] | [ROS Node基础](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.rst) |
| [R06] | [ROS Topic命令行](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.rst) |
| [R07] | [C++发布与订阅](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.rst) |
| [R08] | [Python发布与订阅](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) | — |
| [R09] | [Executor机制](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Executors.rst) |
| [R10] | [Managed Node生命周期设计](https://design.ros2.org/articles/node_lifecycle.html) | — |
| [R11] | [节点参数与名字重映射](https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html) | — |
| [R12] | [Odometry正式消息定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/nav_msgs/msg/Odometry.msg) | — |
| [R13] | [自定义msg与srv](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.rst) |
| [R14] | [QoS策略与兼容矩阵](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst) |
| [R15] | [同步与异步服务调用](https://docs.ros.org/en/humble/How-To-Guides/Sync-Vs-Async.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/How-To-Guides/Sync-Vs-Async.rst) |
| [R16] | [rclcpp Humble Client实现及pending清理](https://raw.githubusercontent.com/ros2/rclcpp/humble/rclcpp/include/rclcpp/client.hpp) | — |
| [R17] | [MAVLink命令协议与ACK](https://mavlink.io/en/services/command.html) | — |
| [R18] | [Python Action服务端与客户端](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.rst) |
| [R19] | [Python节点参数](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html) | — |
| [R20] | [大项目Launch组织](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.rst) |
| [R21] | [ROS 2日志机制](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Concepts/Intermediate/About-Logging.rst) |
| [R21A] | [diagnostic_updater官方包索引](https://index.ros.org/p/diagnostic_updater/) | — |
| [R22] | [Callback groups与死锁](https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/How-To-Guides/Using-callback-groups.rst) |
| [R23] | [ROS 2 Clock and Time设计](https://design.ros2.org/articles/clock_and_time.html) | — |
| [R24] | [REP-103单位、坐标与角度约定](https://www.ros.org/reps/rep-0103.html) | [官方仓库原文](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst) |
| [R25] | [REP-105机器人坐标系职责](https://www.ros.org/reps/rep-0105.html) | [官方仓库原文](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst) |
| [R26] | [tf2静态变换教学与命令接口](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.rst) |
| [R26A] | [tf2四元数基础](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html) | — |
| [R27] | [Image正式消息定义](https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/Image.msg) | — |
| [R27A] | [cv_bridge官方包索引](https://index.ros.org/p/cv_bridge/) | — |
| [R27B] | [image_transport官方包索引](https://index.ros.org/p/image_transport/) | — |
| [R28] | [MAVROS官方ROS 2 README](https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/README.md) | — |
| [R28A] | [MAVROS连接URL](https://raw.githubusercontent.com/mavlink/mavros/ros2/docs/connection_urls.md) | — |
| [R28B] | [MAVROS坐标系转换](https://raw.githubusercontent.com/mavlink/mavros/ros2/docs/frames.md) | — |
| [R28C] | [ArduPilot MAVLink路由](https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html) | — |
| [R29] | [ArduPilot GUIDED模式](https://ardupilot.org/copter/docs/ac2_guidedmode.html) | — |
| [R29A] | [GUIDED支持的控制命令](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html) | — |
| [R30] | [MAVLink控制舵机](https://ardupilot.org/dev/docs/mavlink-move-servo.html) | — |
| [R30A] | [飞控输出功能映射](https://ardupilot.org/copter/docs/common-rcoutput-mapping.html) | — |
| [R31] | [请求飞控遥测频率](https://ardupilot.org/dev/docs/mavlink-requesting-data.html) | — |
| [R31B] | [读取与设置飞控参数](https://ardupilot.org/dev/docs/mavlink-get-set-params.html) | — |
| [R32] | [SITL仿真参数与运行环境](https://ardupilot.org/dev/docs/SITL_simulation_parameters.html) | — |
| [R32A] | [Linux上启动SITL](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html) | — |
| [R33] | [ArduPilot Gazebo插件与SITL](https://ardupilot.org/dev/docs/sitl-with-gazebo.html) | — |
| [R33A] | [ArduPilot ROS 2 + Gazebo版本组合](https://ardupilot.org/dev/docs/ros2-gazebo.html) | — |
| [R34] | [RViz2使用指南](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html) | — |
| [R35] | [rosbag2 Humble README](https://raw.githubusercontent.com/ros2/rosbag2/humble/README.md) | — |
| [R35A] | [rosbag2 Humble播放参数源码](https://raw.githubusercontent.com/ros2/rosbag2/humble/ros2bag/ros2bag/verb/play.py) | — |
| [R36] | [记录与回放QoS覆盖](https://docs.ros.org/en/humble/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.rst) |
| [R37] | [切换RMW及daemon注意事项](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/How-To-Guides/Working-with-multiple-RMW-implementations.rst) |
| [R38] | [ROS Python测试](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Intermediate/Testing/Python.rst) |
| [R39] | [组件组合](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Intermediate/Composition.rst) |
| [R40] | [ROS tracing与性能分析](https://docs.ros.org/en/humble/Tutorials/Advanced/ROS2-Tracing-Trace-and-Analyze.html) | — |
| [R41] | [ROS 2通信安全入门](https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Introducing-ros2-security.html) | [官方仓库原文](https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Tutorials/Advanced/Security/Introducing-ros2-security.rst) |

## E.3 本次交付实际检查范围

| 检查项 | 本次结果 |
|---|---|
| Python语法 / AST | 17个`.py`文件通过解析 |
| ROS包清单XML | 4个`package.xml`通过XML解析 |
| YAML配置 | 2个配置文件通过解析 |
| Shell语法 | `bash -n scripts/run_lab.sh`通过 |
| 纯计算单元测试 | 9个pytest测试通过；涵盖旋转、插值、角度边界和稳定门禁 |
| Python可执行入口 | 10个入口对应模块与main存在 |
| 默认控制隔离 | 教学节点检查`/lab`，源码未创建MAVROS飞行客户端；仍不可替代物理与路由隔离 |
| C++ / ROS编译与DDS联调 | **未执行：本生成环境没有ROS 2安装** |
| Gazebo / ArduPilot SITL | **未执行** |
| 实机 / 舵机 / O4验证 | **未执行** |

以上结果来自交付文件的静态检查与纯函数测试，不是把文档示例推定为真实飞行已验证。

没有ROS运行环境的检查不能替代colcon编译、DDS发现、消息生成、Action取消、真实bag读写或SITL联调。第一次在目标机器运行时，应依第45、46、56章完成软件验收，再决定是否进入更高风险测试等级。

## E.4 已知边界

本工作空间使用简单点模型和固定合成目标；不实现真实目标关联、完整捕获时刻位姿转换、飞控模式控制或真实机构反馈。它使用ROS时间戳与单调超时，不能无条件保证倍速回放等价性。Action练习是通信演示，不是侦察路径控制器。模拟服务去重缓存只在进程内存中，重启不会保留历史动作。

前面对话中的O4单线和12m/s提速方案没有合并进v11；本文对其讨论属于架构和验证要求。本文没有执行真实起飞、上锁、降落或舵机指令。

## E.5 学完后的合格标准

面对一个新节点，你能说清职责、输入输出、消息类型、坐标系、时间戳、QoS、参数应用、超时、控制权限和验证证据；面对一个故障，你能最小复现而不是盲目调参；面对一次交付，下一位队员不需要依赖你的电脑或口头记忆。

这就是ROS在完成CUADC任务中最有价值的能力：让算法、硬件、任务与证据真正连接起来。

[R01]: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
[R01A]: https://raw.githubusercontent.com/ros2/ros2_documentation/humble/source/Installation/_Apt-Repositories.rst
[R02]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
[R03]: https://docs.ros.org/en/humble/Concepts/Basic/About-Discovery.html
[R04]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
[R05]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html
[R06]: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html
[R07]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
[R08]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
[R09]: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Executors.html
[R10]: https://design.ros2.org/articles/node_lifecycle.html
[R11]: https://docs.ros.org/en/humble/How-To-Guides/Node-arguments.html
[R12]: https://raw.githubusercontent.com/ros2/common_interfaces/humble/nav_msgs/msg/Odometry.msg
[R13]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html
[R14]: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html
[R15]: https://docs.ros.org/en/humble/How-To-Guides/Sync-Vs-Async.html
[R16]: https://raw.githubusercontent.com/ros2/rclcpp/humble/rclcpp/include/rclcpp/client.hpp
[R17]: https://mavlink.io/en/services/command.html
[R18]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html
[R19]: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html
[R20]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html
[R21]: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Logging.html
[R21A]: https://index.ros.org/p/diagnostic_updater/
[R22]: https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html
[R23]: https://design.ros2.org/articles/clock_and_time.html
[R24]: https://www.ros.org/reps/rep-0103.html
[R25]: https://www.ros.org/reps/rep-0105.html
[R26]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
[R26A]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html
[R27]: https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/Image.msg
[R27A]: https://index.ros.org/p/cv_bridge/
[R27B]: https://index.ros.org/p/image_transport/
[R28]: https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/README.md
[R28A]: https://raw.githubusercontent.com/mavlink/mavros/ros2/docs/connection_urls.md
[R28B]: https://raw.githubusercontent.com/mavlink/mavros/ros2/docs/frames.md
[R28C]: https://ardupilot.org/dev/docs/mavlink-routing-in-ardupilot.html
[R29]: https://ardupilot.org/copter/docs/ac2_guidedmode.html
[R29A]: https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
[R30]: https://ardupilot.org/dev/docs/mavlink-move-servo.html
[R30A]: https://ardupilot.org/copter/docs/common-rcoutput-mapping.html
[R31]: https://ardupilot.org/dev/docs/mavlink-requesting-data.html
[R31B]: https://ardupilot.org/dev/docs/mavlink-get-set-params.html
[R32]: https://ardupilot.org/dev/docs/SITL_simulation_parameters.html
[R32A]: https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
[R33]: https://ardupilot.org/dev/docs/sitl-with-gazebo.html
[R33A]: https://ardupilot.org/dev/docs/ros2-gazebo.html
[R34]: https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html
[R35]: https://raw.githubusercontent.com/ros2/rosbag2/humble/README.md
[R35A]: https://raw.githubusercontent.com/ros2/rosbag2/humble/ros2bag/ros2bag/verb/play.py
[R36]: https://docs.ros.org/en/humble/How-To-Guides/Overriding-QoS-Policies-For-Recording-And-Playback.html
[R37]: https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html
[R38]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Python.html
[R39]: https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html
[R40]: https://docs.ros.org/en/humble/Tutorials/Advanced/ROS2-Tracing-Trace-and-Analyze.html
[R41]: https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Introducing-ros2-security.html

[P1]: #p1
[P2]: #p2
[P3]: #p3
[P4]: #p4
