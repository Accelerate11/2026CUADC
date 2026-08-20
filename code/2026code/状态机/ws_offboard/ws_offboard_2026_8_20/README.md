# CUADC 2026 实机飞行工作区

这是面向机载 Ubuntu 22.04 / ROS 2 Humble 的纯实机工作区。可将整个
`cuadc_flight_ws` 目录原样复制到 U 盘，再复制到机载电脑。仓库不包含 SITL、
Gazebo、回放、仿真启动脚本或仿真报告，也不声明任何仿真依赖。

## 先看安全边界

- 随包五层配置全部是 **candidate**，不是实飞批准值。
- 两个 aircraft profile 及对应 calibration profile 均保持
  `flight_enable: false`、`calibration_approved: false`，不能直接实飞。
- V6X 与 V5Nano 中的串口 by-id、波特率只是历史候选值；D435i 序列号、相机
  外参、投放偏移、CH/PWM/持续时间也只是候选值。两机配置可能相互冲突，必须
  在所选飞机上逐项现场测量、台架验证并签字，禁止照抄另一架飞机的值。
- 根脚本不会设置或伪造实飞授权、批准 SHA 或子进程运行门。
- 唯一允许的启动文件是 `cuadc_bringup/flight.launch.py`。不要单独启动任务、
  投放或视觉节点来绕过同步预检。

## 从 U 盘导入机载电脑

把 `cuadc_flight_ws_usb_20260820_final.zip` 和同名 `.sha256` 文件原样复制到 U 盘；
不要先在 Windows 或 FAT 文件系统中解压，否则 Linux 可执行权限可能丢失。
在机载 Ubuntu 中进入 U 盘挂载目录后执行：

```bash
sha256sum -c cuadc_flight_ws_usb_20260820_final.zip.sha256
unzip -q cuadc_flight_ws_usb_20260820_final.zip -d "$HOME"
cd "$HOME/cuadc_flight_ws"
sha256sum -c SHA256SUMS
```

三个命令必须全部成功。解压目录固定为 `~/cuadc_flight_ws`；压缩包不携带其他
顶层目录，也不携带任何已有 `build/`、`install/` 或 `log/`。

## 实机流程

在机载电脑上把 U 盘中的完整目录复制到用户目录；不要在 Windows 中拆散 ROS
package。下面假设工作区路径为 `~/cuadc_flight_ws`。

### 1. 构建

系统需预先安装 ROS 2 Humble、MAVROS、RealSense ROS 2 驱动、CUDA 兼容版
PyTorch/Ultralytics、PyYAML 与模型运行依赖。`dependencies.repos` 故意不拉取
第三方源码；系统依赖由机载镜像和 `rosdep` 管理。

```bash
cd ~/cuadc_flight_ws
./scripts/build_onboard.sh
```

脚本先执行 `rosdep check`（若 rosdep 可用），再构建本工作区；它不会联网安装
软件，也不会清理已有数据。

### 2. 创建外部五层配置副本

```bash
./scripts/prepare_flight_profile.sh \
  --destination "$HOME/cuadc_flight_profiles/2026_field/config"
```

目标目录必须位于随包工作区之外，且必须是尚不存在、当前用户可创建的位置；脚本
拒绝覆盖任何已有目录。

复制的五层为：

1. `aircraft`
2. `calibration`
3. `mission`
4. `field`
5. `perception`

该副本仍处于禁飞状态。现场至少核验：

- 选定 V6X 或 V5Nano 的唯一 `/dev/serial/by-id/...`、波特率与目标系统；
- 实际 D435i 型号和精确序列号；
- 相机到机体旋转矩阵、平移、两投放口偏移；
- 两路舵机通道、收拢 PWM、释放 PWM、释放持续时间及机械收拢反馈；
- 场地原点、桶区与侦察区几何、速度和高度；
- aircraft/calibration 两层相同、可追溯的 `approval_reference`。

只有测量、拆桨台架、机构测试和签字均完成后，才可在外部副本中同时设置
`flight_enable: true` 与 `calibration_approved: true`。随包配置不得修改或签字。

### 3. 查看设备

```bash
./scripts/check_devices.sh
```

输出所有 serial by-id 链接及 RealSense 型号/序列号。必须与所选外部 profile
逐字比对。

### 4. 只读预检

V6X 示例：

```bash
./scripts/preflight.sh \
  --aircraft v6x_3E0032 \
  --config-root "$HOME/cuadc_flight_profiles/2026_field/config"
```

V5Nano 将 `--aircraft` 改为 `v5nano_410035`。只读预检通过唯一 launch 的
`preflight_only:=true` 执行，检查 FCU、D435i、模型 SHA、CUDA 黑帧推理、ROS
图冲突与日志空间；不会启动飞控链路、视觉、任务、投放或安全进程，也不会发放
运行门。记录输出的 `CONFIG_BUNDLE_SHA256`，与现场记录一起签字。

### 5. 授权并实飞

仅在同一份外部配置未再修改、签字预检完成后，由授权操作员手工设置：

```bash
export CUADC_FLIGHT_AUTHORIZED=YES_I_COMPLETED_PREFLIGHT
export CUADC_APPROVED_CONFIG_BUNDLE_SHA256=<签字记录中的64位小写SHA-256>

./scripts/run_flight.sh \
  --aircraft v6x_3E0032 \
  --config-root "$HOME/cuadc_flight_profiles/2026_field/config"
```

`run_flight.sh` 不生成上述值，只把请求交给唯一 `flight.launch.py`。launch 会重新
完成同步预检、重新计算五层 bundle SHA，并仅在授权短语、批准 SHA、profile
批准状态和硬件检查全部一致后，向受限子进程注入内部运行门。

## 单一启动后的六个实机进程

1. MAVROS
2. RealSense D435i 驱动
3. 桶分割感知节点
4. 全流程任务状态机 `mission_node`
5. 投放执行器 `payload_node`
6. 独立安全监控 `safety_monitor`

危险物侦察阶段固定为纯航点：3 条航带、6 个端点，不订阅、不启动危险物视觉
节点，也不依赖危险物模型或识别结果。桶视觉只服务投放阶段。

## 紧急人工入口

MAVROS 已在线时：

```bash
./scripts/emergency_land.sh
./scripts/disarm_after_landed.sh
```

两者均要求交互终端输入完整确认短语。`emergency_land.sh` 只调用 MAVROS LAND
服务；`disarm_after_landed.sh` 还要求新鲜的 `landed_state=1`，之后才调用
`arming=false`。它们不扫描或终止其他进程，也不替代遥控器接管和现场安全程序。

## 常用环境变量

- `CUADC_AIRCRAFT`：`v6x_3E0032` 或 `v5nano_410035`
- `CUADC_CONFIG_ROOT`：外部五层配置绝对路径
- `CUADC_MISSION`：默认 `competition_2026`
- `CUADC_ROS_SERVICE_TIMEOUT_S`：人工服务超时秒数，默认 15

实飞授权环境只允许操作员在签字后手工设置；不要写入 `.bashrc`、脚本或 profile。
