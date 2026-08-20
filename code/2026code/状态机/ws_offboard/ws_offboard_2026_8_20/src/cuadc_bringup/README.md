# cuadc_bringup

该包只负责实机启动，不包含 SITL、Gazebo、rosbag 或回放入口。唯一 launch：

```bash
ros2 launch cuadc_bringup flight.launch.py \
  aircraft:=v6x_3E0032 \
  mission:=competition_2026 \
  preflight_only:=true
```

配置固定由五层组成：`aircraft`、`calibration`、`mission`、`field`、
`perception`。`profile_loader.py` 对解析后的五层内容生成路径无关、键排序的
`cuadc.config-bundle.sha256.v1` 规范 SHA-256。

随包 aircraft/calibration 文件是候选值，不是签字值；两架飞机均明确设置
`flight_enable: false`、`calibration_approved: false`。实飞时应将完整 `config`
目录复制到受控目录，现场核验设备、外参和投放机构后修改副本，并通过
`config_root:=/absolute/approved/config` 使用副本。禁止直接把随包候选配置当作
已批准标定。完成对应实机/台架检查后，副本中的 FCU、相机序列号、相机外参、
双投放口外参和投放机构状态还必须逐项改为 `verified_on_aircraft`；仅翻转两个
布尔开关仍会被预检拒绝。

`preflight_only:=false` 还要求：

```bash
export CUADC_FLIGHT_AUTHORIZED=YES_I_COMPLETED_PREFLIGHT
export CUADC_APPROVED_CONFIG_BUNDLE_SHA256=<预检打印的实际 bundle SHA-256>
```

授权短语、批准 SHA、profile 内的 flight/calibration approval 三者缺一不可。
只有实飞预检全部通过后，launch 才会向 mission/payload 子进程注入运行门；
只读预检不会注入运行门，也不会启动任何飞控、视觉或任务进程。

预检还会核对精确 FCU by-id/baud、枚举指定 D435i，并独占启动一次
640x480x30 color+depth、取帧后立即 stop，以拒绝相机被占用或无数据；随后
校验模型/sidecar SHA、执行 CUDA 黑帧分割推理，并检查 ROS graph 冲突。
正式 launch 只启动 mavros、RealSense、perception、mission、payload、safety
六个实机进程，MAVROS 明确 deny `sim_state`。危险物阶段固定为纯航点
3 航带/6 端点；不接入危险物识别节点，safety 仅按任务状态留存取证图像。
