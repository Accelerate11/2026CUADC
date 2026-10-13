# 图像、飞控与控制时间对齐（2026-10-05）

仅 full_mission/mission.yaml 和 drop_only/mission.yaml 为主节点、视觉节点
同时启用 `measurement_time_sync: true`。其余入口默认保留旧行为。
没有修改速度限制、投放高度、投放等待时长、路线航向设定或舵机参数。

## 时间来源

- 原始 RGB、depth 帧开启 librealsense global_time。读取各自
  sensor_timestamp（曝光中点）与 frame_timestamp（微秒计数），将二者
  差值加到 get_timestamp() 的主机时间上；处理 32 位计数回绕。
- 先读取原始帧时间，再做 depth alignment 和 YOLO，不使用处理后时间。
  彩色曝光中点是视觉消息 header.stamp；深度独立核查与它的偏差。
- 不支持元数据、不是 global_time、过旧、未来、重复/回退、主机时钟跳变
  或 RGB/depth 曝光间隔超过 25 ms 时，拒绝该帧。启动有 10 秒适配窗口，
  之后沿用现有连续帧失败处置。不以取帧时刻静默替代。
- 彩色 actual_exposure 在本机测试中不可用，记录 None，不伪造曝光时长。
- 飞控使用 MAVROS MAVLINK TIMESYNC。读取实际 convergence_window、RTT
  与偏差阈值，观察足够数量的有效同步交换后才使用测量历史。默认窗口
  已由任务专用配置改为 100、频率 20 Hz，加主节点额外 11 次确认理论约
  5.55 秒；本机实测回调观察者 5.81 秒就绪。这段等待发生在 WAIT_GUIDED
  之前，而不是遥控器切换之后。链路异常仍拒绝伪造就绪，15 秒不是强制放行期限。
  `config/mavros_time_sync.yaml` 由 `mission_mavros.launch.py` 在系统 APM
  默认配置之后加载；full_mission/drop_only 启动脚本使用此入口。未修改
  系统 MAVROS 配置，其余独立标定/测试入口仍使用系统默认值。
- 位置历史按 odom.header.stamp，姿态历史按 imu/data.header.stamp，分别
  插值到图像曝光时刻，避免直接使用 odom 内缓存的上一条 IMU 姿态。
- 接收时间仍用于通信健康监测。它不能替代测量时间。

## 对准

保存检测帧的历史位姿；最终对准时，先把旧帧桶口坐标投影到世界坐标，
再转换到当前机体坐标求剩余误差。投放误差判定使用相同的修正。
视觉坐标旋转使用对应时刻实测 yaw；任务路线与指令航向仍由路线 YAML 决定。
已有准星/投放点几何定义保持不变，本次不重新标定相机或舵机杆臂。
超过 0.5 秒的检测不能触发“误差满足”的提前投放；原有超时强制投放策略保留。

## 启动顺序

同步模式：WAIT_FCU（准备、同步、静止、视觉、原有舵机初始化）→ PRESTREAM
（原有预发送）→ WAIT_GUIDED → WAIT_ARM → TAKEOFF。

只有准备完成才输出 READY_FOR_GUIDED。切入 GUIDED 时刷新起飞/返航原点，
不再重新等待同步或静止，也不重复预发送。等待期间数据失效则撤销 READY。
如果准备完成时已经处于 GUIDED，必须先切回其他模式再切入，避免延迟自动起飞。
解锁/起飞服务仍需飞控正常响应，不承诺物理上的零延迟。

## 诊断与验证

日志 CAMERA_TIME：时间源、RGB/depth 获取年龄、曝光偏差、曝光元数据、发布年龄。
FCU_TIME：同步有效计数、窗口、RTT、估计残差、就绪状态。
VISION_TIME：曝光时间、使用年龄、等待导航插值的时间。

离线：在工作区 source ROS 环境后运行
`ctest --test-dir build_v16/cuadc_full_mission_20260803 --output-on-failure`。
测试涵盖计数回绕、坏时间、时钟跳变、RGB-D 不匹配、独立姿态插值、旧帧
重复使用、偏航变化、过期投放判定以及 GUIDED 前完成准备/切换后不再等待。

实机诊断：`python3 tools/check_fcu_clock.py --callbacks`（先 source ROS）。
仅单独启动一个 MAVROS、请求姿态和位置遥测 30 Hz；回调测试取消任务定时器，
不执行任务、解锁、起飞或舵机指令。串口已占用或飞行锁被持有时拒绝运行。
结束清理自建进程。此次静态实测：同步 RTT 中位数约 3.3 ms，位置和姿态
各约 30 Hz；收敛后消息年龄中位数约 1.6 ms。不能据此宣称硬件采样误差
精确到 1.6 ms，也不能代替动态飞行验证。

相机实际分辨率、约 100 ms 处理间隔测试：稳定帧获取年龄 27–60 ms，
RGB/depth 曝光中点差约 0.25–2.24 ms；换算耗时中位数约 0.095 ms。
初始化首帧约 140 ms 的 RGB/depth 错配被拒绝。

完整视觉节点单独运行（真实 YOLO、关闭预览和录像）也通过：曝光到发布
约 91–124 ms（所列诊断样本），偶发 RGB/depth 差一帧（约 33.7 ms）会
丢弃该帧，随后正常恢复；不是人为把这类帧当成同时采集。
