# 接入现有状态机：先做影子决策

## 已核对的接口

以仓库 `cuadc_full_mission_node_3_v2_public .cpp` 为对照：

| 原接口 | 类型 | 草案接入用途 |
|---|---|---|
| `/perception/drop_buckets_body` | `geometry_msgs/msg/PoseArray` | 上游选择并锁定桶后取得机体坐标目标 |
| `/mavros/local_position/odom` | `nav_msgs/msg/Odometry` | 位置、速度及姿态来源 |
| `/mavros/state` | `mavros_msgs/msg/State` | 连接、解锁、模式状态 |
| `/mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` | 原状态机的单一位置指令输出 |
| `/mavros/cmd/command` | `mavros_msgs/srv/CommandLong` | 原投放机构执行路径 |
| `/drop_controller/release` | `std_srvs/srv/Trigger` | `sim/cuadc_sim` 中的虚拟投放服务 |

本目录没有订阅以上话题后直接控制实机的程序。原因是公开代码未给出正式标定、同步、目标关联和执行器参数；仅凭一个 `PoseArray` 无法可靠恢复这些语义。

公开 `PoseArray` 将 `orientation.x` 用作尺寸提示、`orientation.y` 用作置信度，这里的 orientation **不是四元数**。目标序号也不应直接当作跨帧稳定 ID；应沿用原任务追踪器的 ID。`basket_detect_seg.py` 目前未发布这个话题，连接视觉到状态机的桥接仍需你在现有项目中完成。

## 可立即运行的日志示例

```bash
python integration/replay_telemetry.py examples/telemetry.jsonl
python integration/replay_telemetry.py examples/telemetry.jsonl --model outputs/q_policy.json
```

第一个命令使用弹道基线，第一条快照应输出候选释放 `true`，第二条因单次候选已锁存而输出 `false`。这里只打印 JSON。

## 快照数据契约

完整例子在 `examples/telemetry.jsonl`。所有时间戳必须使用**同一个时钟域**，不能将 ROS 仿真时间、设备硬件时间和 Unix 时间直接相减。

| 字段 | 单位/含义 |
|---|---|
| `mission_id / target_id / payload_id` | 原任务层给出的稳定身份，不能为空 |
| `now_s` | 当前决策时刻 |
| `vision_stamp_s` | 曝光时刻，不能用回调接收时刻冒充 |
| `odom_stamp_s` | 速度所对应的里程计时刻 |
| `target_body_flu_m` | 曝光时刻桶口在机体 FLU 中的位置 |
| `release_port_body_flu_m` | 当前载荷投放口在同一机体坐标的标定位置 |
| `body_to_enu_xyzw_at_exposure` | 曝光时刻机体 FLU 到局部 ENU 的单位四元数 |
| `velocity_enu_m_s` | 当前飞机 ENU 速度 `[vx,vy,vz]` |
| `last_command_enu_m_s` | 上一次水平指令 `[vx,vy]`，由原程序提供 |
| `body_frame / velocity_frame` | 必须明确为 `FLU` / `ENU` |
| `confidence / target_valid` | 视觉质量与目标有效布尔值 |
| `elapsed_s / stable_s` | 进入本次投放阶段的时间、连续满足门控时间 |
| `connected / armed / guided / pilot_override` | 来自原状态机/飞控，必须为布尔值 |
| `stage` | `ALIGN` 或 `RELEASE` 才处理；只有 `RELEASE` 可产生释放候选 |

`stable_s` 应在任一稳定条件失效时清零，不能直接以进入阶段的时长代替。快照生产端应进行曝光时刻姿态插值；本目录的纯函数不负责多流插值。观测过期、时间倒退、坐标不明、非有限值和目标更换都拒绝释放候选。示例还限制里程计年龄 0.2 s、竖直速度 0.20 m/s；这些是接口演示值，不能替代正式门限。

现有 RealSense 光学系通常与机体 FLU 不同。应使用你已标定的 `p_body = R_camera_to_body × p_camera + t_camera_to_body` 转换，再减投放口位置，最后用曝光时刻姿态旋转到 ENU。不要把视觉脚本中的单位矩阵外参当作正式标定。

`nav_msgs/Odometry` 的速度坐标需核对实际 `child_frame_id` 与发布配置，不能因为 pose 是 ENU 就认为 twist 也是 ENU。公开代码只使用水平速度的模长，难以暴露 x/y 轴混用问题；学习水平修正时必须明确这一点。

## 可选 ROS 2 影子节点

在已有 ROS 2 Humble 环境中：

```bash
source /opt/ros/humble/setup.bash
python3 integration/ros2_shadow_node.py --ros-args -p model_path:=/absolute/path/cuadc_drop_rl/outputs/q_policy.json
```

由原状态机输出 `/rl_drop/snapshot`，消息类型 `std_msgs/msg/String`，内容为上述 JSON。影子节点只发布 `/rl_drop/shadow_decision`（同样为 String）。这两个话题是**本草案新增的软件协议**，不是声称原仓库已存在。节点在回调时使用自己的 ROS 时钟填写 `now_s`，仿真时所有节点需一致启用 `use_sim_time`。

该封装未在当前 Windows 环境实际运行 ROS 2；纯 Python 快照转换、门控及去重已通过测试。节点不自动找到桶、不管理多载荷、不连接舵机，也不修改飞控模式。

## 将候选接入原控制流

建议在 `update_align()` 内读取小幅水平修正候选，在 `update_release()` 内把释放候选与现有位置/速度/稳定条件取逻辑与。需要同时注意：

1. 原公开代码发布**位置**目标，本草案模拟**速度**响应。二者不是即插即用。应选择“将残差积分成限幅位置偏移，继续由原节点唯一发布”或经过验证后整体迁移速度接口。位置偏移积分方案：`offset_next = clip(offset + residual_velocity × dt, offset_limit)`，还需高度保持、命令过期处理与积分清零。这里没有声称该桥接已完成。
2. Q-learning 仅建议释放时机时，可以先完全保留原位置控制；但训练的水平运动模型将不同，必须用其轨迹重放或仿真重新评估。
3. `ShadowSupervisor` 每个实例锁定一个任务/桶/载荷；目标变化不会自动清除锁存。原状态机完成本载荷流程后，显式创建新实例。正式程序还要跨进程重启保存已释放状态；当前影子示例没有持久化锁存。
4. 服务 ACK 表示命令受理不等于瓶子已脱离；保留原程序的机构流程与超时处理。没有新增释放检测传感器，因此无法凭本模型保证脱离确认。
5. 仿真评分不能读取飞机位置就直接当作落点。若接入 Gazebo，需要将释放时刻状态、机构延迟和瓶体运动结果交给新的裁判计算，并从策略话题隔离裁判真值。

文档只给软件整合路径。原项目无需在此阶段改动；拿到现有标定和明确的数据时钟后再实现正式适配。
