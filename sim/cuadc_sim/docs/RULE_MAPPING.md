# 比赛规则到仿真的映射

本文记录“多旋翼无人机侦察与救援”规则在当前 Gazebo 环境中的实现方式。

## 场地坐标

当前 Gazebo 使用 ENU 风格坐标：

```text
x: 比赛前进方向
y: 场地横向
z: 高度
```

H 点中心设为：

```text
(0, 0, 0)
```

## 起降区

规则：

```text
起降区长 33m，宽 8m
起降点直径 80cm
起降点圆心距离准备区 3m
起飞线在起降点前 2m
```

仿真实现：

```text
起降区 x 范围: -3m 到 30m
起降区 y 范围: -4m 到 4m
H 点: (0, 0)
起飞线: x = 2m
H 点直径: 0.8m
```

相关文件：

```text
models/cuadc_field/model.sdf
models/takeoff_pad/model.sdf
config/scene.yaml
```

## 投放区

规则：

```text
起降点前 30m 为投放区
尺寸 8m 长、5m 宽
放置 3 个高度 30cm 白色圆筒
1 号筒直径 15cm
2 号筒直径 20cm
3 号筒直径 25cm
```

结合规则图与前后距离关系，仿真实现为：

```text
投放区中心: (30, 0)
x 方向尺寸: 5m
y 方向尺寸: 8m
```

圆筒：

```text
drop_1: 直径 0.15m，A 区 500 分
drop_2: 直径 0.20m，A 区 300 分
drop_3: 直径 0.25m，A 区 100 分
B 区: 距桶心 0.5m 内，50 分
```

投放桶位置由 `scripts/generate_scene.py` 随机生成。

## 侦察区

规则：

```text
投放区前方 20m 为灾情侦察区
距离起降点 55m
尺寸 8m 长、5m 宽
放置 5 个高 15cm、直径 20cm 白色圆筒
其中 3 个筒内放置 12cm x 12cm 危险化学品标识
```

仿真实现：

```text
侦察区中心: (55, 0)
x 方向尺寸: 5m
y 方向尺寸: 8m
侦察桶: 5 个随机位置
标识: 随机选择 3 个桶，随机选择 3 类标识
```

标识纹理位于：

```text
models/hazard_marker/materials/textures
```

## 飞行器

规则：

```text
仅限电动多旋翼
电池最高 26V / 6S
对角线旋翼轴距不大于 550mm
必须全自动
必须有螺旋桨防护罩和安全开关
```

仿真实现：

```text
使用 ardupilot_gazebo 的 Iris 模型派生
模型名: iris_d435i
机体层: iris_d435i_airframe
加装下视 RGB-D 相机
不修改动力、质量、惯量、旋翼、ArduPilotPlugin 控制逻辑
```

注意：仿真 Iris 主要用于软件流程验证，不等同于最终参赛机物理指标审核。

## 虚拟投放

规则中实际投放载荷为：

```text
2 瓶市售带标签未开封 550ml 饮用水
```

仿真当前不模拟水瓶下落，而是：

```text
飞到桶上方
调用 /drop_controller/release
根据飞机 xy 与目标桶 xy 误差判定 A / B / 无效
```

第一版判定：

```text
误差 <= 桶半径: A 区
误差 <= 0.5m: B 区
否则: 无效
```

后续可扩展为：

- 加入高度、速度、风扰
- 加入简化弹道
- 加入投放延迟
- 加入真实舵机接口

## 侦察识别

当前 world 已贴真实附件 11 标识图，但还没有实现视觉识别算法。

预留统一接口：

```text
/perception/color/image
/perception/depth/image
/perception/color/camera_info
```

未来识别节点建议输出：

```text
/perception/drop_bucket_detection
/perception/hazard_detection
```

第一阶段可用 `generated_scene.yaml` 真值验证飞控流程；第二阶段替换为视觉识别结果。
