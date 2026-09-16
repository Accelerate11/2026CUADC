# ArduPilot 飞行日志（Log）基础阅读教程

> 适合：第一次使用 Mission Planner / Log Browser 看日志的人  
> 目标：能看懂最常用的曲线，并能初步判断振动、姿态跟踪、电机输出、PID 和 EKF 是否有明显问题。

---

# 1. Log 是什么

飞控运行时会不断记录数据，日志就相当于无人机的“黑匣子”。

里面会记录：

- 姿态
- 角速度
- PID
- 电机输出
- IMU 振动
- GPS
- 气压计
- 磁罗盘
- EKF
- 电池
- 报错和飞行事件

所以飞机出现问题后，不要只凭“看起来很抖”判断，而是应该：

```text
现象
 ↓
打开 Log
 ↓
找到对应曲线
 ↓
看谁先变化
 ↓
判断原因
```

---

# 2. 怎么打开日志

Mission Planner 中一般可以：

```text
DataFlash Logs
  ↓
Review a Log
  ↓
选择 .BIN 或 .log
```

打开 Log Browser 后，右边会看到很多项目，例如：

```text
ATT
RATE
PIDR
PIDP
PIDY
RCOU
VIBE
GPS
BARO
MAG
XKF
BAT
RCIN
CTUN
```

初学阶段不用全部学。

优先学会：

```text
VIBE
RCOU
RATE
ATT
PIDR / PIDP / PIDY
BAT
GPS
ERR / MSG / Events
```

---

# 3. 看 Log 的三个基本原则

## 3.1 一次不要画太多曲线

错误：

```text
一次勾十几二十条
```

正确：

```text
一次只看 2～6 条
```

例如：

```text
RATE.RDes
RATE.R
```

或者：

```text
RCOU.C1
RCOU.C2
RCOU.C3
RCOU.C4
```

---

## 3.2 最重要的是“目标”和“实际”

控制系统最重要的问题就是：

```text
我想让飞机怎么动？
实际飞机怎么动？
```

例如 Roll：

```text
RATE.RDes = 目标 Roll 角速度
RATE.R    = 实际 Roll 角速度
```

Pitch：

```text
RATE.PDes
RATE.P
```

Yaw：

```text
RATE.YDes
RATE.Y
```

最基本判断：

```text
目标 ≈ 实际
```

---

## 3.3 先看机械，再看 PID

建议顺序：

```text
VIBE
 ↓
RCOU
 ↓
RATE
 ↓
ATT
 ↓
PID
 ↓
EKF
```

如果机械振动已经很大，PID 曲线也会被污染。

---

# 4. X、Y、Z 是什么

机体系可以先简单理解为：

```text
X：前后
Y：左右
Z：上下
```

所以：

```text
VibeX
VibeY
VibeZ
```

表示飞控在三个方向测到的振动。

注意：

```text
VibeX 很大
```

不等于：

```text
前面的电机坏了
```

因为振动会通过机架传播。

---

# 5. 第一项：VIBE

右边找到：

```text
VIBE
 ├─ VibeX
 ├─ VibeY
 ├─ VibeZ
 └─ Clip
```

---

## 5.1 VibeX / Y / Z 怎么看

可以先粗略这样理解：

```text
< 10 m/s²       很好
10～15 m/s²     比较健康
15～30 m/s²     偏高
> 30 m/s²       需要排查
> 60 m/s²       很严重
```

不要只看 Max。

要看是否持续。

例如：

```text
正常：

5 ─────6─────5─────

异常：

10 ─70────75────80────20
```

后一种很像某个转速段发生了结构共振。

---

## 5.2 Clip 是什么

`Clip` 可以理解成：

```text
IMU 被震到超出加速度计量程的累计次数
```

理想：

```text
Clip = 0
```

如果看到：

```text
0
1
5
30
100
```

说明已经发生 clipping。

这时候优先处理机械结构，而不是继续精调 PID。

---

## 5.3 多颗 IMU 怎么看

可能会看到：

```text
VIBE[0]
VIBE[1]
VIBE[2]
```

如果两颗 IMU 在同一个时间同时变高：

```text
IMU0 ↑
IMU1 ↑
```

通常说明这是真实整机振动。

---

# 6. 第二项：RCOU

找到：

```text
RCOU
 ├─ C1
 ├─ C2
 ├─ C3
 ├─ C4
 ...
```

它表示飞控给各个输出通道的命令。

四旋翼通常先看：

```text
C1
C2
C3
C4
```

---

## 6.1 正常情况

例如：

```text
C1 1300
C2 1320
C3 1290
C4 1310
```

说明四个电机总体比较接近。

---

## 6.2 某个电机长期明显更高

例如：

```text
C1 1250
C2 1280
C3 1500
C4 1260
```

可能检查：

- 重心偏移
- 电机推力不一致
- 桨效率不同
- 电调差异
- 电机安装角
- 姿态控制长期纠偏

---

## 6.3 电机触底 / 触顶

如果某一路长期接近最低允许输出：

```text
不能再降了
```

如果长期接近最大输出：

```text
不能再升了
```

这两种都说明：

```text
控制余量不足
```

这时继续加 PID 增益不一定有意义。

---

# 7. 第三项：RATE

RATE 是判断姿态内环最重要的数据之一。

里面常见：

```text
RDes
R
PDes
P
YDes
Y
ROut
POut
YOut
```

其中：

```text
R = Roll
P = Pitch
Y = Yaw
```

---

# 8. Roll RATE 怎么看

只画：

```text
RATE.RDes
RATE.R
```

其中：

```text
RDes = 目标横滚角速度
R    = 实际横滚角速度
```

好的情况：

```text
目标： ───╭────╮────
实际： ───╭────╮────
```

两条尽量接近。

---

## 8.1 实际跟不上

```text
目标：
      ┌──────
──────┘

实际：
         ╭────
─────────╯
```

可能和：

- P 偏低
- FF 偏低
- 动力响应慢
- 飞机惯量大

有关。

---

## 8.2 超调

```text
目标：
      ┌────────
──────┘

实际：
       /\
      /  \____
─────/
```

可能和：

- P 偏激进
- 阻尼不足
- D 不合适
- 结构弹性

有关。

---

## 8.3 振铃

```text
目标：────────────

实际：──/\/\/\────
```

目标已经稳定，实际还在来回摆。

这是明显需要关注的现象。

---

## 8.4 目标不动，实际自己乱跳

```text
RDes：────────────

R：   ─/\─/\/\──/\─
```

优先检查：

```text
机械振动
Gyro 噪声
Notch Filter
电机/桨
机架共振
```

不要第一反应就是改 P。

---

# 9. Pitch RATE

画：

```text
RATE.PDes
RATE.P
```

看法和 Roll 一样。

如果：

```text
Roll 很好
Pitch 很差
```

可以重点怀疑：

- Pitch 方向惯量更大
- 前后重心偏
- 前后结构刚度差
- Pitch PID 不合适
- 某方向存在共振

---

# 10. Yaw RATE

画：

```text
RATE.YDes
RATE.Y
```

如果：

```text
YDes = -300 deg/s
Y    = -100 deg/s
```

说明 Yaw 明显追不上。

不要直接判断：

```text
Yaw P 太小
```

还要看：

```text
RCOU
PIDY.Flags
电机余量
MOT_YAW_HEADROOM
```

---

# 11. 第四项：ATT

ATT 是姿态角。

常见：

```text
DesRoll
Roll
DesPitch
Pitch
DesYaw
Yaw
```

例如：

```text
ATT.DesRoll
ATT.Roll
```

表示：

```text
目标 Roll 角
实际 Roll 角
```

---

# 12. ATT 和 RATE 有什么区别

简单记：

```text
ATT  = 倾斜了多少度
RATE = 转得多快
```

例如：

```text
Roll = 20°
```

表示飞机当前已经横滚 20°。

而：

```text
RATE.R = 100°/s
```

表示飞机正在以每秒 100° 的速度横滚。

---

# 13. 为什么调 PID 先看 RATE

ArduPilot 姿态控制可以先简单理解成：

```text
目标姿态
   ↓
姿态外环
   ↓
目标角速度
   ↓
RATE PID
   ↓
Motor Mixer
   ↓
电机
```

所以：

```text
ATT.DesRoll
   ↓
RATE.RDes
   ↓
PID
   ↓
RATE.R
```

因此评价内环 PID 最直接的是：

```text
RATE.Des
vs
RATE.Actual
```

---

# 14. 第五项：PIDR / PIDP / PIDY

分别代表：

```text
PIDR = Roll
PIDP = Pitch
PIDY = Yaw
```

里面常见：

```text
Tar
Act
Err
P
I
D
FF
Dmod
SRate
Flags
```

---

# 15. PID 里面每个量是什么

## Tar

```text
Target
```

目标。

## Act

```text
Actual
```

实际。

## Err

```text
Err = Tar - Act
```

目标和实际之间的误差。

---

## P

比例项。

可以先理解成：

```text
现在差多少
立即纠正多少
```

P 太低可能：

```text
反应慢
```

P 太高可能：

```text
容易振荡
```

---

## I

积分项。

用于修正长期的小偏差，例如：

- 重心偏
- 电机推力差异
- 持续外力

如果 I 长时间很大，要检查为什么飞机一直需要补偿。

---

## D

微分项。

主要作用是增加阻尼、抑制快速变化。

但 D 对高频噪声非常敏感。

如果：

```text
机械振动大
 ↓
Gyro 抖
 ↓
D 抖
 ↓
电机也抖
```

所以机械问题没解决之前，不适合精调 D。

---

## FF

Feed Forward，前馈。

简单理解：

```text
还没出现很大误差
控制器就提前给动作
```

它用于提高跟踪性。

FF 不是用来治机械振动的。

---

## Flags

如果 PID 经常显示 LIMIT 类状态，说明：

```text
控制器想继续输出
但是电机 / Mixer 已经没有余量
```

这叫：

```text
饱和
```

---

# 16. 第六项：BAT

常看：

```text
Volt
Curr
CurrTot
```

## Volt

电池电压。

重点看：

```text
起飞前
悬停
大油门
```

如果一加油门电压掉很多，可能有：

- 电池内阻大
- 电池老化
- 电流太大
- 线材压降

## Curr

电流。

可以和：

```text
RCOU
VIBE
```

一起看。

有时候能发现：

```text
油门升高
 ↓
振动突然暴涨
```

---

# 17. 第七项：GPS

常看：

```text
Status
NSats
HDop
Spd
Alt
```

简单理解：

```text
NSats = 卫星数
HDop  = 水平定位精度指标，通常越小越好
```

如果 Loiter 异常，同时 GPS 状态也差，就不能只怪 PID。

---

# 18. 第八项：BARO

BARO 是气压计。

常看：

```text
Alt
Press
Temp
```

如果高度异常，可以对比：

```text
BARO.Alt
GPS.Alt
CTUN.Alt
```

---

# 19. 第九项：MAG

MAG 是磁罗盘。

如果日志中出现：

```text
EKF_YAW_RESET
Compass error
mag anomaly
```

就应该检查：

```text
MAG
```

以及磁罗盘安装、供电线、电机电流产生的磁干扰。

---

# 20. 第十项：EKF / XKF

EKF 是状态估计器。

它把：

```text
IMU
GPS
气压计
磁罗盘
```

融合成：

```text
姿态
速度
位置
航向
```

初学阶段不需要一上来研究全部 XKF 项。

先看：

```text
MSG
ERR
Events
```

有没有：

```text
EKF primary changed
EKF3 lane switch
EKF yaw reset
```

---

# 21. EKF lane switch 是什么

飞控可以同时运行多个 EKF lane。

如果某一路状态质量变差，可能：

```text
Lane 0
 ↓
Lane 1
```

日志会出现：

```text
EKF3 lane switch
EKF primary changed
```

如果它刚好发生在：

```text
VIBE 暴涨
Clip 增长
```

之后，就需要怀疑振动影响了状态估计。

---

# 22. MSG / ERR / Events 很重要

建议每次打开日志先勾：

```text
Events
Errors
MSG
```

先看整个过程：

- Armed
- 起飞
- 模式切换
- EKF 报错
- GPS 报错
- Compass 报错
- Land Complete
- Disarm

这样先知道“什么时候发生了什么”。

---

# 23. CTUN 是什么

可以先理解为：

```text
Copter Control Tuning
```

常用：

```text
Alt
DAlt
ThO
CRt
DCRt
```

其中：

```text
Alt  = 实际高度
DAlt = 目标高度
ThO  = 总体油门输出
```

---

# 24. 最推荐的基础分析顺序

## Step 1：Events / ERR / MSG

先确定：

```text
什么时候 Arm
什么时候起飞
什么飞行模式
有没有 EKF / GPS / Compass 错误
```

---

## Step 2：VIBE

画：

```text
VibeX
VibeY
VibeZ
Clip
```

如果多颗 IMU，就分别看。

---

## Step 3：RCOU

画：

```text
C1
C2
C3
C4
```

判断：

- 是否长期不平衡
- 是否触底
- 是否触顶
- 是否疯狂来回抽动

---

## Step 4：Roll RATE

```text
RATE.RDes
RATE.R
```

---

## Step 5：Pitch RATE

```text
RATE.PDes
RATE.P
```

---

## Step 6：Yaw RATE

```text
RATE.YDes
RATE.Y
```

---

## Step 7：ATT

```text
ATT.DesRoll
ATT.Roll
```

再看：

```text
ATT.DesPitch
ATT.Pitch
```

---

## Step 8：RATE 有问题再看 PID

Roll：

```text
PIDR.Tar
PIDR.Act
PIDR.Err
PIDR.P
PIDR.I
PIDR.D
PIDR.FF
PIDR.Flags
```

Pitch：

```text
PIDP.*
```

Yaw：

```text
PIDY.*
```

---

# 25. 常见问题模式

## 25.1 VIBE 高 + RATE Actual 很毛

```text
VIBE ↑
RATE 实际值高频乱跳
RCOU 高频抽动
```

优先检查：

```text
桨
电机
机臂
飞控减震
结构共振
```

---

## 25.2 目标动了，实际跟不上

可能：

```text
P偏低
FF偏低
动力响应慢
惯量大
```

---

## 25.3 实际严重超过目标

可能：

```text
P偏激进
阻尼不足
D不合适
结构弹性
```

---

## 25.4 目标不动，实际疯狂振荡

优先：

```text
机械 / Gyro / Filter
```

---

## 25.5 Yaw 一直追不上

先看：

```text
RCOU
PIDY.Flags
Motor saturation
MOT_YAW_HEADROOM
```

不要直接加 Yaw P。

---

## 25.6 一个电机长期比其他电机高

检查：

```text
重心
该电机/桨
电调
安装角
机架
```

---

# 26. 时间轴一定要放大

整个 1 分钟只能看趋势。

真正分析问题时，要把异常附近放大。

例如发现：

```text
17:08:23
```

附近出问题。

就只看：

```text
17:08:21 ～ 17:08:25
```

然后观察：

```text
谁先变化？
谁后变化？
```

---

# 27. “谁先变化”比单独看数值更重要

例如：

```text
VIBE 先暴涨
 ↓
RATE.Actual 开始抖
 ↓
PID.D 开始变大
 ↓
RCOU 开始抽动
```

说明机械振动很可能是前因。

如果：

```text
RCOU 先突然变化
 ↓
VIBE 再暴涨
```

可能是某个控制动作激发了结构共振。

---

# 28. 初学阶段最容易犯的错误

## 错误 1：VIBE 高就改 PID

正确：

```text
先解决机械
```

## 错误 2：Yaw 跟不上就加 Yaw P

正确：

```text
先看电机余量和 Flags
```

## 错误 3：只看 Max

Max 可能只是落地碰了一下。

更重要的是：

```text
是否持续
```

## 错误 4：一次改很多参数

建议：

```text
一次只解决一个问题
```

---

# 29. 推荐调参顺序

```text
机械检查
 ↓
VIBE / Clip
 ↓
FFT / Harmonic Notch
 ↓
Roll / Pitch RATE
 ↓
P / D
 ↓
I
 ↓
FF
 ↓
ATT 外环
 ↓
Yaw
 ↓
Loiter / Position
 ↓
AutoTune
```

---

# 30. 每次建议保存这几张图

```text
1. VIBE X/Y/Z + Clip
2. RCOU.C1~C4
3. RATE.RDes + RATE.R
4. RATE.PDes + RATE.P
5. RATE.YDes + RATE.Y
6. ATT.DesRoll + ATT.Roll
7. ATT.DesPitch + ATT.Pitch
8. PIDR Tar/Act/P/I/D/FF/Flags
9. PIDP Tar/Act/P/I/D/FF/Flags
```

---

# 31. 初学者快速检查表

```text
[ ] 有没有 Arm / Disarm
[ ] 飞行模式是什么
[ ] VIBE 是否明显偏高
[ ] Clip 是否为 0
[ ] 多颗 IMU 是否同时异常
[ ] RCOU 是否有电机长期触底
[ ] RCOU 是否有电机长期触顶
[ ] Roll RATE 是否跟目标
[ ] Pitch RATE 是否跟目标
[ ] Yaw RATE 是否跟目标
[ ] 有没有明显超调
[ ] 有没有明显振铃
[ ] PID Flags 是否长期 LIMIT
[ ] 有没有 EKF lane switch
[ ] 有没有 EKF yaw reset
[ ] GPS 有没有报错
[ ] Compass 有没有报错
[ ] 电池有没有明显掉压
```

---

# 32. 最后只记住这两张图

## 控制链

```text
ATT.Des
   ↓
姿态外环
   ↓
RATE.Des
   ↓
PID
   ↓
RCOU
   ↓
Motor
   ↓
飞机运动
   ↓
IMU / EKF
   ↓
反馈
```

## 分析顺序

```text
机械
 ↓
传感器
 ↓
状态估计
 ↓
控制器
 ↓
执行器
```

---

# 33. 一句话总结

看 ArduPilot Log，初学阶段先回答五个问题：

```text
1. 振不振？
2. 电机有没有控制余量？
3. RATE 目标和实际跟不跟？
4. PID 有没有一直撞限幅？
5. EKF 有没有报错？
```

按照：

```text
VIBE
→ RCOU
→ RATE
→ ATT
→ PID
→ EKF
```

这个顺序，就已经可以完成大多数基础飞行问题分析。

---

# 34. 下一步学习方向

看懂本文后，可以继续学：

1. FFT 频谱分析
2. Harmonic Notch
3. Gyro 原始数据
4. EKF Innovation
5. PID P/I/D/FF 定量分析
6. Motor thrust linearization
7. AutoTune 日志分析
8. Position / Velocity Controller
