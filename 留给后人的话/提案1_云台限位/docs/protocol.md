# NUC—STM32 串口协议

物理层：115200 bit/s，8 数据位，无校验，1 停止位。命令是大写 ASCII，以 `\n` 或 `\r\n` 结束。

## 命令

| 命令 | 作用 | 限制 |
|---|---|---|
| `STATUS?` | 立即返回一帧遥测 | 无 |
| `ARM` | 进入稳定闭环 | 仅 `READY` |
| `DISARM` | 停止更新控制器并保持当前 PWM | 无 |
| `ZERO` | 把当前静止姿态定义为 0° | 仅 `READY`、近似 1 g、角速度小 |
| `SET,r,p` | 设置 Roll/Pitch 目标角（度） | 固件限幅为 Roll ±20°、Pitch ±30° |
| `GAIN,R,ka,kr,mr,ms` | 设置 Roll 的四个参数 | 仅 `READY` |
| `GAIN,P,ka,kr,mr,ms` | 设置 Pitch 的四个参数 | 仅 `READY` |

增益字段依次为姿态比例 `ka`、角速度比例 `kr`、最大目标角速度 `mr`、最大舵机变化率 `ms`。

## 响应

- 成功：`ACK,<命令>`
- 失败：`ERR,<原因>`
- 提示：`INFO,<文本>`
- 遥测：

  ```text
  TEL,state,roll,pitch,roll_rate,pitch_rate,accel_norm,servo_roll,servo_pitch,imu_failures
  ```

角度单位为度，角速度单位为度/秒，加速度模长单位为 g。固件主动以 20 Hz 发送遥测，`STATUS?` 也会触发一帧。

示例：

```text
> STATUS?
< TEL,READY,0.231,-0.418,0.031,-0.015,1.002,0.00,0.00,0
> ZERO
< ACK,ZERO
> ARM
< ACK,ARM
```
