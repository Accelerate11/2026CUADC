# 舵机循环测试（V16）

测试前卸下螺旋桨，保持飞控未解锁，并清空投放机构周围区域。完成工作区编译后，从工作区根目录执行：

```bash
bash servo_test/run_servo_test.sh --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200
```

必须替换实际设备，或设置 `CUADC_FCU_URL`。脚本与飞行、标定和时钟检查共用串口锁；输入 `TEST` 后才启动独立 MAVROS。飞控连接且未解锁后，保留原测试次序：CH7 投放/收回，间隔两秒后 CH8 投放/收回，再各重复一次。

投放 PWM 为 1900、保持 0.7 秒，收回 PWM 为 1100。完成或异常退出时，原测试程序会再次尝试收回 CH7/CH8，并关闭本次 MAVROS。实际执行逻辑保存在未修改的 `servo_cycle_test.py`。日志位于 `servo_test/logs/`。
