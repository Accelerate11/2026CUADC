# 仅投放任务（V16）

从工作区根目录运行；先完成编译和本轮标定，路线名 `round_01` 对应 `routes/round_01.yaml`。

```bash
bash drop_only/run_drop_only.sh round_01 --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 --preflight
bash drop_only/run_drop_only.sh round_01 --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 expo=0
```

必须替换实际飞控设备，或显式设置 `CUADC_FCU_URL`。曝光使用 `expo=数值`，0 为自动曝光；省略时读取本目录 `mission.yaml`。任务完成两次投放后直接返航、降落、上锁，不执行侦察航段。串口锁、启动守护、原 V16 预检与飞控时间同步和完整任务入口一致。

本目录 `mission.yaml` 可直接修改，无需重新编译。原配置默认 `intel:gpu`，如需 CPU 可设置 `CUADC_VISION_DEVICE=cpu`。每轮日志在 `drop_only/logs/flight_<时间>/`，任务结束自动生成本地报告，不会自动发送邮件。其余参数见 `full_mission/README.md`。
