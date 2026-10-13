# 飞行后置总结（默认仅本地）

`flight_summary_report.py` 保留 V16 的日志解析、遥测汇总、投放结果、异常分类和报告内容。默认只在日志目录写入 `flight_summary.txt`，不连接 SMTP。缺少可选的 `pymavlink` / `pyulog` 时，报告注明原始飞控数据缺失。

```bash
python3 tools/flight_summary_report.py /path/to/logs/flight_YYYYMMDD_HHMMSS
python3 tools/flight_summary_report.py /path/to/logs/flight_YYYYMMDD_HHMMSS --no-email
```

完整任务及仅投放入口会启动独立的后置监视器；任务 launch 退出、日志刷新后生成本地报告。监视器显式传入 `--no-email`，正常结束和错误分支都不会自动发邮件。固定坐标测试、相机预检和舵机测试保持原版不启动监视器的行为。整机断电时进程无法继续工作，可恢复供电后手动生成报告。

日志中的 `postflight_report_hook.log` 记录后置执行结果；生成失败时写入 `flight_summary_error.txt`。同一日志目录使用原版原子标记目录避免重复处理。

## 可选手动邮件

只有用户手动加 `--send` 才发送。先按 `smtp.env.example` 显式设置发件人、收件人、SMTP 服务器和授权码环境变量；示例均为占位符，不含真实账户或密钥。也可通过 `--smtp-config` 指定不含密钥的配置文件。授权码仅从 `CUADC_SMTP_TOKEN` / `SMTP_TOKEN` 或 `--token-env` 指定变量读取。

```bash
python3 tools/flight_summary_report.py /path/to/logs/flight_YYYYMMDD_HHMMSS --send
```

保留原版附件与限时重试功能；邮件操作失败时，本地报告仍已生成。不要将真实 SMTP 配置、授权码或飞行日志提交到仓库。

## 飞控时钟诊断

完成编译并加载 ROS/工作区环境后，可独立检查 V16 飞控时间同步；该命令会连接飞控并请求遥测频率，但不运行任务节点或执行器命令：

```bash
python3 tools/check_fcu_clock.py --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200
python3 tools/check_fcu_clock.py --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 --callbacks
```

`--callbacks` 使用 `build/cuadc_full_mission_20260803/test_time_alignment`，须启用 `BUILD_TESTING` 编译；可用 `--callback-executable` 指定其他位置。诊断与任务、标定共用串口锁，MAVROS 加载原版 `mavros_time_sync.yaml`。
