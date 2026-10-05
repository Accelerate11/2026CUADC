# cuadc_tools

`ament_python` 包，安装三个工具入口：

| 入口 | 功能 |
| --- | --- |
| `calibrate_route` | 启动独立 MAVROS，记录两点并保存地理航向；结束时关闭其 MAVROS |
| `mavlink_stream_configurator` | 连接已有 MAVROS，请求原包的定位消息发送频率 |
| `flight_telemetry_recorder` | 订阅 MAVROS 与任务状态，记录遥测 CSV |

```bash
ros2 run cuadc_tools calibrate_route --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 --output-dir routes
ros2 run cuadc_tools flight_telemetry_recorder --output /tmp/telemetry.csv --rate-hz 2
```

标定的串口 URL 也可由 `CUADC_FCU_URL` 提供，默认保存目录也可由 `CUADC_ROUTES_DIR` 设置。工具不再写死 `/home/cuadc/CUADCv16` 或某一台飞控的序列号。
