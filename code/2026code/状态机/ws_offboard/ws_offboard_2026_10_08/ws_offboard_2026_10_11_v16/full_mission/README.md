# 完整任务（V16）

以下命令从工作区根目录执行，先按根目录 README 完成依赖安装、编译和本轮两点标定。位置参数是路线文件名，不是数字航向；`round_01` 对应 `routes/round_01.yaml`。

```bash
bash full_mission/run_full_mission.sh round_01 --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 --preflight
bash full_mission/run_full_mission.sh round_01 --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 expo=20
```

必须把 `YOUR_FCU` 替换为实际设备。也可设置 `CUADC_FCU_URL`，再省略 `--fcu-url`。保留串口独占守护，因此机载包装入口只接受串口 URL。`CUADC_ROUTES_DIR` 可选择其他标定目录，`--camera-serial YOUR_D435I_SERIAL` 或 `CUADC_CAMERA_SERIAL` 可显式指定本机相机，覆盖沿用原版的设备设置；请按实际相机选择。曝光用 `expo=数值` 覆盖，`expo=0` 表示自动曝光；省略时使用本目录 `mission.yaml`。

任务依次起飞、搜索识别、两次投放、灾情区域航点侦察、返航、降落及上锁。原 V16 配置允许满足门槛后切入 GUIDED 自动解锁起飞；启动守护、GPS/罗盘偏移提示、姿态检查和飞控时间同步均保留。`--preflight` 只查询部署配置与设备，不启动 MAVROS 或飞行节点；它不能替代连接飞控后的路线/姿态预检。

原 V16 参数默认使用 `intel:gpu`。其他平台应先准备对应依赖，并可显式设置 `CUADC_VISION_DEVICE=cpu`。默认模型仍为 `basket_detect.onnx`。本目录 `mission.yaml` 修改后无需重新编译。每轮日志保存在 `full_mission/logs/flight_<时间>/`，包含运行参数、遥测、检测 CSV、投放快照和录像。结束后自动生成本地 `flight_summary.txt`，不会自动发送邮件；详情见 `tools/README_flight_summary.md`。

仅查看相机画面：

```bash
bash full_mission/preflight_camera.sh --exposure 0
```

相机预览会打开 D435i，但不启动任务或 MAVROS。
