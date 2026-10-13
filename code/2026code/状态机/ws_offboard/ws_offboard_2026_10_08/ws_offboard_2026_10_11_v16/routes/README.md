# 每轮两点航向标定（V16）

**`example.yaml` 是合成格式样例，不能作为实际飞行标定。包内不包含任何真实 GPS 航线。**

完成编译后，从工作区根目录运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 routes/calibrate_route.py --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200
```

必须替换实际设备，也可显式设置 `CUADC_FCU_URL`。按原程序提示记录两点，间距必须严格大于 10 m，再输入文件名（如 `round_01`）。文件默认保存在工作区 `routes/`；`--output-dir` 或 `CUADC_ROUTES_DIR` 可修改保存位置。任务入口读取外部目录时同样设置 `CUADC_ROUTES_DIR`。

```bash
bash full_mission/run_full_mission.sh round_01 --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200
```

启动器每次从两点重新计算地理航向（正北为 0°、顺时针增加），保留原 V16 算法。保存的 `heading_deg` 是参考值。点 1 用于启动前 GPS 偏移提示；真实任务原点仍由 V16 状态机锁定。GPS/罗盘偏差沿用原版提示策略，不修改为新的阻断门槛。

标定与飞行入口共用 `.flight_serial.lock`，避免同时连接飞控；如显式设置 `CUADC_SERIAL_LOCK`，所有入口应使用同一个值。
