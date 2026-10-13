# 室内投放准星预检

该入口只启动主程序的 D435i + YOLO + 深度视觉节点，不启动 MAVROS、飞控或任务状态机。
画面与飞行主程序使用相同的模型、NMS、深度修正、相机外参和投放偏移。先由检测框四周的外环估计地面，再按 0.30 m 桶高恢复桶口平面；在机体 FRD 坐标中，桶口的 z 为地面的 z 减去桶高。准星是机体舵机铅垂线与该平面的交点，不跟随目标框移动：橙色十字是 1 号右前舵机（CH7），洋红色十字是 2 号左后舵机（CH8）。

```bash
bash indoor_projection/preflight_projection.sh       # 自动曝光
bash indoor_projection/preflight_projection.sh 35    # 手动曝光值 35
```

窗口关闭或按 Ctrl-C 结束预检。上述命令从工作区根目录执行；先完成工作区编译。该预检实际读取 `src/cuadc_full_mission_20260803/config/mission_real.yaml`，正式飞行分别读取 `full_mission/mission.yaml` / `drop_only/mission.yaml`，标定后请同步相应参数。原配置默认 `intel:gpu`，也可设置 `CUADC_VISION_DEVICE=cpu`。不需要复制另一套检测逻辑。

每次运行会在 `logs/preflight_<时间>/` 保存 ROS 日志、标注视频和 `detections_camera.csv`。

## 单高度舵机准星标定

1. 将飞机固定或架设在需要标定的高度，并保持机体姿态水平、相机和舵机安装状态与实际飞行一致。
2. 在待标定舵机的位置垂下一根细线，在线端悬挂重物，使细线保持竖直。线端落在桶口平面上的位置，就是该舵机的实际铅垂投影。
3. 调整飞机位置，使细线端准确位于桶口中心。此时桶中心就是该舵机在当前桶口高度平面的真实投影点。
4. 打开室内视觉检查程序：

   ```bash
   bash indoor_projection/preflight_projection.sh
   ```

5. 观察画面中的绿色桶中心十字和对应舵机准星。此时绿色十字代表检测到的桶中心，彩色准星代表程序根据当前标定坐标计算出的舵机投影。调整该舵机的 `payload_release_offsets_body_m` 坐标，直到彩色准星与绿色十字重合。
6. 保持同一高度和姿态重复验证几次；前后左右移动飞机时，准星应随飞机移动，而不是始终固定在桶中心。

标定时只调整对应舵机的机体 `x/y` 坐标，不能修改另一个舵机。`payload_release_offsets_body_m` 使用 FCU FRD：`+x` 向机头前方，`+y` 向机体右方，`+z` 向下。画面方向还受相机安装与外参影响，应以画面标出的 FRD 方向和实际标定结果为准，不要直接套用另一架飞机的图像方向。
