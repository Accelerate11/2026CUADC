# CUADC 快速开始

## 1. 放置目录

建议：

```bash
mkdir -p ~/cuadc_mission/calibration
unzip cuadc_d435i_payload_calibration_2026_09_07.zip -d ~/cuadc_mission/calibration
cd ~/cuadc_mission/calibration/cuadc_d435i_payload_calibration_2026_09_07
chmod +x bin/*.sh tools/*.py
```

## 2. 推荐：当前正式视觉节点 + SSH 网页

终端 A：

```bash
./bin/run_current_cuadc_visual_calibration.sh ~/cuadc_mission
```

终端 B：

```bash
./bin/web_video_server.sh
```

Windows CMD：

```cmd
ssh -L 8080:127.0.0.1:8080 cuadc@NUC实际IP
```

浏览器：

```text
http://127.0.0.1:8080/stream?topic=/cuadc/calib/annotated&type=mjpeg
```

## 3. P1 采样

桶置于 P1 / SERVO9 中轴线正下方：

```bash
python3 tools/sample_payload_xy.py --payload 1 --true-x 0.026 --true-y -0.065 --duration 5
```

## 4. P2 采样

桶置于 P2 / SERVO10 中轴线正下方：

```bash
python3 tools/sample_payload_xy.py --payload 2 --true-x 0.0109 --true-y 0.072 --duration 5
```

## 5. 写入正式任务

```bash
python3 tools/apply_current_cuadc_calibration.py --workspace ~/cuadc_mission --dry-run
python3 tools/apply_current_cuadc_calibration.py --workspace ~/cuadc_mission
python3 tools/verify_current_workspace.py --workspace ~/cuadc_mission
```

然后重新编译正式包。
