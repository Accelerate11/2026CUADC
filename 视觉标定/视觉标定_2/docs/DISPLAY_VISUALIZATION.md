# 显示器可视化

## 方案 A：直接标定叠加话题

NUC 有桌面环境时：

```bash
./bin/show_annotated_local.sh
```

默认查看 `/cuadc/calib/annotated`。也可手工打开 `rqt_image_view` 选择其他图像话题。

## 方案 B：独立 alignment_viewer

```bash
./bin/build.sh
cp config/aircraft.cuadc_v6x_current.yaml config/aircraft.yaml
python3 tools/validate_config.py --config config/aircraft.yaml --mode alignment
python3 tools/generate_config.py
./bin/visual_alignment_display.sh
```

OpenCV 窗口会显示目标、P1/P2、FRD 方向和 dF/dR。

## 方案 C：独立 viewer 无窗口 + 本地 rqt

```bash
./bin/visual_alignment_headless.sh
./bin/show_annotated_local.sh /vision_servo/alignment/image
```
