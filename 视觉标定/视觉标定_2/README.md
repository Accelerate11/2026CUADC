# CUADC D435i 双投放中心标定工具包

面向当前 CUADC V6X + D435i + ROS 2 Humble 工作空间的**视觉中心 / 双投放口准星标定包**。

本包保留三种可视化方案：

1. **网页模式（推荐 SSH 场景）**：标定画面发布为 ROS Image，经 `web_video_server` + SSH 端口转发，在 Windows/Linux 浏览器查看；
2. **本地显示器模式**：NUC 接显示器时，用独立 `alignment_viewer` 的 OpenCV 窗口或 `rqt_image_view` 查看；
3. **直接视觉节点叠加模式**：当前 `drop_bucket_realsense_node.py` 直接发布 `/cuadc/calib/annotated`，绿色 TARGET + P1/P2 准星与正式识别结果完全同源。

> 安全要求：整个静态标定过程必须卸桨、不解锁、不启动自主飞行状态机。标定只涉及相机、视觉节点和静态机械几何。

---

## 1. 当前坐标系

正式视觉节点使用机体 **FLU**：

```text
+X = 机头前方
+Y = 飞机左侧
+Z = 飞机上方
```

D435i 当前安装方向对应：

```text
图像左边 = 飞机前方
图像右边 = 飞机后方
图像上边 = 飞机右侧
图像下边 = 飞机左侧
相机光轴 = 向下
```

独立 `alignment_viewer` 使用 **FRD**：

```text
+X = 机头前方
+Y = 飞机右侧
+Z = 向下
```

FLU 与 FRD 转换：

```text
X_frd =  X_flu
Y_frd = -Y_flu
Z_frd = -Z_flu
```

---

## 2. 本机当前采用的标定值

当前打包版本记录的是 2026-09-07 这一架 V6X 的候选正式值：

### 相机视觉外参（FLU）

```yaml
camera_offset_body_m: [0.0, 0.0, -0.35]
camera_xy_calib_bias_m: [0.047805, -0.014577]
camera_optical_to_body_rotation:
  [-0.971358, 0.237622, 0.0,
    0.237622, 0.971358, 0.0,
    0.0,      0.0,     -1.0]
```

旋转矩阵行列式应保持约 `+1.000`。

### 双投放口（FLU）

```text
P1 / SERVO9
[+0.0260, -0.0650, -0.3200] m
= 前 2.60 cm、右 6.50 cm、下 32 cm

P2 / SERVO10
[+0.0109, +0.0720, -0.3200] m
= 前 1.09 cm、左 7.20 cm、下 32 cm
```

### 正式投放工作高度

```text
SEARCH        2.2 m
ALIGN_COARSE  1.7 m
ALIGN_FINE    1.3 m
RELEASE       1.3 m
```

> P2 是当前最新候选值。重新拆装相机、投放机构、机架或飞控后必须重新验证。

机器可读记录：`config/cuadc_v6x_current_calibration.yaml`。

---

## 3. 目录结构

```text
README.md
config/
  cuadc_v6x_current_calibration.yaml   当前本机 FLU/FRD 标定记录
  aircraft.example.yaml                通用模板
  aircraft.cuadc_v6x_current.yaml      独立 viewer 的当前 FRD 配置
  alignment_base.yaml
bin/
  build.sh                              编译独立 viewer
  visual_alignment.sh                  原独立 viewer
  visual_alignment_display.sh          本地显示器模式
  visual_alignment_headless.sh         无显示器发布 annotated Image
  web_video_server.sh                  网页视频服务器
  show_annotated_local.sh              rqt_image_view 本地查看
  check_cuadc_calibration_topics.sh     检查 ROS 标定话题
  stop_calibration_helpers.sh           停止辅助进程
  run_current_cuadc_visual_calibration.sh 当前 CUADC 视觉节点标定启动模板
tools/
  sample_payload_xy.py                 5 秒中位数/MAD/剩余误差采样
  apply_current_cuadc_calibration.py   将当前值写入 cuadc_mission
  verify_current_workspace.py          检查工作空间正式值
docs/
  QUICKSTART_CUADC.md
  WEB_VISUALIZATION.md
  DISPLAY_VISUALIZATION.md
  CUADC_VISUAL_NODE_INTEGRATION.md
  COORDINATE_SYSTEMS.md
  CALIBRATION_RECORD_2026_09_07.md
  TROUBLESHOOTING.md
ros2_ws/src/vision_servo_calibration/
  ...                                  独立准星 viewer 源码
```

---

## 4. 推荐方案：当前 CUADC 视觉节点直接叠加 + 网页查看

这是当前无显示器、SSH 使用场景最方便的方案。

### 4.1 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-web-video-server \
  ros-humble-rqt-image-view \
  python3-opencv \
  python3-numpy \
  python3-yaml
```

### 4.2 启动当前视觉标定节点

把本包放到例如：

```text
~/cuadc_mission/calibration/cuadc_d435i_payload_calibration_2026_09_07
```

然后：

```bash
cd ~/cuadc_mission/calibration/cuadc_d435i_payload_calibration_2026_09_07
chmod +x bin/*.sh tools/*.py
./bin/run_current_cuadc_visual_calibration.sh ~/cuadc_mission
```

该脚本使用当前记录的：

- `camera_xy_calib_bias_m`
- `camera_optical_to_body_rotation`
- P1/P2
- 自动曝光
- `/cuadc/calib/annotated`

不会启动状态机，不会启动 MAVROS。

### 4.3 检查标定画面话题

新 SSH 终端：

```bash
cd ~/cuadc_mission/calibration/cuadc_d435i_payload_calibration_2026_09_07
./bin/check_cuadc_calibration_topics.sh
```

至少应看到：

```text
/perception/drop_buckets_body
/cuadc/calib/annotated
```

若已启用完整旁路，还可看到：

```text
/cuadc/calib/color/image_raw
/cuadc/calib/color/camera_info
/vision_servo/targets_body
```

---

## 5. 网页模式（SSH / NUC 无显示器）

### NUC 端

```bash
cd ~/cuadc_mission/calibration/cuadc_d435i_payload_calibration_2026_09_07
./bin/web_video_server.sh
```

默认端口 `8080`。如果 8080 已经被旧的 `web_video_server` 占用，脚本会提示并复用，不要重复启动。

### Windows CMD 端

先获得 NUC 实际 IP：

```bash
hostname -I
```

Windows **CMD** 中必须写一整行，不能用 Linux 的 `\` 换行：

```cmd
ssh -L 8080:127.0.0.1:8080 cuadc@NUC实际IP
```

保持该 CMD 窗口不关闭。

浏览器打开：

```text
http://127.0.0.1:8080/stream?topic=/cuadc/calib/annotated&type=mjpeg
```

画面含义：

```text
绿色 TARGET      = 正式视觉最终输出的桶中心
红/橙 P1         = 1号 / SERVO9 投放口准星
黄/紫 P2         = 2号 / SERVO10 投放口准星
FWD              = 飞机前方
RIGHT            = 飞机右侧
```

---

## 6. 本地显示器模式

NUC 接显示器且有桌面环境时，有两种方式。

### 方式 A：直接查看 `/cuadc/calib/annotated`

```bash
./bin/show_annotated_local.sh
```

底层使用 `rqt_image_view`。

### 方式 B：独立 `alignment_viewer`

先编译：

```bash
./bin/build.sh
```

生成当前 FRD 配置：

```bash
cp config/aircraft.cuadc_v6x_current.yaml config/aircraft.yaml
python3 tools/validate_config.py --config config/aircraft.yaml --mode alignment
python3 tools/generate_config.py
```

然后：

```bash
./bin/visual_alignment_display.sh
```

独立 viewer 会显示：

- 绿色：视觉目标；
- 各 Payload 彩色准星；
- `dF / dR`；
- `+X FWD / +Y RIGHT`；
- 自动记录 CSV 和标注视频。

---

## 7. 无显示器独立 viewer + 网页

如果不想在正式视觉节点里画准星，也可以让独立 viewer 生成：

```text
/vision_servo/alignment/image
```

步骤：

```bash
./bin/visual_alignment_headless.sh
./bin/web_video_server.sh
```

浏览器：

```text
http://127.0.0.1:8080/stream?topic=/vision_servo/alignment/image&type=mjpeg
```

此模式要求视觉侧已经发布：

```text
/cuadc/calib/color/image_raw
/cuadc/calib/color/camera_info
/vision_servo/targets_body
```

详见 `docs/CUADC_VISUAL_NODE_INTEGRATION.md`。

---

## 8. 静态 P1/P2 标定流程

### 8.1 机械条件

- 卸桨；
- 飞机保持水平；
- 正式投放高度建议直接在 `1.3 m` 工作高度验证；
- 桶口高度与比赛桶一致；
- 用铅垂线确定舵机/释放机构真实中轴线。

### 8.2 P1

把桶中心放在 P1 / SERVO9 实际中轴线正下方。

当前 P1 真值：

```text
FLU x=+0.0260 y=-0.0650
```

采样：

```bash
python3 tools/sample_payload_xy.py \
  --payload 1 \
  --true-x 0.0260 \
  --true-y -0.0650 \
  --duration 5
```

输出：

```text
median FLU
MAD
remaining dx
remaining dy
```

目标：

```text
|remaining dx| <= 0.005 m
|remaining dy| <= 0.005 m
```

### 8.3 P2

桶移到 P2 / SERVO10 实际中轴线正下方。

当前候选：

```text
FLU x=+0.0109 y=+0.0720
```

```bash
python3 tools/sample_payload_xy.py \
  --payload 2 \
  --true-x 0.0109 \
  --true-y 0.0720 \
  --duration 5
```

同样建议控制在 5 mm 内；如果已经达到 3~5 mm，停止继续追 1 mm 级噪声。

---

## 9. 如何判断应该调什么

### P1/P2 同方向、近似相同偏差

优先调：

```text
camera_xy_calib_bias_m
```

说明更像相机整体 XY 平移误差。

### P1 与 P2 呈明显相反方向残差

优先检查：

```text
camera_optical_to_body_rotation 的 yaw
```

也要重新核对两个机械中轴线。

### P1 很准，P2 单独不准

不要继续破坏已经正确的相机外参，优先修：

```text
P2 payload_release_offsets_body_m
```

这正是当前这架飞机最后一次调整采用的策略。

### Z 不准

先检查：

```text
飞机参考点高度
桶口真实高度
camera_offset_body_m[2]
depth_reference_mode
```

当前 `rim_direct` 模式下，视觉输出参考点已经是桶沿/桶口平面，不应额外再加 0.30 m。

---

## 10. 把当前标定写入正式 `cuadc_mission`

先预览：

```bash
python3 tools/apply_current_cuadc_calibration.py \
  --workspace ~/cuadc_mission \
  --dry-run
```

确认无误后：

```bash
python3 tools/apply_current_cuadc_calibration.py \
  --workspace ~/cuadc_mission
```

然后检查：

```bash
python3 tools/verify_current_workspace.py \
  --workspace ~/cuadc_mission
```

最后重新编译：

```bash
cd ~/cuadc_mission
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cuadc_visual_drop_flight
```

---

## 11. 常见问题

### `AMENT_TRACE_SETUP_FILES: unbound variable`

本包脚本已经在 `source setup.bash` 前暂时 `set +u`，不会再因为 ROS 2 setup 中未定义变量退出。

### `Device or resource busy`

D435i 被旧进程占用：

```bash
pgrep -af "drop_bucket_realsense|realsense2_camera|vision_provider_template"
sudo fuser -v /dev/video* 2>/dev/null
```

停止旧视觉：

```bash
pkill -TERM -f drop_bucket_realsense_node.py 2>/dev/null || true
pkill -TERM -f realsense2_camera 2>/dev/null || true
sleep 2
```

不要同时启动第二个 RealSense driver。

### `bind: Address already in use 0.0.0.0:8080`

通常说明旧 `web_video_server` 已经在运行：

```bash
pgrep -af web_video_server
ss -lntp | grep ':8080'
```

直接复用即可。

### 网页有服务器但没画面

```bash
ros2 topic hz /cuadc/calib/annotated
```

先确认 ROS Image 确实持续发布。

更多见 `docs/TROUBLESHOOTING.md`。

---

## 12. 标定原则

1. **准星不能跟着目标生成**：P1/P2 必须来自机械投放口外参；
2. **图像和目标必须同一采集时间戳**：当前视觉使用 `capture_stamp`，应保留；
3. **相机只允许一个进程占用**；
4. **正式视觉接口与标定可视化解耦**；
5. 优先在最终工作高度验证；
6. P1 已正确时，不要为了修 P2 再破坏全局相机外参；
7. 参数最终写入 `flight_params.yaml` 和状态机前必须重新静态验证。

