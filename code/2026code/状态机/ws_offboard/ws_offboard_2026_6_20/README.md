# ws_offboard

ROS 2 Humble + MAVROS + ArduPilot/CUAV v5 Nano 自动起飞、矩形航点、降落示例工作空间。

## 目录结构

```text
ws_offboard/
├── README.md
├
└── src/
    └── offboard_control/
        ├── CMakeLists.txt
        ├── package.xml
        ├── launch/
        │   ├── mavros_offb_real.launch.py
        │   ├── mavros_offb_sitl.launch.py
        │   ├── mavros_real.launch.py
        │   └── offb_only.launch.py
        └── src/
            └── offb_node.cpp
```

## 安装 MAVROS

```bash
sudo apt update
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

## 构建

把整个 `ws_offboard` 文件夹放到用户主目录：

```bash
cd ~
source /opt/ros/humble/setup.bash
bash ~/ws_offboard/scripts/build_all.sh
```

## 只连接真机 MAVROS，不启动自动飞行

if00：

```bash
bash ~/ws_offboard/scripts/run_mavros_real_if00.sh
```

if02：

```bash
bash ~/ws_offboard/scripts/run_mavros_real_if02.sh
```

检查连接：

```bash
bash ~/ws_offboard/scripts/check_mavros.sh
```

检查本地位置：

```bash
bash ~/ws_offboard/scripts/check_local_position.sh
```

检查服务：

```bash
bash ~/ws_offboard/scripts/list_mavros_services.sh
```

## 真机自动飞行

该命令会启动 MAVROS 和 `offb_node`，程序会自动切 `GUIDED`、解锁、起飞到 2 m、飞 5 m × 5 m 矩形航点、降落。

if00：

```bash
bash ~/ws_offboard/scripts/run_auto_real_if00.sh
```

if02：

```bash
bash ~/ws_offboard/scripts/run_auto_real_if02.sh
```

## 紧急命令

紧急降落：

```bash
bash ~/ws_offboard/scripts/emergency_land.sh
```

落地后上锁：

```bash
bash ~/ws_offboard/scripts/disarm_after_landed.sh
```

## 直接 launch 命令

真机只开 MAVROS：

```bash
ros2 launch offboard_control mavros_real.launch.py \
  fcu_url:=/dev/serial/by-id/usb-ArduPilot_CUAVv5Nano_410035001451333035363236-if00:57600
```

真机 MAVROS + 自动节点：

```bash
ros2 launch offboard_control mavros_offb_real.launch.py \
  fcu_url:=/dev/serial/by-id/usb-ArduPilot_CUAVv5Nano_410035001451333035363236-if00:57600
```

SITL MAVROS + 自动节点：

```bash
ros2 launch offboard_control mavros_offb_sitl.launch.py fcu_url:=udp://:14550@
```
