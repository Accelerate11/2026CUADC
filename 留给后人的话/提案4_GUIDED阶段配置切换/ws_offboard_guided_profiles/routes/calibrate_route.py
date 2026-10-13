#!/usr/bin/env python3
"""Interactively record two GPS points and save the route heading.

The MAVROS GPS stream is sampled continuously.  Press Enter for point 1,
walk the aircraft at least 10 m along the field centre line, then press Enter
for point 2 and provide a YAML basename.  Compass-vs-GPS disagreement is
reported for information only and never changes the saved route.
"""

from __future__ import annotations

import argparse
import math
import fcntl
import os
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval, StreamRate
from std_msgs.msg import Float64


WORKSPACE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORKSPACE / "tools"))
from deployment_config import parse_serial_url, serial_lock_path

ROUTES = Path(os.environ.get("CUADC_ROUTES_DIR", str(WORKSPACE / "routes")))
EARTH_M = 6378137.0


def distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_M * math.asin(min(1.0, math.sqrt(h)))


def bearing_deg(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lat2 = map(math.radians, (a[0], b[0]))
    dlon = math.radians(b[1] - a[1])
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.degrees(math.atan2(y, x)) % 360.0


class Gps(Node):
    def __init__(self) -> None:
        super().__init__("cuadc_route_calibrator")
        self.fix: tuple[float, float] | None = None
        self.compass_deg: float | None = None
        self.status = NavSatStatus.STATUS_NO_FIX
        self.connected = False
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self.compass_cb,
            qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.interval_client = self.create_client(
            MessageInterval, "/mavros/set_message_interval")
        self.stream_client = self.create_client(
            StreamRate, "/mavros/set_stream_rate")

    def cb(self, msg: NavSatFix) -> None:
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            if -90 <= msg.latitude <= 90 and -180 <= msg.longitude <= 180:
                self.fix = (msg.latitude, msg.longitude)
                self.status = msg.status.status

    def compass_cb(self, msg: Float64) -> None:
        if math.isfinite(msg.data):
            self.compass_deg = msg.data % 360.0

    def state_cb(self, msg: State) -> None:
        self.connected = bool(msg.connected)


def configure_mavlink_rates(node: Gps) -> None:
    """Request the FCU streams needed by calibration at 30 Hz."""
    deadline = time.monotonic() + 20.0
    while rclpy.ok() and not node.connected and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.connected:
        raise TimeoutError("等待 MAVROS 与飞控连接超过 20 秒")
    if not node.interval_client.wait_for_service(timeout_sec=10.0):
        raise TimeoutError("MAVROS set_message_interval 服务不可用")
    if not node.stream_client.wait_for_service(timeout_sec=10.0):
        raise TimeoutError("MAVROS set_stream_rate 服务不可用")
    stream_request = StreamRate.Request()
    stream_request.stream_id = StreamRate.Request.STREAM_POSITION
    stream_request.message_rate = 30
    stream_request.on_off = True
    stream_future = node.stream_client.call_async(stream_request)
    rclpy.spin_until_future_complete(node, stream_future, timeout_sec=5.0)
    stream_response = stream_future.result()
    if stream_response is None:
        print("警告：飞控未确认 STREAM_POSITION 30 Hz。", file=sys.stderr)
    else:
        print("已请求 STREAM_POSITION: 30 Hz", flush=True)
    # GLOBAL_POSITION_INT (33) carries both GPS position and compass heading.
    for message_id in (33,):
        request = MessageInterval.Request()
        request.message_id = message_id
        request.message_rate = 30.0
        future = node.interval_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"飞控拒绝 MAVLink 消息 {message_id} 的 30 Hz 请求")
        print(f"已请求 MAVLink 消息 {message_id}: 30 Hz", flush=True)


def spin_until(node: Gps, prompt: str, timeout_s: float = 120.0) -> tuple[float, float]:
    print(prompt, flush=True)
    deadline = time.monotonic() + timeout_s
    while node.fix is None:
        if time.monotonic() >= deadline:
            raise TimeoutError(f"等待 GPS 有效超过 {timeout_s:.0f} 秒")
        rclpy.spin_once(node, timeout_sec=0.2)
    input("GPS 已有效，按 Enter 记录：")
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.05)
    assert node.fix is not None
    return node.fix


def main() -> int:
    parser = argparse.ArgumentParser(description="记录两点 GPS，保存原 V16 场地航向标定文件。")
    parser.add_argument("--fcu-url", default=os.environ.get("CUADC_FCU_URL", ""), help="必须显式提供的飞控串口 URL")
    parser.add_argument("--output-dir", type=Path, default=ROUTES, help="标定文件保存目录")
    args = parser.parse_args()
    try:
        parse_serial_url(args.fcu_url)
    except ValueError as error:
        parser.error(str(error))
    routes = args.output_dir.expanduser().resolve()
    # Share the flight launcher lock so calibration cannot open another FCU
    # connection during a mission. Keep this descriptor for the whole run.
    serial_lock = serial_lock_path().open('a')
    try:
        fcntl.flock(serial_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print('错误：任务或标定程序正在使用飞控，不能重复启动标定。', file=sys.stderr)
        return 3
    mavros = subprocess.Popen(
        ["ros2", "launch", "cuadc_full_mission_20260803", "mission_mavros.launch.py", f"fcu_url:={args.fcu_url}"],
        stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
    )
    time.sleep(1.0)
    if mavros.poll() is not None:
        print("错误：MAVROS 启动失败，请检查飞控设备和串口权限。", file=sys.stderr)
        mavros.terminate()
        return 3
    rclpy.init()
    node = Gps()
    try:
        configure_mavlink_rates(node)
        p1 = spin_until(node, "等待 GPS 有效后，将飞机放在起飞点。")
        print(f"起飞点：纬度 {p1[0]:.8f}，经度 {p1[1]:.8f}")
        p2 = spin_until(node, "沿场地中轴线向前移动至少 10 m，然后准备记录第二点。")
        separation = distance_m(p1, p2)
        print(f"两点距离：{separation:.2f} m")
        if separation <= 10.0:
            print("错误：两点距离必须严格大于 10 m。", file=sys.stderr)
            return 2
        heading = bearing_deg(p1, p2)
        print(f"计算出的中轴线航向：{heading:.2f}°")
        if node.compass_deg is not None:
            delta = abs((node.compass_deg - heading + 180.0) % 360.0 - 180.0)
            print(f"罗盘与中轴线航向差值：{delta:.2f}°（仅判断，不影响保存）")
        else:
            print("警告：第二点 Enter 时未收到 MAVROS 罗盘数据，仅保存 GPS 计算航向。")
        name = input("请输入 YAML 文件名（不含后缀）：").strip()
        if not name or Path(name).name != name or name.endswith(".yaml"):
            print("错误：文件名只能是不含路径和后缀的名称。", file=sys.stderr)
            return 2
        routes.mkdir(parents=True, exist_ok=True)
        path = routes / f"{name}.yaml"
        if path.exists() and input(f"文件已存在，覆盖 {path.name}？输入 yes 确认：").strip().lower() != "yes":
            return 2
        now = datetime.now(timezone.utc).astimezone().isoformat(timespec="seconds")
        path.write_text(
            "# Generated by calibrate_route.py; point_1 is only a GPS drift reference.\n"
            f"created_at: {now}\n"
            "point_1:\n"
            f"  latitude_deg: {p1[0]:.10f}\n  longitude_deg: {p1[1]:.10f}\n"
            "point_2:\n"
            f"  latitude_deg: {p2[0]:.10f}\n  longitude_deg: {p2[1]:.10f}\n"
            f"separation_m: {separation:.3f}\nheading_deg: {heading:.8f}\n",
            encoding="utf-8",
        )
        print(f"已保存：{path}")
        return 0
    except (KeyboardInterrupt, EOFError):
        print("已取消航向标定。", file=sys.stderr)
        return 130
    except TimeoutError as error:
        print(f"错误：{error}，请检查 GPS 天线和飞控定位状态。", file=sys.stderr)
        return 4
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        mavros.terminate()
        try:
            mavros.wait(timeout=5)
        except subprocess.TimeoutExpired:
            mavros.kill()


if __name__ == "__main__":
    raise SystemExit(main())
