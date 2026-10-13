#!/usr/bin/env python3
"""Validate route telemetry and report GPS/compass differences.

The calibrated route defines the mission frame.  Final GPS movement and
compass disagreement are advisory only: they are printed for the pilot but
never pause or reject a flight.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval, StreamRate
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float64

EARTH_M = 6378137.0


def distance_m(a, b):
    lat1, lon1 = map(math.radians, a); lat2, lon2 = map(math.radians, b)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    h = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_M * math.asin(min(1.0, math.sqrt(h)))


def bearing_deg(a, b):
    lat1, lat2 = map(math.radians, (a[0], b[0]))
    dlon = math.radians(b[1] - a[1])
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.degrees(math.atan2(y, x)) % 360.0


class Stream(Node):
    def __init__(self):
        super().__init__("cuadc_route_preflight")
        self.gps = None; self.compass = None; self.connected = False
        self.attitude_count = 0
        self.attitude_arrival = None
        self.create_subscription(
            Imu, '/mavros/imu/data', self.attitude_cb, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self.gps_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self.compass_cb,
            qos_profile_sensor_data)
        self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.interval_client = self.create_client(
            MessageInterval, "/mavros/set_message_interval")
        self.stream_client = self.create_client(
            StreamRate, "/mavros/set_stream_rate")
    def gps_cb(self, msg):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude):
            self.gps = (msg.latitude, msg.longitude)
    def compass_cb(self, msg):
        if math.isfinite(msg.data): self.compass = msg.data % 360.0
    def state_cb(self, msg): self.connected = bool(msg.connected)

    def attitude_cb(self, msg):
        q = msg.orientation
        norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
        if msg.orientation_covariance[0] < 0 or not math.isfinite(norm) or abs(norm - 1.0) > 0.05:
            return
        self.attitude_count += 1
        self.attitude_arrival = time.monotonic()


def spin_retry_pause(node, seconds):
    deadline = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))


def request_with_retry(node, client, request, label, attempts=5):
    """Retry telemetry configuration only; never use for actuator commands.

    MAVROS can return success=False when its internal command service times
    out, even while the command plugin is still waiting for a late ACK.
    Space failed attempts at least six seconds apart to let that transaction
    drain. Successful requests have no extra delay. Local future cancellation
    removes the pending client request, but cannot cancel an FCU transaction.
    """
    reason = "未收到响应"
    for attempt in range(1, attempts + 1):
        if not rclpy.ok():
            raise RuntimeError(f"{label}：ROS 已关闭，停止重试")
        started = time.monotonic()
        future = None
        try:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
            if not future.done():
                reason = "等待 MAVROS 服务响应超时"
            else:
                response = future.result()
                if response is not None and getattr(response, 'success', True):
                    print(f"[OK] {label} 服务请求成功（第 {attempt}/{attempts} 次）", flush=True)
                    return response
                reason = "MAVROS 返回失败（可能为内部超时，不能据此认定飞控拒绝）"
        except Exception as error:
            reason = f"服务调用异常：{error}"
        finally:
            if future is not None and not future.done():
                client.remove_pending_request(future)
                future.cancel()
        print(f"[WARN] {label} 第 {attempt}/{attempts} 次失败：{reason}",
              file=sys.stderr, flush=True)
        if attempt < attempts:
            spin_retry_pause(node, max(1.0, 6.0 - (time.monotonic() - started)))
    raise RuntimeError(f"{label} 连续 {attempts} 次请求失败；最后原因：{reason}")


def configure_mavlink_rates(node):
    deadline = time.monotonic() + 20.0
    while rclpy.ok() and not node.connected and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if not node.connected or not node.interval_client.wait_for_service(timeout_sec=10.0):
        raise TimeoutError("MAVROS/飞控连接或 set_message_interval 服务不可用")
    if not node.stream_client.wait_for_service(timeout_sec=10.0):
        raise TimeoutError("MAVROS set_stream_rate 服务不可用")
    stream_request = StreamRate.Request()
    stream_request.stream_id = StreamRate.Request.STREAM_POSITION
    stream_request.message_rate = 50
    stream_request.on_off = True
    # StreamRate has an empty response: service success is not an FCU ACK.
    request_with_retry(node, node.stream_client, stream_request, "STREAM_POSITION 50 Hz")
    # ATTITUDE (30) supplies real roll/pitch, LOCAL_POSITION_NED (32) feeds
    # local position, and GLOBAL_POSITION_INT (33) carries GPS plus heading.
    for message_id in (30, 32, 33):
        request = MessageInterval.Request()
        request.message_id = message_id; request.message_rate = 50.0
        request_with_retry(node, node.interval_client, request,
                           f"MAVLink 消息 {message_id} 50 Hz")


def main():
    if len(sys.argv) != 2: print("usage: route_preflight.py ROUTE.yaml", file=sys.stderr); return 2
    route = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
    p1 = (float(route["point_1"]["latitude_deg"]), float(route["point_1"]["longitude_deg"]))
    p2 = (float(route["point_2"]["latitude_deg"]), float(route["point_2"]["longitude_deg"]))
    # Recompute on every flight from the calibrated GPS baseline.  The saved
    # heading_deg is retained for compatibility/reference only.
    heading = bearing_deg(p1, p2)
    rclpy.init(); node = Stream()
    try:
        configure_mavlink_rates(node)
        print("等待起飞前最终 GPS 定位和罗盘数据...", flush=True)
        deadline = time.monotonic() + 60.0
        while (node.gps is None or node.compass is None or node.attitude_count < 3
               or node.attitude_arrival is None
               or time.monotonic() - node.attitude_arrival > 0.5):
            if time.monotonic() >= deadline:
                print("错误：等待 GPS/罗盘/有效姿态数据超过 60 秒。", file=sys.stderr)
                return 7
            rclpy.spin_once(node, timeout_sec=0.2)
        print('[OK] 已接收到有效飞控姿态，横滚/俯仰投影补偿有数据输入。')
        drift = distance_m(p1, node.gps)
        print(f"最终 GPS：{node.gps[0]:.8f}, {node.gps[1]:.8f}；相对标定点偏移 {drift:.2f} m")
        if drift > 4.0:
            print("警告：GPS 相对标定点偏移超过 4 m；按当前场地配置继续，不等待确认。",
                  file=sys.stderr, flush=True)
        elif drift > 1.0:
            print("提示：GPS 相对标定点偏移超过 1 m；按当前场地配置继续，不等待确认。",
                  file=sys.stderr, flush=True)
        else:
            print("[OK] GPS 相对标定点偏移在 1 m 内。", flush=True)
        delta = abs((node.compass - heading + 180.0) % 360.0 - 180.0)
        print(f"罗盘航向 {node.compass:.2f}°，路线航向 {heading:.2f}°，偏差 {delta:.2f}°")
        if delta > 30.0:
            print("警告：罗盘偏差超过 30°；罗盘仅作提示，按路线航向继续，不等待确认。",
                  file=sys.stderr, flush=True)
        elif delta > 7.0:
            print("提示：罗盘偏差超过 7°；罗盘仅作提示，按路线航向继续，不等待确认。",
                  file=sys.stderr, flush=True)
        else:
            print("[OK] 罗盘与路线航向偏差在 7° 内。", flush=True)
        print(f"ROUTE_HEADING_DEG={heading:.8f}")
        return 0
    except KeyboardInterrupt:
        print("已取消起飞前检查。", file=sys.stderr)
        return 130
    except (TimeoutError, RuntimeError) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 8
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__": raise SystemExit(main())
