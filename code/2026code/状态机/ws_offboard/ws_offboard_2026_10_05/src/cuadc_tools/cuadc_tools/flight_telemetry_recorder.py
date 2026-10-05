#!/usr/bin/env python3
"""低频、可及时落盘的 MAVROS 遥测记录，用于飞行后分析。"""

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64, String


FIELDS = (
    "timestamp_s", "absolute_time", "record_reason", "mission_state",
    "fcu_mode", "connected", "armed", "latitude_deg", "longitude_deg",
    "gps_altitude_m", "battery_voltage_v", "battery_current_a",
    "battery_remaining_pct", "velocity_x_m_s", "velocity_y_m_s",
    "velocity_z_m_s", "horizontal_speed_m_s", "vertical_speed_m_s",
    "roll_deg", "pitch_deg", "yaw_enu_deg", "heading_deg",
    "compass_heading_deg",
)


def quaternion_to_rpy(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class FlightTelemetryRecorder(Node):
    def __init__(self, output_path: Path, rate_hz: float):
        super().__init__("flight_telemetry_recorder")
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = output_path.open("w", newline="", encoding="utf-8", buffering=1)
        self._writer = csv.DictWriter(self._file, fieldnames=FIELDS)
        self._writer.writeheader()
        self._values = {field: "" for field in FIELDS}

        self.create_subscription(State, "/mavros/state", self._state_callback, 10)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._gps_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Float64, "/mavros/global_position/compass_hdg", self._compass_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            BatteryState, "/mavros/battery", self._battery_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, "/mavros/local_position/velocity_local",
            self._velocity_callback, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self._pose_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            String, "/cuadc/mission_state", self._mission_state_callback, 20)
        self.create_timer(1.0 / max(0.5, rate_hz), lambda: self._record("periodic"))
        self.get_logger().info(
            f"Recording structured flight telemetry at {rate_hz:g} Hz: {output_path}")

    def _state_callback(self, message: State):
        self._values.update(
            fcu_mode=message.mode,
            connected=str(bool(message.connected)).lower(),
            armed=str(bool(message.armed)).lower(),
        )

    def _gps_callback(self, message: NavSatFix):
        self._values.update(
            latitude_deg=message.latitude,
            longitude_deg=message.longitude,
            gps_altitude_m=message.altitude,
        )

    def _compass_callback(self, message: Float64):
        if math.isfinite(message.data):
            self._values["compass_heading_deg"] = message.data % 360.0

    def _battery_callback(self, message: BatteryState):
        remaining = message.percentage * 100.0 if math.isfinite(message.percentage) else ""
        self._values.update(
            battery_voltage_v=message.voltage if math.isfinite(message.voltage) else "",
            battery_current_a=message.current if math.isfinite(message.current) else "",
            battery_remaining_pct=remaining,
        )

    def _velocity_callback(self, message: TwistStamped):
        velocity = message.twist.linear
        self._values.update(
            velocity_x_m_s=velocity.x,
            velocity_y_m_s=velocity.y,
            velocity_z_m_s=velocity.z,
            horizontal_speed_m_s=math.hypot(velocity.x, velocity.y),
            vertical_speed_m_s=velocity.z,
        )

    def _pose_callback(self, message: PoseStamped):
        orientation = message.pose.orientation
        roll, pitch, yaw = quaternion_to_rpy(
            orientation.x, orientation.y, orientation.z, orientation.w)
        self._values.update(
            roll_deg=roll,
            pitch_deg=pitch,
            yaw_enu_deg=yaw,
            heading_deg=(90.0 - yaw) % 360.0,
        )

    def _mission_state_callback(self, message: String):
        if message.data != self._values["mission_state"]:
            self._values["mission_state"] = message.data
            self._record("state_change")

    def _record(self, reason: str):
        timestamp = time.time()
        row = dict(self._values)
        row.update(
            timestamp_s=f"{timestamp:.6f}",
            absolute_time=time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(timestamp)),
            record_reason=reason,
        )
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        if not self._file.closed:
            self._record("shutdown")
            self._file.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = FlightTelemetryRecorder(args.output, args.rate_hz)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch 与终端进程组可能几乎同时发送 SIGINT；
        # 关闭流程保持幂等，避免正常中断任务时
        # 将 CSV 记录节点误报为进程崩溃。
        try:
            node.close()
        except KeyboardInterrupt:
            pass
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass
        except Exception:
            pass


if __name__ == "__main__":
    main()
