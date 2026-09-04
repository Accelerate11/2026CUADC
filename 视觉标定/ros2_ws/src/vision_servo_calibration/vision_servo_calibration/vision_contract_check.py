#!/usr/bin/env python3
"""Runtime checker for third-party vision providers."""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class VisionContractCheck(Node):
    def __init__(self) -> None:
        super().__init__("vision_contract_check")
        self.declare_parameter("detections_topic", "/vision_servo/targets_body")
        self.declare_parameter("expected_frame_id", "fcu_body_frd")
        self.declare_parameter("duration_s", 5.0)
        self.declare_parameter("require_contract_sentinel", True)
        self.topic = str(self.get_parameter("detections_topic").value)
        self.expected_frame_id = str(self.get_parameter("expected_frame_id").value)
        self.duration_s = float(self.get_parameter("duration_s").value)
        self.require_sentinel = bool(self.get_parameter("require_contract_sentinel").value)
        self.started = time.monotonic()
        self.messages = 0
        self.detections = 0
        self.errors = []
        self.create_subscription(PoseArray, self.topic, self._callback, qos_profile_sensor_data)
        self.finished = False
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(f"Checking {self.topic} for {self.duration_s:.1f}s")

    def _error(self, text: str) -> None:
        if text not in self.errors:
            self.errors.append(text)
            self.get_logger().error(text)

    def _callback(self, message: PoseArray) -> None:
        self.messages += 1
        if message.header.stamp.sec == 0 and message.header.stamp.nanosec == 0:
            self._error("header.stamp is zero")
        if message.header.frame_id != self.expected_frame_id:
            self._error(
                f"frame_id={message.header.frame_id!r}, expected {self.expected_frame_id!r}"
            )
        for index, pose in enumerate(message.poses):
            self.detections += 1
            xyz = (pose.position.x, pose.position.y, pose.position.z)
            if not all(math.isfinite(float(value)) for value in xyz):
                self._error(f"detection {index}: non-finite position")
            confidence = float(pose.orientation.y)
            if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                self._error(f"detection {index}: confidence must be in [0,1]")
            if self.require_sentinel and abs(float(pose.orientation.w) - 1.0) > 1e-6:
                self._error(f"detection {index}: orientation.w contract sentinel must be 1.0")

    def _tick(self) -> None:
        if self.finished or time.monotonic() - self.started < self.duration_s:
            return
        self.finished = True
        if self.messages == 0:
            self._error("no PoseArray messages received")
        if self.errors:
            self.get_logger().error(
                f"CONTRACT FAIL: messages={self.messages}, detections={self.detections}, errors={len(self.errors)}"
            )
        else:
            self.get_logger().info(
                f"CONTRACT PASS: messages={self.messages}, detections={self.detections}"
            )
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionContractCheck()
    status = 0
    try:
        rclpy.spin(node)
        status = 1 if node.errors or node.messages == 0 else 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(status)


if __name__ == "__main__":
    main()
