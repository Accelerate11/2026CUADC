#!/usr/bin/env python3
"""Intentionally empty detector template.

Replace only `detect_targets_body_frd()` or ignore this file and publish the
same PoseArray contract from your existing vision node.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

import math
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


@dataclass
class Target3D:
    x_fwd_m: float
    y_right_m: float
    z_down_m: float
    confidence: float = 1.0
    size_m: float = 0.0
    depth_metric_m: float = 0.0


class VisionProviderTemplate(Node):
    def __init__(self) -> None:
        super().__init__("vision_provider_template")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("output_topic", "/vision_servo/targets_body")
        self.declare_parameter("frame_id", "fcu_body_frd")
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(PoseArray, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)
        self.get_logger().warning(
            "Vision provider template is intentionally empty. It will publish heartbeat frames "
            "with poses=[] until detect_targets_body_frd() is implemented."
        )

    def detect_targets_body_frd(self, image_bgr) -> Iterable[Target3D]:
        """TODO: implement your own detector/localizer here.

        Return zero or more targets already expressed in FCU body FRD:
          +X forward, +Y right, +Z down, metres.

        For a camera-frame detector, transform your 3D point with:
          p_body = R_body_camera @ p_camera + t_body_camera
        before returning it.
        """
        del image_bgr
        return []

    def _image_cb(self, message: Image) -> None:
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            targets = list(self.detect_targets_body_frd(image_bgr))
        except Exception as error:
            self.get_logger().error(f"Vision provider error: {error}")
            targets = []

        array = PoseArray()
        array.header = message.header
        array.header.frame_id = self.frame_id
        for target in targets:
            values = (
                target.x_fwd_m,
                target.y_right_m,
                target.z_down_m,
                target.confidence,
                target.size_m,
                target.depth_metric_m,
            )
            if not all(math.isfinite(float(value)) for value in values):
                self.get_logger().warning("Skipping non-finite target")
                continue
            pose = Pose()
            pose.position.x = float(target.x_fwd_m)
            pose.position.y = float(target.y_right_m)
            pose.position.z = float(target.z_down_m)
            pose.orientation.x = float(target.size_m)
            pose.orientation.y = float(max(0.0, min(1.0, target.confidence)))
            pose.orientation.z = float(target.depth_metric_m)
            pose.orientation.w = 1.0
            array.poses.append(pose)
        self.publisher.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionProviderTemplate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
