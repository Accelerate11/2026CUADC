#!/usr/bin/env python3
"""Minimal Gazebo Harmonic (Transport 13) to ROS 2 image bridge."""

from __future__ import annotations

from gz.msgs10.image_pb2 import (
    Image as GzImage,
    RGB_INT8,
    BGR_INT8,
    RGBA_INT8,
    BGRA_INT8,
    L_INT8,
    L_INT16,
)
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


ENCODINGS = {
    RGB_INT8: "rgb8",
    BGR_INT8: "bgr8",
    RGBA_INT8: "rgba8",
    BGRA_INT8: "bgra8",
    L_INT8: "mono8",
    L_INT16: "mono16",
}


class GzImageBridge(Node):
    def __init__(self) -> None:
        super().__init__("gz_harmonic_image_bridge")
        self.declare_parameter("gz_topic", "/d435i/image")
        self.declare_parameter("ros_topic", "/uav/d435i/color/image")
        self.declare_parameter("frame_id", "d435i_color_optical_frame")
        self.gz_topic = str(self.get_parameter("gz_topic").value)
        self.ros_topic = str(self.get_parameter("ros_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        qos = rclpy.qos.QoSProfile(
            depth=2,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Image, self.ros_topic, qos)
        self.gz_node = GzNode()
        self.gz_node.subscribe(GzImage, self.gz_topic, self.image_cb)
        self.received = 0
        self.get_logger().info(f"Gazebo Transport 13 bridge: {self.gz_topic} -> {self.ros_topic}")

    def image_cb(self, source: GzImage) -> None:
        encoding = ENCODINGS.get(source.pixel_format_type)
        if encoding is None:
            if self.received == 0:
                self.get_logger().error(f"Unsupported Gazebo pixel format {source.pixel_format_type}")
            return
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.height = source.height
        message.width = source.width
        message.encoding = encoding
        message.is_bigendian = False
        message.step = source.step
        message.data = source.data
        self.publisher.publish(message)
        self.received += 1
        if self.received == 1:
            self.get_logger().info(
                f"First frame: {message.width}x{message.height} {message.encoding}, step={message.step}"
            )


def main() -> None:
    rclpy.init()
    node = GzImageBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
