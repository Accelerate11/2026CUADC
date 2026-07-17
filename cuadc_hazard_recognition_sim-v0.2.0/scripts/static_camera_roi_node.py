#!/usr/bin/env python3
"""Simulation-only top-view renderer to equivalent onboard D435i ROI."""

from pathlib import Path
import math
import threading

import cv2
from gz.msgs10.image_pb2 import Image as GzImage, RGB_INT8
from gz.transport13 import Node as GzNode
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import yaml


class StaticCameraRoiNode(Node):
    def __init__(self):
        super().__init__("static_camera_roi_node")
        self.declare_parameter("generated_scene_path", "")
        self.declare_parameter("gz_topic", "/hazard_d435i/image")
        self.declare_parameter("ros_topic", "/uav/d435i/color/image")
        self.declare_parameter("source_horizontal_fov", 1.5)
        self.declare_parameter("crop_width_m", 1.8)
        self.declare_parameter("crop_height_m", 1.02)
        self.declare_parameter("output_width", 848)
        self.declare_parameter("output_height", 480)
        self.declare_parameter("local_to_world_yaw_deg", 0.0)

        gp = self.get_parameter
        scene_path = Path(str(gp("generated_scene_path").value))
        scene = yaml.safe_load(scene_path.read_text())
        camera = scene["camera"]
        vehicle = scene["vehicle"]
        self.camera_x = float(camera["x"])
        self.camera_y = float(camera["y"])
        self.camera_z = float(camera["z"])
        self.spawn_x = float(vehicle["pose"][0])
        self.spawn_y = float(vehicle["pose"][1])
        self.source_hfov = float(gp("source_horizontal_fov").value)
        self.crop_width_m = float(gp("crop_width_m").value)
        self.crop_height_m = float(gp("crop_height_m").value)
        self.output_width = int(gp("output_width").value)
        self.output_height = int(gp("output_height").value)
        self.yaw = math.radians(float(gp("local_to_world_yaw_deg").value))
        self.lock = threading.Lock()
        self.local_xy = None
        self.frames = 0
        self.background_image = None
        self.source_frames = 0

        self.publisher = self.create_publisher(Image, str(gp("ros_topic").value), qos_profile_sensor_data)
        self.create_subscription(Odometry, "/mavros/local_position/odom", self.odom_callback, qos_profile_sensor_data)
        self.gz_node = GzNode()
        self.gz_topic = str(gp("gz_topic").value)
        self.gz_node.subscribe(GzImage, self.gz_topic, self.image_callback)
        self.get_logger().info(
            f"Simulation ROI camera: {self.gz_topic} -> {gp('ros_topic').value}, "
            f"spawn=({self.spawn_x:.3f},{self.spawn_y:.3f}), crop={self.crop_width_m:.2f}x{self.crop_height_m:.2f}m"
        )

    def odom_callback(self, message: Odometry):
        with self.lock:
            self.local_xy = (message.pose.pose.position.x, message.pose.pose.position.y)

    def image_callback(self, source: GzImage):
        if source.pixel_format_type != RGB_INT8:
            return
        source_image = np.frombuffer(source.data, dtype=np.uint8).reshape(source.height, source.width, 3)
        self.source_frames += 1
        if self.background_image is None:
            if self.source_frames < 30:
                return
            self.background_image = source_image.copy()
            self.get_logger().info(
                f"Cached clean Gazebo background {source.width}x{source.height} at frame {self.source_frames}"
            )
        with self.lock:
            local_xy = self.local_xy
        if local_xy is None:
            return
        lx, ly = local_xy
        dx = math.cos(self.yaw) * lx - math.sin(self.yaw) * ly
        dy = math.sin(self.yaw) * lx + math.cos(self.yaw) * ly
        world_x, world_y = self.spawn_x + dx, self.spawn_y + dy

        source_image = self.background_image
        view_width_m = 2.0 * self.camera_z * math.tan(self.source_hfov * 0.5)
        pixels_per_m = source.width / view_width_m
        # Downward camera at roll=0,pitch=+90,yaw=0: +world y maps left,
        # +world x maps up in the rendered image.
        center_u = source.width * 0.5 - (world_y - self.camera_y) * pixels_per_m
        center_v = source.height * 0.5 - (world_x - self.camera_x) * pixels_per_m
        crop_w = max(2, int(round(self.crop_width_m * pixels_per_m)))
        crop_h = max(2, int(round(self.crop_height_m * pixels_per_m)))
        x1, y1 = int(round(center_u - crop_w * 0.5)), int(round(center_v - crop_h * 0.5))
        x2, y2 = x1 + crop_w, y1 + crop_h

        pad_left, pad_top = max(0, -x1), max(0, -y1)
        pad_right, pad_bottom = max(0, x2 - source.width), max(0, y2 - source.height)
        padded = cv2.copyMakeBorder(
            source_image, pad_top, pad_bottom, pad_left, pad_right,
            cv2.BORDER_CONSTANT, value=(125, 130, 125),
        )
        x1, y1 = x1 + pad_left, y1 + pad_top
        roi = padded[y1:y1 + crop_h, x1:x1 + crop_w]
        output = cv2.resize(roi, (self.output_width, self.output_height), interpolation=cv2.INTER_LINEAR)

        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "d435i_color_optical_frame"
        message.height = self.output_height
        message.width = self.output_width
        message.encoding = "rgb8"
        message.is_bigendian = False
        message.step = self.output_width * 3
        message.data = output.tobytes()
        self.publisher.publish(message)
        self.frames += 1
        if self.frames == 1:
            self.get_logger().info(
                f"First ROI at world=({world_x:.3f},{world_y:.3f}), source={source.width}x{source.height}"
            )


def main():
    rclpy.init()
    node = StaticCameraRoiNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
