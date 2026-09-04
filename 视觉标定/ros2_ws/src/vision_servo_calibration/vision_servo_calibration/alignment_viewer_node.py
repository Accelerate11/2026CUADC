#!/usr/bin/env python3
"""Independent crosshair viewer for any compatible vision provider."""
from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .projection import (
    project_body_point,
    unpack_payload_offsets,
    validate_rotation_matrix,
)


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class AlignmentViewer(Node):
    def __init__(self) -> None:
        super().__init__("alignment_viewer")
        self.declare_parameter("detections_topic", "/vision_servo/targets_body")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("expected_frame_id", "fcu_body_frd")
        self.declare_parameter("camera_to_body_rotation", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("camera_to_body_translation_m", [0.0, 0.0, 0.0])
        self.declare_parameter("payload_release_offsets_body_m", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("target_height_m", 0.0)
        self.declare_parameter("sync_tolerance_s", 0.35)
        self.declare_parameter("min_camera_z_m", 0.05)
        self.declare_parameter("live_view_enabled", True)
        self.declare_parameter("annotated_image_topic", "/vision_servo/alignment/image")
        self.declare_parameter("detection_csv_path", "")
        self.declare_parameter("video_path", "")
        self.declare_parameter("video_fps", 10.0)

        self.detections_topic = str(self.get_parameter("detections_topic").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.expected_frame_id = str(self.get_parameter("expected_frame_id").value)
        self.rotation = validate_rotation_matrix(self.get_parameter("camera_to_body_rotation").value)
        self.translation = np.asarray(self.get_parameter("camera_to_body_translation_m").value, dtype=np.float64)
        if self.translation.shape != (3,) or not np.all(np.isfinite(self.translation)):
            raise ValueError("camera_to_body_translation_m must be a finite 3-vector")
        self.payload_offsets = unpack_payload_offsets(self.get_parameter("payload_release_offsets_body_m").value)
        self.target_height_m = float(self.get_parameter("target_height_m").value)
        self.sync_tolerance_s = float(self.get_parameter("sync_tolerance_s").value)
        self.min_camera_z_m = float(self.get_parameter("min_camera_z_m").value)
        self.live_view_enabled = bool(self.get_parameter("live_view_enabled").value)
        if not math.isfinite(self.target_height_m) or self.target_height_m < 0.0:
            raise ValueError("target_height_m must be finite and non-negative")
        if not math.isfinite(self.sync_tolerance_s) or self.sync_tolerance_s <= 0.0:
            raise ValueError("sync_tolerance_s must be positive")

        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.latest_detections: Optional[PoseArray] = None
        self.latest_detection_stamp_ns = 0
        self.warned_frame = False

        self.csv_file = None
        self.csv_writer = None
        self._open_csv(str(self.get_parameter("detection_csv_path").value))

        self.video_path = str(self.get_parameter("video_path").value).strip()
        self.video_fps = float(self.get_parameter("video_fps").value)
        self.video_writer = None

        self.annotated_pub = self.create_publisher(
            Image, str(self.get_parameter("annotated_image_topic").value), qos_profile_sensor_data
        )
        self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, qos_profile_sensor_data)
        self.create_subscription(PoseArray, self.detections_topic, self._detections_cb, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"Alignment viewer ready: image={self.image_topic}, detections={self.detections_topic}, "
            f"payloads={len(self.payload_offsets)}"
        )

    def _open_csv(self, path_text: str) -> None:
        if not path_text.strip():
            return
        path = Path(path_text).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "capture_time_ns",
            "detection_index",
            "body_frd_x_m",
            "body_frd_y_m",
            "body_frd_z_m",
            "diameter_m",
            "confidence",
            "depth_metric_m",
            "contract_version",
            "status",
        ])
        self.csv_file.flush()
        self.get_logger().info(f"Detection CSV: {path}")

    def _camera_info_cb(self, message: CameraInfo) -> None:
        if message.k[0] <= 0.0 or message.k[4] <= 0.0:
            self.get_logger().warning("Ignoring CameraInfo with invalid focal length")
            return
        self.camera_info = message

    def _detections_cb(self, message: PoseArray) -> None:
        self.latest_detections = message
        self.latest_detection_stamp_ns = stamp_ns(message.header.stamp)
        if message.header.frame_id != self.expected_frame_id and not self.warned_frame:
            self.warned_frame = True
            self.get_logger().warning(
                f"Vision frame_id={message.header.frame_id!r}, expected {self.expected_frame_id!r}. "
                "Coordinates will still be drawn as configured FRD values."
            )
        if self.csv_writer is not None:
            for index, pose in enumerate(message.poses):
                values = (
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
                status = "accepted" if all(math.isfinite(float(v)) for v in values) else "invalid"
                self.csv_writer.writerow([
                    self.latest_detection_stamp_ns,
                    index,
                    f"{pose.position.x:.6f}",
                    f"{pose.position.y:.6f}",
                    f"{pose.position.z:.6f}",
                    f"{pose.orientation.x:.6f}",
                    f"{pose.orientation.y:.6f}",
                    f"{pose.orientation.z:.6f}",
                    f"{pose.orientation.w:.3f}",
                    status,
                ])
            self.csv_file.flush()

    def _project(self, body_xyz) -> Optional[tuple[int, int]]:
        if self.camera_info is None:
            return None
        k = self.camera_info.k
        pixel = project_body_point(
            body_xyz,
            self.rotation,
            self.translation,
            float(k[0]),
            float(k[4]),
            float(k[2]),
            float(k[5]),
            self.min_camera_z_m,
        )
        if pixel is None:
            return None
        return int(round(pixel[0])), int(round(pixel[1]))

    @staticmethod
    def _draw_text(image, text, origin, scale=0.50, color=(255, 255, 255)) -> None:
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    def _draw_axes(self, image) -> None:
        origin = np.asarray([92.0, float(image.shape[0] - 76)], dtype=np.float64)
        axes = (
            ("+X FWD", np.array([1.0, 0.0, 0.0]), (0, 255, 255)),
            ("+Y RIGHT", np.array([0.0, 1.0, 0.0]), (255, 255, 0)),
        )
        for label, body_axis, color in axes:
            camera_axis = self.rotation.T @ body_axis
            direction = np.asarray([camera_axis[0], camera_axis[1]])
            norm = float(np.linalg.norm(direction))
            if norm < 1e-6:
                continue
            endpoint = origin + 54.0 * direction / norm
            start = tuple(np.rint(origin).astype(int))
            end = tuple(np.rint(endpoint).astype(int))
            cv2.arrowedLine(image, start, end, (0, 0, 0), 5, cv2.LINE_AA, tipLength=0.20)
            cv2.arrowedLine(image, start, end, color, 2, cv2.LINE_AA, tipLength=0.20)
            self._draw_text(image, label, (end[0] + 6, end[1] - 5), 0.45, color)
        self._draw_text(image, "FCU BODY FRD", (20, image.shape[0] - 18), 0.50)

    def _image_cb(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().error(f"cv_bridge failed: {error}")
            return
        display = image.copy()
        self._draw_axes(display)

        image_stamp = stamp_ns(message.header.stamp)
        detections = self.latest_detections
        fresh = detections is not None
        if fresh and image_stamp > 0 and self.latest_detection_stamp_ns > 0:
            age_s = abs(image_stamp - self.latest_detection_stamp_ns) / 1e9
            fresh = age_s <= self.sync_tolerance_s
        if self.camera_info is None:
            self._draw_text(display, "WAITING CameraInfo", (20, 32), 0.60, (0, 0, 255))
        elif not fresh:
            self._draw_text(display, "NO FRESH TARGET FRAME", (20, 32), 0.60, (0, 0, 255))
        else:
            self._draw_detections(display, detections)

        out = self.bridge.cv2_to_imgmsg(display, encoding="bgr8")
        out.header = message.header
        self.annotated_pub.publish(out)
        self._write_video(display)

        if self.live_view_enabled:
            cv2.imshow("Vision Servo Alignment", display)
            cv2.waitKey(1)

    def _draw_detections(self, image, message: PoseArray) -> None:
        payload_colors = [
            (0, 165, 255),
            (255, 0, 255),
            (255, 128, 0),
            (0, 200, 255),
            (255, 255, 0),
            (255, 0, 128),
        ]
        if not message.poses:
            self._draw_text(image, "VISION OK / 0 TARGETS", (20, 32), 0.58, (0, 220, 0))
            return
        self._draw_text(image, f"VISION OK / {len(message.poses)} TARGET(S)", (20, 32), 0.58, (0, 220, 0))
        for target_index, pose in enumerate(message.poses):
            body = np.asarray([pose.position.x, pose.position.y, pose.position.z], dtype=np.float64)
            if not np.all(np.isfinite(body)):
                continue
            target_pixel = self._project(body)
            if target_pixel is not None and 0 <= target_pixel[0] < image.shape[1] and 0 <= target_pixel[1] < image.shape[0]:
                color = (57, 255, 20)
                cv2.drawMarker(image, target_pixel, color, cv2.MARKER_CROSS, 24, 3, cv2.LINE_AA)
                self._draw_text(image, f"TARGET {target_index + 1}", (target_pixel[0] + 12, target_pixel[1] + 22), 0.48, color)

            plane_z = float(body[2] - self.target_height_m)
            for payload_index, offset in enumerate(self.payload_offsets):
                payload_plane = np.asarray([offset[0], offset[1], plane_z], dtype=np.float64)
                payload_pixel = self._project(payload_plane)
                if payload_pixel is None:
                    continue
                if not (0 <= payload_pixel[0] < image.shape[1] and 0 <= payload_pixel[1] < image.shape[0]):
                    continue
                color = payload_colors[payload_index % len(payload_colors)]
                cv2.drawMarker(image, payload_pixel, color, cv2.MARKER_CROSS, 28, 3, cv2.LINE_AA)
                self._draw_text(
                    image,
                    f"P{payload_index + 1}",
                    (payload_pixel[0] + 12, payload_pixel[1] - 10),
                    0.50,
                    color,
                )
                error_forward = body[0] - offset[0]
                error_right = body[1] - offset[1]
                line_y = 60 + 24 * (target_index * len(self.payload_offsets) + payload_index)
                self._draw_text(
                    image,
                    f"T{target_index + 1}/P{payload_index + 1}: dF={error_forward:+.3f}m dR={error_right:+.3f}m",
                    (20, line_y),
                    0.48,
                    color,
                )

    def _write_video(self, image) -> None:
        if not self.video_path:
            return
        if self.video_writer is None:
            path = Path(self.video_path).expanduser().resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self.video_writer = cv2.VideoWriter(
                str(path), fourcc, max(1.0, self.video_fps), (image.shape[1], image.shape[0])
            )
            if not self.video_writer.isOpened():
                self.get_logger().error(f"Unable to open video writer: {path}")
                self.video_writer = None
                self.video_path = ""
                return
            self.get_logger().info(f"Annotated video: {path}")
        self.video_writer.write(image)

    def close(self) -> None:
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        if self.live_view_enabled:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlignmentViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
