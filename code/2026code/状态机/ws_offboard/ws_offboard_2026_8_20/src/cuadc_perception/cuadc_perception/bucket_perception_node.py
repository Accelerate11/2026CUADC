#!/usr/bin/env python3
"""Detect drop buckets from RealSense ROS topics and publish body-FLU poses.

This node consumes only aligned color/depth frames from the separately launched
RealSense driver. It has no dangerous-object or reconnaissance input. An empty
PoseArray is published for every synchronized frame when no candidate passes
the quality gates, preserving the mission heartbeat contract.
"""

from __future__ import annotations

from collections import Counter
import hashlib
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO

from . import bucket_algorithms as algorithms


TRUSTED_MODEL_SHA256 = (
    "061b67ace71d5f036f7003a3699640dc0a8522e3c3162116884b67b552cd87bf"
)
DEFAULT_CAMERA_TO_BODY_R = [
    0.0, -1.0, 0.0,
    -1.0, 0.0, 0.0,
    0.0, 0.0, -1.0,
]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_rotation(values: Sequence[float], tolerance: float) -> np.ndarray:
    rotation = np.asarray(values, dtype=np.float64)
    if rotation.size != 9 or not np.all(np.isfinite(rotation)):
        raise ValueError("camera_to_body_rotation must contain 9 finite values")
    rotation = rotation.reshape(3, 3)
    if tolerance <= 0.0 or tolerance > 0.01:
        raise ValueError("extrinsic_orthogonality_tolerance must be in (0, 0.01]")
    error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3), ord=np.inf))
    determinant = float(np.linalg.det(rotation))
    if error > tolerance or abs(determinant - 1.0) > tolerance:
        raise ValueError(
            f"camera extrinsic is not a proper rotation: error={error}, det={determinant}"
        )
    return rotation


def quality_gate(metrics: Mapping[str, float], limits: Mapping[str, float]) -> str:
    checks = (
        (metrics["confidence"] < limits["min_confidence"], "confidence_below_min"),
        (metrics["depth_m"] < limits["min_depth_m"], "depth_below_min"),
        (metrics["depth_m"] > limits["max_depth_m"], "depth_above_max"),
        (
            metrics["depth_valid_ratio"] < limits["min_depth_valid_ratio"],
            "depth_valid_ratio_below_min",
        ),
        (metrics["depth_iqr_m"] > limits["max_depth_iqr_m"], "depth_iqr_above_max"),
        (metrics["axis_ratio"] > limits["max_axis_ratio"], "axis_ratio_above_max"),
        (metrics["diameter_m"] < limits["min_diameter_m"], "diameter_below_min"),
        (metrics["diameter_m"] > limits["max_diameter_m"], "diameter_above_max"),
        (metrics["mask_area_px"] < limits["min_mask_area_px"], "mask_area_below_min"),
    )
    if not all(math.isfinite(float(value)) for value in metrics.values()):
        return "non_finite_metric"
    for failed, reason in checks:
        if failed:
            return reason
    if metrics["depth_valid_ratio"] > 1.0 or metrics["axis_ratio"] < 1.0:
        return "invalid_metric_range"
    return "ok"


class BucketPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("bucket_perception")
        self.bridge = CvBridge()
        self.camera_info: Optional[CameraInfo] = None
        self.frame_count = 0
        self.accepted_total = 0
        self.rejected_total = 0
        self.consecutive_errors = 0
        self.last_rejections: Counter[str] = Counter()
        self.fatal = False

        self._declare_parameters()
        self._load_parameters()
        self._load_model()

        self.pose_pub = self.create_publisher(
            PoseArray, self.output_topic, qos_profile_sensor_data
        )
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.color_sub = Subscriber(
            self, Image, self.color_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_sub = Subscriber(
            self, Image, self.depth_topic, qos_profile=qos_profile_sensor_data
        )
        self.synchronizer = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=self.sync_queue_size,
            slop=self.max_input_skew_s,
            allow_headerless=False,
        )
        self.synchronizer.registerCallback(self._frame_callback)
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self.get_logger().info(
            "Bucket perception ready: color=%s depth=%s model=%s camera=%s"
            % (self.color_topic, self.depth_topic, self.model_path, self.camera_serial)
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "model_path": "",
            "model_sha256": TRUSTED_MODEL_SHA256,
            "camera_serial": "",
            "color_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "output_topic": "/perception/drop_buckets_body",
            "frame_id": "base_link",
            "sync_queue_size": 3,
            "max_input_skew_s": 0.08,
            "depth_scale_m_per_unit": 0.001,
            "target_class": 0,
            "confidence_threshold": 0.25,
            "nms_iou_threshold": 0.01,
            "inference_image_size": 640,
            "device": "0",
            "half": True,
            "depth_mode": "rim_direct",
            "bucket_height_m": 0.30,
            "rim_band_thickness_pixels": 11,
            "depth_filter_size": 5,
            "duplicate_mask_iou_threshold": 0.20,
            "ground_ring_inner_scale": 1.15,
            "ground_ring_outer_scale": 1.75,
            "camera_to_body_rotation": DEFAULT_CAMERA_TO_BODY_R,
            "camera_to_body_translation_m": [0.0, 0.0, 0.0],
            "extrinsic_orthogonality_tolerance": 0.001,
            "min_quality_confidence": 0.25,
            "min_depth_m": 0.15,
            "max_depth_m": 6.0,
            "min_depth_valid_ratio": 0.35,
            "max_depth_iqr_m": 0.20,
            "max_ellipse_axis_ratio": 2.50,
            "min_diameter_m": 0.08,
            "max_diameter_m": 0.35,
            "min_mask_area_px": 100.0,
            "max_consecutive_frame_errors": 5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        share = Path(get_package_share_directory("cuadc_perception"))
        configured_model = str(self.get_parameter("model_path").value).strip()
        self.model_path = Path(configured_model).expanduser().resolve() if configured_model else (
            share / "models" / "basket_v3.pt"
        ).resolve()
        self.expected_model_hash = str(self.get_parameter("model_sha256").value).lower()
        self.camera_serial = str(self.get_parameter("camera_serial").value).strip()
        self.color_topic = str(self.get_parameter("color_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.max_input_skew_s = float(self.get_parameter("max_input_skew_s").value)
        self.depth_scale = float(self.get_parameter("depth_scale_m_per_unit").value)
        self.target_class = int(self.get_parameter("target_class").value)
        self.confidence = float(self.get_parameter("confidence_threshold").value)
        self.nms_iou = float(self.get_parameter("nms_iou_threshold").value)
        self.image_size = int(self.get_parameter("inference_image_size").value)
        self.device = str(self.get_parameter("device").value).strip() or None
        self.half = bool(self.get_parameter("half").value)
        self.depth_mode = str(self.get_parameter("depth_mode").value)
        self.bucket_height = float(self.get_parameter("bucket_height_m").value)
        self.rim_band = int(self.get_parameter("rim_band_thickness_pixels").value)
        self.max_errors = int(self.get_parameter("max_consecutive_frame_errors").value)
        self.depth_filter_size = int(self.get_parameter("depth_filter_size").value)
        self.duplicate_mask_iou = float(
            self.get_parameter("duplicate_mask_iou_threshold").value
        )
        self.ground_ring_inner_scale = float(
            self.get_parameter("ground_ring_inner_scale").value
        )
        self.ground_ring_outer_scale = float(
            self.get_parameter("ground_ring_outer_scale").value
        )
        tolerance = float(self.get_parameter("extrinsic_orthogonality_tolerance").value)
        self.rotation = validate_rotation(
            self.get_parameter("camera_to_body_rotation").value, tolerance
        )
        self.translation = np.asarray(
            self.get_parameter("camera_to_body_translation_m").value,
            dtype=np.float64,
        )
        self.limits = {
            "min_confidence": float(self.get_parameter("min_quality_confidence").value),
            "min_depth_m": float(self.get_parameter("min_depth_m").value),
            "max_depth_m": float(self.get_parameter("max_depth_m").value),
            "min_depth_valid_ratio": float(
                self.get_parameter("min_depth_valid_ratio").value
            ),
            "max_depth_iqr_m": float(self.get_parameter("max_depth_iqr_m").value),
            "max_axis_ratio": float(
                self.get_parameter("max_ellipse_axis_ratio").value
            ),
            "min_diameter_m": float(self.get_parameter("min_diameter_m").value),
            "max_diameter_m": float(self.get_parameter("max_diameter_m").value),
            "min_mask_area_px": float(self.get_parameter("min_mask_area_px").value),
        }
        if not self.model_path.is_file():
            raise RuntimeError(f"model is missing: {self.model_path}")
        if self.expected_model_hash != TRUSTED_MODEL_SHA256:
            raise RuntimeError("configured model hash is not the trusted release hash")
        if sha256_file(self.model_path) != TRUSTED_MODEL_SHA256:
            raise RuntimeError("basket_v3.pt bytes do not match the trusted SHA-256")
        if not self.camera_serial:
            raise RuntimeError("camera_serial must be supplied by the aircraft profile")
        if self.depth_mode not in ("rim_direct", "wall_minus_height"):
            raise ValueError("depth_mode must be rim_direct or wall_minus_height")
        if self.translation.shape != (3,) or not np.all(np.isfinite(self.translation)):
            raise ValueError("camera_to_body_translation_m must be a finite 3-vector")
        if self.sync_queue_size < 1 or not 0.0 < self.max_input_skew_s <= 0.25:
            raise ValueError("invalid RGB-D synchronization limits")
        if (
            not math.isfinite(self.depth_scale)
            or self.depth_scale <= 0.0
            or self.max_errors < 1
            or self.depth_filter_size < 1
            or self.depth_filter_size % 2 == 0
            or not 0.0 <= self.duplicate_mask_iou <= 1.0
        ):
            raise ValueError("invalid depth scale, filter size, IoU, or frame-error limit")
        if (
            self.target_class < 0
            or not 0.0 <= self.confidence <= 1.0
            or not 0.0 <= self.nms_iou <= 1.0
            or self.image_size < 32
            or self.rim_band < 1
            or not math.isfinite(self.bucket_height)
            or self.bucket_height <= 0.0
        ):
            raise ValueError("invalid inference or bucket geometry parameter")
        if not all(math.isfinite(value) for value in self.limits.values()):
            raise ValueError("quality limits must all be finite")
        if not (
            0.0 <= self.limits["min_confidence"] <= 1.0
            and 0.0 < self.limits["min_depth_m"] < self.limits["max_depth_m"]
            and 0.0 <= self.limits["min_depth_valid_ratio"] <= 1.0
            and self.limits["max_depth_iqr_m"] >= 0.0
            and self.limits["max_axis_ratio"] >= 1.0
            and 0.0 < self.limits["min_diameter_m"]
            < self.limits["max_diameter_m"]
            and self.limits["min_mask_area_px"] > 0.0
        ):
            raise ValueError("quality limits are inconsistent")

    def _load_model(self) -> None:
        self.model = YOLO(str(self.model_path), task="segment")

    def _camera_info_callback(self, message: CameraInfo) -> None:
        if message.width < 1 or message.height < 1 or len(message.k) != 9:
            return
        if message.k[0] <= 0.0 or message.k[4] <= 0.0:
            return
        self.camera_info = message

    @staticmethod
    def _stamp_seconds(message: Image) -> float:
        return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9

    def _frame_callback(self, color_message: Image, depth_message: Image) -> None:
        if self.fatal:
            # Stop the heartbeat so the mission's existing freshness gate and
            # the safety monitor both fail closed after a persistent fault.
            return
        capture_stamp = color_message.header.stamp
        output = PoseArray()
        output.header.stamp = capture_stamp
        output.header.frame_id = self.frame_id
        self.frame_count += 1
        try:
            skew = abs(self._stamp_seconds(color_message) - self._stamp_seconds(depth_message))
            if skew > self.max_input_skew_s:
                raise RuntimeError(f"RGB-D timestamp skew {skew:.3f}s exceeds limit")
            if self.camera_info is None:
                raise RuntimeError("color CameraInfo has not arrived")
            color = self.bridge.imgmsg_to_cv2(color_message, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            depth = np.asarray(depth)
            if color.shape[:2] != depth.shape[:2] or depth.ndim != 2:
                raise RuntimeError(
                    f"aligned RGB-D shape mismatch: color={color.shape}, depth={depth.shape}"
                )
            if (
                int(self.camera_info.width) != color.shape[1]
                or int(self.camera_info.height) != color.shape[0]
            ):
                raise RuntimeError("CameraInfo dimensions do not match the aligned images")
            if depth_message.encoding in ("16UC1", "mono16"):
                depth_scale = self.depth_scale
            elif depth_message.encoding == "32FC1":
                depth_scale = 1.0
            else:
                raise RuntimeError(
                    f"unsupported depth encoding: {depth_message.encoding!r}"
                )
            intrinsics = SimpleNamespace(
                width=int(self.camera_info.width),
                height=int(self.camera_info.height),
                fx=float(self.camera_info.k[0]),
                fy=float(self.camera_info.k[4]),
                ppx=float(self.camera_info.k[2]),
                ppy=float(self.camera_info.k[5]),
            )
            accepted, rejections = self._infer(
                color, depth, intrinsics, depth_scale
            )
            for item in accepted:
                output.poses.append(item)
            self.accepted_total += len(accepted)
            self.rejected_total += len(rejections)
            self.last_rejections = Counter(rejections)
            self.consecutive_errors = 0
        except Exception as error:  # heartbeat remains empty for this capture
            self.consecutive_errors += 1
            self.get_logger().error(
                f"perception frame error {self.consecutive_errors}/{self.max_errors}: {error}"
            )
            if self.consecutive_errors >= self.max_errors:
                self.fatal = True
                self._publish_diagnostic(DiagnosticStatus.ERROR, f"fatal: {error}")
        finally:
            self.pose_pub.publish(output)

    def _infer(
        self,
        color: np.ndarray,
        depth: np.ndarray,
        intrinsics: SimpleNamespace,
        depth_scale: float,
    ) -> Tuple[list[Pose], list[str]]:
        arguments: Dict[str, Any] = {
            "classes": [self.target_class],
            "conf": self.confidence,
            "iou": self.nms_iou,
            "imgsz": self.image_size,
            "verbose": False,
            "half": self.half,
        }
        if self.device is not None:
            arguments["device"] = self.device
        results = self.model(color, **arguments)
        if not results:
            raise RuntimeError("YOLO returned no result object")
        result = results[0]
        if result.masks is None or result.boxes is None:
            return [], []
        masks = result.masks.data.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        boxes = result.boxes.xyxy.cpu().numpy()
        candidates = []
        for mask, confidence, box in zip(masks, confidences, boxes):
            if mask.shape != color.shape[:2]:
                mask = cv2.resize(
                    mask,
                    (color.shape[1], color.shape[0]),
                    interpolation=cv2.INTER_NEAREST,
                )
            candidates.append(
                {"mask": mask, "confidence": float(confidence), "box": box}
            )
        candidates = algorithms.remove_duplicate_masks(
            candidates, self.duplicate_mask_iou
        )

        accepted: list[Pose] = []
        rejected: list[str] = []
        for candidate in candidates:
            pose, reason = self._analyse_candidate(
                candidate, depth, intrinsics, depth_scale
            )
            if pose is None:
                rejected.append(reason)
            else:
                accepted.append(pose)
        return accepted, rejected

    def _analyse_candidate(
        self,
        candidate: Mapping[str, Any],
        depth: np.ndarray,
        intrinsics: SimpleNamespace,
        depth_scale: float,
    ) -> Tuple[Optional[Pose], str]:
        mask = np.asarray(candidate["mask"])
        fitted = algorithms.fit_ellipse_from_mask(mask)
        if fitted is None:
            return None, "ellipse_fit_failed"
        ellipse, _ = fitted
        (center_x, center_y), (axis_a, axis_b), _ = ellipse
        short_axis = min(float(axis_a), float(axis_b))
        long_axis = max(float(axis_a), float(axis_b))
        if short_axis <= 0.0:
            return None, "ellipse_axis_invalid"
        u, v = int(round(center_x)), int(round(center_y))
        if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
            return None, "ellipse_center_outside_image"

        rim = algorithms.ellipse_band_mask(depth.shape, ellipse, self.rim_band)
        rim_stats = algorithms.depth_summary(depth, rim, depth_scale)
        mask_stats = algorithms.depth_summary(
            depth, (mask > 0).astype(np.uint8), depth_scale
        )
        wall_depth = rim_stats["p15"]
        active_stats = rim_stats
        if wall_depth is None:
            wall_depth = algorithms.filtered_depth_meters(
                depth, u, v, depth_scale, self.depth_filter_size
            )
            active_stats = mask_stats
        if wall_depth is None:
            wall_depth = mask_stats["median"]
        if wall_depth is None:
            return None, "depth_unavailable"
        wall_depth = float(wall_depth)
        if active_stats["iqr"] is None:
            return None, "depth_statistics_unavailable"
        depth_metric = wall_depth if self.depth_mode == "rim_direct" else (
            wall_depth - self.bucket_height
        )
        diameter = long_axis * wall_depth / float(intrinsics.fx)
        metrics = {
            "confidence": float(candidate["confidence"]),
            "depth_m": depth_metric,
            "depth_valid_ratio": float(active_stats["valid_ratio"]),
            "depth_iqr_m": float(active_stats["iqr"]),
            "axis_ratio": long_axis / short_axis,
            "diameter_m": diameter,
            "mask_area_px": float(np.count_nonzero(mask > 0)),
        }
        reason = quality_gate(metrics, self.limits)
        if reason != "ok":
            return None, reason
        camera_xyz = np.asarray(
            [
                (float(center_x) - intrinsics.ppx) / intrinsics.fx * depth_metric,
                (float(center_y) - intrinsics.ppy) / intrinsics.fy * depth_metric,
                depth_metric,
            ],
            dtype=np.float64,
        )
        body = self.rotation @ camera_xyz + self.translation
        if not np.all(np.isfinite(body)):
            return None, "body_point_non_finite"
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, body)
        pose.orientation.x = float(diameter)
        pose.orientation.y = float(candidate["confidence"])
        pose.orientation.z = float(depth_metric)
        pose.orientation.w = 1.0
        return pose, "ok"

    def _publish_status(self) -> None:
        level = DiagnosticStatus.ERROR if self.fatal else (
            DiagnosticStatus.WARN if self.consecutive_errors else DiagnosticStatus.OK
        )
        message = "fatal" if self.fatal else (
            "degraded" if self.consecutive_errors else "running"
        )
        self._publish_diagnostic(level, message)

    def _publish_diagnostic(self, level: int, message: str) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = f"{self.get_fully_qualified_name()}/basket_perception"
        status.hardware_id = self.camera_serial
        status.message = message
        values = {
            "frames": self.frame_count,
            "accepted_total": self.accepted_total,
            "rejected_total": self.rejected_total,
            "consecutive_errors": self.consecutive_errors,
            "model_sha256": self.expected_model_hash,
        }
        values.update(
            {f"last_rejections.{key}": value for key, value in self.last_rejections.items()}
        )
        for key, value in values.items():
            status.values.append(KeyValue(key=str(key), value=str(value)))
        array.status.append(status)
        self.diagnostic_pub.publish(array)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[BucketPerceptionNode] = None
    try:
        node = BucketPerceptionNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
