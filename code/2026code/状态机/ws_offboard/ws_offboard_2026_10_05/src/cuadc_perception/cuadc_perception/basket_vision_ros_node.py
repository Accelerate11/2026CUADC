#!/usr/bin/env python3
"""原 D435i 与 YOLO 桶检测器的 ROS 2 适配层。

检测实现保留在同目录的 ``basket_detect_seg_analysis.py`` 中。
本适配层导入该模块，复用相机控制、分割去重、椭圆拟合、
深度统计、反投影和诊断录像功能。

发布接口（``geometry_msgs/msg/PoseArray``）：

* 默认话题：``/perception/drop_buckets_body``；
* ``header.stamp``：``wait_for_frames`` 返回后立即记录的 ROS 采集时刻，
  早于深度对齐与推理；
* ``header.frame_id``：飞控机体 FRD 坐标系，默认 ``fcu_body_frd``；
* ``position``：桶中心在 FRD 坐标系中的 XYZ，单位为米；
* ``orientation.x``：报告的桶直径，单位为米；
* ``orientation.y``：YOLO 置信度，范围 [0, 1]；
* ``orientation.z``：桶口中心的 FRD X 坐标，单位为米；
* ``orientation.w``：桶口中心的 FRD Y 坐标，单位为米。

每次获得有效 RGB-D 帧时都会发布消息；没有目标通过质量检查时，
发布空 PoseArray。任务节点按航点执行危险物区域侦察，
不接收危险物视觉输入。
"""

from __future__ import annotations

import csv
import importlib
import math
import os
import queue
import signal
import sys
import threading
import time
import traceback
import cv2
from collections import Counter
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


# 飞控与 MAVLink 使用机体 FRD 坐标：+X 前、+Y 右、+Z 下。
# 当前向下安装方式中，图像左方对应 +X，图像上方对应 +Y。
# RealSense 光学坐标：+X 向图像右方、+Y 向图像下方、+Z 沿光轴前方。
# 飞控机体坐标：+X 前、+Y 右、+Z 下。
DEFAULT_CAMERA_TO_BODY_R = np.asarray(
    [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
DEFAULT_CAMERA_TO_BODY_T_M = np.asarray([0.033, 0.0, 0.320], dtype=np.float64)


class FatalVisionError(RuntimeError):
    """无法恢复的模型、相机、配置或工作线程错误。"""


PACKAGE_NAME = "cuadc_perception"


def resolve_default_model_path(
    script_path: Optional[Path] = None,
    package_share_getter: Optional[Any] = None,
) -> Path:
    """在安装目录或源码目录中查找随包模型。

    安装后的 ROS 可执行入口位于 lib/<package>，模型位于
    share/<package>。已安装包优先使用 ament 索引；源码目录候选
    用于支持直接开发运行。
    """

    candidates: List[Path] = []
    lookup_errors: List[str] = []
    getter = package_share_getter
    if getter is None:
        try:
            from ament_index_python.packages import get_package_share_directory

            getter = get_package_share_directory
        except Exception as error:
            lookup_errors.append("ament import failed: {!r}".format(error))

    if getter is not None:
        try:
            share_directory = Path(str(getter(PACKAGE_NAME))).resolve()
            candidates.append(share_directory / "models" / "basket_v3.pt")
        except Exception as error:
            lookup_errors.append("ament lookup failed: {!r}".format(error))

    resolved_script = Path(script_path or __file__).resolve()
    candidates.append(resolved_script.parent.parent / "models" / "basket_v3.pt")

    unique_candidates: List[Path] = []
    for candidate in candidates:
        candidate = candidate.resolve()
        if candidate not in unique_candidates:
            unique_candidates.append(candidate)
        if candidate.is_file():
            return candidate

    checked = ", ".join(str(path) for path in unique_candidates) or "none"
    lookup = "; ".join(lookup_errors) or "ament lookup completed"
    raise FatalVisionError(
        "Unable to locate the default basket model. Checked: {}. {}. "
        "Install the package model or set ROS parameter model_path explicitly."
        .format(checked, lookup)
    )


def validate_rotation_matrix(
    rotation: Sequence[float], tolerance: float = 1.0e-3
) -> np.ndarray:
    """返回 3×3 正旋转矩阵，不合格时抛出 ``ValueError``。

    同时检查正交性和行列式。反射变换会使释放时的横向目标修正
    被镜像，因此不能接受。
    """

    array = np.asarray(rotation, dtype=np.float64)
    if array.size != 9:
        raise ValueError("camera_to_body_rotation must contain exactly 9 values")
    array = array.reshape(3, 3)
    if not np.all(np.isfinite(array)):
        raise ValueError("camera_to_body_rotation contains a non-finite value")
    if not math.isfinite(float(tolerance)) or tolerance <= 0.0:
        raise ValueError("extrinsic_orthogonality_tolerance must be positive")
    orthogonality_error = float(
        np.linalg.norm(array.T.dot(array) - np.eye(3), ord=np.inf)
    )
    determinant = float(np.linalg.det(array))
    if orthogonality_error > tolerance:
        raise ValueError(
            "camera_to_body_rotation is not orthogonal: "
            "error={:.6g}, tolerance={:.6g}".format(
                orthogonality_error, tolerance
            )
        )
    if abs(determinant - 1.0) > tolerance:
        raise ValueError(
            "camera_to_body_rotation must be a proper rotation: "
            "det={:.6g}, tolerance={:.6g}".format(determinant, tolerance)
        )
    return array


def transform_camera_to_body(
    camera_xyz_m: Sequence[float],
    camera_to_body_rotation: Sequence[float],
    camera_to_body_translation_m: Sequence[float],
) -> np.ndarray:
    """按 ``p_body = R_body_camera * p_camera + t_body_camera`` 转换坐标。"""

    point = np.asarray(camera_xyz_m, dtype=np.float64)
    translation = np.asarray(camera_to_body_translation_m, dtype=np.float64)
    if point.shape != (3,) or not np.all(np.isfinite(point)):
        raise ValueError("camera_xyz_m must be a finite 3-vector")
    if translation.shape != (3,) or not np.all(np.isfinite(translation)):
        raise ValueError("camera_to_body_translation_m must be a finite 3-vector")
    rotation = np.asarray(camera_to_body_rotation, dtype=np.float64)
    if rotation.size != 9:
        raise ValueError("camera_to_body_rotation must contain exactly 9 values")
    return rotation.reshape(3, 3).dot(point) + translation


def encode_pose_payload(
    body_xyz_m: Sequence[float],
    diameter_m: float,
    confidence: float,
    depth_metric_m: float,
) -> Tuple[float, float, float, float, float, float, float]:
    """构建供 C++ 任务节点读取的七个标量字段。"""

    body = np.asarray(body_xyz_m, dtype=np.float64)
    if body.shape != (3,) or not np.all(np.isfinite(body)):
        raise ValueError("body_xyz_m must be a finite 3-vector")
    scalars = np.asarray(
        [diameter_m, confidence, depth_metric_m], dtype=np.float64
    )
    if not np.all(np.isfinite(scalars)):
        raise ValueError("pose metadata contains a non-finite value")
    if diameter_m <= 0.0:
        raise ValueError("diameter_m must be positive")
    if confidence < 0.0 or confidence > 1.0:
        raise ValueError("confidence must be in [0, 1]")
    if depth_metric_m <= 0.0:
        raise ValueError("depth_metric_m must be positive")
    return (
        float(body[0]),
        float(body[1]),
        float(body[2]),
        float(diameter_m),
        float(confidence),
        float(depth_metric_m),
        1.0,
    )


def quality_gate(
    metrics: Mapping[str, Any], limits: Mapping[str, float]
) -> Tuple[bool, str]:
    """执行确定的检测质量检查，不依赖 ROS 或相机接口。"""

    required = (
        "confidence",
        "depth_m",
        "depth_valid_ratio",
        "depth_iqr_m",
        "axis_ratio",
        "diameter_m",
        "box_area_px",
    )
    values: Dict[str, float] = {}
    for key in required:
        value = metrics.get(key)
        try:
            values[key] = float(value)
        except (TypeError, ValueError):
            return False, "{}_unavailable".format(key)
        if not math.isfinite(values[key]):
            return False, "{}_non_finite".format(key)

    checks = (
        (
            values["confidence"] < float(limits["min_confidence"]),
            "confidence_below_min",
        ),
        (values["depth_m"] < float(limits["min_depth_m"]), "depth_below_min"),
        (values["depth_m"] > float(limits["max_depth_m"]), "depth_above_max"),
        (
            values["depth_valid_ratio"]
            < float(limits["min_depth_valid_ratio"]),
            "depth_valid_ratio_below_min",
        ),
        (
            values["depth_iqr_m"] > float(limits["max_depth_iqr_m"]),
            "depth_iqr_above_max",
        ),
        (
            values["axis_ratio"] > float(limits["max_axis_ratio"]),
            "axis_ratio_above_max",
        ),
        (
            values["diameter_m"] < float(limits["min_diameter_m"]),
            "diameter_below_min",
        ),
        (
            values["diameter_m"] > float(limits["max_diameter_m"]),
            "diameter_above_max",
        ),
        (
            values["box_area_px"] < float(limits["min_box_area_px"]),
            "box_area_below_min",
        ),
    )
    for failed, reason in checks:
        if failed:
            return False, reason
    if values["depth_valid_ratio"] > 1.0:
        return False, "depth_valid_ratio_above_one"
    if values["axis_ratio"] < 1.0:
        return False, "axis_ratio_below_one"
    return True, "ok"


def _load_original_detector_module() -> Any:
    """从安装后的 Python 包中导入原检测模块。"""

    return importlib.import_module(
        ".basket_detect_seg_analysis", package="cuadc_perception"
    )


class BasketVisionRosNode:
    """组合封装 ``rclpy.node.Node``。

    将 rclpy 导入放在模块作用域之外，使纯接口测试能在没有
    ROS、RealSense 或 Ultralytics 的开发机上运行。
    """

    def __init__(self, node: Any, rclpy_module: Any, ros_types: SimpleNamespace):
        self.node = node
        self.rclpy = rclpy_module
        self.ros = ros_types
        self.stop_event = threading.Event()
        self.video_stop_event = threading.Event()
        self.cleanup_lock = threading.Lock()
        self.resources_cleaned = False
        self.worker: Optional[threading.Thread] = None
        self.worker_exception: Optional[BaseException] = None
        self.pipeline = None
        self.profile = None
        self.video_writer = None
        self.video_queue: queue.Queue = queue.Queue(maxsize=2)
        self.video_worker: Optional[threading.Thread] = None
        self.live_view_enabled = False
        self.live_view_queue: queue.Queue = queue.Queue(maxsize=1)
        self.live_view_worker: Optional[threading.Thread] = None
        self.live_view_stop_event = threading.Event()
        self.live_view_failed = False
        self.release_snapshot_dir = ""
        self.snapshot_lock = threading.Lock()
        self.pending_snapshots: Dict[int, float] = {}
        self.video_dropped_frames = 0
        self.video_write_failed = False
        self.video_timeline_start_ns: Optional[int] = None
        self.video_timeline_frames_written = 0
        self.vision = None
        self.model = None
        self.align_to_color = None
        self.intrinsics = None
        self.depth_scale = None
        self.color_format = None
        self.color_format_name = "unknown"
        self.actual_camera_serial = "unknown"
        self.detection_log_file = None
        self.detection_log_writer = None

        self.frame_count = 0
        self.accepted_count = 0
        self.rejected_count = 0
        self.consecutive_frame_errors = 0
        self.last_inference_ms = 0.0
        self.last_status_monotonic = 0.0
        self.last_capture_stamp_text = "none"
        self.last_raw_detection_count = 0
        self.last_accepted_frame_count = 0
        self.last_rejected_frame_count = 0
        self.last_rejection_counts: Counter[str] = Counter()

        diagnostics_topic = self._parameter(
            "diagnostics_topic", "/diagnostics"
        )
        output_topic = self._parameter(
            "output_topic", "/perception/drop_buckets_body"
        )
        diagnostic_qos = self.ros.QoSProfile(
            history=self.ros.HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=self.ros.ReliabilityPolicy.RELIABLE,
            durability=self.ros.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.diagnostic_publisher = self.node.create_publisher(
            self.ros.DiagnosticArray, diagnostics_topic, diagnostic_qos
        )
        self.pose_publisher = self.node.create_publisher(
            self.ros.PoseArray, output_topic, self.ros.qos_profile_sensor_data
        )
        self.snapshot_subscription = self.node.create_subscription(
            self.ros.String, "/cuadc/release_snapshot", self._snapshot_trigger, 10
        )
        self.output_topic = output_topic

    def _parameter(self, name: str, default: Any) -> Any:
        if not self.node.has_parameter(name):
            self.node.declare_parameter(name, default)
        return self.node.get_parameter(name).value

    def _snapshot_trigger(self, message: Any) -> None:
        try:
            payload = int(str(message.data).strip())
            if payload < 1:
                return
            with self.snapshot_lock:
                # 收到释放触发后延迟一秒采集快照，
                # 记录投放后的场景，而不是执行器动作的瞬间。
                self.pending_snapshots[payload] = time.monotonic() + 1.0
        except (TypeError, ValueError):
            self.node.get_logger().warning("Invalid release snapshot trigger")

    @staticmethod
    def _positive_int(name: str, value: Any) -> int:
        converted = int(value)
        if converted <= 0:
            raise ValueError("{} must be positive".format(name))
        return converted

    @staticmethod
    def _nonnegative_int(name: str, value: Any) -> int:
        converted = int(value)
        if converted < 0:
            raise ValueError("{} must be non-negative".format(name))
        return converted

    @staticmethod
    def _finite_float(name: str, value: Any) -> float:
        converted = float(value)
        if not math.isfinite(converted):
            raise ValueError("{} must be finite".format(name))
        return converted

    def initialize(self) -> None:
        """加载原检测模块，并校验参数、模型和相机。"""

        self._publish_diagnostic(
            self.ros.DiagnosticStatus.WARN,
            "initializing",
            {"output_topic": self.output_topic},
            force=True,
        )
        self.vision = _load_original_detector_module()
        required_symbols = (
            "YOLO",
            "rs",
            "cv2",
            "configure_camera",
            "require_usb3",
            "frame_to_bgr",
            "depth_summary",
            "filtered_depth_meters",
            "pixel_to_camera_xyz",
        )
        missing = [name for name in required_symbols if not hasattr(self.vision, name)]
        if missing:
            raise FatalVisionError(
                "basket_detect_seg_analysis.py is missing required symbols: {}".format(
                    ", ".join(missing)
                )
            )
        self._load_parameters()
        self.release_snapshot_dir = str(self._parameter("release_snapshot_dir", "")).strip()
        self._load_model()
        self._start_camera()
        self._open_detection_log()
        self._open_diagnostic_video_if_enabled()
        self._open_live_view_if_enabled()
        self._publish_diagnostic(
            self.ros.DiagnosticStatus.OK,
            "ready",
            self._status_values(),
            force=True,
        )
        self.node.get_logger().info(
            "Basket vision ready: topic={} frame={} depth_mode={} camera={} "
            "format={} model={}".format(
                self.output_topic,
                self.frame_id,
                self.depth_mode,
                self.actual_camera_serial,
                self.color_format_name,
                self.model_path,
            )
        )

    def _load_parameters(self) -> None:
        configured_model_path = str(self._parameter("model_path", "")).strip()
        if configured_model_path:
            selected_model = Path(
                os.path.expandvars(os.path.expanduser(configured_model_path))
            ).resolve()
        else:
            selected_model = resolve_default_model_path()
        self.model_path = str(selected_model)
        if not selected_model.is_file():
            raise FatalVisionError(
                "model_path is not a file: {}. Set model_path to the installed "
                "basket_v3.pt or leave it empty to use the package default."
                .format(selected_model)
            )

        self.camera_serial = str(self._parameter("camera_serial", "")).strip()
        self.exposure = self._finite_float(
            "exposure",
            self._parameter(
                "exposure", float(getattr(self.vision, "DEFAULT_EXPOSURE", 0.0))
            ),
        )
        if self.exposure < 0.0:
            raise ValueError("exposure must be zero (auto) or positive")
        self.color_width = self._positive_int(
            "color_width",
            self._parameter(
                "color_width", int(getattr(self.vision, "COLOR_WIDTH", 1920))
            ),
        )
        self.color_height = self._positive_int(
            "color_height",
            self._parameter(
                "color_height", int(getattr(self.vision, "COLOR_HEIGHT", 1080))
            ),
        )
        self.depth_width = self._positive_int(
            "depth_width",
            self._parameter(
                "depth_width", int(getattr(self.vision, "DEPTH_WIDTH", 848))
            ),
        )
        self.depth_height = self._positive_int(
            "depth_height",
            self._parameter(
                "depth_height", int(getattr(self.vision, "DEPTH_HEIGHT", 480))
            ),
        )
        self.camera_fps = self._positive_int(
            "camera_fps",
            self._parameter(
                "camera_fps", int(getattr(self.vision, "FRAME_RATE", 30))
            ),
        )
        self.warmup_frames = self._nonnegative_int(
            "warmup_frames", self._parameter("warmup_frames", 15)
        )
        self.frame_timeout_ms = self._positive_int(
            "frame_timeout_ms", self._parameter("frame_timeout_ms", 1500)
        )
        if self.frame_timeout_ms > 10000:
            raise ValueError("frame_timeout_ms must be <= 10000 for clean shutdown")

        self.target_class = int(
            self._parameter(
                "target_class", int(getattr(self.vision, "TARGET_CLASS", 0))
            )
        )
        self.confidence_threshold = self._finite_float(
            "confidence_threshold",
            self._parameter(
                "confidence_threshold",
                float(getattr(self.vision, "CONFIDENCE_THRESHOLD", 0.25)),
            ),
        )
        self.inference_image_size = self._positive_int(
            "inference_image_size",
            self._parameter(
                "inference_image_size",
                int(getattr(self.vision, "INFERENCE_IMAGE_SIZE", 640)),
            ),
        )
        self.nms_iou_threshold = self._finite_float(
            "nms_iou_threshold",
            self._parameter(
                "nms_iou_threshold",
                float(getattr(self.vision, "NMS_IOU_THRESHOLD", 0.01)),
            ),
        )
        duplicate_iou = self._finite_float(
            "duplicate_mask_iou_threshold",
            self._parameter(
                "duplicate_mask_iou_threshold",
                float(
                    getattr(self.vision, "DUPLICATE_MASK_IOU_THRESHOLD", 0.20)
                ),
            ),
        )
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("confidence_threshold must be in [0, 1]")
        if not 0.0 <= self.nms_iou_threshold <= 1.0:
            raise ValueError("nms_iou_threshold must be in [0, 1]")
        if not 0.0 <= duplicate_iou <= 1.0:
            raise ValueError("duplicate_mask_iou_threshold must be in [0, 1]")
        self.vision.DUPLICATE_MASK_IOU_THRESHOLD = duplicate_iou
        self.device = self._parameter("device", "")
        if isinstance(self.device, str) and not self.device.strip():
            self.device = None
        self.half = bool(self._parameter("half", False))
        self.using_tensorrt = Path(self.model_path).suffix.lower() == ".engine"

        self.bucket_height_m = self._finite_float(
            "bucket_height_m",
            self._parameter(
                "bucket_height_m",
                float(getattr(self.vision, "BUCKET_HEIGHT_METERS", 0.30)),
            ),
        )
        if self.bucket_height_m < 0.0:
            raise ValueError("bucket_height_m must be non-negative")
        self.bucket_height_compensation_ratio = self._finite_float(
            "bucket_height_compensation_ratio",
            self._parameter("bucket_height_compensation_ratio", 0.85),
        )
        if not 0.0 <= self.bucket_height_compensation_ratio <= 1.0:
            raise ValueError("bucket_height_compensation_ratio must be in [0, 1]")
        self.depth_mode = str(
            self._parameter("depth_mode", "wall_minus_height")
        ).strip().lower()
        if self.depth_mode not in ("wall_minus_height", "rim_direct"):
            raise ValueError(
                "depth_mode must be 'wall_minus_height' or 'rim_direct'"
            )
        self.rim_band_thickness_pixels = self._positive_int(
            "rim_band_thickness_pixels",
            self._parameter(
                "rim_band_thickness_pixels",
                int(getattr(self.vision, "RIM_BAND_THICKNESS_PIXELS", 11)),
            ),
        )
        self.vision.DEPTH_FILTER_SIZE = self._positive_int(
            "depth_filter_size",
            self._parameter(
                "depth_filter_size",
                int(getattr(self.vision, "DEPTH_FILTER_SIZE", 5)),
            ),
        )
        self.vision.GROUND_RING_INNER_SCALE = self._finite_float(
            "ground_ring_inner_scale",
            self._parameter(
                "ground_ring_inner_scale",
                float(getattr(self.vision, "GROUND_RING_INNER_SCALE", 1.15)),
            ),
        )
        self.vision.GROUND_RING_OUTER_SCALE = self._finite_float(
            "ground_ring_outer_scale",
            self._parameter(
                "ground_ring_outer_scale",
                float(getattr(self.vision, "GROUND_RING_OUTER_SCALE", 1.75)),
            ),
        )
        if (
            self.vision.GROUND_RING_INNER_SCALE <= 1.0
            or self.vision.GROUND_RING_OUTER_SCALE
            <= self.vision.GROUND_RING_INNER_SCALE
        ):
            raise ValueError(
                "ground ring scales must satisfy 1 < inner < outer"
            )
        self.vision.POWER_LINE_FREQUENCY_HZ = int(
            self._parameter("power_line_frequency_hz", 50)
        )
        if self.vision.POWER_LINE_FREQUENCY_HZ not in (50, 60):
            raise ValueError("power_line_frequency_hz must be 50 or 60")

        rotation_values = self._parameter(
            "camera_to_body_rotation", DEFAULT_CAMERA_TO_BODY_R.reshape(-1).tolist()
        )
        translation_values = self._parameter(
            "camera_to_body_translation_m", DEFAULT_CAMERA_TO_BODY_T_M.tolist()
        )
        extrinsic_tolerance = self._finite_float(
            "extrinsic_orthogonality_tolerance",
            self._parameter("extrinsic_orthogonality_tolerance", 1.0e-3),
        )
        self.camera_to_body_rotation = validate_rotation_matrix(
            rotation_values, extrinsic_tolerance
        )
        self.camera_to_body_translation_m = np.asarray(
            translation_values, dtype=np.float64
        )
        if (
            self.camera_to_body_translation_m.shape != (3,)
            or not np.all(np.isfinite(self.camera_to_body_translation_m))
        ):
            raise ValueError(
                "camera_to_body_translation_m must be a finite 3-vector"
            )

        self.quality_limits = {
            "min_confidence": self._finite_float(
                "min_quality_confidence",
                self._parameter(
                    "min_quality_confidence", self.confidence_threshold
                ),
            ),
            "min_depth_m": self._finite_float(
                "min_depth_m", self._parameter("min_depth_m", 0.15)
            ),
            "max_depth_m": self._finite_float(
                "max_depth_m", self._parameter("max_depth_m", 6.0)
            ),
            "min_depth_valid_ratio": self._finite_float(
                "min_depth_valid_ratio",
                self._parameter("min_depth_valid_ratio", 0.35),
            ),
            "max_depth_iqr_m": self._finite_float(
                "max_depth_iqr_m", self._parameter("max_depth_iqr_m", 0.20)
            ),
            "max_axis_ratio": self._finite_float(
                "max_box_axis_ratio",
                self._parameter(
                    "max_box_axis_ratio",
                    self._parameter("max_ellipse_axis_ratio", 2.50),
                ),
            ),
            "min_diameter_m": self._finite_float(
                "min_diameter_m", self._parameter("min_diameter_m", 0.08)
            ),
            "max_diameter_m": self._finite_float(
                "max_diameter_m", self._parameter("max_diameter_m", 0.35)
            ),
            "min_box_area_px": self._finite_float(
                "min_box_area_px", self._parameter("min_mask_area_px", 100.0)
            ),
        }
        if not 0.0 <= self.quality_limits["min_confidence"] <= 1.0:
            raise ValueError("min_quality_confidence must be in [0, 1]")
        if not 0.0 <= self.quality_limits["min_depth_valid_ratio"] <= 1.0:
            raise ValueError("min_depth_valid_ratio must be in [0, 1]")
        if self.quality_limits["min_depth_m"] >= self.quality_limits["max_depth_m"]:
            raise ValueError("min_depth_m must be less than max_depth_m")
        if self.quality_limits["min_diameter_m"] >= self.quality_limits["max_diameter_m"]:
            raise ValueError("min_diameter_m must be less than max_diameter_m")
        if self.quality_limits["max_depth_iqr_m"] < 0.0:
            raise ValueError("max_depth_iqr_m must be non-negative")
        if self.quality_limits["max_axis_ratio"] < 1.0:
            raise ValueError("max_box_axis_ratio must be >= 1")
        if self.quality_limits["min_box_area_px"] < 0.0:
            raise ValueError("min_box_area_px must be non-negative")

        self.frame_id = str(self._parameter("frame_id", "fcu_body_frd")).strip()
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")
        self.max_consecutive_frame_errors = self._positive_int(
            "max_consecutive_frame_errors",
            self._parameter("max_consecutive_frame_errors", 5),
        )
        self.diagnostic_period_s = self._finite_float(
            "diagnostic_period_s", self._parameter("diagnostic_period_s", 1.0)
        )
        if self.diagnostic_period_s <= 0.0:
            raise ValueError("diagnostic_period_s must be positive")

        self.diagnostic_video_enabled = bool(
            self._parameter("diagnostic_video_enabled", False)
        )
        self.diagnostic_video_path = str(
            self._parameter("diagnostic_video_path", "")
        ).strip()
        self.diagnostic_video_fps = self._positive_int(
            "diagnostic_video_fps",
            self._parameter("diagnostic_video_fps", self.camera_fps),
        )
        self.diagnostic_video_bitrate = self._positive_int(
            "diagnostic_video_bitrate",
            self._parameter("diagnostic_video_bitrate", 8_000_000),
        )
        self.diagnostic_video_software_encoder = bool(
            self._parameter("diagnostic_video_software_encoder", False)
        )
        self.live_view_enabled = bool(
            self._parameter("live_view_enabled", True)
        )
        self.diagnostic_detection_csv_path = str(
            self._parameter("diagnostic_detection_csv_path", "")
        ).strip()
        offsets = self._parameter(
            "payload_release_offsets_body_m",
            [0.029, -0.070, -0.320, -0.031, 0.055, -0.320],
        )
        self.payload_release_offsets_body_m = np.asarray(offsets, dtype=np.float64).reshape(-1, 3)
        if self.payload_release_offsets_body_m.shape[0] < 2 or not np.all(np.isfinite(self.payload_release_offsets_body_m)):
            raise ValueError("payload_release_offsets_body_m must contain two finite 3-vectors")

    def _open_detection_log(self) -> None:
        if not self.diagnostic_detection_csv_path:
            return
        try:
            path = Path(os.path.expandvars(
                os.path.expanduser(self.diagnostic_detection_csv_path)
            )).resolve()
            path.parent.mkdir(parents=True, exist_ok=True)
            self.detection_log_file = path.open(
                "w", newline="", encoding="utf-8"
            )
            self.detection_log_writer = csv.writer(self.detection_log_file)
            self.detection_log_writer.writerow([
                "capture_time_ns", "frame", "detection_index", "center_source",
                "camera_x_m", "camera_y_m", "camera_z_m",
                "body_frd_x_m", "body_frd_y_m", "body_frd_z_m", "depth_m",
                "rim_body_frd_x_m", "rim_body_frd_y_m", "rim_body_frd_z_m",
                "diameter_m", "confidence", "status",
            ])
            self.detection_log_file.flush()
            self.node.get_logger().info(
                "Detection coordinate log enabled: {}".format(path)
            )
        except Exception as error:
            self.detection_log_file = None
            self.detection_log_writer = None
            self.node.get_logger().error(
                "Detection coordinate log disabled: {}".format(error)
            )

    def _write_detection_log(
        self, capture_stamp: Any, detection_index: int,
        detection: Mapping[str, Any], status: str = "accepted",
    ) -> None:
        if self.detection_log_writer is None:
            return
        payload = detection["payload"]
        camera = detection["camera_xyz_m"]
        body = detection["body_xyz_m"]
        rim_body = detection.get("rim_body_xyz_m", body)
        stamp_ns = int(capture_stamp.sec) * 1_000_000_000 + int(capture_stamp.nanosec)
        self.detection_log_writer.writerow([
            stamp_ns, self.frame_count, detection_index,
            detection.get("center_source", "unknown"),
            "{:.6f}".format(float(camera[0])),
            "{:.6f}".format(float(camera[1])),
            "{:.6f}".format(float(camera[2])),
            "{:.6f}".format(float(body[0])),
            "{:.6f}".format(float(body[1])),
            "{:.6f}".format(float(body[2])),
            "{:.6f}".format(float(payload[5])),
            "{:.6f}".format(float(rim_body[0])),
            "{:.6f}".format(float(rim_body[1])),
            "{:.6f}".format(float(rim_body[2])),
            "{:.6f}".format(float(payload[3])),
            "{:.6f}".format(float(payload[4])), status,
        ])
        self.detection_log_file.flush()

    def _load_model(self) -> None:
        load_start = time.monotonic()
        self.node.get_logger().info("Loading YOLO detection model: {}".format(self.model_path))
        try:
            self.model = self.vision.YOLO(self.model_path, task="detect")
        except Exception as error:
            raise FatalVisionError("YOLO model load failed: {}".format(error)) from error
        self.node.get_logger().info(
            "YOLO model loaded in {:.2f}s".format(time.monotonic() - load_start)
        )

    def _start_camera(self) -> None:
        rs = self.vision.rs
        color_formats = (
            (rs.format.yuyv, "YUYV"),
            (rs.format.rgb8, "RGB8"),
            (rs.format.bgr8, "BGR8"),
        )
        failures: List[str] = []
        for stream_format, format_name in color_formats:
            pipeline = rs.pipeline()
            config = rs.config()
            if self.camera_serial:
                config.enable_device(self.camera_serial)
            config.enable_stream(
                rs.stream.color,
                self.color_width,
                self.color_height,
                stream_format,
                self.camera_fps,
            )
            config.enable_stream(
                rs.stream.depth,
                self.depth_width,
                self.depth_height,
                rs.format.z16,
                self.camera_fps,
            )
            try:
                profile = pipeline.start(config)
                self.pipeline = pipeline
                self.profile = profile
                self.color_format = stream_format
                self.color_format_name = format_name
                break
            except Exception as error:
                failures.append("{}: {}".format(format_name, error))
                try:
                    pipeline.stop()
                except Exception:
                    pass
        if self.pipeline is None or self.profile is None:
            raise FatalVisionError(
                "No requested D435i RGB-D profile started ({})".format(
                    "; ".join(failures)
                )
            )

        try:
            device = self.profile.get_device()
            self.vision.require_usb3(device)
            self.vision.configure_camera(device, self.exposure)
            if device.supports(rs.camera_info.serial_number):
                self.actual_camera_serial = device.get_info(
                    rs.camera_info.serial_number
                )
            if self.camera_serial and self.actual_camera_serial != self.camera_serial:
                raise FatalVisionError(
                    "requested camera serial {} but opened {}".format(
                        self.camera_serial, self.actual_camera_serial
                    )
                )
            self.depth_scale = float(
                device.first_depth_sensor().get_depth_scale()
            )
            if not math.isfinite(self.depth_scale) or self.depth_scale <= 0.0:
                raise FatalVisionError(
                    "invalid RealSense depth scale: {}".format(self.depth_scale)
                )
            self.intrinsics = (
                self.profile.get_stream(rs.stream.color)
                .as_video_stream_profile()
                .intrinsics
            )
            self.align_to_color = rs.align(rs.stream.color)
            for _ in range(self.warmup_frames):
                if self.stop_event.is_set() or not self.rclpy.ok(
                    context=self.node.context
                ):
                    raise FatalVisionError("shutdown requested during camera warm-up")
                self.pipeline.wait_for_frames(self.frame_timeout_ms)
        except Exception:
            self._cleanup_resources()
            raise

    def _open_diagnostic_video_if_enabled(self) -> None:
        if not self.diagnostic_video_enabled:
            return
        if self.diagnostic_video_path:
            video_path = Path(
                os.path.expandvars(os.path.expanduser(self.diagnostic_video_path))
            ).resolve()
        else:
            video_path = Path("/tmp/cuadc_basket_vision") / "basket_{}.mp4".format(
                datetime.now().strftime("%Y%m%d_%H%M%S")
            )
        try:
            self.video_writer, encoder_name = self.vision.create_video_writer(
                video_path,
                int(self.intrinsics.width),
                int(self.intrinsics.height),
                self.diagnostic_video_fps,
                self.diagnostic_video_bitrate,
                self.diagnostic_video_software_encoder,
            )
            self.diagnostic_video_path = str(video_path)
            self.node.get_logger().warn(
                "Diagnostic video enabled ({}): {}".format(
                    encoder_name, self.diagnostic_video_path
                )
            )
            self.video_worker = threading.Thread(
                target=self._video_worker_main,
                name="basket_video_writer",
                daemon=False,
            )
            self.video_worker.start()
        except Exception as error:
            # 可选录像失败时，任务感知仍需继续工作。
            self.video_writer = None
            self.diagnostic_video_enabled = False
            self.node.get_logger().error(
                "Diagnostic video disabled after open failure: {}".format(error)
            )

    def _open_live_view_if_enabled(self) -> None:
        if not self.live_view_enabled:
            return
        if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
            self.live_view_enabled = False
            self.node.get_logger().warning(
                "Live view disabled: no graphical display is available"
            )
            return
        try:
            self.live_view_worker = threading.Thread(
                target=self._live_view_worker_main,
                name="basket_live_view",
                daemon=True,
            )
            self.live_view_worker.start()
            self.node.get_logger().info("Live view enabled: window=CUADC live view")
        except Exception as error:
            self.live_view_enabled = False
            self.live_view_failed = True
            self.node.get_logger().warning(
                "Live view disabled after GUI open failure: {}".format(error)
            )

    def _submit_live_view_frame(self, display: np.ndarray) -> None:
        if not self.live_view_enabled:
            return
        try:
            self.live_view_queue.put_nowait(display)
        except queue.Full:
            try:
                self.live_view_queue.get_nowait()
                self.live_view_queue.task_done()
            except queue.Empty:
                pass
            try:
                self.live_view_queue.put_nowait(display)
            except queue.Full:
                pass

    def _live_view_worker_main(self) -> None:
        cv2 = self.vision.cv2
        try:
            # Linux 的 HighGUI 后端要求窗口操作来自同一线程；
            # 窗口创建和使用都在当前工作线程完成。
            cv2.namedWindow("CUADC live view", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("CUADC live view", 960, 540)
            first_frame = True
            while not self.live_view_stop_event.is_set():
                try:
                    display = self.live_view_queue.get(timeout=0.1)
                except queue.Empty:
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        self.live_view_enabled = False
                        break
                    continue
                try:
                    if first_frame:
                        self.node.get_logger().info(
                            "Live view first frame: shape={} dtype={} range=[{}, {}]".format(
                                display.shape,
                                display.dtype,
                                int(display.min()),
                                int(display.max()),
                            )
                        )
                        first_frame = False
                    cv2.imshow("CUADC live view", display)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        self.live_view_enabled = False
                        break
                    try:
                        if cv2.getWindowProperty(
                            "CUADC live view", cv2.WND_PROP_VISIBLE
                        ) < 1:
                            self.live_view_enabled = False
                            break
                    except Exception:
                        pass
                finally:
                    self.live_view_queue.task_done()
        except Exception as error:
            self.live_view_failed = True
            self.live_view_enabled = False
            self.node.get_logger().warning(
                "Live view disabled after display failure: {}".format(error)
            )
        finally:
            try:
                cv2.destroyWindow("CUADC live view")
            except Exception:
                pass

    def start(self) -> None:
        if self.worker is not None:
            raise RuntimeError("vision worker already started")
        self.worker = threading.Thread(
            target=self._worker_main,
            name="basket_vision_worker",
            daemon=False,
        )
        self.worker.start()

    def _worker_main(self) -> None:
        try:
            while (
                not self.stop_event.is_set()
                and self.rclpy.ok(context=self.node.context)
            ):
                try:
                    frames = self.pipeline.wait_for_frames(self.frame_timeout_ms)
                    # 在对齐和推理增加延迟前，记录采集返回时刻。
                    capture_stamp = self.node.get_clock().now().to_msg()
                    aligned_frames = self.align_to_color.process(frames)
                    self._process_captured_frames(aligned_frames, capture_stamp)
                    self.consecutive_frame_errors = 0
                    self._publish_running_diagnostic()
                except Exception as error:
                    if self.stop_event.is_set() or not self.rclpy.ok(
                        context=self.node.context
                    ):
                        break
                    self.consecutive_frame_errors += 1
                    self.node.get_logger().error(
                        "Vision frame error {}/{}: {}".format(
                            self.consecutive_frame_errors,
                            self.max_consecutive_frame_errors,
                            error,
                        )
                    )
                    self._publish_diagnostic(
                        self.ros.DiagnosticStatus.WARN,
                        "recoverable frame error",
                        {
                            "consecutive_errors": self.consecutive_frame_errors,
                            "error": repr(error),
                        },
                        force=True,
                    )
                    if (
                        self.consecutive_frame_errors
                        >= self.max_consecutive_frame_errors
                    ):
                        raise FatalVisionError(
                            "{} consecutive frame errors; last={}".format(
                                self.consecutive_frame_errors, error
                            )
                        ) from error
        except BaseException as error:
            self.worker_exception = error
            self.report_fatal(error)
            self.stop_event.set()
            try:
                self.node.context.try_shutdown()
            except Exception:
                pass
        finally:
            self._cleanup_resources()

    def _process_captured_frames(
        self, aligned_frames: Any, capture_stamp: Any
    ) -> bool:
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        if not color_frame or not depth_frame:
            # 缺失图像流时，进入错误诊断并按致命错误阈值处理。
            raise RuntimeError(
                "Aligned RGB-D capture missing color or depth frame"
            )

        message = self.ros.PoseArray()
        message.header.stamp = capture_stamp
        message.header.frame_id = self.frame_id
        self.frame_count += 1
        self.last_capture_stamp_text = "{}.{}".format(
            capture_stamp.sec, str(capture_stamp.nanosec).zfill(9)
        )
        display = None
        raw_detection_count = 0
        accepted: List[Dict[str, Any]] = []
        rejections: List[Dict[str, Any]] = []
        try:
            color_image = self.vision.frame_to_bgr(
                color_frame, self.color_format
            )
            self._save_due_release_snapshots(color_image)
            depth_raw = np.asanyarray(depth_frame.get_data())
            if color_image.shape[:2] != depth_raw.shape[:2]:
                raise RuntimeError(
                    "aligned color/depth shape mismatch: {} vs {}".format(
                        color_image.shape[:2], depth_raw.shape[:2]
                    )
                )
            if self.video_writer is not None or self.live_view_enabled:
                display = color_image.copy()
            inference_start = time.monotonic()
            result = self._run_inference(color_image)
            self.last_inference_ms = (time.monotonic() - inference_start) * 1000.0
            accepted, rejections, raw_detection_count = self._analyse_result(
                result, color_image, depth_raw, display
            )
            for detection_index, detection in enumerate(accepted):
                self._write_detection_log(
                    capture_stamp, detection_index, detection
                )
                message.poses.append(self._pose_from_payload(
                    detection["payload"], detection["rim_body_xyz_m"]
                ))
            self.accepted_count += len(accepted)
            self.rejected_count += len(rejections)
            self.last_raw_detection_count = raw_detection_count
            self.last_accepted_frame_count = len(accepted)
            self.last_rejected_frame_count = len(rejections)
            self.last_rejection_counts = Counter(
                item["reason"] for item in rejections
            )
            if display is not None:
                self._draw_fcu_xy_axes(display)
                self._submit_live_view_frame(display)
                self._write_diagnostic_frame(
                    display,
                    len(accepted),
                    len(rejections),
                    raw_detection_count,
                    capture_stamp,
                )
        finally:
            # 这是每次采集的心跳；YOLO 未检测到目标或所有候选
            # 都被拒绝时，主动发布空数组。
            self.pose_publisher.publish(message)
        return True

    def _save_due_release_snapshots(self, color_image: np.ndarray) -> None:
        if not self.release_snapshot_dir:
            return
        now = time.monotonic()
        with self.snapshot_lock:
            due = [payload for payload, deadline in self.pending_snapshots.items() if now >= deadline]
            for payload in due:
                self.pending_snapshots.pop(payload, None)
        for payload in due:
            try:
                os.makedirs(self.release_snapshot_dir, exist_ok=True)
                height, width = color_image.shape[:2]
                target_height = 480
                target_width = max(1, int(round(width * target_height / height)))
                resized = cv2.resize(
                    color_image, (target_width, target_height),
                    interpolation=cv2.INTER_AREA,
                )
                path = os.path.join(self.release_snapshot_dir, f"release_{payload}.jpg")
                if not cv2.imwrite(path, resized, [int(cv2.IMWRITE_JPEG_QUALITY), 35]):
                    raise RuntimeError("cv2.imwrite returned false")
                self.node.get_logger().info("Release snapshot saved: {}".format(path))
            except Exception as error:
                self.node.get_logger().error("Release snapshot failed: {}".format(error))

    def _run_inference(self, color_image: np.ndarray) -> Any:
        predict_args: Dict[str, Any] = {
            "classes": [self.target_class],
            "conf": self.confidence_threshold,
            "iou": self.nms_iou_threshold,
            "imgsz": self.inference_image_size,
            "verbose": False,
        }
        if not self.using_tensorrt:
            predict_args["half"] = self.half
        if self.device is not None:
            predict_args["device"] = self.device
        results = self.model(color_image, **predict_args)
        if not results:
            raise RuntimeError("YOLO returned no result object")
        return results[0]

    def _extract_candidates(
        self, result: Any, color_shape: Tuple[int, int]
    ) -> Tuple[List[Dict[str, Any]], int]:
        if result.boxes is None:
            return [], 0
        confidences = result.boxes.conf.cpu().numpy()
        boxes = result.boxes.xyxy.cpu().numpy()
        raw_count = int(len(boxes))
        candidates: List[Dict[str, Any]] = []
        for confidence, box in zip(confidences, boxes):
            candidates.append(
                {
                    "confidence": float(confidence),
                    "box": np.asarray(box, dtype=np.float64),
                }
            )
        # ONNX 流程直接输出检测框，因此在此执行严格的最终非极大值抑制。
        candidates.sort(key=lambda item: item["confidence"], reverse=True)
        selected: List[Dict[str, Any]] = []
        for candidate in candidates:
            box = candidate["box"]
            suppressed = False
            for kept in selected:
                other = kept["box"]
                ix1 = max(float(box[0]), float(other[0]))
                iy1 = max(float(box[1]), float(other[1]))
                ix2 = min(float(box[2]), float(other[2]))
                iy2 = min(float(box[3]), float(other[3]))
                intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
                area = max(0.0, float(box[2] - box[0])) * max(0.0, float(box[3] - box[1]))
                other_area = max(0.0, float(other[2] - other[0])) * max(0.0, float(other[3] - other[1]))
                union = area + other_area - intersection
                if union > 0.0 and intersection / union >= self.nms_iou_threshold:
                    suppressed = True
                    break
            if not suppressed:
                selected.append(candidate)
        return selected, raw_count

    def _analyse_result(
        self,
        result: Any,
        color_image: np.ndarray,
        depth_raw: np.ndarray,
        display: Optional[np.ndarray],
    ) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]], int]:
        candidates, raw_count = self._extract_candidates(
            result, color_image.shape[:2]
        )
        accepted: List[Dict[str, Any]] = []
        rejections: List[Dict[str, Any]] = []
        for candidate in candidates:
            try:
                detection, reason = self._analyse_candidate(
                    candidate, depth_raw
                )
            except Exception as error:
                detection = None
                reason = "candidate_analysis_error"
                self.node.get_logger().warning(
                    "Candidate analysis failed: {}".format(error)
                )
            if detection is None:
                rejected = {
                    "reason": reason,
                    "box": candidate["box"],
                    "confidence": candidate["confidence"],
                }
                rejections.append(rejected)
                if display is not None:
                    self._draw_rejected(display, rejected)
                continue
            accepted.append(detection)
            if display is not None:
                self._draw_accepted(display, detection, draw_release_markers=(len(accepted) == 1))
        return accepted, rejections, raw_count

    def _analyse_candidate(
        self,
        candidate: Mapping[str, Any],
        depth_raw: np.ndarray,
    ) -> Tuple[Optional[Dict[str, Any]], str]:
        confidence = float(candidate["confidence"])
        x1, y1, x2, y2 = [float(value) for value in candidate["box"]]
        box_center_x = max(0.0, min(float(depth_raw.shape[1] - 1), (x1 + x2) * 0.5))
        box_center_y = max(0.0, min(float(depth_raw.shape[0] - 1), (y1 + y2) * 0.5))
        box_width = max(1.0, x2 - x1)
        box_height = max(1.0, y2 - y1)
        long_axis = max(box_width, box_height)
        short_axis = min(box_width, box_height)
        axis_ratio = long_axis / short_axis
        box_area_px = int(box_width * box_height)
        u = int(round(box_center_x))
        v = int(round(box_center_y))
        center_depth = self.vision.filtered_depth_meters(
            depth_raw, u, v, self.depth_scale
        )
        # 飞机靠近桶时，检测框中心可能看到不透明桶壁或桶底。
        # 因此从检测框外侧的环带估计地面平面，
        # 再结合已知桶高恢复桶口平面。
        # 这样桶口估计不会依赖检测框中心处
        # 随高度变化的桶内可见内容。
        ground_depth = self._estimate_ground_depth_from_bbox(
            depth_raw, x1, y1, x2, y2
        )
        ground_depth_source = "outer_ground_ring"
        if ground_depth is None:
            if center_depth is None:
                return None, "center_and_outer_ground_depth_unavailable"
            ground_depth = max(0.10, float(center_depth) - self.bucket_height_m)
            ground_depth_source = "center_fallback"
        highest_wall_depth = float(ground_depth + self.bucket_height_m)

        if self.depth_mode == "wall_minus_height":
            depth_metric_m = ground_depth
        else:
            depth_metric_m = highest_wall_depth
        center_source = "box"
        center_x, center_y = box_center_x, box_center_y
        # 环带深度表示地面距离；检测轮廓对应桶口，
        # 其位置比地面朝相机方向近一个桶高。
        # 此处若直接采用地面距离，下降时会高估桶直径，
        # 且在近距离时偏差更明显。
        diameter_depth_m = max(
            0.05,
            float(ground_depth)
            - float(self.bucket_height_m) * self.bucket_height_compensation_ratio,
        )
        legacy_diameter_m = (
            long_axis * diameter_depth_m / float(self.intrinsics.fx)
        )
        metrics = {
            "confidence": confidence,
            "depth_m": depth_metric_m,
            "depth_valid_ratio": 1.0,
            "depth_iqr_m": 0.0,
            "axis_ratio": axis_ratio,
            "diameter_m": legacy_diameter_m,
            "box_area_px": box_area_px,
        }
        passed, reason = quality_gate(metrics, self.quality_limits)
        if not passed:
            return None, reason

        camera_xyz_m = self.vision.pixel_to_camera_xyz(
            center_x, center_y, depth_metric_m, self.intrinsics
        )
        # 另保留桶口或桶沿平面上的第二个点。
        # 上方修正点用于以地面为参考估计深度和直径，
        # 而释放十字准星应在不透明桶壁实际观测到的
        # 桶口高度处计算。
        rim_depth_m = max(0.05, float(depth_metric_m) - self.bucket_height_m)
        rim_camera_xyz_m = self.vision.pixel_to_camera_xyz(
            center_x, center_y, rim_depth_m, self.intrinsics
        )
        body_xyz_m = transform_camera_to_body(
            camera_xyz_m,
            self.camera_to_body_rotation,
            self.camera_to_body_translation_m,
        )
        rim_body_xyz_m = transform_camera_to_body(
            rim_camera_xyz_m,
            self.camera_to_body_rotation,
            self.camera_to_body_translation_m,
        )
        payload = encode_pose_payload(
            body_xyz_m, legacy_diameter_m, confidence, depth_metric_m
        )
        return {
            "payload": payload,
            "ellipse": None,
            "box": candidate["box"],
            "depth_source": "box_center",
            "center_source": center_source,
            "camera_xyz_m": camera_xyz_m,
            "body_xyz_m": body_xyz_m,
            "rim_body_xyz_m": rim_body_xyz_m,
            "depth_metric_m": depth_metric_m,
            "highest_wall_depth_m": highest_wall_depth,
            "diameter_axis_a_m": float(short_axis * diameter_depth_m / self.intrinsics.fx),
            "diameter_axis_b_m": float(long_axis * diameter_depth_m / self.intrinsics.fx),
            "ground_depth_m": ground_depth,
            "ground_depth_source": ground_depth_source,
            "metrics": metrics,
        }, "ok"

    def _estimate_ground_depth_from_bbox(
        self, depth_raw: np.ndarray, x1: float, y1: float, x2: float, y2: float
    ) -> Optional[float]:
        """从检测框紧邻的外侧区域返回稳健的地面深度估计。"""
        height, width = depth_raw.shape[:2]
        left = max(0, int(math.floor(x1)))
        top = max(0, int(math.floor(y1)))
        right = min(width - 1, int(math.ceil(x2)))
        bottom = min(height - 1, int(math.ceil(y2)))
        if right <= left or bottom <= top:
            return None
        box_w = right - left + 1
        box_h = bottom - top + 1
        pad = max(8, int(round(0.18 * max(box_w, box_h))))
        outer_left = max(0, left - pad)
        outer_top = max(0, top - pad)
        outer_right = min(width - 1, right + pad)
        outer_bottom = min(height - 1, bottom + pad)
        sample = np.zeros((height, width), dtype=np.uint8)
        sample[outer_top:outer_bottom + 1, outer_left:outer_right + 1] = 1
        sample[top:bottom + 1, left:right + 1] = 0
        values = depth_raw[sample > 0].astype(np.float64) * float(self.depth_scale)
        values = values[np.isfinite(values) & (values > 0.10) & (values < 10.0)]
        if values.size < 20:
            return None
        # 对周围地面样本取中位数前，
        # 先剔除孤立的前景和背景污染点。
        low, high = np.percentile(values, [10.0, 90.0])
        clipped = values[(values >= low) & (values <= high)]
        if clipped.size < 10:
            clipped = values
        return float(np.median(clipped))

    def _pose_from_payload(
        self, payload: Sequence[float], rim_body_xyz_m: Sequence[float]
    ) -> Any:
        pose = self.ros.Pose()
        pose.position.x = float(payload[0])
        pose.position.y = float(payload[1])
        pose.position.z = float(payload[2])
        pose.orientation.x = float(payload[3])
        pose.orientation.y = float(payload[4])
        pose.orientation.z = float(rim_body_xyz_m[0])
        pose.orientation.w = float(rim_body_xyz_m[1])
        return pose

    def _draw_accepted(
        self,
        display: np.ndarray,
        detection: Mapping[str, Any],
        draw_release_markers: bool = True,
    ) -> None:
        cv2 = self.vision.cv2
        ellipse = detection["ellipse"]
        box = np.asarray(detection["box"], dtype=int)
        x1, y1, x2, y2 = [int(value) for value in box]
        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
        # 荧光绿色标记表示检测目标的几何中心，
        # 与橙色和品红色的舵机投放点标记区分。
        center_u = int(round((x1 + x2) * 0.5))
        center_v = int(round((y1 + y2) * 0.5))
        target_center_color = (57, 255, 20)  # BGR 色序，荧光绿。
        cv2.drawMarker(
            display,
            (center_u, center_v),
            target_center_color,
            cv2.MARKER_CROSS,
            24,
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            "TARGET CENTER",
            (center_u + 12, center_v + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            target_center_color,
            2,
            cv2.LINE_AA,
        )
        if ellipse is not None:
            cv2.ellipse(display, ellipse, (0, 255, 255), 2)
        payload = detection["payload"]
        label = "OK conf={:.2f} D={:.3f}m depth={:.3f}m {} center={}".format(
            payload[4], payload[3], payload[5], detection["depth_source"],
            detection["center_source"],
        )
        self.vision.put_text_with_outline(
            display, label, (max(0, x1), max(24, y1 - 8)), 0.48
        )
        # 十字准星固定于飞机，表示每个瓶体垂直线
        # 与当前桶口平面的交点；垂直线由标定的机体 x/y 偏移确定。
        # 不能根据目标的 x/y 位置生成十字准星，
        # 否则标记会跟随桶移动，
        # 掩盖实际对准误差。
        ground_body = np.asarray(detection["body_xyz_m"], dtype=np.float64)
        rim_z_body = float(ground_body[2] - self.bucket_height_m)
        if not draw_release_markers:
            return
        translation = self.camera_to_body_translation_m
        camera_rotation = self.camera_to_body_rotation
        for index, offset in enumerate(self.payload_release_offsets_body_m[:2]):
            servo_rim_body = np.array(
                [float(offset[0]), float(offset[1]), rim_z_body],
                dtype=np.float64,
            )
            camera_point = camera_rotation.T.dot(servo_rim_body - translation)
            if camera_point[2] <= 0.05:
                continue
            u = int(round(self.intrinsics.fx * camera_point[0] / camera_point[2] + self.intrinsics.ppx))
            v = int(round(self.intrinsics.fy * camera_point[1] / camera_point[2] + self.intrinsics.ppy))
            if 0 <= u < display.shape[1] and 0 <= v < display.shape[0]:
                color = (0, 165, 255) if index == 0 else (255, 0, 255)
                # OpenCV 的 Hershey 字体不能可靠显示中文。
                # 逐帧叠加信息保留 ASCII，避免在检测循环中
                # 增加 PIL 和字体转换的开销。
                label = "1 Right-Front Servo (CH7)" if index == 0 else "2 Left-Rear Servo (CH8)"
                cv2.drawMarker(display, (u, v), color, cv2.MARKER_CROSS, 28, 3, cv2.LINE_AA)
                cv2.putText(display, label, (u + 12, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)

    def _draw_fcu_xy_axes(self, display: np.ndarray) -> None:
        """叠加显示配置后的飞控 FRD +X 和 +Y 在图像中的方向。"""
        cv2 = self.vision.cv2
        origin = np.asarray([92.0, float(display.shape[0] - 76)], dtype=np.float64)
        axes = (
            ("+X FWD", np.array([1.0, 0.0, 0.0]), (0, 255, 255)),
            ("+Y RIGHT", np.array([0.0, 1.0, 0.0]), (255, 255, 0)),
        )
        for label, body_axis, color in axes:
            camera_axis = self.camera_to_body_rotation.T.dot(body_axis)
            image_direction = np.asarray([camera_axis[0], camera_axis[1]])
            length = float(np.linalg.norm(image_direction))
            if length < 1e-6:
                continue
            endpoint = origin + 54.0 * image_direction / length
            start = tuple(np.rint(origin).astype(int))
            end = tuple(np.rint(endpoint).astype(int))
            cv2.arrowedLine(display, start, end, (0, 0, 0), 5, cv2.LINE_AA, tipLength=0.20)
            cv2.arrowedLine(display, start, end, color, 2, cv2.LINE_AA, tipLength=0.20)
            text_origin = (end[0] + 6, end[1] - 5)
            cv2.putText(display, label, text_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(display, label, text_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
        cv2.putText(display, "FCU BODY FRD", (20, display.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(display, "FCU BODY FRD", (20, display.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1, cv2.LINE_AA)

    def _draw_rejected(self, display: np.ndarray, rejection: Mapping[str, Any]) -> None:
        cv2 = self.vision.cv2
        box = np.asarray(rejection["box"], dtype=int)
        x1, y1, x2, y2 = [int(value) for value in box]
        cv2.rectangle(display, (x1, y1), (x2, y2), (0, 0, 255), 2)
        self.vision.put_text_with_outline(
            display,
            "REJECT {}".format(rejection["reason"]),
            (max(0, x1), max(24, y1 - 8)),
            0.44,
        )

    def _write_diagnostic_frame(
        self,
        display: np.ndarray,
        accepted_count: int,
        rejected_count: int,
        raw_count: int,
        capture_stamp: Any,
    ) -> None:
        if self.video_writer is None:
            return
        self.vision.put_text_with_outline(
            display,
            "frame={} raw={} accepted={} rejected={} infer={:.1f}ms".format(
                self.frame_count,
                raw_count,
                accepted_count,
                rejected_count,
                self.last_inference_ms,
            ),
            (20, 32),
            0.58,
        )
        self.vision.put_text_with_outline(
            display,
            "ros_time={}.{}".format(
                capture_stamp.sec, str(capture_stamp.nanosec).zfill(9)
            ),
            (20, 62),
            0.58,
        )
        try:
            stamp_ns = (
                int(capture_stamp.sec) * 1_000_000_000
                + int(capture_stamp.nanosec)
            )
            self.video_queue.put_nowait((stamp_ns, display))
        except queue.Full:
            self.video_dropped_frames += 1

    def _video_worker_main(self) -> None:
        while not self.video_stop_event.is_set() or not self.video_queue.empty():
            try:
                stamp_ns, frame = self.video_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if self.video_writer is not None:
                    if self.video_timeline_start_ns is None:
                        self.video_timeline_start_ns = stamp_ns
                    elapsed_s = max(
                        0.0,
                        (stamp_ns - self.video_timeline_start_ns) / 1_000_000_000.0,
                    )
                    target_frame_count = (
                        int(round(elapsed_s * self.diagnostic_video_fps)) + 1
                    )
                    while self.video_timeline_frames_written < target_frame_count:
                        self.video_writer.write(frame)
                        self.video_timeline_frames_written += 1
            except Exception as error:
                self.video_write_failed = True
                self.node.get_logger().error(
                    "Diagnostic video disabled after write failure: {}".format(error)
                )
                break
            finally:
                self.video_queue.task_done()

    def _status_values(self) -> Dict[str, Any]:
        values: Dict[str, Any] = {
            "frames_published": self.frame_count,
            "detections_accepted_total": self.accepted_count,
            "detections_rejected_total": self.rejected_count,
            "last_raw_detection_count": self.last_raw_detection_count,
            "last_accepted_frame_count": self.last_accepted_frame_count,
            "last_rejected_frame_count": self.last_rejected_frame_count,
            "consecutive_frame_errors": self.consecutive_frame_errors,
            "last_inference_ms": "{:.3f}".format(self.last_inference_ms),
            "last_capture_stamp": self.last_capture_stamp_text,
            "camera_serial": self.actual_camera_serial,
            "camera_format": self.color_format_name,
            "depth_mode": getattr(self, "depth_mode", "unknown"),
            "frame_id": getattr(self, "frame_id", "unknown"),
            "diagnostic_video_enabled": bool(self.video_writer is not None),
            "diagnostic_video_dropped_frames": self.video_dropped_frames,
            "diagnostic_video_write_failed": self.video_write_failed,
            "live_view_enabled": bool(self.live_view_enabled),
            "live_view_failed": self.live_view_failed,
        }
        for reason, count in sorted(self.last_rejection_counts.items()):
            values["last_rejections.{}".format(reason)] = count
        return values

    def _publish_running_diagnostic(self) -> None:
        now_monotonic = time.monotonic()
        if now_monotonic - self.last_status_monotonic < self.diagnostic_period_s:
            return
        self.last_status_monotonic = now_monotonic
        rejection_text = ",".join(
            "{}={}".format(reason, count)
            for reason, count in sorted(self.last_rejection_counts.items())
        ) or "none"
        self.node.get_logger().info(
            "VISION frame={} raw={} accepted={} rejected={} infer={:.1f}ms reasons={}".format(
                self.frame_count,
                self.last_raw_detection_count,
                self.last_accepted_frame_count,
                self.last_rejected_frame_count,
                self.last_inference_ms,
                rejection_text,
            )
        )
        self._publish_diagnostic(
            self.ros.DiagnosticStatus.OK,
            "running",
            self._status_values(),
            force=True,
        )

    def _publish_diagnostic(
        self,
        level: int,
        message: str,
        values: Optional[Mapping[str, Any]] = None,
        force: bool = False,
    ) -> None:
        del force  # 在调用处显式保留该标记，用于区分重要和致命诊断报告。
        try:
            diagnostic_array = self.ros.DiagnosticArray()
            diagnostic_array.header.stamp = self.node.get_clock().now().to_msg()
            status = self.ros.DiagnosticStatus()
            if isinstance(self.ros.DiagnosticStatus.OK, (bytes, bytearray)):
                status.level = (
                    bytes(level[:1])
                    if isinstance(level, (bytes, bytearray))
                    else bytes((int(level),))
                )
            else:
                status.level = (
                    level[0]
                    if isinstance(level, (bytes, bytearray))
                    else int(level)
                )
            status.name = "{}/basket_vision".format(self.node.get_fully_qualified_name())
            status.hardware_id = self.actual_camera_serial
            status.message = str(message)
            for key, value in (values or {}).items():
                item = self.ros.KeyValue()
                item.key = str(key)
                item.value = str(value)
                status.values.append(item)
            diagnostic_array.status.append(status)
            self.diagnostic_publisher.publish(diagnostic_array)
        except Exception as error:
            self.node.get_logger().error(
                "Unable to publish diagnostic status: {}".format(error)
            )

    def report_fatal(self, error: BaseException) -> None:
        trace = "".join(
            traceback.format_exception(type(error), error, error.__traceback__)
        )
        self.node.get_logger().fatal("Basket vision fatal: {}".format(error))
        self._publish_diagnostic(
            self.ros.DiagnosticStatus.ERROR,
            "FATAL: {}".format(error),
            {
                "exception_type": type(error).__name__,
                "traceback": trace[-4000:],
            },
            force=True,
        )

    def request_stop(self) -> None:
        self.stop_event.set()

    def stop(self) -> bool:
        """停止工作线程；需要强制终止进程时返回 false。"""

        self.stop_event.set()
        worker = self.worker
        if worker is not None and worker is not threading.current_thread():
            frame_timeout_ms = getattr(self, "frame_timeout_ms", 1500)
            join_timeout_s = max(3.0, frame_timeout_ms / 1000.0 + 2.0)
            worker.join(timeout=join_timeout_s)
            if worker.is_alive():
                self.node.get_logger().fatal(
                    "Vision worker did not exit within {:.1f}s. ROS objects will "
                    "not be destroyed while the non-daemon worker is live."
                    .format(join_timeout_s)
                )
                return False
        self._cleanup_resources()
        return True

    def _cleanup_resources(self) -> None:
        with self.cleanup_lock:
            if self.resources_cleaned:
                return
            self.resources_cleaned = True
            video_worker = self.video_worker
            self.video_stop_event.set()
            if video_worker is not None and video_worker is not threading.current_thread():
                self.node.get_logger().info(
                    "Finalizing diagnostic video; waiting for queued frames"
                )
                video_worker.join()
            self.video_worker = None
            self.live_view_stop_event.set()
            live_view_worker = self.live_view_worker
            if live_view_worker is not None and live_view_worker is not threading.current_thread():
                live_view_worker.join(timeout=2.0)
            self.live_view_worker = None
            try:
                if self.vision is not None:
                    self.vision.cv2.destroyWindow("CUADC live view")
            except Exception:
                pass
            if self.video_writer is not None:
                try:
                    self.video_writer.release()
                    self.node.get_logger().info(
                        "Diagnostic video finalized: {}".format(
                            self.diagnostic_video_path
                        )
                    )
                except Exception as error:
                    self.node.get_logger().error(
                        "Diagnostic video close failed: {}".format(error)
                    )
                self.video_writer = None
            if self.detection_log_file is not None:
                try:
                    self.detection_log_file.flush()
                    self.detection_log_file.close()
                except Exception as error:
                    self.node.get_logger().error(
                        "Detection coordinate log close failed: {}".format(error)
                    )
                self.detection_log_file = None
                self.detection_log_writer = None
            if self.pipeline is not None:
                try:
                    self.pipeline.stop()
                except Exception as error:
                    self.node.get_logger().error(
                        "RealSense pipeline stop failed: {}".format(error)
                    )
                self.pipeline = None
def _load_ros_types() -> SimpleNamespace:
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Pose, PoseArray
    from std_msgs.msg import String
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )

    return SimpleNamespace(
        DiagnosticArray=DiagnosticArray,
        DiagnosticStatus=DiagnosticStatus,
        KeyValue=KeyValue,
        Pose=Pose,
        PoseArray=PoseArray,
        String=String,
        DurabilityPolicy=DurabilityPolicy,
        HistoryPolicy=HistoryPolicy,
        QoSProfile=QoSProfile,
        ReliabilityPolicy=ReliabilityPolicy,
        qos_profile_sensor_data=qos_profile_sensor_data,
    )


def main(args: Optional[Iterable[str]] = None) -> int:
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as error:
        print("FATAL: ROS 2 rclpy is required: {}".format(error), file=sys.stderr)
        return 2

    rclpy.init(args=list(args) if args is not None else None)
    node = Node("basket_vision_ros_node")
    adapter: Optional[BasketVisionRosNode] = None
    exit_code = 0
    worker_stopped = True
    try:
        adapter = BasketVisionRosNode(node, rclpy, _load_ros_types())
        adapter.initialize()
        adapter.start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        exit_code = 1
        if adapter is not None:
            adapter.report_fatal(error)
        else:
            node.get_logger().fatal("Basket vision fatal: {}".format(error))
    finally:
        if adapter is not None:
            adapter.request_stop()
            # 等待相机线程退出时，launch 可能多次转发同一次 Ctrl-C。
            # 清理过程保持原子性，使 VideoWriter 能在进程结束前
            # 完成 MP4 容器收尾。
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            worker_stopped = adapter.stop()
            if adapter.worker_exception is not None:
                exit_code = 1
        if not worker_stopped:
            exit_code = 1
            node.get_logger().fatal(
                "Forcing process exit with the live worker's ROS objects intact."
            )
            try:
                sys.stderr.flush()
                sys.stdout.flush()
            except Exception:
                pass
            os._exit(exit_code)

        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
