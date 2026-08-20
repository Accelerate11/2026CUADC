"""D435i + YOLO segmentation with annotated video, FPS, and CSV diagnostics."""

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


# D435i camera defaults. Change this value to adjust the default manual exposure.
DEFAULT_EXPOSURE = 25.0

TARGET_CLASS = 0
CONFIDENCE_THRESHOLD = 0.25
INFERENCE_IMAGE_SIZE = 640
NMS_IOU_THRESHOLD = 0.01
DUPLICATE_MASK_IOU_THRESHOLD = 0.20
MODEL_PATH = "/home/nvidia/video/basket_new.engine"
DEPTH_FILTER_SIZE = 5
BUCKET_HEIGHT_METERS = 0.30
RIM_BAND_THICKNESS_PIXELS = 11
RIM_DEPTH_PERCENTILE = 15
GROUND_RING_INNER_SCALE = 1.15
GROUND_RING_OUTER_SCALE = 1.75
CSV_FLUSH_INTERVAL_FRAMES = 30
COLOR_WIDTH, COLOR_HEIGHT, FRAME_RATE = 1920, 1080, 30
DEPTH_WIDTH, DEPTH_HEIGHT = 848, 480
POWER_LINE_FREQUENCY_HZ = 50

# These remain identity until a calibrated camera-to-reference transform is supplied.
CAMERA_TO_WORLD_R = np.eye(3, dtype=np.float64)
CAMERA_TO_WORLD_T = np.zeros(3, dtype=np.float64)

CSV_FIELDS = [
    "host_time_iso",
    "camera_timestamp_ms",
    "frame_index",
    "elapsed_s",
    "fps_instant",
    "fps_ema",
    "fps_average",
    "capture_wait_ms",
    "depth_align_ms",
    "frame_convert_ms",
    "inference_ms",
    "yolo_preprocess_ms",
    "yolo_backend_inference_ms",
    "yolo_postprocess_ms",
    "analysis_overlay_ms",
    "video_write_ms",
    "video_timeline_fps",
    "video_frames_written_this_cycle",
    "video_frames_written_total",
    "raw_detection_count",
    "selected_detection_count",
    "color_width_px",
    "color_height_px",
    "color_format",
    "depth_width_px",
    "depth_height_px",
    "depth_scale_m_per_unit",
    "color_exposure_setting",
    "intrinsics_fx_px",
    "intrinsics_fy_px",
    "intrinsics_ppx_px",
    "intrinsics_ppy_px",
    "inference_image_size_px",
    "nms_iou_threshold",
    "frame_ground_depth_median_approx_m",
    "frame_ground_depth_iqr_m",
    "frame_ground_depth_valid_ratio",
    "detection_index",
    "status",
    "confidence",
    "bbox_x1_px",
    "bbox_y1_px",
    "bbox_x2_px",
    "bbox_y2_px",
    "bbox_width_px",
    "bbox_height_px",
    "mask_area_px",
    "mask_area_ratio",
    "ellipse_center_u_px",
    "ellipse_center_v_px",
    "ellipse_axis_a_px",
    "ellipse_axis_b_px",
    "ellipse_angle_deg",
    "rim_depth_p15_m",
    "rim_depth_median_m",
    "rim_depth_iqr_m",
    "rim_depth_std_m",
    "rim_depth_valid_count",
    "rim_depth_valid_ratio",
    "center_depth_m",
    "mask_depth_median_m",
    "mask_depth_iqr_m",
    "mask_depth_valid_ratio",
    "ground_depth_median_m",
    "ground_depth_iqr_m",
    "ground_depth_std_m",
    "ground_depth_valid_count",
    "ground_depth_valid_ratio",
    "camera_ground_distance_approx_m",
    "bucket_height_config_m",
    "bucket_height_estimate_approx_m",
    "corrected_depth_m",
    "camera_x_m",
    "camera_y_m",
    "camera_z_m",
    "world_x_m",
    "world_y_m",
    "world_z_m",
    "diameter_reported_m",
    "diameter_legacy_major_m",
    "diameter_axis_a_deprojected_m",
    "diameter_axis_b_deprojected_m",
    "diameter_axis_ratio",
]


def fit_ellipse_from_mask(mask: np.ndarray):
    binary_mask = (mask > 0).astype(np.uint8) * 255
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    usable = [contour for contour in contours if len(contour) >= 5 and cv2.contourArea(contour) > 30]
    if not usable:
        return None
    contour = max(usable, key=cv2.contourArea)
    return cv2.fitEllipse(contour), contour


def valid_depth_values(depth_raw: np.ndarray, sample_mask: np.ndarray, depth_scale: float) -> np.ndarray:
    values = depth_raw[sample_mask > 0].astype(np.float64) * depth_scale
    return values[np.isfinite(values) & (values > 0.10) & (values < 10.0)]


def summarize_depth(values: np.ndarray, sample_count: int) -> dict:
    if values.size == 0:
        return {
            "valid_count": 0,
            "valid_ratio": 0.0,
            "p15": None,
            "median": None,
            "iqr": None,
            "std": None,
        }
    p15, p25, median, p75 = np.percentile(values, [15, 25, 50, 75])
    return {
        "valid_count": int(values.size),
        "valid_ratio": float(values.size / max(1, sample_count)),
        "p15": float(p15),
        "median": float(median),
        "iqr": float(p75 - p25),
        "std": float(np.std(values)),
    }


def depth_summary(depth_raw: np.ndarray, sample_mask: np.ndarray, depth_scale: float) -> dict:
    sample_count = int(np.count_nonzero(sample_mask))
    return summarize_depth(valid_depth_values(depth_raw, sample_mask, depth_scale), sample_count)


def filtered_depth_meters(depth_raw: np.ndarray, u: int, v: int, depth_scale: float) -> float | None:
    half = DEPTH_FILTER_SIZE // 2
    y0, y1 = max(0, v - half), min(depth_raw.shape[0], v + half + 1)
    x0, x1 = max(0, u - half), min(depth_raw.shape[1], u + half + 1)
    values = depth_raw[y0:y1, x0:x1].astype(np.float64) * depth_scale
    values = values[np.isfinite(values) & (values > 0.10) & (values < 10.0)]
    return float(np.median(values)) if values.size else None


def ellipse_band_mask(shape: tuple[int, int], ellipse, thickness: int) -> np.ndarray:
    band = np.zeros(shape, dtype=np.uint8)
    cv2.ellipse(band, ellipse, 255, thickness)
    return band


def ground_ring_mask(shape: tuple[int, int], ellipse, occupied_mask: np.ndarray) -> np.ndarray:
    """Build a local ground annulus and exclude every detected bucket mask."""
    (center_x, center_y), (axis_a, axis_b), angle = ellipse
    inner = (
        (center_x, center_y),
        (max(2.0, axis_a * GROUND_RING_INNER_SCALE), max(2.0, axis_b * GROUND_RING_INNER_SCALE)),
        angle,
    )
    outer = (
        (center_x, center_y),
        (max(3.0, axis_a * GROUND_RING_OUTER_SCALE), max(3.0, axis_b * GROUND_RING_OUTER_SCALE)),
        angle,
    )
    inner_mask = np.zeros(shape, dtype=np.uint8)
    outer_mask = np.zeros(shape, dtype=np.uint8)
    cv2.ellipse(inner_mask, inner, 255, -1)
    cv2.ellipse(outer_mask, outer, 255, -1)
    ring = cv2.subtract(outer_mask, inner_mask)
    ring[occupied_mask > 0] = 0
    return ring


def mask_iou(first: np.ndarray, second: np.ndarray) -> float:
    first_binary = first > 0
    second_binary = second > 0
    union = np.count_nonzero(first_binary | second_binary)
    if union == 0:
        return 0.0
    return float(np.count_nonzero(first_binary & second_binary) / union)


def boxes_overlap(first: np.ndarray, second: np.ndarray) -> bool:
    left = max(float(first[0]), float(second[0]))
    top = max(float(first[1]), float(second[1]))
    right = min(float(first[2]), float(second[2]))
    bottom = min(float(first[3]), float(second[3]))
    return right > left and bottom > top


def remove_duplicate_masks(candidates: list[dict]) -> list[dict]:
    """Preserve the original duplicate-removal behavior for comparable recordings."""
    selected = []
    for candidate in sorted(candidates, key=lambda item: item["confidence"], reverse=True):
        if any(
            boxes_overlap(candidate["box"], kept["box"])
            or mask_iou(candidate["mask"], kept["mask"]) >= DUPLICATE_MASK_IOU_THRESHOLD
            for kept in selected
        ):
            continue
        selected.append(candidate)
    return selected


def pixel_to_camera_xyz(u: float, v: float, depth_meters: float, intrinsics: rs.intrinsics) -> np.ndarray:
    point = rs.rs2_deproject_pixel_to_point(intrinsics, [float(u), float(v)], depth_meters)
    return np.asarray(point, dtype=np.float64)


def ellipse_axis_diameters_meters(ellipse, depth_meters: float, intrinsics: rs.intrinsics) -> tuple[float, float]:
    """Deproject both ellipse axes to expose fx/fy and off-center scale effects."""
    (center_x, center_y), (axis_a, axis_b), angle_deg = ellipse
    theta = np.deg2rad(angle_deg)
    direction_a = np.array([np.cos(theta), np.sin(theta)])
    direction_b = np.array([-np.sin(theta), np.cos(theta)])

    def deprojected_length(axis_pixels: float, direction: np.ndarray) -> float:
        first = np.array([center_x, center_y]) - 0.5 * axis_pixels * direction
        second = np.array([center_x, center_y]) + 0.5 * axis_pixels * direction
        first_xyz = pixel_to_camera_xyz(first[0], first[1], depth_meters, intrinsics)
        second_xyz = pixel_to_camera_xyz(second[0], second[1], depth_meters, intrinsics)
        return float(np.linalg.norm(second_xyz - first_xyz))

    return deprojected_length(axis_a, direction_a), deprojected_length(axis_b, direction_b)


def camera_to_world(camera_xyz: np.ndarray) -> np.ndarray:
    return CAMERA_TO_WORLD_R @ camera_xyz + CAMERA_TO_WORLD_T


def find_rgb_sensor(device: rs.device) -> rs.sensor:
    for sensor in device.query_sensors():
        if sensor.supports(rs.camera_info.name):
            name = sensor.get_info(rs.camera_info.name)
            if "RGB" in name or "Color" in name:
                return sensor
    raise RuntimeError("D435i RGB sensor was not found")


def require_usb3(device: rs.device) -> None:
    if not device.supports(rs.camera_info.usb_type_descriptor):
        print("Warning: camera USB connection type is unavailable")
        return
    usb_type = device.get_info(rs.camera_info.usb_type_descriptor).strip()
    print(f"Negotiated USB connection: {usb_type}")
    if not usb_type.startswith("3."):
        raise RuntimeError(
            f"D435i negotiated USB {usb_type}; RGB-D inference requires USB 3.x. "
            "Check the Jetson USB port, cable, and camera power."
        )


def set_option(sensor: rs.sensor, option: rs.option, value: float, name: str) -> None:
    if not sensor.supports(option):
        print(f"  [SKIP] {name}: unsupported by this device/firmware")
        return
    value_range = sensor.get_option_range(option)
    if value < value_range.min or value > value_range.max:
        raise ValueError(
            f"{name}={value:g} is outside the supported range "
            f"[{value_range.min:g}, {value_range.max:g}]"
        )
    sensor.set_option(option, value)
    actual = sensor.get_option(option)
    print(f"  [OK]   {name}: requested={value:g}, actual={actual:g}")


def configure_camera(device: rs.device, exposure: float) -> None:
    color_sensor = find_rgb_sensor(device)
    print("Applying RGB controls:")
    set_option(color_sensor, rs.option.enable_auto_exposure, 0.0, "enable_auto_exposure")
    set_option(color_sensor, rs.option.exposure, exposure, "exposure")
    set_option(color_sensor, rs.option.gain, 64.0, "gain")
    set_option(color_sensor, rs.option.auto_exposure_priority, 0.0, "auto_exposure_priority")
    set_option(color_sensor, rs.option.backlight_compensation, 0.0, "backlight_compensation")
    set_option(color_sensor, rs.option.brightness, -10.0, "brightness")
    set_option(color_sensor, rs.option.contrast, 50.0, "contrast")
    set_option(color_sensor, rs.option.gamma, 300.0, "gamma")
    set_option(color_sensor, rs.option.hue, 0.0, "hue")
    set_option(color_sensor, rs.option.saturation, 64.0, "saturation")
    set_option(color_sensor, rs.option.sharpness, 50.0, "sharpness")
    set_option(color_sensor, rs.option.enable_auto_white_balance, 1.0, "enable_auto_white_balance")
    power_line_value = 1.0 if POWER_LINE_FREQUENCY_HZ == 50 else 2.0
    set_option(color_sensor, rs.option.power_line_frequency, power_line_value, "power_line_frequency")
    for sensor in device.query_sensors():
        if sensor.supports(rs.option.global_time_enabled):
            sensor.set_option(rs.option.global_time_enabled, 1.0)


def start_rgbd_stream(args: argparse.Namespace):
    """Try the same Linux-friendly color formats as the verified RGB recorder."""
    color_formats = [
        (rs.format.yuyv, "YUYV"),
        (rs.format.rgb8, "RGB8"),
        (rs.format.bgr8, "BGR8"),
    ]
    last_error = None
    for color_format, format_name in color_formats:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, args.width, args.height, color_format, args.fps)
        config.enable_stream(
            rs.stream.depth,
            args.depth_width,
            args.depth_height,
            rs.format.z16,
            args.fps,
        )
        try:
            print(
                f"Trying RGB-D profile: color {args.width}x{args.height} {format_name} "
                f"+ depth {args.depth_width}x{args.depth_height} Z16 @ {args.fps} FPS"
            )
            profile = pipeline.start(config)
            print(f"Selected color format: {format_name}")
            return pipeline, profile, color_format, format_name
        except RuntimeError as error:
            last_error = error
            print(f"  [SKIP] {format_name}: {error}")
            try:
                pipeline.stop()
            except RuntimeError:
                pass
    raise RuntimeError(f"No requested RGB-D profile could start. Last error: {last_error}")


def frame_to_bgr(color_frame: rs.video_frame, stream_format: rs.format) -> np.ndarray:
    raw = np.asanyarray(color_frame.get_data())
    height = color_frame.get_height()
    width = color_frame.get_width()
    if stream_format == rs.format.yuyv:
        if raw.ndim == 2:
            raw = raw.view(np.uint8).reshape(height, width, 2)
        elif raw.ndim == 3 and raw.shape[-1] == 1:
            raw = raw.reshape(height, width, 2)
        elif raw.ndim != 3 or raw.shape[-1] != 2:
            raise RuntimeError(f"Unexpected YUYV frame shape: {raw.shape}, dtype={raw.dtype}")
        return cv2.cvtColor(np.ascontiguousarray(raw), cv2.COLOR_YUV2BGR_YUY2)
    if stream_format == rs.format.rgb8:
        return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
    return raw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="D435i basket segmentation video and CSV recorder")
    parser.add_argument(
        "exposure",
        nargs="?",
        type=float,
        default=DEFAULT_EXPOSURE,
        help=f"Manual D435i color exposure value; default: {DEFAULT_EXPOSURE:g}",
    )
    parser.add_argument("--model", default=MODEL_PATH, help="Path to the YOLO segmentation .pt or .engine file")
    parser.add_argument("--output-dir", default=None, help="Output directory; default is basket_video beside this script")
    parser.add_argument("--conf", type=float, default=CONFIDENCE_THRESHOLD)
    parser.add_argument(
        "--bucket-height",
        type=float,
        default=BUCKET_HEIGHT_METERS,
        help="Known physical bucket height in meters; stored beside the depth-derived estimate",
    )
    parser.add_argument("--width", type=int, default=COLOR_WIDTH)
    parser.add_argument("--height", type=int, default=COLOR_HEIGHT)
    parser.add_argument("--fps", type=int, default=FRAME_RATE)
    parser.add_argument(
        "--video-fps",
        type=int,
        default=None,
        help="Output MP4 timeline FPS; default follows camera --fps",
    )
    parser.add_argument(
        "--video-bitrate",
        type=int,
        default=8_000_000,
        help="Jetson H.264 bitrate in bits/s; default: 8000000",
    )
    parser.add_argument(
        "--software-encoder",
        action="store_true",
        help="Force the CPU mp4v fallback instead of GStreamer software H.264",
    )
    parser.add_argument("--depth-width", type=int, default=DEPTH_WIDTH)
    parser.add_argument("--depth-height", type=int, default=DEPTH_HEIGHT)
    parser.add_argument(
        "--imgsz",
        type=int,
        default=None,
        help="YOLO inference size; default: 640",
    )
    parser.add_argument(
        "--device",
        default=None,
        help="Ultralytics device, for example 0 or cpu; default lets Ultralytics choose",
    )
    parser.add_argument("--half", action="store_true", help="Use FP16 inference on a compatible CUDA device")
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=15,
        help="RGB-D frames discarded before inference starts",
    )
    # Retained only so older launch commands do not fail; recording is always headless.
    parser.add_argument("--headless", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def blank_csv_row(frame_data: dict, detection_index="", status="no_detection") -> dict:
    row = {field: "" for field in CSV_FIELDS}
    row.update(frame_data)
    row["detection_index"] = detection_index
    row["status"] = status
    return row


def put_text_with_outline(image: np.ndarray, text: str, origin: tuple[int, int], scale=0.55) -> None:
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 255, 255), 2, cv2.LINE_AA)


def create_video_writer(
    path: Path,
    width: int,
    height: int,
    fps: int,
    bitrate: int,
    software_encoder: bool,
) -> tuple[cv2.VideoWriter, str]:
    path.parent.mkdir(parents=True, exist_ok=True)
    gstreamer_enabled = any(
        "GStreamer" in line and "YES" in line
        for line in cv2.getBuildInformation().splitlines()
    )
    escaped_path = str(path).replace("\\", "\\\\").replace('"', '\\"')
    if gstreamer_enabled and not software_encoder:
        keyframe_interval = max(1, fps * 2)
        x264_bitrate_kbps = max(100, int(round(bitrate / 1000.0)))
        pipeline = (
            "appsrc ! queue max-size-buffers=4 ! "
            "videoconvert ! video/x-raw,format=I420 ! "
            f"x264enc bitrate={x264_bitrate_kbps} speed-preset=ultrafast "
            f"tune=zerolatency key-int-max={keyframe_interval} ! "
            "h264parse ! qtmux ! "
            f'filesink location="{escaped_path}" sync=false'
        )
        writer = cv2.VideoWriter(
            pipeline,
            cv2.CAP_GSTREAMER,
            0,
            float(fps),
            (width, height),
            True,
        )
        if writer.isOpened():
            return writer, "GStreamer x264enc H.264 (CPU)"
        writer.release()
        print(
            "Warning: GStreamer x264enc writer unavailable; falling back to CPU mp4v.",
            file=sys.stderr,
        )

    writer = cv2.VideoWriter(
        str(path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        float(fps),
        (width, height),
    )
    if not writer.isOpened():
        if gstreamer_enabled:
            raise RuntimeError(
                "Unable to open GStreamer x264enc or CPU mp4v writer. "
                "Install gstreamer1.0-plugins-ugly and verify mp4v support."
            )
        raise RuntimeError("Unable to open CPU mp4v VideoWriter")
    return writer, "CPU mp4v"


TIMING_KEYS = (
    "capture",
    "align",
    "convert",
    "yolo_wall",
    "yolo_preprocess",
    "yolo_inference",
    "yolo_postprocess",
    "analysis_overlay",
    "video_write",
    "csv_write",
    "total_frame",
)


def add_timing_sample(totals: dict[str, float], sample: dict[str, float]) -> None:
    for key in TIMING_KEYS:
        totals[key] += sample[key]


def timing_average_text(totals: dict[str, float], count: int) -> str:
    if count <= 0:
        return "no completed frames"
    average = {key: totals[key] / count for key in TIMING_KEYS}
    effective_fps = 1000.0 / average["total_frame"] if average["total_frame"] > 0.0 else 0.0
    return (
        f"frames={count}, effective_fps={effective_fps:.2f}, "
        f"capture={average['capture']:.1f} ms, align={average['align']:.1f} ms, "
        f"convert={average['convert']:.1f} ms, yolo_wall={average['yolo_wall']:.1f} ms "
        f"[pre={average['yolo_preprocess']:.1f}, infer={average['yolo_inference']:.1f}, "
        f"post={average['yolo_postprocess']:.1f}], "
        f"analysis+overlay={average['analysis_overlay']:.1f} ms, "
        f"video_write={average['video_write']:.1f} ms, csv_write={average['csv_write']:.1f} ms, "
        f"total={average['total_frame']:.1f} ms"
    )


def main() -> int:
    args = parse_args()
    using_tensorrt = Path(args.model).suffix.lower() == ".engine"
    if args.imgsz is None:
        args.imgsz = INFERENCE_IMAGE_SIZE
    video_fps = args.fps if args.video_fps is None else args.video_fps
    if args.warmup_frames < 0:
        raise ValueError("--warmup-frames must be zero or positive")
    if args.imgsz <= 0:
        raise ValueError("--imgsz must be positive")
    if video_fps <= 0:
        raise ValueError("--video-fps must be positive")
    if args.video_bitrate <= 0:
        raise ValueError("--video-bitrate must be positive")
    model_load_start = time.perf_counter()
    print(f"Loading YOLO model: {args.model}", flush=True)
    model = YOLO(args.model, task="segment")
    print(f"YOLO model loaded in {time.perf_counter() - model_load_start:.2f} s.", flush=True)
    output_dir = Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parent / "basket_video"
    output_dir.mkdir(parents=True, exist_ok=True)

    try:
        pipeline, profile, color_format, color_format_name = start_rgbd_stream(args)
    except RuntimeError as error:
        print(f"Unable to start D435i: {error}", file=sys.stderr)
        return 1

    device = profile.get_device()
    require_usb3(device)
    configure_camera(device, args.exposure)
    depth_scale = device.first_depth_sensor().get_depth_scale()
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
    align_to_color = rs.align(rs.stream.color)

    print(f"Warming up RGB-D stream for {args.warmup_frames} frames ...", flush=True)
    for _ in range(args.warmup_frames):
        pipeline.wait_for_frames(5000)
    print("RGB-D stream warm-up complete.", flush=True)

    session_name = f"basket_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    video_path = output_dir / f"{session_name}.mp4"
    csv_path = output_dir / f"{session_name}.csv"
    try:
        writer, video_encoder_name = create_video_writer(
            video_path,
            intrinsics.width,
            intrinsics.height,
            video_fps,
            args.video_bitrate,
            args.software_encoder,
        )
    except RuntimeError as error:
        pipeline.stop()
        print(f"Unable to open video writer: {error}", file=sys.stderr)
        return 1

    print(
        f"Color intrinsics: {intrinsics.width}x{intrinsics.height}, "
        f"fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}, "
        f"ppx={intrinsics.ppx:.2f}, ppy={intrinsics.ppy:.2f}"
    )
    print(f"Model: {args.model}")
    print(f"Color format: {color_format_name}")
    precision_text = "engine-defined" if using_tensorrt else str(args.half)
    print(
        f"YOLO inference: imgsz={args.imgsz}, device={args.device or 'auto'}, "
        f"half={precision_text}"
    )
    print(f"Manual color exposure: {args.exposure:g}")
    print(f"Video encoder: {video_encoder_name}")
    print(
        f"Video timeline: {video_fps} FPS; camera timestamps fill skipped intervals "
        "with repeated annotated frames"
    )
    print(f"Annotated video: {video_path}")
    print(f"Diagnostics CSV: {csv_path}")
    print("Preview window disabled; press Ctrl+C to stop recording.")

    start_clock = time.perf_counter()
    previous_frame_clock = None
    fps_ema = 0.0
    frame_index = 0
    completed_frames = 0
    steady_frames = 0
    timing_totals = {key: 0.0 for key in TIMING_KEYS}
    steady_timing_totals = {key: 0.0 for key in TIMING_KEYS}
    last_progress_clock = start_clock
    video_camera_start_ms = None
    video_frames_written_total = 0

    try:
        with csv_path.open("w", newline="", encoding="utf-8-sig") as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            csv_writer.writeheader()

            while True:
                frame_cycle_start = time.perf_counter()
                capture_start = time.perf_counter()
                frames = pipeline.wait_for_frames(5000)
                capture_wait_ms = (time.perf_counter() - capture_start) * 1000.0
                align_start = time.perf_counter()
                aligned_frames = align_to_color.process(frames)
                align_ms = (time.perf_counter() - align_start) * 1000.0
                color_frame = aligned_frames.get_color_frame()
                depth_frame = aligned_frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue

                frame_clock = time.perf_counter()
                frame_index += 1
                elapsed_s = frame_clock - start_clock
                fps_instant = 0.0 if previous_frame_clock is None else 1.0 / max(1.0e-6, frame_clock - previous_frame_clock)
                previous_frame_clock = frame_clock
                if fps_instant > 0.0:
                    fps_ema = fps_instant if fps_ema == 0.0 else 0.90 * fps_ema + 0.10 * fps_instant
                fps_average = frame_index / max(elapsed_s, 1.0e-6)

                convert_start = time.perf_counter()
                color_image = frame_to_bgr(color_frame, color_format)
                depth_raw = np.asanyarray(depth_frame.get_data())
                frame_convert_ms = (time.perf_counter() - convert_start) * 1000.0
                inference_start = time.perf_counter()
                if frame_index == 1:
                    print("First RGB-D frame received; starting first YOLO inference ...", flush=True)
                predict_args = {
                    "classes": [TARGET_CLASS],
                    "conf": args.conf,
                    "iou": NMS_IOU_THRESHOLD,
                    "imgsz": args.imgsz,
                    "verbose": False,
                }
                if not using_tensorrt:
                    predict_args["half"] = args.half
                if args.device is not None:
                    predict_args["device"] = args.device
                result = model(color_image, **predict_args)[0]
                inference_ms = (time.perf_counter() - inference_start) * 1000.0
                result_speed = result.speed or {}
                yolo_preprocess_ms = float(result_speed.get("preprocess", 0.0))
                yolo_backend_inference_ms = float(result_speed.get("inference", 0.0))
                yolo_postprocess_ms = float(result_speed.get("postprocess", 0.0))
                if frame_index == 1:
                    print(f"First YOLO inference complete in {inference_ms:.1f} ms.", flush=True)
                analysis_start = time.perf_counter()
                display = color_image.copy()
                selected_candidates = []
                raw_detection_count = 0

                if result.masks is not None and result.boxes is not None:
                    masks = result.masks.data.cpu().numpy()
                    confidences = result.boxes.conf.cpu().numpy()
                    boxes = result.boxes.xyxy.cpu().numpy()
                    raw_detection_count = len(boxes)
                    candidates = []
                    for mask, confidence, box in zip(masks, confidences, boxes):
                        if mask.shape != color_image.shape[:2]:
                            mask = cv2.resize(
                                mask,
                                (color_image.shape[1], color_image.shape[0]),
                                interpolation=cv2.INTER_NEAREST,
                            )
                        candidates.append({"mask": mask, "confidence": float(confidence), "box": box})
                    selected_candidates = remove_duplicate_masks(candidates)

                camera_timestamp_ms = float(color_frame.get_timestamp())
                frame_data = {
                    "host_time_iso": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    "camera_timestamp_ms": camera_timestamp_ms,
                    "frame_index": frame_index,
                    "elapsed_s": elapsed_s,
                    "fps_instant": fps_instant,
                    "fps_ema": fps_ema,
                    "fps_average": fps_average,
                    "video_timeline_fps": video_fps,
                    "capture_wait_ms": capture_wait_ms,
                    "depth_align_ms": align_ms,
                    "frame_convert_ms": frame_convert_ms,
                    "inference_ms": inference_ms,
                    "yolo_preprocess_ms": yolo_preprocess_ms,
                    "yolo_backend_inference_ms": yolo_backend_inference_ms,
                    "yolo_postprocess_ms": yolo_postprocess_ms,
                    "raw_detection_count": raw_detection_count,
                    "selected_detection_count": len(selected_candidates),
                    "color_width_px": color_image.shape[1],
                    "color_height_px": color_image.shape[0],
                    "color_format": color_format_name,
                    "depth_width_px": depth_raw.shape[1],
                    "depth_height_px": depth_raw.shape[0],
                    "depth_scale_m_per_unit": depth_scale,
                    "color_exposure_setting": args.exposure,
                    "intrinsics_fx_px": intrinsics.fx,
                    "intrinsics_fy_px": intrinsics.fy,
                    "intrinsics_ppx_px": intrinsics.ppx,
                    "intrinsics_ppy_px": intrinsics.ppy,
                    "inference_image_size_px": args.imgsz,
                    "nms_iou_threshold": NMS_IOU_THRESHOLD,
                }

                occupied_mask = np.zeros(color_image.shape[:2], dtype=np.uint8)
                for candidate in selected_candidates:
                    occupied_mask[candidate["mask"] > 0] = 255

                frame_ground_mask = np.zeros(depth_raw.shape, dtype=np.uint8)
                frame_y0, frame_y1 = depth_raw.shape[0] // 4, depth_raw.shape[0] * 3 // 4
                frame_x0, frame_x1 = depth_raw.shape[1] // 4, depth_raw.shape[1] * 3 // 4
                frame_ground_mask[frame_y0:frame_y1, frame_x0:frame_x1] = 255
                frame_ground_mask[occupied_mask > 0] = 0
                frame_ground_stats = depth_summary(depth_raw, frame_ground_mask, depth_scale)
                frame_data.update({
                    "frame_ground_depth_median_approx_m": frame_ground_stats["median"],
                    "frame_ground_depth_iqr_m": frame_ground_stats["iqr"],
                    "frame_ground_depth_valid_ratio": frame_ground_stats["valid_ratio"],
                })

                rows = []
                for detection_index, candidate in enumerate(selected_candidates):
                    mask = candidate["mask"]
                    confidence = candidate["confidence"]
                    box = candidate["box"].astype(int)
                    x1, y1, x2, y2 = box
                    row = blank_csv_row(frame_data, detection_index, "ellipse_fit_failed")
                    mask_area = int(np.count_nonzero(mask > 0))
                    row.update({
                        "confidence": confidence,
                        "bbox_x1_px": x1,
                        "bbox_y1_px": y1,
                        "bbox_x2_px": x2,
                        "bbox_y2_px": y2,
                        "bbox_width_px": max(0, x2 - x1),
                        "bbox_height_px": max(0, y2 - y1),
                        "mask_area_px": mask_area,
                        "mask_area_ratio": mask_area / float(mask.size),
                        "bucket_height_config_m": args.bucket_height,
                    })

                    binary_mask = mask > 0
                    display[binary_mask] = (
                        0.55 * display[binary_mask] + 0.45 * np.array([0, 180, 0])
                    ).astype(np.uint8)
                    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    fitted = fit_ellipse_from_mask(mask)
                    if fitted is None:
                        put_text_with_outline(display, f"bucket {confidence:.2f} ellipse failed", (x1, max(25, y1 - 8)))
                        rows.append(row)
                        continue

                    ellipse, _ = fitted
                    (center_x, center_y), (axis_a, axis_b), angle_deg = ellipse
                    u, v = int(round(center_x)), int(round(center_y))
                    row.update({
                        "ellipse_center_u_px": center_x,
                        "ellipse_center_v_px": center_y,
                        "ellipse_axis_a_px": axis_a,
                        "ellipse_axis_b_px": axis_b,
                        "ellipse_angle_deg": angle_deg,
                        "diameter_axis_ratio": max(axis_a, axis_b) / max(1.0e-6, min(axis_a, axis_b)),
                    })
                    if not (0 <= u < color_image.shape[1] and 0 <= v < color_image.shape[0]):
                        row["status"] = "ellipse_center_outside_image"
                        rows.append(row)
                        continue

                    rim_mask = ellipse_band_mask(depth_raw.shape, ellipse, RIM_BAND_THICKNESS_PIXELS)
                    rim_stats = depth_summary(depth_raw, rim_mask, depth_scale)
                    mask_stats = depth_summary(depth_raw, (mask > 0).astype(np.uint8), depth_scale)
                    ground_mask = ground_ring_mask(depth_raw.shape, ellipse, occupied_mask)
                    ground_stats = depth_summary(depth_raw, ground_mask, depth_scale)
                    center_depth = filtered_depth_meters(depth_raw, u, v, depth_scale)

                    highest_wall_depth = rim_stats["p15"]
                    if highest_wall_depth is None:
                        highest_wall_depth = center_depth
                    if highest_wall_depth is None:
                        highest_wall_depth = mask_stats["median"]

                    ground_depth = ground_stats["median"]
                    bucket_height_estimate = None
                    if ground_depth is not None and highest_wall_depth is not None:
                        bucket_height_estimate = ground_depth - highest_wall_depth

                    row.update({
                        "rim_depth_p15_m": rim_stats["p15"],
                        "rim_depth_median_m": rim_stats["median"],
                        "rim_depth_iqr_m": rim_stats["iqr"],
                        "rim_depth_std_m": rim_stats["std"],
                        "rim_depth_valid_count": rim_stats["valid_count"],
                        "rim_depth_valid_ratio": rim_stats["valid_ratio"],
                        "center_depth_m": center_depth,
                        "mask_depth_median_m": mask_stats["median"],
                        "mask_depth_iqr_m": mask_stats["iqr"],
                        "mask_depth_valid_ratio": mask_stats["valid_ratio"],
                        "ground_depth_median_m": ground_depth,
                        "ground_depth_iqr_m": ground_stats["iqr"],
                        "ground_depth_std_m": ground_stats["std"],
                        "ground_depth_valid_count": ground_stats["valid_count"],
                        "ground_depth_valid_ratio": ground_stats["valid_ratio"],
                        "camera_ground_distance_approx_m": ground_depth,
                        "bucket_height_estimate_approx_m": bucket_height_estimate,
                    })

                    if highest_wall_depth is None or highest_wall_depth <= args.bucket_height:
                        row["status"] = "invalid_bucket_depth"
                        put_text_with_outline(display, f"bucket {confidence:.2f} depth invalid", (x1, max(25, y1 - 8)))
                        rows.append(row)
                        continue

                    # Keep the original depth correction so recordings remain comparable.
                    corrected_depth = highest_wall_depth - args.bucket_height
                    camera_xyz = pixel_to_camera_xyz(u, v, corrected_depth, intrinsics)
                    world_xyz = camera_to_world(camera_xyz)
                    legacy_diameter = max(axis_a, axis_b) * highest_wall_depth / intrinsics.fx
                    diameter_a, diameter_b = ellipse_axis_diameters_meters(ellipse, highest_wall_depth, intrinsics)

                    row.update({
                        "status": "ok",
                        "corrected_depth_m": corrected_depth,
                        "camera_x_m": camera_xyz[0],
                        "camera_y_m": camera_xyz[1],
                        "camera_z_m": camera_xyz[2],
                        "world_x_m": world_xyz[0],
                        "world_y_m": world_xyz[1],
                        "world_z_m": world_xyz[2],
                        "diameter_reported_m": legacy_diameter,
                        "diameter_legacy_major_m": legacy_diameter,
                        "diameter_axis_a_deprojected_m": diameter_a,
                        "diameter_axis_b_deprojected_m": diameter_b,
                    })
                    rows.append(row)

                    cv2.ellipse(display, ellipse, (0, 255, 255), 2)
                    cv2.circle(display, (u, v), 5, (0, 0, 255), -1)
                    label_y = max(24, y1 - 34)
                    put_text_with_outline(display, f"bucket conf={confidence:.2f} D={legacy_diameter:.3f}m", (x1, label_y))
                    height_text = "Hest=n/a" if bucket_height_estimate is None else f"Hest={bucket_height_estimate:.3f}m"
                    ground_text = "ground=n/a" if ground_depth is None else f"ground={ground_depth:.3f}m"
                    put_text_with_outline(display, f"{ground_text} {height_text}", (x1, label_y + 24), 0.50)
                    put_text_with_outline(
                        display,
                        f"Cam XYZ=({camera_xyz[0]:.3f}, {camera_xyz[1]:.3f}, {camera_xyz[2]:.3f})m",
                        (x1, label_y + 48),
                        0.50,
                    )

                if not rows:
                    rows.append(blank_csv_row(frame_data))

                fps_text = "warming up" if frame_index == 1 else f"{fps_ema:.1f} (avg {fps_average:.1f})"
                put_text_with_outline(
                    display,
                    f"FPS {fps_text}  infer {inference_ms:.1f} ms",
                    (20, 34),
                    0.72,
                )
                put_text_with_outline(
                    display,
                    f"detections {len(selected_candidates)}  conf >= {args.conf:.2f}",
                    (20, 65),
                    0.62,
                )
                put_text_with_outline(
                    display,
                    f"capture {capture_wait_ms:.1f} ms  align {align_ms:.1f} ms",
                    (20, 94),
                    0.56,
                )
                analysis_overlay_ms = (time.perf_counter() - analysis_start) * 1000.0
                write_start = time.perf_counter()
                if video_camera_start_ms is None:
                    video_camera_start_ms = camera_timestamp_ms
                    target_video_frames = 1
                else:
                    camera_elapsed_s = max(
                        0.0,
                        (camera_timestamp_ms - video_camera_start_ms) / 1000.0,
                    )
                    target_video_frames = max(
                        video_frames_written_total + 1,
                        int(round(camera_elapsed_s * video_fps)) + 1,
                    )

                # Limit a timestamp discontinuity to one second of catch-up data.
                target_video_frames = min(
                    target_video_frames,
                    video_frames_written_total + max(1, video_fps),
                )
                video_frames_written_this_cycle = max(
                    1,
                    target_video_frames - video_frames_written_total,
                )
                for _ in range(video_frames_written_this_cycle):
                    writer.write(display)
                video_frames_written_total += video_frames_written_this_cycle
                video_write_ms = (time.perf_counter() - write_start) * 1000.0

                for row in rows:
                    row["analysis_overlay_ms"] = analysis_overlay_ms
                    row["video_write_ms"] = video_write_ms
                    row["video_frames_written_this_cycle"] = video_frames_written_this_cycle
                    row["video_frames_written_total"] = video_frames_written_total
                csv_write_start = time.perf_counter()
                for row in rows:
                    csv_writer.writerow({key: "" if value is None else value for key, value in row.items()})
                if frame_index % CSV_FLUSH_INTERVAL_FRAMES == 0:
                    csv_file.flush()
                csv_write_ms = (time.perf_counter() - csv_write_start) * 1000.0

                total_frame_ms = (time.perf_counter() - frame_cycle_start) * 1000.0
                timing_sample = {
                    "capture": capture_wait_ms,
                    "align": align_ms,
                    "convert": frame_convert_ms,
                    "yolo_wall": inference_ms,
                    "yolo_preprocess": yolo_preprocess_ms,
                    "yolo_inference": yolo_backend_inference_ms,
                    "yolo_postprocess": yolo_postprocess_ms,
                    "analysis_overlay": analysis_overlay_ms,
                    "video_write": video_write_ms,
                    "csv_write": csv_write_ms,
                    "total_frame": total_frame_ms,
                }
                add_timing_sample(timing_totals, timing_sample)
                completed_frames += 1
                if completed_frames > 1:
                    add_timing_sample(steady_timing_totals, timing_sample)
                    steady_frames += 1

                progress_clock = time.perf_counter()
                if progress_clock - last_progress_clock >= 5.0:
                    print(f"[TIMING] {timing_average_text(timing_totals, completed_frames)}", flush=True)
                    last_progress_clock = progress_clock
    except KeyboardInterrupt:
        print("Recording stopped by user.")
    finally:
        writer.release()
        pipeline.stop()
        print("\nFinal average timing (all completed frames):")
        print(f"  {timing_average_text(timing_totals, completed_frames)}")
        print("Final steady-state timing (first inference frame excluded):")
        print(f"  {timing_average_text(steady_timing_totals, steady_frames)}")
        video_duration_s = video_frames_written_total / float(video_fps)
        print(
            f"Video timeline: {video_frames_written_total} frames at {video_fps} FPS "
            f"({video_duration_s:.2f} s)"
        )

    print(f"Saved video: {video_path}")
    print(f"Saved CSV: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
