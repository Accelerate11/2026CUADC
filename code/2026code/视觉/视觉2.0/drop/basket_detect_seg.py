"""D435i + YOLO segmentation: fit the bucket mouth ellipse and report its 3D position."""

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


TARGET_CLASS = 0
CONFIDENCE_THRESHOLD = 0.25
INFERENCE_IMAGE_SIZE = 1280
NMS_IOU_THRESHOLD = 0.01
DUPLICATE_MASK_IOU_THRESHOLD = 0.20
MODEL_PATH = "basket-seg.pt"
DEPTH_FILTER_SIZE = 5
BUCKET_HEIGHT_METERS = 0.30  # Subtract this from the measured bottom/ground depth to obtain the bucket-mouth depth.
RIM_BAND_THICKNESS_PIXELS = 11
RIM_DEPTH_PERCENTILE = 15  # Near-side percentile rejects isolated invalid/outlier depth values.
COLOR_WIDTH, COLOR_HEIGHT, FRAME_RATE = 1920, 1080, 30
DEPTH_WIDTH, DEPTH_HEIGHT = 1280, 720  # D435i maximum depth profile at 30 FPS.
POWER_LINE_FREQUENCY_HZ = 50

# Camera-to-world extrinsics. Replace these with calibrated values before using world coordinates.
# Point convention: world_xyz = CAMERA_TO_WORLD_R @ camera_xyz + CAMERA_TO_WORLD_T
CAMERA_TO_WORLD_R = np.eye(3, dtype=np.float64)
CAMERA_TO_WORLD_T = np.zeros(3, dtype=np.float64)


def fit_ellipse_from_mask(mask: np.ndarray):
    """Fit an ellipse to the largest external contour of one binary YOLO mask."""
    binary_mask = (mask > 0).astype(np.uint8) * 255
    contours, _ = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    usable = [contour for contour in contours if len(contour) >= 5 and cv2.contourArea(contour) > 30]
    if not usable:
        return None
    contour = max(usable, key=cv2.contourArea)
    return cv2.fitEllipse(contour), contour


def filtered_depth_meters(depth_raw: np.ndarray, u: int, v: int, depth_scale: float) -> float | None:
    """Use the local median to reject invalid and noisy depth pixels at the ellipse center."""
    half = DEPTH_FILTER_SIZE // 2
    y0, y1 = max(0, v - half), min(depth_raw.shape[0], v + half + 1)
    x0, x1 = max(0, u - half), min(depth_raw.shape[1], u + half + 1)
    values = depth_raw[y0:y1, x0:x1].astype(np.float64) * depth_scale
    values = values[(values > 0.10) & (values < 10.0)]
    return float(np.median(values)) if values.size else None


def mask_depth_meters(mask: np.ndarray, depth_raw: np.ndarray, depth_scale: float) -> float | None:
    """Fallback depth estimate from all valid pixels in the fitted object mask."""
    if mask.shape != depth_raw.shape:
        mask = cv2.resize(mask.astype(np.uint8), (depth_raw.shape[1], depth_raw.shape[0]), interpolation=cv2.INTER_NEAREST)
    values = depth_raw[mask.astype(bool)].astype(np.float64) * depth_scale
    values = values[(values > 0.10) & (values < 10.0)]
    return float(np.median(values)) if values.size else None


def rim_depth_meters(ellipse, depth_raw: np.ndarray, depth_scale: float) -> float | None:
    """Estimate the highest visible bucket-wall depth from a band around the fitted mouth ellipse."""
    rim_band = np.zeros(depth_raw.shape, dtype=np.uint8)
    cv2.ellipse(rim_band, ellipse, 255, RIM_BAND_THICKNESS_PIXELS)
    values = depth_raw[rim_band > 0].astype(np.float64) * depth_scale
    values = values[(values > 0.10) & (values < 10.0)]
    return float(np.percentile(values, RIM_DEPTH_PERCENTILE)) if values.size else None


def mask_iou(first: np.ndarray, second: np.ndarray) -> float:
    first_binary = first > 0
    second_binary = second > 0
    union = np.count_nonzero(first_binary | second_binary)
    if union == 0:
        return 0.0
    return float(np.count_nonzero(first_binary & second_binary) / union)


def boxes_overlap(first: np.ndarray, second: np.ndarray) -> bool:
    """Return true for any non-zero intersection; buckets are known not to overlap."""
    left = max(float(first[0]), float(second[0]))
    top = max(float(first[1]), float(second[1]))
    right = min(float(first[2]), float(second[2]))
    bottom = min(float(first[3]), float(second[3]))
    return right > left and bottom > top


def remove_duplicate_masks(candidates: list[dict]) -> list[dict]:
    """Keep only the highest-confidence result for every overlapping bucket box."""
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


def pixel_to_camera_xyz(u: int, v: int, depth_meters: float, intrinsics: rs.intrinsics) -> np.ndarray:
    point = rs.rs2_deproject_pixel_to_point(intrinsics, [float(u), float(v)], depth_meters)
    return np.asarray(point, dtype=np.float64)


def camera_to_world(camera_xyz: np.ndarray) -> np.ndarray:
    return CAMERA_TO_WORLD_R @ camera_xyz + CAMERA_TO_WORLD_T


def set_option(sensor: rs.sensor, option: rs.option, value: float, name: str) -> None:
    """Set an available RealSense option and report unsupported firmware options."""
    if sensor.supports(option):
        sensor.set_option(option, value)
    else:
        print(f"Warning: D435i does not support {name}; skipped.")


def configure_camera(device: rs.device) -> None:
    """Apply the requested manual D435i color-camera configuration."""
    color_sensor = device.first_color_sensor()

    # Disable auto exposure before applying manual exposure and gain.
    set_option(color_sensor, rs.option.enable_auto_exposure, 0.0, "enable_auto_exposure")
    set_option(color_sensor, rs.option.exposure, 120.0, "exposure")
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

    # RealSense SDK: 1 = 50 Hz, 2 = 60 Hz. China mains frequency is 50 Hz.
    power_line_value = 1.0 if POWER_LINE_FREQUENCY_HZ == 50 else 2.0
    set_option(color_sensor, rs.option.power_line_frequency, power_line_value, "power_line_frequency")

    # Global time can be exposed by either the color or depth sensor, depending on firmware.
    for sensor in device.query_sensors():
        if sensor.supports(rs.option.global_time_enabled):
            sensor.set_option(rs.option.global_time_enabled, 1.0)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Ubuntu D435i YOLO segmentation video recorder")
    parser.add_argument("--model", default=MODEL_PATH, help="Path to the YOLO segmentation .pt file")
    parser.add_argument("--output-dir", default=None, help="Video output directory; default is basket_video beside this script")
    parser.add_argument("--conf", type=float, default=CONFIDENCE_THRESHOLD)
    parser.add_argument("--width", type=int, default=COLOR_WIDTH)
    parser.add_argument("--height", type=int, default=COLOR_HEIGHT)
    parser.add_argument("--fps", type=int, default=FRAME_RATE)
    parser.add_argument("--display", action="store_true", help="Show a window; omit on headless SSH systems")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = YOLO(args.model)
    output_dir = Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parent / "basket_video"
    output_dir.mkdir(parents=True, exist_ok=True)
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FRAME_RATE)

    try:
        profile = pipeline.start(config)
    except RuntimeError as error:
        print(f"Unable to start D435i: {error}", file=sys.stderr)
        return 1

    device = profile.get_device()
    configure_camera(device)
    depth_scale = device.first_depth_sensor().get_depth_scale()
    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().intrinsics
    print(
        f"Color intrinsics: {intrinsics.width}x{intrinsics.height}, "
        f"fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}, "
        f"ppx={intrinsics.ppx:.2f}, ppy={intrinsics.ppy:.2f}"
    )
    align_to_color = rs.align(rs.stream.color)
    video_path = output_dir / f"basket_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
    writer = cv2.VideoWriter(
        str(video_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (intrinsics.width, intrinsics.height),
    )
    if not writer.isOpened():
        pipeline.stop()
        print(f"Unable to open video writer: {video_path}", file=sys.stderr)
        return 1
    print(f"Model: {args.model}")
    print(f"Recording annotated video to: {video_path}")
    print("Press Ctrl+C to stop recording.")

    try:
        while True:
            frames = align_to_color.process(pipeline.wait_for_frames())
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_raw = np.asanyarray(depth_frame.get_data())
            result = model(
                color_image,
                classes=[TARGET_CLASS],
                conf=args.conf,
                iou=NMS_IOU_THRESHOLD,
                imgsz=INFERENCE_IMAGE_SIZE,
                verbose=False,
            )[0]
            display = color_image.copy()
            selected_candidates = []

            if result.masks is not None and result.boxes is not None:
                masks = result.masks.data.cpu().numpy()
                confidences = result.boxes.conf.cpu().numpy()
                boxes = result.boxes.xyxy.cpu().numpy()
                candidates = []
                for mask, confidence, box in zip(masks, confidences, boxes):
                    if mask.shape != color_image.shape[:2]:
                        mask = cv2.resize(mask, (color_image.shape[1], color_image.shape[0]), interpolation=cv2.INTER_NEAREST)
                    candidates.append({"mask": mask, "confidence": float(confidence), "box": box})

                selected_candidates = remove_duplicate_masks(candidates)
                cv2.putText(
                    display,
                    f"detections: {len(selected_candidates)}  conf >= {args.conf:.2f}",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

                for candidate in selected_candidates:
                    mask = candidate["mask"]
                    box = candidate["box"].astype(int)
                    confidence = candidate["confidence"]
                    binary_mask = mask > 0
                    display[binary_mask] = (0.55 * display[binary_mask] + 0.45 * np.array([0, 180, 0])).astype(np.uint8)
                    x1, y1, x2, y2 = box
                    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        display,
                        f"cylinder_mouth {confidence:.2f}",
                        (x1, max(25, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    fitted = fit_ellipse_from_mask(mask)
                    if fitted is None:
                        continue
                    ellipse, _ = fitted
                    (center_x, center_y), (axis_a, axis_b), _ = ellipse
                    u, v = int(round(center_x)), int(round(center_y))
                    if not (0 <= u < color_image.shape[1] and 0 <= v < color_image.shape[0]):
                        continue

                    highest_wall_depth_meters = rim_depth_meters(ellipse, depth_raw, depth_scale)
                    if highest_wall_depth_meters is None:
                        highest_wall_depth_meters = filtered_depth_meters(depth_raw, u, v, depth_scale)
                    if highest_wall_depth_meters is None:
                        highest_wall_depth_meters = mask_depth_meters(mask, depth_raw, depth_scale)
                    if highest_wall_depth_meters is None or highest_wall_depth_meters <= BUCKET_HEIGHT_METERS:
                        continue

                    # Preserve the requested correction after finding the highest visible wall ring.
                    depth_meters = highest_wall_depth_meters - BUCKET_HEIGHT_METERS

                    camera_xyz = pixel_to_camera_xyz(u, v, depth_meters, intrinsics)
                    world_xyz = camera_to_world(camera_xyz)
                    # Image scale is determined by the physical, uncorrected wall depth.
                    # The major ellipse axis is the least-foreshortened diameter of the circular mouth.
                    diameter_meters = max(axis_a, axis_b) * highest_wall_depth_meters / intrinsics.fx

                    cv2.ellipse(display, ellipse, (0, 255, 255), 2)
                    cv2.circle(display, (u, v), 5, (0, 0, 255), -1)
                    text = f"W: {world_xyz[0]:.3f}, {world_xyz[1]:.3f}, {world_xyz[2]:.3f} m"
                    cv2.putText(display, text, (u + 10, max(22, v - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(display, f"diameter: {diameter_meters:.3f} m", (u + 10, v + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 2, cv2.LINE_AA)
                    print(
                        f"wall_depth={highest_wall_depth_meters:.3f} m, "
                        f"world_xyz=({world_xyz[0]:.3f}, {world_xyz[1]:.3f}, {world_xyz[2]:.3f}) m, "
                        f"diameter={diameter_meters:.3f} m"
                    )

            cv2.putText(
                display,
                f"detections: {len(selected_candidates)}  conf >= {args.conf:.2f}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            writer.write(display)
            if args.display:
                cv2.imshow("D435i YOLO Segmentation Ellipse", display)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        writer.release()
        pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
