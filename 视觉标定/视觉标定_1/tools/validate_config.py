#!/usr/bin/env python3
"""Validate alignment and/or optional MAVROS servo configuration."""
import argparse
import math
from pathlib import Path

import yaml


def require(value, name):
    if value is None or value == "" or value == []:
        raise ValueError(f"{name} is not configured")
    return value


def finite_vector(value, size, name):
    value = require(value, name)
    if not isinstance(value, list) or len(value) != size:
        raise ValueError(f"{name} must contain {size} values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def validate_rotation(values):
    rows = [values[0:3], values[3:6], values[6:9]]
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))
    if any(abs(dot(row, row) - 1.0) > 1e-3 for row in rows):
        raise ValueError("camera rotation rows must have unit length")
    if any(abs(dot(rows[i], rows[j])) > 1e-3 for i in range(3) for j in range(i + 1, 3)):
        raise ValueError("camera rotation rows must be orthogonal")
    det = (
        rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
        - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
        + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0])
    )
    if abs(det - 1.0) > 1e-3:
        raise ValueError("camera rotation determinant must be +1")


def validate_alignment(data):
    camera = data["camera"]
    vision = data["vision"]
    payloads = data["payloads"]
    target = data["target"]
    require(camera.get("image_topic"), "camera.image_topic")
    require(camera.get("camera_info_topic"), "camera.camera_info_topic")
    rotation = finite_vector(camera.get("rotation_camera_to_frd"), 9, "camera.rotation_camera_to_frd")
    validate_rotation(rotation)
    finite_vector(camera.get("translation_frd_m"), 3, "camera.translation_frd_m")
    finite_vector(payloads.get("release_offsets_frd_m"), 6, "payloads.release_offsets_frd_m")
    require(vision.get("detections_topic"), "vision.detections_topic")
    require(vision.get("frame_id"), "vision.frame_id")
    height = float(require(target.get("height_m"), "target.height_m"))
    if not math.isfinite(height) or height < 0.0:
        raise ValueError("target.height_m must be finite and non-negative")


def validate_servo(data):
    fcu = data.get("fcu", {}) or {}
    payloads = data["payloads"]
    require(fcu.get("device"), "fcu.device")
    baud = int(require(fcu.get("baud"), "fcu.baud"))
    if baud <= 0:
        raise ValueError("fcu.baud must be positive")
    for key in ("channels", "stowed_pwm", "release_pwm", "release_duration_s"):
        values = finite_vector(payloads.get(key), 2, "payloads." + key)
        if key == "channels" and any(int(item) <= 0 for item in values):
            raise ValueError("payload channels must be positive")
    durations = [float(v) for v in payloads["release_duration_s"]]
    if any(v < 0.0 for v in durations):
        raise ValueError("payload release durations must be non-negative")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--mode", choices=("alignment", "servo", "all"), default="alignment")
    args = parser.parse_args()
    data = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    if args.mode in ("alignment", "all"):
        validate_alignment(data)
    if args.mode in ("servo", "all"):
        validate_servo(data)
    print(f"OK: {args.config} ({args.mode})")


if __name__ == "__main__":
    main()
