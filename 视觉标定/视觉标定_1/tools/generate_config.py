#!/usr/bin/env python3
"""Generate alignment ROS parameters from the public aircraft config."""
import argparse
import math
import shlex
from pathlib import Path

import yaml


def required(value, name):
    if value is None or value == "" or value == []:
        raise ValueError(f"{name} is not configured")
    return value


def finite_list(value, count, name):
    value = required(value, name)
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f"{name} must contain {count} values")
    result = [float(v) for v in value]
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{name} must contain finite values")
    return result


def validate_rotation(flat):
    rows = [flat[0:3], flat[3:6], flat[6:9]]
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--aircraft", type=Path, default=Path("config/aircraft.yaml"))
    parser.add_argument("--base", type=Path, default=Path("config/alignment_base.yaml"))
    parser.add_argument("--output", type=Path, default=Path("config/alignment_generated.yaml"))
    parser.add_argument("--env-output", type=Path, default=Path("config/runtime.env"))
    args = parser.parse_args()

    aircraft = yaml.safe_load(args.aircraft.read_text(encoding="utf-8"))
    data = yaml.safe_load(args.base.read_text(encoding="utf-8"))

    camera = aircraft["camera"]
    vision = aircraft["vision"]
    payloads = aircraft["payloads"]
    target = aircraft["target"]

    image_topic = str(required(camera.get("image_topic"), "camera.image_topic"))
    info_topic = str(required(camera.get("camera_info_topic"), "camera.camera_info_topic"))
    rotation = finite_list(camera.get("rotation_camera_to_frd"), 9, "camera.rotation_camera_to_frd")
    validate_rotation(rotation)
    translation = finite_list(camera.get("translation_frd_m"), 3, "camera.translation_frd_m")
    offsets = finite_list(payloads.get("release_offsets_frd_m"), 6, "payloads.release_offsets_frd_m")
    detections_topic = str(required(vision.get("detections_topic"), "vision.detections_topic"))
    frame_id = str(required(vision.get("frame_id"), "vision.frame_id"))
    height = float(required(target.get("height_m"), "target.height_m"))
    if not math.isfinite(height) or height < 0.0:
        raise ValueError("target.height_m must be finite and non-negative")

    viewer = data["alignment_viewer"]["ros__parameters"]
    viewer.update({
        "detections_topic": detections_topic,
        "image_topic": image_topic,
        "camera_info_topic": info_topic,
        "expected_frame_id": frame_id,
        "camera_to_body_rotation": rotation,
        "camera_to_body_translation_m": translation,
        "payload_release_offsets_body_m": offsets,
        "target_height_m": height,
    })
    template = data["vision_provider_template"]["ros__parameters"]
    template.update({
        "image_topic": image_topic,
        "output_topic": detections_topic,
        "frame_id": frame_id,
    })
    checker = data["vision_contract_check"]["ros__parameters"]
    checker.update({
        "detections_topic": detections_topic,
        "expected_frame_id": frame_id,
    })

    args.output.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    print(f"Generated {args.output}")

    fcu = aircraft.get("fcu", {}) or {}
    device = fcu.get("device")
    baud = fcu.get("baud")
    if device not in (None, "") and baud not in (None, ""):
        env = {
            "VS_FCU_DEVICE": str(device),
            "VS_FCU_BAUD": str(int(baud)),
        }
        args.env_output.write_text(
            "".join(f"{key}={shlex.quote(value)}\n" for key, value in env.items()),
            encoding="utf-8",
        )
        print(f"Generated optional {args.env_output}")
    elif args.env_output.exists():
        args.env_output.unlink()
        print("FCU not configured; removed stale runtime.env")


if __name__ == "__main__":
    main()
