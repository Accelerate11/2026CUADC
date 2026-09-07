#!/usr/bin/env python3
"""Check that the current packaged calibration is visible in the CUADC workspace."""
import argparse
from pathlib import Path

EXPECTED = {
    "camera_xy_calib_bias_m": ["0.047805", "-0.014577"],
    "camera_optical_to_body_rotation": ["-0.971358", "0.237622", "0.971358", "-1.0"],
    "payload_release_offsets_body_m": ["0.026", "-0.065", "0.0109", "0.0720", "-0.320"],
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", type=Path, default=Path.home() / "cuadc_mission")
    args = ap.parse_args()
    ws = args.workspace.expanduser().resolve()
    cpp = ws / "src/cuadc_visual_drop_flight/src/visual_drop_mission_node.cpp"
    yaml_path = ws / "src/cuadc_visual_drop_flight/config/flight_params.yaml"
    vision = ws / "src/cuadc_visual_drop_flight/scripts/drop_bucket_realsense_node.py"
    ok = True
    for p in (cpp, yaml_path, vision):
        if not p.is_file():
            print("[FAIL] missing", p)
            ok = False
    if not ok:
        raise SystemExit(2)

    y = yaml_path.read_text(encoding="utf-8")
    s = cpp.read_text(encoding="utf-8")
    v = vision.read_text(encoding="utf-8")

    for key, tokens in EXPECTED.items():
        target = y if key.startswith("camera_") else (y + "\n" + s)
        matched = key in target and all(t in target for t in tokens)
        print(f"[{'OK' if matched else 'FAIL'}] {key}")
        ok = ok and matched

    alt_ok = "fine_align_alt_m" in (y + s) and "1.30" in (y + s)
    print(f"[{'OK' if alt_ok else 'FAIL'}] fine_align_alt_m=1.30")
    ok = ok and alt_ok

    overlay = "calibration_overlay_enabled" in v or "CUADC_HEADLESS_OVERLAY_V1" in v
    print(f"[{'OK' if overlay else '--'}] visual overlay hook")
    if not ok:
        raise SystemExit(2)
    print("[SUCCESS] current packaged calibration is present")


if __name__ == "__main__":
    main()
