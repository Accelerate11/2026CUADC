#!/usr/bin/env python3
"""Apply the packaged current calibration to a cuadc_mission workspace."""
import argparse
import re
import shutil
import tempfile
from pathlib import Path

CURRENT = {
    "camera_xy_calib_bias_m": "[0.047805, -0.014577]",
    "camera_optical_to_body_rotation": (
        "[-0.971358, 0.237622, 0.0, 0.237622, 0.971358, 0.0, 0.0, 0.0, -1.0]"
    ),
    "payload_release_offsets_body_m": "[0.026, -0.065, -0.320, 0.0109, 0.0720, -0.320]",
    "fine_align_alt_m": "1.30",
}


def replace_yaml_scalar(text: str, key: str, value: str):
    lines = text.splitlines(True)
    out = []
    found = 0
    i = 0
    key_re = re.compile(rf"^(\s*){re.escape(key)}\s*:")
    while i < len(lines):
        m = key_re.match(lines[i])
        if not m:
            out.append(lines[i])
            i += 1
            continue
        found += 1
        indent = m.group(1)
        base = len(indent)
        out.append(f"{indent}{key}: {value}\n")
        i += 1
        while i < len(lines):
            line = lines[i]
            if not line.strip():
                out.append(line)
                i += 1
                continue
            current_indent = len(line) - len(line.lstrip())
            if current_indent <= base:
                break
            if line.lstrip().startswith("-"):
                i += 1
                continue
            break
    return "".join(out), found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workspace", type=Path, default=Path.home() / "cuadc_mission")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    ws = args.workspace.expanduser().resolve()
    cpp = ws / "src/cuadc_visual_drop_flight/src/visual_drop_mission_node.cpp"
    yaml_path = ws / "src/cuadc_visual_drop_flight/config/flight_params.yaml"
    vision = ws / "src/cuadc_visual_drop_flight/scripts/drop_bucket_realsense_node.py"
    for p in (cpp, yaml_path, vision):
        if not p.is_file():
            raise SystemExit(f"[ERROR] missing {p}")

    y = yaml_path.read_text(encoding="utf-8")
    counts = {}
    for key, value in CURRENT.items():
        y, n = replace_yaml_scalar(y, key, value)
        counts[key] = n
    required_yaml = ("camera_xy_calib_bias_m", "camera_optical_to_body_rotation")
    for key in required_yaml:
        if counts[key] != 1:
            raise SystemExit(f"[ERROR] YAML {key} match count={counts[key]}, expected 1")

    s = cpp.read_text(encoding="utf-8")
    s, n_release = re.subn(
        r'(declare_parameter<std::vector<double>>\(\s*"payload_release_offsets_body_m"\s*,\s*std::vector<double>\{)[^}]+(\}\s*\);)',
        r'\g<1>0.026, -0.065, -0.320, 0.0109, 0.0720, -0.320\g<2>',
        s, count=1, flags=re.S,
    )
    if n_release != 1:
        raise SystemExit(f"[ERROR] C++ release-offset default match count={n_release}")
    s = re.sub(
        r'declare_parameter<double>\("fine_align_alt_m",\s*[-+0-9.]+\);',
        'declare_parameter<double>("fine_align_alt_m", 1.30);',
        s,
    )
    s = re.sub(r'(?m)^(\s*)fine_align_alt_m_\s*=\s*0\.9(?:0)?\s*;', r'\1fine_align_alt_m_ = 1.30;', s)

    v = vision.read_text(encoding="utf-8")
    v2, n_overlay = re.subn(
        r'(self\.declare_parameter\(\s*["\']calibration_release_offsets_body_m["\']\s*,\s*\[)[\s\S]*?(\]\s*,?\s*\))',
        r'\g<1>\n                0.026, -0.065, -0.320,\n                0.0109, 0.0720, -0.320,\n            \g<2>',
        v, count=1,
    )
    if n_overlay == 0:
        v2 = v

    print("Workspace:", ws)
    print("YAML matches:", counts)
    print("C++ release default: OK")
    print("Vision overlay default:", "updated" if n_overlay else "not present / skipped")
    print("P1 FLU = [0.0260, -0.0650, -0.3200]")
    print("P2 FLU = [0.0109, +0.0720, -0.3200]")
    print("fine/release altitude = 1.30 m")

    if args.dry_run:
        print("[DRY-RUN] no files written")
        return

    tmp = Path(tempfile.mkdtemp(prefix="cuadc_calib_backup_"))
    shutil.copy2(cpp, tmp / cpp.name)
    shutil.copy2(yaml_path, tmp / yaml_path.name)
    shutil.copy2(vision, tmp / vision.name)
    print("Temporary backup:", tmp)

    yaml_path.write_text(y, encoding="utf-8")
    cpp.write_text(s, encoding="utf-8")
    vision.write_text(v2, encoding="utf-8")
    print("[SUCCESS] calibration applied")


if __name__ == "__main__":
    main()
