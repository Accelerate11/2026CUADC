#!/usr/bin/env python3
"""Estimate a release X/Y after its real plumb point was centered on a visual target."""
import argparse
import csv
import statistics
from pathlib import Path

import yaml


def percentile(values, fraction):
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * fraction)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("detections_csv", type=Path)
    parser.add_argument("--payload", type=int, choices=(1, 2), required=True)
    parser.add_argument("--aircraft", type=Path, default=Path("config/aircraft.yaml"))
    parser.add_argument("--last-seconds", type=float, default=3.0)
    parser.add_argument("--write", action="store_true", help="write estimated X/Y into aircraft.yaml")
    args = parser.parse_args()
    with args.detections_csv.open(newline="", encoding="utf-8") as source:
        rows = [r for r in csv.DictReader(source) if r.get("status") == "accepted"]
    if not rows:
        raise SystemExit("No accepted detections")
    time_key = "capture_time_ns"
    newest = max(int(r[time_key]) for r in rows)
    rows = [r for r in rows if int(r[time_key]) >= newest - int(args.last_seconds * 1e9)]
    x_key = "body_frd_x_m" if "body_frd_x_m" in rows[0] else "body_x_m"
    y_key = "body_frd_y_m" if "body_frd_y_m" in rows[0] else "body_y_m"
    xs, ys = [float(r[x_key]) for r in rows], [float(r[y_key]) for r in rows]
    x, y = statistics.median(xs), statistics.median(ys)
    spread_x = percentile(xs, 0.75) - percentile(xs, 0.25)
    spread_y = percentile(ys, 0.75) - percentile(ys, 0.25)
    print("payload {} estimated FRD X/Y = {:.4f}, {:.4f} m (n={}, IQR={:.3f}/{:.3f} m)".format(args.payload, x, y, len(rows), spread_x, spread_y))
    if not args.write:
        return
    data = yaml.safe_load(args.aircraft.read_text(encoding="utf-8"))
    offsets = data["payloads"]["release_offsets_frd_m"]
    if offsets is None or len(offsets) != 6:
        raise SystemExit("Fill six rough payload offset values first; Z must be physically measured")
    start = (args.payload - 1) * 3
    offsets[start], offsets[start + 1] = round(x, 6), round(y, 6)
    args.aircraft.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print("Updated {} (Z unchanged)".format(args.aircraft))


if __name__ == "__main__":
    main()
