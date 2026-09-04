#!/usr/bin/env python3
"""Solve robust effective release-offset corrections from repeated drops."""
import argparse
import csv
import statistics
from collections import defaultdict
from pathlib import Path

import yaml


def mad(values):
    center = statistics.median(values)
    return statistics.median(abs(v - center) for v in values)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trials_csv", type=Path)
    parser.add_argument("--aircraft", type=Path, default=Path("config/aircraft.yaml"))
    parser.add_argument("--output", type=Path, default=Path("logs/drop_solution.yaml"))
    args = parser.parse_args()
    groups = defaultdict(list)
    with args.trials_csv.open(newline="", encoding="utf-8") as source:
        for row in csv.DictReader(source):
            groups[(int(row["payload"]), float(row["height_m"]))].append(
                (float(row["miss_forward_m"]), float(row["miss_right_m"])))
    aircraft = yaml.safe_load(args.aircraft.read_text(encoding="utf-8"))
    offsets = aircraft["payloads"]["release_offsets_frd_m"]
    if offsets is None or len(offsets) != 6:
        raise SystemExit("aircraft.yaml must contain two calibrated release offsets")
    result = {"coordinate_frame": "FCU FRD", "rule": "recommended_xy = static_xy + median_landing_miss_xy", "solutions": []}
    for (payload, height), values in sorted(groups.items()):
        forward = [v[0] for v in values]
        right = [v[1] for v in values]
        correction = [statistics.median(forward), statistics.median(right)]
        start = (payload - 1) * 3
        recommended = [offsets[start] + correction[0], offsets[start + 1] + correction[1], offsets[start + 2]]
        result["solutions"].append({
            "payload": payload, "height_m": height, "trial_count": len(values),
            "median_miss_frd_xy_m": correction,
            "miss_mad_xy_m": [mad(forward), mad(right)],
            "static_release_offset_frd_m": offsets[start:start + 3],
            "recommended_effective_offset_frd_m": recommended,
            "enough_trials": len(values) >= 3,
        })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(result, sort_keys=False, allow_unicode=True), encoding="utf-8")
    print("Wrote {}".format(args.output))
    for item in result["solutions"]:
        print("payload {} height {:.2f} m: n={}, effective offset={}".format(
            item["payload"], item["height_m"], item["trial_count"],
            [round(v, 4) for v in item["recommended_effective_offset_frd_m"]]))


if __name__ == "__main__":
    main()
