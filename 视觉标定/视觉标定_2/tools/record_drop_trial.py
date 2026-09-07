#!/usr/bin/env python3
"""Append a measured elevated-drop result in FCU heading-locked directions."""
import argparse
import csv
from datetime import datetime
from pathlib import Path


FIELDS = ["time", "payload", "height_m", "miss_forward_m", "miss_right_m", "wind_m_s", "notes"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--payload", type=int, choices=(1, 2), required=True)
    parser.add_argument("--height", type=float, required=True)
    parser.add_argument("--miss-forward", type=float, required=True, help="landing point minus bucket center; forward positive")
    parser.add_argument("--miss-right", type=float, required=True, help="landing point minus bucket center; right positive")
    parser.add_argument("--wind", type=float, default=float("nan"))
    parser.add_argument("--notes", default="")
    parser.add_argument("--output", type=Path, default=Path("logs/drop_trials.csv"))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    new = not args.output.exists()
    with args.output.open("a", newline="", encoding="utf-8") as target:
        writer = csv.DictWriter(target, fieldnames=FIELDS)
        if new:
            writer.writeheader()
        writer.writerow({"time": datetime.now().astimezone().isoformat(), "payload": args.payload,
                         "height_m": args.height, "miss_forward_m": args.miss_forward,
                         "miss_right_m": args.miss_right, "wind_m_s": args.wind, "notes": args.notes})
    print("Recorded {}".format(args.output))


if __name__ == "__main__":
    main()
