#!/usr/bin/env python3
"""Synchronously log CUAV ATTITUDE and AKPF gimbal telemetry; sends no flight commands."""

from __future__ import annotations

import argparse
import csv
import math
import time

import serial
from pymavlink import mavutil


def parse_gimbal(line: str) -> dict[str, object] | None:
    fields = line.strip().split(",")
    if len(fields) != 10 or fields[0] != "TEL":
        return None
    try:
        return {
            "state": fields[1],
            "gimbal_roll_deg": float(fields[2]),
            "gimbal_pitch_deg": float(fields[3]),
            "gimbal_roll_rate_dps": float(fields[4]),
            "gimbal_pitch_rate_dps": float(fields[5]),
            "accel_norm_g": float(fields[6]),
            "servo_roll_deg": float(fields[7]),
            "servo_pitch_deg": float(fields[8]),
            "imu_failures": int(fields[9]),
        }
    except ValueError:
        return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gimbal", required=True, help="gimbal serial device")
    parser.add_argument("--mavlink", required=True, help="e.g. udp:127.0.0.1:14550")
    parser.add_argument("--csv", required=True, help="output CSV path")
    parser.add_argument("--seconds", type=float, default=3600.0)
    args = parser.parse_args()

    mav = mavutil.mavlink_connection(args.mavlink)
    gimbal = serial.Serial(args.gimbal, 115200, timeout=0.02)
    fieldnames = [
        "host_time_s", "cuav_roll_deg", "cuav_pitch_deg", "cuav_yaw_deg",
        "state", "gimbal_roll_deg", "gimbal_pitch_deg", "gimbal_roll_rate_dps",
        "gimbal_pitch_rate_dps", "accel_norm_g", "servo_roll_deg",
        "servo_pitch_deg", "imu_failures",
    ]
    cuav = {"cuav_roll_deg": math.nan, "cuav_pitch_deg": math.nan, "cuav_yaw_deg": math.nan}
    deadline = time.monotonic() + args.seconds
    with open(args.csv, "w", newline="", encoding="utf-8") as output:
        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        while time.monotonic() < deadline:
            message = mav.recv_match(type="ATTITUDE", blocking=False)
            if message is not None:
                cuav = {
                    "cuav_roll_deg": math.degrees(message.roll),
                    "cuav_pitch_deg": math.degrees(message.pitch),
                    "cuav_yaw_deg": math.degrees(message.yaw),
                }
            raw = gimbal.readline()
            if not raw:
                continue
            telemetry = parse_gimbal(raw.decode("ascii", errors="replace"))
            if telemetry is None:
                continue
            row = {"host_time_s": time.time(), **cuav, **telemetry}
            writer.writerow(row)
            output.flush()
            print(f"CUAV R/P {cuav['cuav_roll_deg']:6.1f}/{cuav['cuav_pitch_deg']:6.1f}  "
                  f"Gimbal R/P {telemetry['gimbal_roll_deg']:6.2f}/"
                  f"{telemetry['gimbal_pitch_deg']:6.2f}  {telemetry['state']}")
    gimbal.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
