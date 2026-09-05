#!/usr/bin/env python3
"""Small command/telemetry client for the STM32 gimbal controller."""

from __future__ import annotations

import argparse
import sys
import time

import serial


def send(port: serial.Serial, line: str) -> None:
    port.write((line + "\n").encode("ascii"))
    port.flush()


def read_lines(port: serial.Serial, duration: float) -> None:
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        raw = port.readline()
        if raw:
            print(raw.decode("ascii", errors="replace").rstrip())


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="AKPF gimbal serial client")
    parser.add_argument("--port", required=True, help="e.g. /dev/ttyACM0 or COM5")
    parser.add_argument("--baud", type=int, default=115200)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status")
    sub.add_parser("arm")
    sub.add_parser("disarm")
    sub.add_parser("zero")
    monitor = sub.add_parser("monitor")
    monitor.add_argument("--seconds", type=float, default=3600.0)
    target = sub.add_parser("set")
    target.add_argument("roll", type=float)
    target.add_argument("pitch", type=float)
    gain = sub.add_parser("gain")
    gain.add_argument("axis", choices=("R", "P"))
    gain.add_argument("attitude_kp", type=float)
    gain.add_argument("rate_kp", type=float)
    gain.add_argument("max_rate", type=float)
    gain.add_argument("max_slew", type=float)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    with serial.Serial(args.port, args.baud, timeout=0.2) as port:
        time.sleep(0.1)
        if args.command == "monitor":
            read_lines(port, args.seconds)
            return 0
        if args.command == "status":
            command = "STATUS?"
        elif args.command == "arm":
            command = "ARM"
        elif args.command == "disarm":
            command = "DISARM"
        elif args.command == "zero":
            command = "ZERO"
        elif args.command == "set":
            command = f"SET,{args.roll:.3f},{args.pitch:.3f}"
        elif args.command == "gain":
            command = (f"GAIN,{args.axis},{args.attitude_kp:.4f},{args.rate_kp:.4f},"
                       f"{args.max_rate:.2f},{args.max_slew:.2f}")
        else:
            raise AssertionError(args.command)
        send(port, command)
        read_lines(port, 1.0)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except serial.SerialException as exc:
        print(f"serial error: {exc}", file=sys.stderr)
        raise SystemExit(2)
