#!/usr/bin/env python3
"""解析显式串口 URL；供保留串口守护的机载启动入口共用。"""
import argparse
import os
from pathlib import Path
import re


WORKSPACE = Path(__file__).resolve().parent.parent


def parse_serial_url(value):
    """返回绝对设备路径与波特率，不打开串口。"""
    match = re.fullmatch(r"serial://(/[^\r\n\t?#]+):([0-9]+)", value)
    if not match or int(match.group(2)) <= 0:
        raise ValueError("请显式指定串口 URL，例如 serial:///dev/serial/by-id/YOUR_FCU:115200；机载入口仅接受 serial://")
    return match.group(1), int(match.group(2))


def serial_lock_path():
    return Path(os.environ.get("CUADC_SERIAL_LOCK", str(WORKSPACE / ".flight_serial.lock"))).expanduser()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fcu_url", help="必须显式提供的飞控串口 URL")
    args = parser.parse_args()
    try:
        device, baud = parse_serial_url(args.fcu_url)
    except ValueError as error:
        parser.error(str(error))
    print(f"{device}\t{baud}")


if __name__ == "__main__":
    main()
