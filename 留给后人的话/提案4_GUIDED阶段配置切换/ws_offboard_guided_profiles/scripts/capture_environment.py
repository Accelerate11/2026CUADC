#!/usr/bin/env python3
"""记录当前机器的可复现环境信息，不读取密钥、不连接设备。"""
import argparse
import importlib.metadata
import json
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

def command(args):
    if not shutil.which(args[0]):
        return {"available": False}
    result = subprocess.run(args, capture_output=True, text=True, timeout=30)
    return {"returncode": result.returncode, "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("log/environment.json"))
    args = parser.parse_args()
    packages = {}
    for distribution in importlib.metadata.distributions():
        name = distribution.metadata.get("Name")
        if name:
            packages[name] = distribution.version
    report = {
        "captured_utc": datetime.now(timezone.utc).isoformat(),
        "purpose": "当前执行环境快照；不代表原比赛飞机的完整环境",
        "platform": platform.platform(), "python": sys.version,
        "python_packages": dict(sorted(packages.items())),
        "ros_packages": command(["dpkg-query", "-W", "-f=${Package}=${Version}\n", "ros-humble-*"]),
        "compiler": command(["g++", "--version"]),
        "cmake": command(["cmake", "--version"]),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(args.output)

if __name__ == "__main__":
    main()
