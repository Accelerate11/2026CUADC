#!/usr/bin/env python3
"""Stop old ROS/FCU clients before launch; never send FCU commands.

Detect ROS through process arguments and loaded ROS libraries, not a broad
`pkill python`. Check open serial file descriptors, including pymavlink clients.
Unknown/unreadable process information causes verification to fail closed.
"""
import argparse
import os
from pathlib import Path
import signal
import time


def ancestors():
    result = set()
    pid = os.getpid()
    while pid > 1:
        result.add(pid)
        data = Path(f'/proc/{pid}/status').read_text()
        pid = int(next(line.split()[1] for line in data.splitlines()
                       if line.startswith('PPid:')))
    return result


def processes(devices, excluded):
    found = {}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) in excluded:
            continue
        try:
            args = proc.joinpath('cmdline').read_bytes().split(b'\0')
            args = [a.decode(errors='replace') for a in args if a]
            if not args:
                continue
            maps = proc.joinpath('maps').read_text()
            ros = any('/librcl.' in s or '/librclcpp.' in s or '/librmw_' in s
                      for s in maps.splitlines())
            ros |= any(Path(a).name in ('ros2', 'mavros_node', 'calibrate_route.py',
                                       'route_preflight.py', 'run_full_mission.sh',
                                       'run_drop_only.sh') for a in args)
            ros |= any('ros2cli.daemon' in a for a in args)
            ros |= '--ros-args' in args
            serial = []
            for fd in proc.joinpath('fd').iterdir():
                try:
                    target = os.path.realpath(fd)
                    if target in devices:
                        serial.append(target)
                except FileNotFoundError:
                    pass
            if ros or serial:
                found[int(proc.name)] = (args, sorted(set(serial)))
        except (FileNotFoundError, ProcessLookupError):
            continue
        except PermissionError:
            # Other users' processes cannot be safely inspected or terminated.
            # fuser below independently detects their ownership of the FCU.
            continue
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['clean', 'verify'])
    parser.add_argument('device')
    parser.add_argument('--mavros-pid', type=int)
    args = parser.parse_args()
    devices = {os.path.realpath(args.device)}
    # X7 exposes two USB interfaces: neither may have a competing reader.
    for p in Path('/dev/serial/by-id').glob('*ArduPilot*'):
        devices.add(os.path.realpath(p))
    excluded = ancestors()
    if args.action == 'clean':
        for sig, duration in ((signal.SIGINT, 5), (signal.SIGTERM, 3),
                              (signal.SIGKILL, 2)):
            for pid, (argv, serial) in processes(devices, excluded).items():
                print(f'[CLEAN] signal={sig.name} pid={pid} command={argv[:2]} serial={serial}', flush=True)
                try:
                    os.kill(pid, sig)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + duration
            while time.monotonic() < deadline:
                if not processes(devices, excluded):
                    break
                time.sleep(0.2)
        # Require a quiet interval to catch immediate automatic respawns.
        for _ in range(10):
            if processes(devices, excluded):
                raise RuntimeError('旧 ROS/串口进程未退出，或被自动重启；拒绝启动任务')
            time.sleep(0.2)
    else:
        found = processes(devices, excluded)
        mavros = [pid for pid, (argv, _) in found.items()
                  if any(Path(a).name == 'mavros_node' for a in argv)]
        if mavros != [args.mavros_pid]:
            raise RuntimeError(f'MAVROS 数量或 PID 不符：{mavros}')
        owners = {pid for pid, (_, serial) in found.items() if serial}
        if owners != {args.mavros_pid}:
            raise RuntimeError(f'飞控串口所有者不唯一：{owners}')
    # fuser scans independently (including clients not recognized as ROS).
    import subprocess
    for device in sorted(devices):
        result = subprocess.run(['fuser', device], capture_output=True, text=True)
        if result.returncode not in (0, 1):
            raise RuntimeError(f'无法检查串口占用：{device}: {result.stderr}')
        owners = set(map(int, result.stdout.split()))
        allowed = {args.mavros_pid} if args.action == 'verify' else set()
        if owners - allowed:
            raise RuntimeError(f'串口仍有其他占用：{device}: {owners}')
    print(f'[OK] ROS/串口检查通过：{args.action}', flush=True)


if __name__ == '__main__':
    main()
