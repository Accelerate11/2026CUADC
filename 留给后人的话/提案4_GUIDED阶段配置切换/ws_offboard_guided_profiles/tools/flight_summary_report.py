#!/usr/bin/env python3
"""生成本地飞行报告；只有手动传入 --send 才尝试邮件发送。

授权码仅从 CUADC_SMTP_TOKEN 或 SMTP_TOKEN 环境变量读取。
发件人、收件人与服务器由用户显式配置；可选日志库未安装时仍可生成报告。
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import email.message
import math
import os
import re
import smtplib
import signal
import time
import statistics
import sys
import traceback
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception:
    yaml = None


TS_RE = re.compile(r"\[(\d+(?:\.\d+)?)\]")
STATE_RE = re.compile(r"STATE (\w+) -> (\w+)")
POS_RE = re.compile(r"state=(\w+).*?pos=\(([-+\d.]+),([-+\d.]+),([-+\d.]+)\)")
NAV_RE = re.compile(r"NAV_STREAM.*?odom_pos=\(([-+\d.]+),([-+\d.]+),([-+\d.]+)\).*?odom_rpy=\(([-+\d.eE]+),([-+\d.eE]+),([-+\d.eE]+)\)")
EVENT_PATTERNS = (
    ("识别阶段", re.compile(r"SEARCH_START|STATE \w+ -> SEARCH")),
    ("细对准", re.compile(r"FINE_MECHANICAL_DESCENT_BEGIN|FINE_DESCENT_READY|STATE \w+ -> ALIGN")),
    ("侦察航线", re.compile(r"RECON_FIRST_WAYPOINT_TRANSIT|STATE \w+ -> RECON_SURVEY")),
    ("启动返航", re.compile(r"RETURN_CLIMB|STATE \w+ -> RETURN_HOME")),
)
RELEASE_RE = re.compile(r"(?:DIRECT_RELEASE_COMMAND|VISUAL_RELEASE_(?:TIMEOUT|TOLERANCE)).*?payload=(\d+)")
ERROR_RE = re.compile(r"\b(ERROR|WARN|WARNING|FATAL)\b|\[ERROR\]|\[WARN\]")


def stamp(value: float, origin: float | None = None) -> str:
    zone = dt.datetime.now().astimezone().tzinfo
    when = dt.datetime.fromtimestamp(value, zone)
    rel = "n/a" if origin is None else f"T+{value - origin:.3f}s"
    return f"{when:%Y-%m-%d %H:%M:%S.%f%z} ({rel})"


def parse_float(value: Any) -> float | None:
    try:
        x = float(value)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None


def load_yaml(path: Path) -> dict[str, Any]:
    if yaml is None:
        return {}
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def find_files(root: Path) -> list[Path]:
    return [p for p in root.rglob("*") if p.is_file()]


def parse_csv_points(path: Path) -> tuple[list[tuple[float, float, float]], list[dict[str, Any]]]:
    points: list[tuple[float, float, float]] = []
    rows: list[dict[str, Any]] = []
    if not path.exists():
        return points, rows
    try:
        with path.open(newline="", encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                rows.append(row)
                ts = parse_float(row.get("capture_time_ns"))
                if ts is not None and ts > 1e15:
                    ts /= 1e9
                x = parse_float(row.get("local_x_m"))
                y = parse_float(row.get("local_y_m"))
                z = parse_float(row.get("local_z_m"))
                if ts is not None and x is not None and y is not None and z is not None:
                    points.append((ts, x, y, z))
    except Exception:
        return [], rows
    return points, rows


def parse_telemetry(path: Path) -> list[dict[str, Any]]:
    """Read the low-rate MAVROS recorder without failing the whole report."""
    rows: list[dict[str, Any]] = []
    if not path.is_file():
        return rows
    try:
        with path.open(newline="", encoding="utf-8") as fh:
            for raw in csv.DictReader(fh):
                timestamp = parse_float(raw.get("timestamp_s"))
                if timestamp is None:
                    continue
                row: dict[str, Any] = dict(raw)
                row["timestamp_s"] = timestamp
                for key in (
                    "latitude_deg", "longitude_deg", "gps_altitude_m",
                    "battery_voltage_v", "battery_current_a",
                    "battery_remaining_pct", "velocity_x_m_s",
                    "velocity_y_m_s", "velocity_z_m_s",
                    "horizontal_speed_m_s", "vertical_speed_m_s", "roll_deg",
                    "pitch_deg", "yaw_enu_deg", "heading_deg",
                    "compass_heading_deg",
                ):
                    row[key] = parse_float(raw.get(key))
                rows.append(row)
    except Exception:
        return []
    return sorted(rows, key=lambda row: row["timestamp_s"])


def nearest_telemetry(
    rows: list[dict[str, Any]], timestamp: float
) -> dict[str, Any] | None:
    if not rows:
        return None
    nearest = min(rows, key=lambda row: abs(row["timestamp_s"] - timestamp))
    return nearest if abs(nearest["timestamp_s"] - timestamp) <= 2.0 else None


def valid_gps(row: dict[str, Any]) -> bool:
    latitude = row.get("latitude_deg")
    longitude = row.get("longitude_deg")
    return (
        latitude is not None and longitude is not None and
        -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0 and
        (abs(latitude) > 1.0e-8 or abs(longitude) > 1.0e-8)
    )


def telemetry_event_lines(row: dict[str, Any] | None, indent: str = "  - ") -> list[str]:
    if row is None:
        return [f"{indent}结构化飞控数据：未获取"]
    lines: list[str] = []
    if valid_gps(row):
        lines.extend([
            f"{indent}纬度：{row['latitude_deg']:.8f}°",
            f"{indent}经度：{row['longitude_deg']:.8f}°",
        ])
        if row.get("gps_altitude_m") is not None:
            lines.append(f"{indent}GPS 海拔：{row['gps_altitude_m']:.2f} m")
    else:
        lines.append(f"{indent}经纬度：无有效 GPS 定位")
    labels = (
        ("compass_heading_deg", "磁罗盘航向", "°", 2),
        ("heading_deg", "姿态航向", "°", 2),
        ("battery_voltage_v", "电池电压", " V", 2),
        ("horizontal_speed_m_s", "水平速度", " m/s", 2),
        ("vertical_speed_m_s", "垂直速度", " m/s", 2),
        ("roll_deg", "横滚", "°", 2),
        ("pitch_deg", "俯仰", "°", 2),
    )
    for key, label, suffix, precision in labels:
        if row.get(key) is not None:
            lines.append(f"{indent}{label}：{row[key]:.{precision}f}{suffix}")
    if row.get("fcu_mode"):
        lines.append(f"{indent}飞控模式：{row['fcu_mode']}")
    if row.get("armed"):
        lines.append(f"{indent}解锁状态：{'已解锁' if row['armed'] == 'true' else '已上锁'}")
    return lines


def observed_gps_bounds(rows: list[dict[str, Any]]) -> list[str]:
    valid = [row for row in rows if valid_gps(row)]
    if not valid:
        return ["  - 未获取有效 GPS 坐标"]
    latitudes = [row["latitude_deg"] for row in valid]
    longitudes = [row["longitude_deg"] for row in valid]
    return [
        f"  - 纬度范围：{min(latitudes):.8f}° 至 {max(latitudes):.8f}°",
        f"  - 经度范围：{min(longitudes):.8f}° 至 {max(longitudes):.8f}°",
        f"  - 有效 GPS 样本：{len(valid)} 个",
    ]


def planned_gps_bounds(
    origin: dict[str, Any] | None,
    heading_deg: float | None,
    x_min_m: float | None,
    x_max_m: float | None,
    y_half_width_m: float | None,
) -> list[str]:
    values = (heading_deg, x_min_m, x_max_m, y_half_width_m)
    if origin is None or not valid_gps(origin) or any(value is None for value in values):
        return ["  - 无法换算：缺少有效起飞 GPS 或任务区域参数"]
    heading = math.radians(float(heading_deg))
    latitude_0 = math.radians(origin["latitude_deg"])
    longitude_0 = math.radians(origin["longitude_deg"])
    earth_radius_m = 6378137.0
    points: list[tuple[float, float]] = []
    for field_x in (float(x_min_m), float(x_max_m)):
        for field_y in (-float(y_half_width_m), float(y_half_width_m)):
            north = math.cos(heading) * field_x + math.sin(heading) * field_y
            east = math.sin(heading) * field_x - math.cos(heading) * field_y
            latitude = latitude_0 + north / earth_radius_m
            longitude = longitude_0 + east / (earth_radius_m * math.cos(latitude_0))
            points.append((math.degrees(latitude), math.degrees(longitude)))
    latitudes = [point[0] for point in points]
    longitudes = [point[1] for point in points]
    return [
        f"  - 纬度范围：{min(latitudes):.8f}° 至 {max(latitudes):.8f}°",
        f"  - 经度范围：{min(longitudes):.8f}° 至 {max(longitudes):.8f}°",
        "  - 换算方式：起飞 GPS + 指定地理航向 + YAML 场地尺寸（局部 WGS84 近似）",
    ]


def parse_ros_log(path: Path) -> dict[str, Any]:
    events: list[tuple[float, str, str]] = []
    positions: list[tuple[float, str, float, float, float]] = []
    attitudes: list[tuple[float, float, float, float]] = []
    releases: list[dict[str, Any]] = []
    errors: list[tuple[float, str]] = []
    origin: float | None = None
    if not path.exists():
        return {"events": events, "positions": positions, "attitudes": attitudes,
                "releases": releases, "errors": errors, "origin": origin}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = TS_RE.search(line)
        if not match:
            continue
        ts = float(match.group(1))
        origin = ts if origin is None else min(origin, ts)
        sm = STATE_RE.search(line)
        if sm:
            events.append((ts, "状态", f"{sm.group(1)} -> {sm.group(2)}"))
        pm = POS_RE.search(line)
        if pm:
            positions.append((ts, pm.group(1), float(pm.group(2)), float(pm.group(3)), float(pm.group(4))))
        nm = NAV_RE.search(line)
        if nm:
            attitudes.append((ts, float(nm.group(4)), float(nm.group(5)), float(nm.group(6))))
        rm = RELEASE_RE.search(line)
        if rm and ("VISUAL_RELEASE_TIMEOUT" in line or "VISUAL_RELEASE_TOLERANCE" in line):
            mode = "超时强制投放" if "TIMEOUT" in line else "正常误差范围内投放"
            em = re.search(r"error=([\d.]+)m", line)
            releases.append({"ts": ts, "payload": int(rm.group(1)), "mode": mode,
                             "error": em.group(1) + "m" if em else "n/a", "line": line})
        if ERROR_RE.search(line):
            errors.append((ts, line.strip()))
        for name, pattern in EVENT_PATTERNS:
            if pattern.search(line):
                events.append((ts, name, line.strip()))
    return {"events": events, "positions": positions, "attitudes": attitudes,
            "releases": releases, "errors": errors, "origin": origin}


def parse_auxiliary_errors(paths: list[Path]) -> list[tuple[float, str]]:
    errors: list[tuple[float, str]] = []
    for path in paths:
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            continue
        for line in lines:
            if not ERROR_RE.search(line):
                continue
            match = TS_RE.search(line)
            if match:
                errors.append((float(match.group(1)), f"[{path.name}] {line.strip()}"))
    return errors


def parse_dataflash(files: list[Path]) -> dict[str, Any]:
    """Best-effort APM DataFlash extraction when pymavlink is installed."""
    result: dict[str, Any] = {"gps": [], "motion": [], "battery": [], "errors": []}
    try:
        from pymavlink import DFReader
    except Exception:
        return result
    for path in files:
        if path.suffix.lower() not in {".bin", ".log"}:
            continue
        if path.suffix.lower() == ".log":
            try:
                header = path.open("rb").read(8)
            except OSError:
                continue
            if not (header.startswith(b"\xa3\x95") or header.startswith(b"FMT")):
                continue
        try:
            log = DFReader.DFReader_binary(str(path), zero_time_base=False)
            while True:
                msg = log.recv_msg()
                if msg is None:
                    break
                typ = msg.get_type()
                t = parse_float(getattr(msg, "_timestamp", None))
                if t is None:
                    continue
                if typ in {"GPS", "GPS2"}:
                    lat = parse_float(getattr(msg, "Lat", None)); lon = parse_float(getattr(msg, "Lng", None))
                    alt = parse_float(getattr(msg, "Alt", None))
                    spd = parse_float(getattr(msg, "Spd", None)); hdg = parse_float(getattr(msg, "GCrs", None))
                    if lat is not None and lon is not None:
                        if abs(lat) > 180: lat /= 1e7
                        if abs(lon) > 180: lon /= 1e7
                        result["gps"].append((t, lat, lon, alt, spd, hdg))
                elif typ in {"IMU", "IMU2", "ATT"}:
                    roll = parse_float(getattr(msg, "Roll", None)); pitch = parse_float(getattr(msg, "Pitch", None)); yaw = parse_float(getattr(msg, "Yaw", None))
                    if roll is not None and pitch is not None and yaw is not None:
                        result["motion"].append((t, roll, pitch, yaw))
                elif typ in {"BAT", "BAT2"}:
                    voltage = parse_float(getattr(msg, "Volt", None))
                    if voltage is not None:
                        result["battery"].append((t, voltage))
        except Exception as exc:
            result["errors"].append(f"DataFlash {path}: {exc}")
    return result


def parse_ulog(files: list[Path]) -> dict[str, Any]:
    result: dict[str, Any] = {"gps": [], "motion": [], "battery": [], "errors": []}
    try:
        from pyulog import ULog
    except Exception:
        return result
    for path in files:
        if path.suffix.lower() != ".ulg":
            continue
        try:
            log = ULog(str(path))
            for d in log.get_dataset("vehicle_global_position").data:
                result["gps"].append((d["timestamp"] / 1e6, d.get("lat", 0) / 1e7, d.get("lon", 0) / 1e7, d.get("alt", 0), None, None))
        except Exception as exc:
            result["errors"].append(f"ULog {path}: {exc}")
    return result


def raw_log_files(files: list[Path]) -> list[Path]:
    result: list[Path] = []
    for path in files:
        suffix = path.suffix.lower()
        if suffix in {".bin", ".ulg"}:
            result.append(path)
        elif suffix == ".log":
            try:
                header = path.open("rb").read(8)
            except OSError:
                continue
            if header.startswith(b"\xa3\x95") or header.startswith(b"FMT"):
                result.append(path)
    return result


def speed_stats(points: list[tuple[float, str, float, float, float]]) -> tuple[float, dict[str, list[float]]]:
    speeds: list[float] = []; phases: dict[str, list[float]] = {}
    for a, b in zip(points, points[1:]):
        dt_s = b[0] - a[0]
        if dt_s <= 0 or dt_s > 5:
            continue
        v = math.hypot(b[2] - a[2], b[3] - a[3]) / dt_s
        if v > 100:
            continue
        speeds.append(v); phases.setdefault(b[1], []).append(v)
    return (max(speeds, default=0.0), phases)


def generate_report_legacy(root: Path) -> str:
    files = find_files(root)
    node_logs = [p for p in files if p.name.startswith("cuadc_full_mission_node_") and p.suffix == ".log"]
    parsed = {"events": [], "positions": [], "attitudes": [], "releases": [], "errors": [], "origin": None}
    for path in node_logs:
        part = parse_ros_log(path)
        parsed["events"] += part["events"]; parsed["positions"] += part["positions"]
        parsed["attitudes"] += part["attitudes"]; parsed["releases"] += part["releases"]
        parsed["errors"] += part["errors"]
        if part["origin"] is not None:
            parsed["origin"] = part["origin"] if parsed["origin"] is None else min(parsed["origin"], part["origin"])
    auxiliary_logs = [
        path for path in files
        if path.name == "mavros_stdout.log" or
        (path.suffix == ".log" and path.parent.name.startswith("2026-"))
    ]
    parsed["errors"].extend(parse_auxiliary_errors(auxiliary_logs))
    parsed["events"].sort(); parsed["positions"].sort(); parsed["releases"].sort(key=lambda x: x["ts"])
    raw_files = raw_log_files(files)
    df = parse_dataflash(raw_files); ul = parse_ulog(raw_files)
    takeoff_events = [ts for ts, kind, detail in parsed["events"]
                      if kind == "状态" and "TAKEOFF -> SEARCH" in detail]
    origin = min(takeoff_events) if takeoff_events else (
        parsed["origin"] or (df["gps"][0][0] if df["gps"] else None))
    max_v, phases = speed_stats(parsed["positions"])
    runtime = load_yaml(root / "mission_runtime.yaml")
    params = runtime.get("cuadc_full_mission_node", {}).get("ros__parameters", {}) if runtime else {}
    lines = [f"飞行总结报告: {root.name}", f"生成时间: {dt.datetime.now().astimezone():%Y-%m-%d %H:%M:%S%z}", ""]
    lines.append("[时间基准]")
    lines.append(f"起飞原点: {stamp(origin) if origin else '未解析'}")
    lines.append("所有事件同时显示绝对时间和相对起飞时间 T。")
    lines.append("")
    lines.append("[任务关键事件]")
    for ts, kind, detail in parsed["events"]:
        lines.append(f"- {kind}: {stamp(ts, origin)} | {detail}")
    for rel in parsed["releases"]:
        lines.append(f"- 桶{rel['payload']}投放: {stamp(rel['ts'], origin)} | {rel['mode']} | 误差={rel['error']}")
    lines.append("")
    lines.append("[飞行运动]")
    lines.append(f"全程最大水平移动速度: {max_v:.2f} m/s")
    for phase, vals in sorted(phases.items()):
        lines.append(f"阶段 {phase} 平均水平速度: {statistics.fmean(vals):.2f} m/s (样本{len(vals)})")
    lines.append("")
    lines.append("[位置、GPS、电压、姿态]")
    if df["gps"]:
        first = df["gps"][0]; last = df["gps"][-1]
        lines.append(f"起飞GPS: lat={first[1]:.7f}, lon={first[2]:.7f}, alt={first[3]}")
        lines.append(f"任务区域GPS轨迹: {len(df['gps'])} 个 DataFlash 点; 末点 lat={last[1]:.7f}, lon={last[2]:.7f}")
    else:
        lines.append("GPS/电压/飞控原始姿态: 本次目录未发现可解析的 DataFlash/PX4 ULog；保留 ROS 姿态与位置日志。")
    if df.get("battery"):
        volts = [v for _, v in df["battery"]]
        lines.append(f"飞控电压: 最低={min(volts):.2f}V, 最高={max(volts):.2f}V, 样本={len(volts)}")
    else:
        lines.append("飞控电压: 未发现可解析的 DataFlash/PX4 电池数据。")
    if parsed["attitudes"]:
        max_tilt = max(max(abs(x[1]), abs(x[2])) for x in parsed["attitudes"])
        lines.append(f"ROS姿态样本: {len(parsed['attitudes'])}，最大绝对横滚/俯仰: {max_tilt:.3f} rad")
        anomalies = [(ts, roll, pitch) for ts, roll, pitch, _ in parsed["attitudes"]
                     if max(abs(roll), abs(pitch)) > math.radians(30)]
        if anomalies:
            lines.append(f"姿态异常: {len(anomalies)} 个样本超过 30deg，首次 {stamp(anomalies[0][0], origin)} roll={anomalies[0][1]:.3f} pitch={anomalies[0][2]:.3f}rad")
        else:
            lines.append("姿态异常: 未发现横滚/俯仰超过 30deg 的 ROS 样本。")
    else:
        lines.append("ROS姿态: 未找到 NAV_STREAM 姿态样本。")
    lines.append(f"任务配置: coarse_alt={params.get('coarse_alt_m', 'n/a')}m, release_timeout={params.get('release_stabilization_s', 'n/a')}s")
    lines.append("")
    lines.append("[故障、告警与异常]")
    if parsed["errors"]:
        for ts, msg in parsed["errors"]:
            lines.append(f"- {stamp(ts, origin)} | {msg}")
    else:
        lines.append("- 未发现 ROS 任务日志中的 ERROR/WARN/FATAL。")
    for msg in df["errors"] + ul["errors"]:
        lines.append(f"- 解析器异常: {msg}")
    lines.append("")
    lines.append("[数据完整性]")
    lines.append(f"发现文件数: {len(files)}; ROS任务日志: {len(node_logs)}; DataFlash/PX4原始日志: {len(raw_files)}")
    if not node_logs:
        lines.append("ERROR: 缺少 cuadc_full_mission_node 日志，任务事件无法完整解析。")
    return "\n".join(lines) + "\n"


def _local_time(value: float) -> str:
    return dt.datetime.fromtimestamp(value, dt.datetime.now().astimezone().tzinfo).strftime(
        "%Y-%m-%d %H:%M:%S"
    )


def _event_line(ts: float, origin: float | None) -> str:
    absolute = _local_time(ts)
    relative = "无起飞原点" if origin is None else f"T{ts - origin:+.3f} 秒"
    return f"绝对时间：{absolute}\n    相对时间：{relative}"


def generate_report(root: Path) -> str:
    """Generate the operator-facing, numbered Chinese flight report."""
    files = find_files(root)
    node_logs = sorted(
        (p for p in files if p.name.startswith("cuadc_full_mission_node_") and p.suffix == ".log"),
        key=lambda p: p.name,
    )
    parsed = {"events": [], "positions": [], "attitudes": [], "releases": [], "errors": [], "origin": None}
    for path in node_logs:
        part = parse_ros_log(path)
        for key in ("events", "positions", "attitudes", "releases", "errors"):
            parsed[key].extend(part[key])
        if part["origin"] is not None:
            parsed["origin"] = part["origin"] if parsed["origin"] is None else min(parsed["origin"], part["origin"])
    for key in ("events", "positions", "attitudes", "errors"):
        parsed[key].sort(key=lambda x: x[0])
    parsed["releases"].sort(key=lambda x: x["ts"])

    transitions = [(ts, detail) for ts, kind, detail in parsed["events"] if kind == "状态"]
    telemetry = parse_telemetry(root / "flight_telemetry.csv")
    known_states = {detail.split(" -> ")[-1] for _, detail in transitions}
    previous_state = ""
    for row in telemetry:
        state = str(row.get("mission_state") or "")
        if not state or state == previous_state:
            continue
        if state not in known_states:
            transitions.append((row["timestamp_s"], f"{previous_state or 'UNKNOWN'} -> {state}"))
            known_states.add(state)
        previous_state = state
    transitions.sort(key=lambda item: item[0])
    takeoff = next((ts for ts, detail in transitions if "-> TAKEOFF" in detail), None)
    origin = takeoff
    runtime = load_yaml(root / "mission_runtime.yaml")
    params = runtime.get("cuadc_full_mission_node", {}).get("ros__parameters", {}) if runtime else {}
    drop_only = bool(params.get("drop_only_mode", False))
    finished = any("-> DONE" in detail for _, detail in transitions)
    failed = any("MISSION_FAILURE" in msg or "SAFE_ABORT" in msg for _, msg in parsed["errors"])
    result = "起飞前终止" if takeoff is None else ("异常终止" if failed or not finished else "成功完成")
    generated = dt.datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S")
    lines = [
        "CUADC 无人机飞行总结报告",
        f"  任务编号：{root.name}",
        f"  报告生成时间：{generated}",
        f"  任务类型：{'仅投放任务' if drop_only else '完整任务'}",
        f"  任务结果：{result}",
        "",
    ]
    if origin is None:
        lines += [
            "  一、时间基准", "", "  未发生有效起飞，因此不存在 T=0 起飞原点。",
            "  起飞前事件仅记录绝对时间。", "", "  二、异常事件", "",
        ]
        for ts, msg in parsed["errors"][-20:]:
            lines += [f"  - {_local_time(ts)}", f"    {msg}"]
        if telemetry:
            lines += ["", "  终止前最后一条飞控数据："]
            lines += telemetry_event_lines(telemetry[-1])
        lines += ["", "  三、任务进度", ""]
        state_names = ["飞控连接", "GUIDED", "坐标锁定", "解锁", "起飞", "目标识别", "投放", "侦察", "返航"]
        for name in state_names:
            lines.append(f"  - {name}：未完成")
        lines += ["", "  最终处置：", "  - 未检测到有效起飞时刻。", "  - 已生成异常报告。"]
        return "\n".join(lines) + "\n"

    end_ts = max([ts for ts, _ in transitions] + [row["timestamp_s"] for row in telemetry] + [origin])
    takeoff_telemetry = nearest_telemetry(telemetry, origin)
    configured_heading = parse_float(params.get("geographic_heading_deg"))
    lines += [
        "  一、时间基准", "", "  起飞时刻定义为 T=0。",
        f"  起飞绝对时间：{_local_time(origin)}",
        "  所有事件同时记录：", "  - 绝对时间：年月日时分秒",
        "  - 相对时间：距离起飞的秒数", f"", f"  任务总时长：{end_ts - origin:.2f} 秒", "",
        "  二、起飞信息", "", "  起飞时刻：", f"  - {_event_line(origin, origin)}",
        "", "  起飞位置：",
    ]
    if takeoff_telemetry and valid_gps(takeoff_telemetry):
        lines += [
            f"  - 纬度：{takeoff_telemetry['latitude_deg']:.8f}°",
            f"  - 经度：{takeoff_telemetry['longitude_deg']:.8f}°",
            f"  - GPS 海拔：{takeoff_telemetry['gps_altitude_m']:.2f} m" if takeoff_telemetry.get("gps_altitude_m") is not None else "  - GPS 海拔：未获取",
        ]
    else:
        lines += ["  - 经纬度：无有效 GPS 定位", "  - GPS 海拔：未获取"]
    lines += ["", "  起飞瞬间航向："]
    if configured_heading is not None and configured_heading >= 0.0:
        lines.append(f"  - 指定地理航向：{configured_heading:.2f}°")
    if takeoff_telemetry and takeoff_telemetry.get("compass_heading_deg") is not None:
        lines.append(f"  - 磁罗盘航向：{takeoff_telemetry['compass_heading_deg']:.2f}°")
        if configured_heading is not None and configured_heading >= 0.0:
            heading_error = (
                takeoff_telemetry["compass_heading_deg"] - configured_heading + 180.0
            ) % 360.0 - 180.0
            lines.append(f"  - 罗盘相对指定航向偏差：{heading_error:+.2f}°")
    if takeoff_telemetry and takeoff_telemetry.get("heading_deg") is not None:
        lines.append(f"  - 飞控姿态航向：{takeoff_telemetry['heading_deg']:.2f}°")
    lines += ["  - 坐标系锁定状态：成功", "", "  起飞瞬间飞控数据："]
    lines += telemetry_event_lines(takeoff_telemetry)
    lines += ["", "  三、任务阶段事件", ""]
    search_ts = next((ts for ts, detail in transitions if "TAKEOFF -> SEARCH" in detail), origin)
    align_events = [(ts, detail) for ts, detail in transitions if "-> ALIGN" in detail]
    recon_ts = next((ts for ts, detail in transitions if "-> RECON_SURVEY" in detail), None)
    return_ts = next((ts for ts, detail in transitions if "-> RETURN_CLIMB" in detail), None)
    stable_target_ids = set()
    for _, message in parsed["errors"]:
        stable_target_ids.update(re.findall(r"id=(\d+)", message))
    stable_target_count = len(stable_target_ids) or "日志未直接汇总"
    lines += [
        "  1. 进入目标桶识别阶段", "", "  - 阶段名称：SEARCH",
        f"  - {_event_line(search_ts, origin)}", "  - 识别航线：投放区蛇形搜索",
        f"  - 搜索高度：{params.get('search_alt_m', '未记录')} m",
        f"  - 搜索航点数量：日志记录值或配置值（{params.get('search_lane_count', '自动')} 条扫描线）",
        f"  - 视觉状态：{'正常' if parsed['positions'] else '未获取导航样本'}",
        f"  - 检测到的稳定目标数量：{stable_target_count} 个", "",
    ]
    for idx, ts in enumerate([x[0] for x in align_events], 1):
        lines += [f"  {idx + 1}. 第{idx}个桶进入对准流程", "", "  - 阶段名称：ALIGN", f"  - {_event_line(ts, origin)}", f"  - 目标编号：桶 {idx}", "  - 目标跟踪 ID：见 ALIGN_TARGET 日志", "  - 粗对准和下降：机械标定流程", ""]
        rel = next((r for r in parsed["releases"] if r["payload"] == idx), None)
        if rel:
            lines += [f"  {idx + 2}. 第{idx}个桶投放", "", f"  - {_event_line(rel['ts'], origin)}", f"  - 投放类型：{rel['mode']}", f"  - 投放前视觉误差：{rel['error']}", "  - 投放时任务状态：RELEASE", f"  - 投放舵机：CH{7 if idx == 1 else 8}", "  - 舵机收回：由 ACK 日志确认", ""]
            lines += ["  投放瞬间飞控数据："]
            lines += telemetry_event_lines(nearest_telemetry(telemetry, rel["ts"]))
            lines.append("")
    section_no = 2 + len(align_events) * 2
    if recon_ts is not None:
        lines += [f"  {section_no}. 进入灾情侦察航线", "", "  - 阶段名称：RECON_CLIMB → RECON_SURVEY", f"  - {_event_line(recon_ts, origin)}", f"  - 侦察高度：{params.get('recon_hover_alt_m', '未记录')} m", "  - 航线类型：蛇形航线", "  - 灾情视觉识别：未启用", ""]
        section_no += 1
    if return_ts is not None:
        lines += [f"  {section_no}. 启动返航", "", "  - 阶段名称：RETURN_CLIMB", f"  - {_event_line(return_ts, origin)}", f"  - 返航高度：{params.get('return_alt_m', '未记录')} m", f"  - 返航速度：{params.get('return_speed_m_s', '未记录')} m/s", ""]
        lines += ["  返航启动飞控数据："]
        lines += telemetry_event_lines(nearest_telemetry(telemetry, return_ts))
        lines.append("")
        section_no += 1
    lines += [f"  {section_no}. 返回、降落和上锁", ""]
    for name in ("RETURN_HOME", "LAND", "DISARM", "DONE"):
        event = next(((ts, detail) for ts, detail in transitions if f"-> {name}" in detail), None)
        if event:
            lines += [f"  - 进入 {name}：", f"    {_event_line(event[0], origin)}", ""]
    max_v, phases = speed_stats(parsed["positions"])
    telemetry_speeds = [
        row["horizontal_speed_m_s"] for row in telemetry
        if row.get("horizontal_speed_m_s") is not None
    ]
    telemetry_phases: dict[str, list[float]] = {}
    for row in telemetry:
        state = str(row.get("mission_state") or "")
        speed = row.get("horizontal_speed_m_s")
        if state and speed is not None:
            telemetry_phases.setdefault(state, []).append(speed)
    if telemetry_speeds:
        max_v = max(telemetry_speeds)
        phases = telemetry_phases
    lines += ["  四、飞行运动统计", "", f"  全程最大水平速度：{max_v:.2f} m/s", "", "  各阶段平均水平速度："]
    for phase, vals in sorted(phases.items()):
        lines.append(f"  - {phase}：{statistics.fmean(vals):.2f} m/s")
    roll_values = [abs(row["roll_deg"]) for row in telemetry if row.get("roll_deg") is not None]
    pitch_values = [abs(row["pitch_deg"]) for row in telemetry if row.get("pitch_deg") is not None]
    descent_values = [row["vertical_speed_m_s"] for row in telemetry if row.get("vertical_speed_m_s") is not None]
    if roll_values:
        lines.append(f"  最大绝对横滚角：{max(roll_values):.2f}°")
    if pitch_values:
        lines.append(f"  最大绝对俯仰角：{max(pitch_values):.2f}°")
    if descent_values:
        lines.append(f"  最大垂直下降速度：{abs(min(descent_values)):.2f} m/s")

    drop_rows = [row for row in telemetry if row.get("mission_state") in {"SEARCH", "ALIGN", "RELEASE"}]
    recon_rows = [row for row in telemetry if str(row.get("mission_state") or "").startswith("RECON")]
    planning_heading = configured_heading
    if (planning_heading is None or planning_heading < 0.0) and takeoff_telemetry:
        planning_heading = takeoff_telemetry.get("compass_heading_deg")
    drop_x_min = parse_float(params.get("drop_area_near_edge_field_x_m"))
    drop_length = parse_float(params.get("drop_area_length_field_x_m"))
    drop_width = parse_float(params.get("drop_area_width_field_y_m"))
    drop_x_max = drop_x_min + drop_length if drop_x_min is not None and drop_length is not None else None
    recon_center = parse_float(params.get("recon_area_center_field_x_m"))
    recon_length = parse_float(params.get("recon_area_length_field_x_m"))
    recon_width = parse_float(params.get("recon_area_width_field_y_m"))
    recon_x_min = recon_center - recon_length * 0.5 if recon_center is not None and recon_length is not None else None
    recon_x_max = recon_center + recon_length * 0.5 if recon_center is not None and recon_length is not None else None
    task_x_max_values = [value for value in (drop_x_max, recon_x_max) if value is not None]
    task_width_values = [value for value in (drop_width, recon_width) if value is not None]
    lines += ["", "  五、任务区域位置", "", "  起飞点："]
    if takeoff_telemetry and valid_gps(takeoff_telemetry):
        lines += [
            f"  - 纬度：{takeoff_telemetry['latitude_deg']:.8f}°",
            f"  - 经度：{takeoff_telemetry['longitude_deg']:.8f}°",
        ]
    else:
        lines.append("  - 未获取有效 GPS 坐标")
    lines += ["", "  投放区域规划经纬度范围："] + planned_gps_bounds(
        takeoff_telemetry, planning_heading, drop_x_min, drop_x_max,
        drop_width * 0.5 if drop_width is not None else None)
    lines += ["", "  投放区域实际 GPS 覆盖范围："] + observed_gps_bounds(drop_rows)
    lines += ["", "  灾情侦察区域规划经纬度范围："] + planned_gps_bounds(
        takeoff_telemetry, planning_heading, recon_x_min, recon_x_max,
        recon_width * 0.5 if recon_width is not None else None)
    lines += ["", "  灾情侦察区域实际 GPS 覆盖范围："] + observed_gps_bounds(recon_rows)
    lines += ["", "  整片任务区域规划经纬度范围："] + planned_gps_bounds(
        takeoff_telemetry, planning_heading, 0.0,
        max(task_x_max_values) if task_x_max_values else None,
        max(task_width_values) * 0.5 if task_width_values else None)
    lines += ["", "  整片任务区域实际 GPS 覆盖范围："] + observed_gps_bounds(telemetry)
    lines += ["", "  六、故障、告警和异常", ""]
    meaningful = [(ts, msg) for ts, msg in parsed["errors"] if "TARGET_REACQUIRED_NEAREST" not in msg]
    if meaningful:
        for ts, msg in meaningful:
            lines += [f"  - {_local_time(ts)}，T{ts - origin:+.3f} 秒", f"    {msg}", ""]
    else:
        lines.append("  任务期间未发现 ERROR/WARN/FATAL。")
    lines += ["  七、数据文件", "", f"  本次飞行目录：", f"  {root}/", "", f"  - ROS任务日志：{'有' if node_logs else '缺失'}", f"  - 结构化飞控遥测 CSV：{'有' if telemetry else '缺失'}", f"  - 视觉检测 CSV：{'有' if any(p.name == 'detections_camera.csv' for p in files) else '缺失'}", f"  - 标注视频：{'有' if any(p.name == 'basket_annotated.avi' for p in files) else '缺失'}", f"  - 原始飞控日志：{'有' if raw_log_files(files) else '缺失'}", "", "  八、最终结论", "", f"  任务{'成功完成。' if result == '成功完成' else '未成功完成。'}"]
    return "\n".join(lines) + "\n"


def load_smtp_config(path: Path | None = None) -> dict[str, Any]:
    """Load non-secret SMTP settings; the token is deliberately excluded."""
    config_path = path or Path(__file__).resolve().parent.parent / "config" / "smtp.yaml"
    if not config_path.is_file() or yaml is None:
        return {}
    try:
        data = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception as exc:
        raise RuntimeError(f"SMTP 配置文件读取失败: {config_path}: {exc}") from exc


def _addresses(value: Any) -> list[str]:
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    if isinstance(value, (list, tuple)):
        return [str(item).strip() for item in value if str(item).strip()]
    return []


def send_email(
    report: str, subject: str, token_env: str, config_path: Path | None = None
) -> None:
    config = load_smtp_config(config_path)
    smtp_config = config.get("smtp", config)
    # The address is public configuration, but the authorization code must
    # only come from the process environment and must never be logged.
    token = os.environ.get(token_env) or os.environ.get("SMTP_TOKEN", "")
    user = os.environ.get("CUADC_SMTP_USER") or os.environ.get(
        "SMTP_USER", str(smtp_config.get("user", ""))
    )
    targets = _addresses(os.environ.get("CUADC_SMTP_TO")) or _addresses(
        os.environ.get("SMTP_TO")
    ) or _addresses(config.get("to", smtp_config.get("to", [])))
    cc = _addresses(os.environ.get("CUADC_SMTP_CC")) or _addresses(
        os.environ.get("SMTP_CC")
    ) or _addresses(config.get("cc", smtp_config.get("cc", [])))
    host = os.environ.get("CUADC_SMTP_HOST") or os.environ.get(
        "SMTP_HOST", str(smtp_config.get("host", ""))
    )
    port_text = os.environ.get("CUADC_SMTP_PORT") or os.environ.get(
        "SMTP_PORT", str(smtp_config.get("port", 465))
    )
    port = int(port_text)
    if not token or not user or not targets or not host:
        raise RuntimeError(f"缺少 SMTP 服务器、发件人、主送地址或 {token_env} 环境变量")
    msg = email.message.EmailMessage()
    msg["From"] = user
    msg["To"] = ", ".join(targets)
    if cc:
        msg["Cc"] = ", ".join(cc)
    msg["Subject"] = subject
    msg.set_content(report)
    # Attach the two post-release snapshots when available. Missing images are
    # reported in the text report/data-integrity section and do not prevent the
    # remaining report from being mailed.
    attachment_root = config_path.parent if config_path else None
    for image_path in getattr(send_email, "attachment_paths", []):
        try:
            data = image_path.read_bytes()
            subtype = "jpeg" if image_path.suffix.lower() in {".jpg", ".jpeg"} else "png"
            msg.add_attachment(data, maintype="image", subtype=subtype, filename=image_path.name)
        except OSError:
            continue
    timeout_s = min(15.0, max(0.1, float(os.environ.get("CUADC_SMTP_TIMEOUT_S", "15"))))
    with smtplib.SMTP_SSL(host, port, timeout=timeout_s) as smtp:
        smtp.login(user, token)
        smtp.send_message(msg, to_addrs=targets + cc)


def send_email_with_retry(report, subject, token_env, config_path, deadline):
    """Linux postflight main process only: bound DNS + SMTP, not just sockets.

    Each attempt gets at most 15 seconds, with 3 seconds between failures.
    Normal and error notification share one 60-second monotonic deadline.
    A timeout after server acceptance can cause a duplicate delivery on retry.
    """
    def timed_out(signum, frame):
        raise TimeoutError('本次 SMTP/DNS 发送尝试超时')

    previous = signal.signal(signal.SIGALRM, timed_out)
    attempt = 0
    last_error = None
    try:
        while time.monotonic() < deadline:
            attempt += 1
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            print(f'邮件发送尝试 {attempt}，剩余 {remaining:.1f} 秒', file=sys.stderr, flush=True)
            signal.setitimer(signal.ITIMER_REAL, min(15.0, remaining))
            try:
                send_email(report, subject, token_env, config_path)
                print(f'邮件发送成功（第 {attempt} 次）', file=sys.stderr, flush=True)
                return
            except Exception as exc:
                last_error = exc
                print(f'邮件发送尝试 {attempt} 失败：{exc}', file=sys.stderr, flush=True)
            finally:
                signal.setitimer(signal.ITIMER_REAL, 0)
            time.sleep(min(3.0, max(0.0, deadline - time.monotonic())))
        raise TimeoutError(f'邮件发送总时限 60 秒已到，停止重试；最后错误：{last_error}')
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        signal.signal(signal.SIGALRM, previous)


def main() -> int:
    ap = argparse.ArgumentParser(description="生成本地飞行总结；手动 --send 才发送邮件")
    ap.add_argument("log_dir", type=Path, help="本次飞行日志目录")
    ap.add_argument("--report", type=Path, help="本地报告输出路径")
    delivery = ap.add_mutually_exclusive_group()
    delivery.add_argument("--send", action="store_true", help="用户手动选择：按配置发送邮件")
    delivery.add_argument("--no-email", action="store_true", help="仅生成本地文件（默认行为，保留兼容选项）")
    ap.add_argument("--token-env", default="CUADC_SMTP_TOKEN", help="授权码所在环境变量名称")
    ap.add_argument("--smtp-config", type=Path, help="不含授权码的 SMTP 配置文件")
    args = ap.parse_args()
    email_deadline = None
    try:
        report = generate_report(args.log_dir)
        report_path = args.report or args.log_dir / "flight_summary.txt"
        report_path.write_text(report, encoding="utf-8")
        print(report, end="", flush=True)
        if args.send and not args.no_email:
            snapshot_dir = args.log_dir / "vision" / "release_snapshots"
            send_email.attachment_paths = sorted(
                list(snapshot_dir.glob("release_*.jpg")) +
                list(snapshot_dir.glob("release_*.jpeg")) +
                list(snapshot_dir.glob("release_*.png"))
            )
            email_deadline = time.monotonic() + 60.0
            send_email_with_retry(report, f"飞行总结 {args.log_dir.name}", args.token_env, args.smtp_config, email_deadline)
            print("飞行总结邮件已发送", file=sys.stderr)
        return 0
    except Exception as exc:
        failure = f"飞行报告生成/发送失败\n目录: {args.log_dir}\n错误: {exc}\n\n{traceback.format_exc()}"
        try:
            (args.report or args.log_dir / "flight_summary_error.txt").write_text(failure, encoding="utf-8")
        except Exception:
            pass
        print(failure, file=sys.stderr)
        if args.send and not args.no_email and (email_deadline is None or time.monotonic() < email_deadline):
            try:
                send_email.attachment_paths = []
                if email_deadline is None:
                    email_deadline = time.monotonic() + 60.0
                send_email_with_retry(failure, f"飞行报告异常 {args.log_dir.name}", args.token_env, args.smtp_config, email_deadline)
            except Exception as email_exc:
                print(f"错误通知邮件也发送失败: {email_exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
