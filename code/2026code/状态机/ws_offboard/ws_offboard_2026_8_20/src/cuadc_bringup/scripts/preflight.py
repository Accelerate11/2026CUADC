#!/usr/bin/env python3
"""Fail-closed hardware, model, graph, and authorization preflight checks."""

from __future__ import annotations

import argparse
import hashlib
import hmac
import os
import re
import shutil
import stat
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import List

try:
    from profile_loader import LoadedProfile, ProfileError, load_profile
except ImportError:  # pragma: no cover - installed ROS invocation fallback
    from ament_index_python.packages import get_package_share_directory

    sys.path.insert(0, str(Path(get_package_share_directory("cuadc_bringup")) / "scripts"))
    from profile_loader import LoadedProfile, ProfileError, load_profile


AUTHORIZATION_ENV = "CUADC_FLIGHT_AUTHORIZED"
AUTHORIZATION_PHRASE = "YES_I_COMPLETED_PREFLIGHT"
APPROVED_BUNDLE_ENV = "CUADC_APPROVED_CONFIG_BUNDLE_SHA256"
RUNTIME_GATE_VALUE = "CUADC_FLIGHT_LAUNCH_PREFLIGHT_PASSED"
CONFIG_BUNDLE_ENV = "CUADC_CONFIG_BUNDLE_SHA256"
TRUSTED_MODEL_SHA256 = "061b67ace71d5f036f7003a3699640dc0a8522e3c3162116884b67b552cd87bf"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class PreflightError(RuntimeError):
    """Raised before any flight process starts when a required check fails."""


@dataclass
class PreflightReport:
    bundle_sha256: str
    checks: List[str] = field(default_factory=list)
    notices: List[str] = field(default_factory=list)

    def passed(self, name: str, detail: str) -> None:
        self.checks.append(f"PASS {name}: {detail}")

    def notice(self, detail: str) -> None:
        self.notices.append(f"NOTICE: {detail}")

    def lines(self) -> List[str]:
        return [
            f"CONFIG_BUNDLE_SHA256={self.bundle_sha256}",
            *self.checks,
            *self.notices,
        ]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve_model_path(profile: LoadedProfile) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError as exc:
        raise PreflightError("ament_index_python is required to resolve basket_v3.pt") from exc

    package = str(profile.model["package"])
    package_share = Path(get_package_share_directory(package)).resolve(strict=True)
    model_path = (package_share / str(profile.model["relative_path"])).resolve(strict=True)
    try:
        model_path.relative_to(package_share)
    except ValueError as exc:
        raise PreflightError("model path escapes the cuadc_perception package") from exc
    return model_path


def _require_flight_authorization(profile: LoadedProfile, report: PreflightReport) -> None:
    if not profile.flight_approved:
        raise PreflightError(
            "profile approval is closed: aircraft and calibration layers must both set "
            "flight_enable=true and calibration_approved=true in the approved config copy"
        )
    aircraft_reference = str(profile.approval.get("approval_reference", "")).strip()
    calibration_reference = str(
        profile.calibration_approval.get("approval_reference", "")
    ).strip()
    if not aircraft_reference or aircraft_reference != calibration_reference:
        raise PreflightError(
            "aircraft/calibration approval_reference must be the same non-empty field record"
        )
    calibration_payload = profile.calibration.get("payload", {})
    verification_states = {
        "FCU device/baud": profile.fcu.get("verification_status"),
        "payload bench": profile.aircraft.get("payload", {}).get("verification_status"),
        "aircraft speed/altitude/tilt limits": profile.aircraft.get("hard_limits", {}).get(
            "verification_status"
        ),
        "camera serial": profile.camera.get("serial_verification_status"),
        "camera extrinsic": profile.camera.get("extrinsic_verification_status"),
        "payload offsets": calibration_payload.get("offsets_verification_status"),
    }
    unverified = {
        name: state
        for name, state in verification_states.items()
        if state != "verified_on_aircraft"
    }
    if unverified:
        raise PreflightError(
            "approved profile still contains candidate/unmeasured hardware fields; "
            f"complete bench/airframe checks and set verified_on_aircraft: {unverified}"
        )
    if os.environ.get(AUTHORIZATION_ENV) != AUTHORIZATION_PHRASE:
        raise PreflightError(
            f"{AUTHORIZATION_ENV} must exactly equal the documented preflight phrase"
        )
    approved = os.environ.get(APPROVED_BUNDLE_ENV, "")
    if not _SHA256.fullmatch(approved):
        raise PreflightError(
            f"{APPROVED_BUNDLE_ENV} must be exactly 64 lowercase hexadecimal characters"
        )
    if not hmac.compare_digest(approved, profile.bundle_sha256):
        raise PreflightError(
            "approved bundle SHA does not match the canonical five-layer configuration"
        )
    report.passed("flight authorization", "phrase, profile approval, and bundle SHA agree")


def _check_fcu(profile: LoadedProfile, report: PreflightReport) -> None:
    device = Path(str(profile.fcu["device"]))
    if not device.is_symlink():
        raise PreflightError(f"FCU path is not the exact by-id symlink: {device}")
    try:
        resolved = device.resolve(strict=True)
        mode = resolved.stat().st_mode
    except OSError as exc:
        raise PreflightError(f"FCU by-id symlink is broken or unreadable: {device}: {exc}") from exc
    if not stat.S_ISCHR(mode):
        raise PreflightError(f"FCU by-id target is not a character device: {resolved}")
    if not os.access(device, os.R_OK | os.W_OK):
        raise PreflightError(f"current user lacks read/write access to FCU device: {device}")
    baud = int(profile.fcu["baud"])
    expected_url = f"serial://{device}:{baud}"
    if profile.fcu["fcu_url"] != expected_url:
        raise PreflightError("FCU URL changed after profile validation")
    report.passed("FCU", f"{device} -> {resolved}, baud={baud}")


def _check_realsense(profile: LoadedProfile, report: PreflightReport) -> None:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        raise PreflightError("pyrealsense2 is not importable in the flight environment") from exc
    expected = str(profile.camera["serial"])
    try:
        devices = list(rs.context().query_devices())
    except Exception as exc:  # pyrealsense2 exposes device-specific exception classes
        raise PreflightError(f"RealSense enumeration failed: {exc}") from exc
    matches = []
    enumerated = []
    for device in devices:
        try:
            serial_number = device.get_info(rs.camera_info.serial_number)
            name = device.get_info(rs.camera_info.name)
        except Exception as exc:
            raise PreflightError(f"unable to read RealSense identity: {exc}") from exc
        enumerated.append(f"{name}:{serial_number}")
        if serial_number == expected:
            matches.append((name, device))
    if len(matches) != 1:
        raise PreflightError(
            f"expected exactly one D435i serial {expected}; "
            f"matches={[name for name, _ in matches]}, "
            f"enumerated={enumerated}"
        )
    product_name, matched_device = matches[0]
    normalised_name = product_name.upper().replace(" ", "")
    if "D435I" not in normalised_name:
        raise PreflightError(
            f"serial {expected} reports {product_name!r}, not an Intel RealSense D435i"
        )

    sensor_names = []
    try:
        for sensor in matched_device.query_sensors():
            sensor_names.append(sensor.get_info(rs.camera_info.name))
    except Exception as exc:
        raise PreflightError(f"unable to enumerate D435i sensors: {exc}") from exc
    normalised_sensors = " ".join(sensor_names).lower()
    if "rgb" not in normalised_sensors or not any(
        token in normalised_sensors for token in ("depth", "stereo")
    ):
        raise PreflightError(
            f"D435i does not expose both RGB and depth sensors: {sensor_names}"
        )

    realsense = profile.realsense_parameters()

    def parse_stream_profile(value: object, label: str) -> tuple[int, int, int]:
        match = re.fullmatch(r"([0-9]{2,4})x([0-9]{2,4})x([0-9]{1,3})", str(value))
        if match is None:
            raise PreflightError(f"{label} must use WIDTHxHEIGHTxFPS")
        width, height, fps = (int(part) for part in match.groups())
        if width != 640 or height != 480 or fps != 30:
            raise PreflightError(f"{label} must remain exactly 640x480x30")
        return width, height, fps

    color_width, color_height, color_fps = parse_stream_profile(
        realsense.get("rgb_camera.color_profile"), "RGB profile"
    )
    depth_width, depth_height, depth_fps = parse_stream_profile(
        realsense.get("depth_module.depth_profile"), "depth profile"
    )
    pipeline = rs.pipeline()
    configuration = rs.config()
    configuration.enable_device(expected)
    configuration.enable_stream(
        rs.stream.color, color_width, color_height, rs.format.bgr8, color_fps
    )
    configuration.enable_stream(
        rs.stream.depth, depth_width, depth_height, rs.format.z16, depth_fps
    )
    try:
        pipeline.start(configuration)
        try:
            frames = pipeline.wait_for_frames(5000)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                raise PreflightError("D435i did not return both color and depth frames")
            if (
                color_frame.get_width() != color_width
                or color_frame.get_height() != color_height
                or depth_frame.get_width() != depth_width
                or depth_frame.get_height() != depth_height
            ):
                raise PreflightError("D435i returned frames at an unexpected resolution")
        finally:
            pipeline.stop()
    except PreflightError:
        raise
    except Exception as exc:
        raise PreflightError(
            f"D435i stream ownership/data-path test failed for serial {expected}: {exc}"
        ) from exc
    report.passed(
        "D435i",
        f"serial={expected}, product={product_name}, sensors={sensor_names}, "
        "color+depth=640x480x30 start/frame/stop",
    )


def _check_model(profile: LoadedProfile, model_path: Path, report: PreflightReport) -> None:
    configured = str(profile.model["sha256"])
    if configured != TRUSTED_MODEL_SHA256:
        raise PreflightError("profile model SHA is not the trusted release SHA")
    if not model_path.is_file():
        raise PreflightError(f"model file is missing: {model_path}")
    actual = sha256_file(model_path)
    if not hmac.compare_digest(actual, TRUSTED_MODEL_SHA256):
        raise PreflightError(f"basket_v3.pt SHA mismatch: actual={actual}")
    sidecar = model_path.with_name(model_path.name + ".sha256")
    if not sidecar.is_file():
        raise PreflightError(f"model SHA sidecar is missing: {sidecar}")
    try:
        sidecar_token = sidecar.read_text(encoding="ascii").split()[0]
    except (OSError, UnicodeError, IndexError) as exc:
        raise PreflightError(f"model SHA sidecar cannot be read: {sidecar}") from exc
    if not hmac.compare_digest(sidecar_token, TRUSTED_MODEL_SHA256):
        raise PreflightError("model SHA sidecar disagrees with the trusted release SHA")
    report.passed("model integrity", f"{model_path}, sha256={actual}")


_BLACK_FRAME_PROGRAM = r"""
import hashlib
import sys

import numpy as np
import torch
from ultralytics import YOLO

model_path, expected_hash, image_size_text = sys.argv[1:4]
digest = hashlib.sha256(open(model_path, "rb").read()).hexdigest()
if digest != expected_hash:
    raise SystemExit("worker model hash mismatch")
if not torch.cuda.is_available():
    raise SystemExit("torch.cuda.is_available() is false")
torch.cuda.set_device(0)
image_size = int(image_size_text)
frame = np.zeros((image_size, image_size, 3), dtype=np.uint8)
model = YOLO(model_path, task="segment")
results = model.predict(
    source=frame,
    device="0",
    half=True,
    imgsz=image_size,
    verbose=False,
    save=False,
    stream=False,
)
torch.cuda.synchronize(0)
if results is None or len(results) != 1:
    raise SystemExit("black-frame inference did not return exactly one result")
print(torch.cuda.get_device_name(0))
"""


def _check_cuda_black_frame(
    profile: LoadedProfile, model_path: Path, report: PreflightReport
) -> None:
    if profile.model.get("require_cuda") is not True:
        raise PreflightError("release profile must require CUDA")
    if profile.model.get("require_black_frame_inference") is not True:
        raise PreflightError("release profile must require black-frame inference")
    timeout = float(profile.model.get("black_frame_timeout_s", 120.0))
    if not 10.0 <= timeout <= 180.0:
        raise PreflightError("black_frame_timeout_s must be in [10, 180]")
    image_size = int(
        profile.perception["perception_parameters"].get("inference_image_size", 640)
    )
    command = [
        sys.executable,
        "-c",
        _BLACK_FRAME_PROGRAM,
        str(model_path),
        TRUSTED_MODEL_SHA256,
        str(image_size),
    ]
    try:
        completed = subprocess.run(
            command,
            check=False,
            text=True,
            capture_output=True,
            timeout=timeout,
            env=os.environ.copy(),
        )
    except subprocess.TimeoutExpired as exc:
        raise PreflightError(f"CUDA black-frame inference timed out after {timeout:.0f}s") from exc
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()[-1200:]
        raise PreflightError(f"CUDA black-frame inference failed: {detail}")
    device_name = completed.stdout.strip().splitlines()[-1] if completed.stdout.strip() else "GPU 0"
    report.passed("CUDA black-frame inference", device_name)


def _check_ros_graph(report: PreflightReport) -> None:
    ros2 = shutil.which("ros2")
    if ros2 is None:
        raise PreflightError("ros2 CLI is not available for the graph conflict gate")
    try:
        completed = subprocess.run(
            [ros2, "node", "list", "--no-daemon", "--spin-time", "2.0", "--all"],
            check=False,
            text=True,
            capture_output=True,
            timeout=15.0,
            env=os.environ.copy(),
        )
    except subprocess.TimeoutExpired as exc:
        raise PreflightError("ROS graph conflict query timed out") from exc
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout).strip()[-1200:]
        raise PreflightError(f"ROS graph conflict query failed: {detail}")
    nodes = sorted({line.strip() for line in completed.stdout.splitlines() if line.strip()})
    reserved = {
        "mavros",
        "camera",
        "realsense2_camera",
        "bucket_perception_node",
        "mission_node",
        "offb_node",
        "payload_node",
        "cuadc_payload_node",
        "safety_monitor",
    }
    conflicts = []
    prohibited_fragments = ("gazebo", "gzserver", "rosbag", "sitl", "simulator")
    for node in nodes:
        basename = node.rstrip("/").rsplit("/", 1)[-1]
        lowered = node.lower()
        if basename in reserved or any(fragment in lowered for fragment in prohibited_fragments):
            conflicts.append(node)
    if conflicts:
        raise PreflightError(f"ROS graph/process namespace conflict: {conflicts}")
    report.passed("ROS graph conflict gate", f"no reserved/simulation nodes; existing={nodes}")


def _check_log_target(profile: LoadedProfile, report: PreflightReport) -> None:
    parameters = profile.safety_parameters()
    configured = str(parameters.get("log_directory", ""))
    if configured and not Path(configured).expanduser().is_absolute():
        raise PreflightError("safety log_directory must be empty or an absolute path")
    evidence = str(parameters.get("recon_evidence_directory", ""))
    if evidence and not Path(evidence).expanduser().is_absolute():
        raise PreflightError("recon_evidence_directory must be empty or an absolute path")
    minimum = int(parameters.get("minimum_free_megabytes", 512)) * 1024 * 1024
    log_target = (
        Path(configured).expanduser() if configured else Path.home() / "cuadc_logs"
    ).absolute()
    targets = [("safety log target", log_target)]
    if evidence:
        targets.append(("recon evidence target", Path(evidence).expanduser().absolute()))
    for label, target in targets:
        ancestor = target
        while not ancestor.exists() and ancestor != ancestor.parent:
            ancestor = ancestor.parent
        if not ancestor.is_dir() or not os.access(ancestor, os.W_OK | os.X_OK):
            raise PreflightError(f"{label} has no writable parent: {target}")
        free = shutil.disk_usage(ancestor).free
        if free < minimum:
            raise PreflightError(
                f"{label} filesystem has {free} bytes free, below required {minimum}"
            )
        report.passed(label, f"{target}, free_bytes={free}")


def run_preflight(
    profile: LoadedProfile,
    model_path: Path,
    *,
    require_flight_authorization: bool,
) -> PreflightReport:
    """Run all checks synchronously before launch returns any process actions."""

    report = PreflightReport(bundle_sha256=profile.bundle_sha256)
    if require_flight_authorization:
        _require_flight_authorization(profile, report)
    else:
        report.notice(
            "read-only preflight: no runtime gate is issued and profile approval is not assumed"
        )
        if not profile.flight_approved:
            report.notice(
                "candidate profile remains disabled (flight_enable/calibration_approved are false)"
            )
    _check_fcu(profile, report)
    _check_realsense(profile, report)
    _check_model(profile, model_path, report)
    _check_cuda_black_frame(profile, model_path, report)
    _check_ros_graph(report)
    _check_log_target(profile, report)
    return report


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-root", required=True)
    parser.add_argument("--aircraft", choices=("v6x_3E0032", "v5nano_410035"), required=True)
    parser.add_argument("--mission", default="competition_2026")
    parser.add_argument("--model", help="explicit model path; normally resolved from ament index")
    parser.add_argument(
        "--require-flight-authorization",
        action="store_true",
        help="also require authorization phrase, approved bundle SHA, and profile approval",
    )
    return parser


def main(argv: List[str] | None = None) -> int:
    arguments = _build_parser().parse_args(argv)
    try:
        profile = load_profile(arguments.config_root, arguments.aircraft, arguments.mission)
        model_path = (
            Path(arguments.model).expanduser().resolve(strict=True)
            if arguments.model
            else resolve_model_path(profile)
        )
        report = run_preflight(
            profile,
            model_path,
            require_flight_authorization=arguments.require_flight_authorization,
        )
    except (OSError, ProfileError, PreflightError) as exc:
        print(f"PREFLIGHT FAILED: {exc}", file=sys.stderr)
        return 2
    for line in report.lines():
        print(line)
    print("PREFLIGHT PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
