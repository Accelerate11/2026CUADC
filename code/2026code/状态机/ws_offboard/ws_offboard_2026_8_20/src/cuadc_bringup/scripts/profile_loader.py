#!/usr/bin/env python3
"""Load and validate the five-layer CUADC real-aircraft configuration bundle.

The bundle digest is intentionally independent of the absolute config_root and
of YAML whitespace.  It covers the parsed content, selected layer identities,
and the digest schema identifier.  Duplicate YAML keys and non-JSON values are
rejected so two tools cannot silently interpret the approved bytes differently.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, MutableMapping, Tuple

import yaml


BUNDLE_SCHEMA = "cuadc.config-bundle.sha256.v1"
LAYER_ORDER = ("aircraft", "calibration", "mission", "field", "perception")
SUPPORTED_AIRCRAFT = ("v6x_3E0032", "v5nano_410035")
SUPPORTED_SCHEMAS = {
    "aircraft": "cuadc.aircraft.v1",
    "calibration": "cuadc.calibration.v1",
    "mission": "cuadc.mission.v1",
    "field": "cuadc.field.v1",
    "perception": "cuadc.perception.v1",
}
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,95}$")
_LOWER_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_CAMERA_SERIAL = re.compile(r"^[0-9]{6,20}$")

# These sets mirror the parameters declared by the corresponding release
# executables.  Exact-key checks prevent a renamed/removed parameter from being
# silently ignored and prevent a safety-critical C++ default from taking over.
MISSION_PARAMETER_ALLOWLIST = frozenset(
    {
        "aircraft_max_drop_speed_m_s",
        "aircraft_max_horizontal_speed_m_s",
        "aircraft_max_mission_altitude_m",
        "aircraft_max_release_tilt_deg",
        "alignment_timeout_s",
        "auto_arm_on_guided",
        "bucket_body_filter_alpha",
        "bucket_confidence_filter_alpha",
        "bucket_detection_topic",
        "bucket_diameter_filter_alpha",
        "bucket_diameter_filter_window",
        "bucket_distinct_min_diameter_m",
        "bucket_distinct_min_separation_m",
        "bucket_max_diameter_deviation_m",
        "bucket_max_position_deviation_m",
        "bucket_min_confirmations",
        "bucket_min_track_confidence",
        "bucket_position_filter_alpha",
        "bucket_required_count",
        "bucket_selection_max_age_s",
        "bucket_target_ranking_stable_s",
        "bucket_track_max_gap_s",
        "coarse_alt_m",
        "coarse_error_m",
        "coarse_stable_s",
        "compass_timeout_s",
        "config_bundle_sha256",
        "cross_track_camera_fov_deg",
        "detection_timeout_s",
        "diameter_track_gate_m",
        "direct_release_height_tolerance_m",
        "direct_release_max_angular_rate_deg_s",
        "direct_release_max_horizontal_speed_m_s",
        "direct_release_max_tilt_deg",
        "direct_release_max_vertical_speed_m_s",
        "direct_release_stable_s",
        "direct_release_timeout_s",
        "direct_release_xy_tolerance_m",
        "disarm_timeout_s",
        "drop_approach_speed_m_s",
        "drop_area_length_field_x_m",
        "drop_area_near_edge_field_x_m",
        "drop_area_width_field_y_m",
        "enforce_final_drop_heights",
        "extended_state_timeout_s",
        "fine_alt_m",
        "fine_error_m",
        "fine_stable_s",
        "flight_enable",
        "heading_lock_max_variation_deg",
        "heading_lock_stability_s",
        "known_bucket_memory_s",
        "land_timeout_s",
        "landing_confirm_stable_s",
        "landing_max_horizontal_speed_m_s",
        "landing_max_relative_altitude_m",
        "landing_max_vertical_speed_m_s",
        "lock_mission_yaw_to_initial_heading",
        "max_search_passes",
        "min_segment_time_s",
        "mission_epoch",
        "mission_timeout_s",
        "nav_interpolation_max_gap_s",
        "odom_history_s",
        "odom_timeout_s",
        "payload_action_timeout_s",
        "payload_count",
        "payload_initialization_timeout_s",
        "payload_release_offsets_body_m",
        "position_lock_stability_s",
        "prestream_hold_s",
        "recon_area_center_field_x_m",
        "recon_area_length_field_x_m",
        "recon_area_width_field_y_m",
        "recon_cross_margin_m",
        "recon_edge_margin_m",
        "recon_hover_alt_m",
        "recon_lane_count",
        "recon_speed_m_s",
        "recon_waypoint_hold_s",
        "recover_hold_s",
        "release_max_yaw_error_deg",
        "release_pose_refresh_reset_m",
        "release_positioning_speed_m_s",
        "released_bucket_exclusion_m",
        "return_climb_timeout_s",
        "return_home_timeout_s",
        "search_alt_m",
        "search_cross_margin_m",
        "search_edge_margin_m",
        "search_lane_count",
        "search_lane_overlap_ratio",
        "search_speed_m_s",
        "search_vision_acquire_min_frames",
        "search_vision_acquire_timeout_s",
        "safety_status_timeout_s",
        "service_ack_timeout_s",
        "stationary_speed_max_m_s",
        "takeoff_alt_m",
        "takeoff_timeout_s",
        "target_guidance_max_age_s",
        "track_gate_m",
        "transit_speed_m_s",
        "vision_future_tolerance_s",
        "vision_heartbeat_timeout_s",
        "vision_max_pipeline_delay_s",
        "vision_min_messages_before_takeoff",
        "vision_transform_tolerance_s",
        "waypoint_accept_radius_m",
        "yaw_to_target",
    }
)

PAYLOAD_PARAMETER_ALLOWLIST = frozenset(
    {
        "flight_enable",
        "allow_legacy_release_service",
        "command_ack_timeout_ms",
        "servo_channels",
        "servo_stowed_pwm",
        "servo_release_pwm",
        "servo_release_duration_s",
    }
)

SAFETY_PARAMETER_ALLOWLIST = frozenset(
    {
        "flight_enable",
        "config_bundle_sha256",
        "log_directory",
        "recon_evidence_directory",
        "recon_image_topic",
        "recon_capture_interval_s",
        "recon_jpeg_quality",
        "mavros_timeout_s",
        "odom_timeout_s",
        "vision_timeout_s",
        "payload_timeout_s",
        "mission_status_timeout_s",
        "mission_timeout_s",
        "startup_grace_s",
        "minimum_free_megabytes",
    }
)

PERCEPTION_PARAMETER_ALLOWLIST = frozenset(
    {
        "model_path",
        "model_sha256",
        "camera_serial",
        "color_topic",
        "depth_topic",
        "camera_info_topic",
        "output_topic",
        "frame_id",
        "sync_queue_size",
        "max_input_skew_s",
        "depth_scale_m_per_unit",
        "target_class",
        "confidence_threshold",
        "nms_iou_threshold",
        "inference_image_size",
        "device",
        "half",
        "depth_mode",
        "bucket_height_m",
        "rim_band_thickness_pixels",
        "depth_filter_size",
        "duplicate_mask_iou_threshold",
        "ground_ring_inner_scale",
        "ground_ring_outer_scale",
        "camera_to_body_rotation",
        "camera_to_body_translation_m",
        "extrinsic_orthogonality_tolerance",
        "min_quality_confidence",
        "min_depth_m",
        "max_depth_m",
        "min_depth_valid_ratio",
        "max_depth_iqr_m",
        "max_ellipse_axis_ratio",
        "min_diameter_m",
        "max_diameter_m",
        "min_mask_area_px",
        "max_consecutive_frame_errors",
    }
)


class ProfileError(RuntimeError):
    """Raised when a profile is ambiguous, malformed, or unsafe to load."""


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_unique_mapping(
    loader: _UniqueKeyLoader, node: yaml.nodes.MappingNode, deep: bool = False
) -> MutableMapping[str, Any]:
    mapping: MutableMapping[str, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str):
            raise ProfileError("every YAML mapping key must be a string")
        if key in mapping:
            raise ProfileError(f"duplicate YAML key is forbidden: {key!r}")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict):
        raise ProfileError(f"{label} must be a YAML mapping")
    return value


def _require_string(value: Any, label: str, *, nonempty: bool = True) -> str:
    if not isinstance(value, str) or (nonempty and not value.strip()):
        qualifier = "non-empty " if nonempty else ""
        raise ProfileError(f"{label} must be a {qualifier}string")
    return value


def _require_bool(value: Any, label: str) -> bool:
    # bool is a subclass of int, so use an exact type check here and below.
    if type(value) is not bool:
        raise ProfileError(f"{label} must be exactly true or false")
    return value


def _require_int(value: Any, label: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ProfileError(f"{label} must be an integer in [{minimum}, {maximum}]")
    return value


def _normalise_json(value: Any, label: str = "root") -> Any:
    if value is None or isinstance(value, (str, bool)):
        return value
    if type(value) is int:
        return value
    if type(value) is float:
        if not math.isfinite(value):
            raise ProfileError(f"{label} contains NaN or infinity")
        # Collapse signed zero, which has no configuration meaning but has two
        # possible JSON spellings.
        return 0.0 if value == 0.0 else value
    if isinstance(value, list):
        return [_normalise_json(item, f"{label}[]") for item in value]
    if isinstance(value, dict):
        result: Dict[str, Any] = {}
        for key, item in value.items():
            if not isinstance(key, str):
                raise ProfileError(f"{label} contains a non-string mapping key")
            result[key] = _normalise_json(item, f"{label}.{key}")
        return result
    raise ProfileError(f"{label} contains unsupported YAML value {type(value).__name__}")


def _load_yaml(path: Path) -> Mapping[str, Any]:
    if not path.is_file():
        raise ProfileError(f"configuration layer is not a regular file: {path}")
    if path.stat().st_size > 1024 * 1024:
        raise ProfileError(f"configuration layer exceeds 1 MiB: {path}")
    try:
        with path.open("r", encoding="utf-8") as stream:
            loaded = yaml.load(stream, Loader=_UniqueKeyLoader)
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise ProfileError(f"unable to parse {path}: {exc}") from exc
    return _require_mapping(_normalise_json(loaded, path.name), str(path))


def _safe_layer_path(root: Path, kind: str, identifier: str) -> Path:
    if kind not in SUPPORTED_SCHEMAS:
        raise ProfileError(f"unknown configuration layer kind: {kind}")
    if not _SAFE_ID.fullmatch(identifier):
        raise ProfileError(f"unsafe {kind} identifier: {identifier!r}")
    candidate = root / kind / f"{identifier}.yaml"
    try:
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root)
    except (OSError, ValueError) as exc:
        raise ProfileError(f"{kind} layer escapes or is missing from config_root: {candidate}") from exc
    return resolved


def _layer_identity(layer: Mapping[str, Any], kind: str, expected: str) -> None:
    if layer.get("schema") != SUPPORTED_SCHEMAS[kind]:
        raise ProfileError(
            f"{kind} schema must be exactly {SUPPORTED_SCHEMAS[kind]!r}"
        )
    if layer.get(kind) != expected:
        raise ProfileError(f"{kind} identity does not match selected layer {expected!r}")


def _copy_parameters(value: Any, label: str) -> Dict[str, Any]:
    return dict(_require_mapping(value, label))


def _merge_disjoint(destination: Dict[str, Any], source: Mapping[str, Any], label: str) -> None:
    duplicates = sorted(set(destination).intersection(source))
    if duplicates:
        raise ProfileError(f"{label} attempts to override parameters: {duplicates}")
    destination.update(source)


def _require_exact_parameter_keys(
    parameters: Mapping[str, Any], expected: frozenset[str], label: str
) -> None:
    actual = frozenset(parameters)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing or unknown:
        raise ProfileError(
            f"{label} parameter contract mismatch; missing={missing}, unknown={unknown}"
        )


@dataclass(frozen=True)
class LoadedProfile:
    config_root: Path
    aircraft_name: str
    mission_name: str
    layers: Mapping[str, Mapping[str, Any]]
    layer_paths: Mapping[str, Path]
    bundle_sha256: str
    canonical_json: bytes

    @property
    def aircraft(self) -> Mapping[str, Any]:
        return self.layers["aircraft"]

    @property
    def calibration(self) -> Mapping[str, Any]:
        return self.layers["calibration"]

    @property
    def mission(self) -> Mapping[str, Any]:
        return self.layers["mission"]

    @property
    def field(self) -> Mapping[str, Any]:
        return self.layers["field"]

    @property
    def perception(self) -> Mapping[str, Any]:
        return self.layers["perception"]

    @property
    def approval(self) -> Mapping[str, Any]:
        return _require_mapping(self.aircraft.get("approval"), "aircraft.approval")

    @property
    def calibration_approval(self) -> Mapping[str, Any]:
        return _require_mapping(
            self.calibration.get("approval"), "calibration.approval"
        )

    @property
    def flight_approved(self) -> bool:
        return all(
            (
                _require_bool(self.approval.get("flight_enable"), "aircraft.approval.flight_enable"),
                _require_bool(
                    self.approval.get("calibration_approved"),
                    "aircraft.approval.calibration_approved",
                ),
                _require_bool(
                    self.calibration_approval.get("flight_enable"),
                    "calibration.approval.flight_enable",
                ),
                _require_bool(
                    self.calibration_approval.get("calibration_approved"),
                    "calibration.approval.calibration_approved",
                ),
            )
        )

    @property
    def fcu(self) -> Mapping[str, Any]:
        return _require_mapping(self.aircraft.get("fcu"), "aircraft.fcu")

    @property
    def camera(self) -> Mapping[str, Any]:
        return _require_mapping(self.calibration.get("camera"), "calibration.camera")

    @property
    def model(self) -> Mapping[str, Any]:
        return _require_mapping(self.perception.get("model"), "perception.model")

    def mission_parameters(self, mission_epoch: int) -> Dict[str, Any]:
        if type(mission_epoch) is not int or not 1 <= mission_epoch <= (2**63 - 1):
            raise ProfileError("mission_epoch must be a non-zero positive int64")
        result: Dict[str, Any] = {}
        _merge_disjoint(
            result,
            _copy_parameters(self.field.get("mission_parameters"), "field.mission_parameters"),
            "field.mission_parameters",
        )
        _merge_disjoint(
            result,
            _copy_parameters(
                self.mission.get("mission_parameters"), "mission.mission_parameters"
            ),
            "mission.mission_parameters",
        )
        _merge_disjoint(
            result,
            _copy_parameters(
                self.aircraft.get("mission_parameters"), "aircraft.mission_parameters"
            ),
            "aircraft.mission_parameters",
        )
        calibration_payload = _require_mapping(
            self.calibration.get("payload"), "calibration.payload"
        )
        hard_limits = _require_mapping(
            self.aircraft.get("hard_limits"), "aircraft.hard_limits"
        )
        result.update(
            {
                "flight_enable": self.flight_approved,
                "config_bundle_sha256": self.bundle_sha256,
                "mission_epoch": mission_epoch,
                "aircraft_max_horizontal_speed_m_s": hard_limits[
                    "max_horizontal_speed_m_s"
                ],
                "aircraft_max_drop_speed_m_s": hard_limits["max_drop_speed_m_s"],
                "aircraft_max_mission_altitude_m": hard_limits[
                    "max_mission_altitude_m"
                ],
                "aircraft_max_release_tilt_deg": hard_limits["max_release_tilt_deg"],
                "payload_release_offsets_body_m": list(
                    calibration_payload["release_offsets_body_m"]
                ),
            }
        )
        return result

    def payload_parameters(self) -> Dict[str, Any]:
        payload = _require_mapping(self.aircraft.get("payload"), "aircraft.payload")
        return {
            "flight_enable": self.flight_approved,
            "allow_legacy_release_service": False,
            "servo_channels": list(payload["servo_channels"]),
            "servo_stowed_pwm": list(payload["stowed_pwm"]),
            "servo_release_pwm": list(payload["release_pwm"]),
            "servo_release_duration_s": list(payload["release_duration_s"]),
            "command_ack_timeout_ms": payload["command_ack_timeout_ms"],
        }

    def perception_parameters(self, model_path: Path) -> Dict[str, Any]:
        result = _copy_parameters(
            self.perception.get("perception_parameters"),
            "perception.perception_parameters",
        )
        result.update(
            {
                "model_path": str(model_path),
                "model_sha256": self.model["sha256"],
                "camera_serial": self.camera["serial"],
                "camera_to_body_rotation": list(self.camera["camera_to_body_rotation"]),
                "camera_to_body_translation_m": list(
                    self.camera["camera_to_body_translation_m"]
                ),
            }
        )
        return result

    def realsense_parameters(self) -> Dict[str, Any]:
        result = _copy_parameters(
            self.perception.get("realsense_parameters"),
            "perception.realsense_parameters",
        )
        result["serial_no"] = self.camera["serial"]
        return result

    def safety_parameters(self) -> Dict[str, Any]:
        result = _copy_parameters(
            self.mission.get("safety_parameters"), "mission.safety_parameters"
        )
        result["flight_enable"] = self.flight_approved
        result["config_bundle_sha256"] = self.bundle_sha256
        return result


def _validate_profile(profile: LoadedProfile) -> None:
    approval = profile.approval
    calibration_approval = profile.calibration_approval
    for label, source in (
        ("aircraft.approval", approval),
        ("calibration.approval", calibration_approval),
    ):
        _require_bool(source.get("flight_enable"), f"{label}.flight_enable")
        _require_bool(
            source.get("calibration_approved"), f"{label}.calibration_approved"
        )
        _require_string(
            source.get("approval_reference", ""),
            f"{label}.approval_reference",
            nonempty=False,
        )

    fcu = profile.fcu
    device = _require_string(fcu.get("device"), "aircraft.fcu.device")
    if not device.startswith("/dev/serial/by-id/") or device.endswith("/"):
        raise ProfileError("aircraft.fcu.device must be one exact /dev/serial/by-id entry")
    baud = _require_int(fcu.get("baud"), "aircraft.fcu.baud", 1200, 4_000_000)
    expected_fcu_url = f"serial://{device}:{baud}"
    if fcu.get("fcu_url") != expected_fcu_url:
        raise ProfileError(
            f"aircraft.fcu.fcu_url must exactly equal {expected_fcu_url!r}"
        )
    _require_string(fcu.get("gcs_url", ""), "aircraft.fcu.gcs_url", nonempty=False)
    if fcu.get("protocol") != "v2.0":
        raise ProfileError("aircraft.fcu.protocol must remain v2.0")
    _require_int(fcu.get("target_system"), "aircraft.fcu.target_system", 1, 255)
    _require_int(fcu.get("target_component"), "aircraft.fcu.target_component", 1, 255)

    camera = profile.camera
    camera_serial = _require_string(camera.get("serial"), "calibration.camera.serial")
    if not _CAMERA_SERIAL.fullmatch(camera_serial):
        raise ProfileError("calibration.camera.serial must contain only 6-20 decimal digits")
    rotation = camera.get("camera_to_body_rotation")
    translation = camera.get("camera_to_body_translation_m")
    if not isinstance(rotation, list) or len(rotation) != 9:
        raise ProfileError("camera_to_body_rotation must be a flat nine-element matrix")
    if not isinstance(translation, list) or len(translation) != 3:
        raise ProfileError("camera_to_body_translation_m must be a three-element vector")
    for label, values in (("rotation", rotation), ("translation", translation)):
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            raise ProfileError(f"camera {label} must contain finite numeric values")
    rows = [rotation[index : index + 3] for index in range(0, 9, 3)]
    tolerance = float(
        _require_mapping(
            profile.perception.get("perception_parameters"),
            "perception.perception_parameters",
        ).get("extrinsic_orthogonality_tolerance", 0.001)
    )
    if not math.isfinite(tolerance) or not 0.0 < tolerance <= 0.01:
        raise ProfileError("extrinsic_orthogonality_tolerance must be in (0, 0.01]")
    for row_index, row in enumerate(rows):
        for column_index, other in enumerate(rows):
            dot = sum(float(row[k]) * float(other[k]) for k in range(3))
            expected = 1.0 if row_index == column_index else 0.0
            if abs(dot - expected) > tolerance:
                raise ProfileError("camera extrinsic rotation is not orthonormal")
    determinant = (
        rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7])
        - rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6])
        + rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6])
    )
    if abs(float(determinant) - 1.0) > tolerance:
        raise ProfileError("camera extrinsic rotation must be right-handed with determinant +1")

    model = profile.model
    if model.get("package") != "cuadc_perception":
        raise ProfileError("the release model must come from package cuadc_perception")
    relative_model = Path(_require_string(model.get("relative_path"), "model.relative_path"))
    if relative_model.is_absolute() or ".." in relative_model.parts:
        raise ProfileError("model.relative_path must stay inside the perception package")
    model_hash = _require_string(model.get("sha256"), "model.sha256")
    if not _LOWER_SHA256.fullmatch(model_hash):
        raise ProfileError("model.sha256 must be exactly 64 lowercase hexadecimal characters")
    _require_bool(model.get("require_cuda"), "model.require_cuda")
    _require_bool(
        model.get("require_black_frame_inference"),
        "model.require_black_frame_inference",
    )
    realsense_parameters = _require_mapping(
        profile.perception.get("realsense_parameters"),
        "perception.realsense_parameters",
    )
    for key in ("enable_color", "enable_depth", "align_depth.enable", "enable_sync"):
        if _require_bool(
            realsense_parameters.get(key), f"perception.realsense_parameters.{key}"
        ) is not True:
            raise ProfileError(f"RealSense parameter {key} must remain true")
    if realsense_parameters.get("rgb_camera.color_profile") != "640x480x30":
        raise ProfileError("RealSense color profile must remain 640x480x30")
    if realsense_parameters.get("depth_module.depth_profile") != "640x480x30":
        raise ProfileError("RealSense depth profile must remain 640x480x30")

    payload = _require_mapping(profile.aircraft.get("payload"), "aircraft.payload")
    channels = payload.get("servo_channels")
    stowed_pwm = payload.get("stowed_pwm")
    release_pwm = payload.get("release_pwm")
    durations = payload.get("release_duration_s")
    if not isinstance(channels, list) or len(channels) != 2:
        raise ProfileError("aircraft.payload.servo_channels must contain exactly two channels")
    if any(type(channel) is not int or not 1 <= channel <= 16 for channel in channels):
        raise ProfileError("payload servo channels must be integers in [1, 16]")
    if channels[0] == channels[1]:
        raise ProfileError("payload servo channels must be different")
    for name, values in (("stowed_pwm", stowed_pwm), ("release_pwm", release_pwm)):
        if not isinstance(values, list) or len(values) != 2:
            raise ProfileError(f"aircraft.payload.{name} must contain exactly two PWM values")
        if any(type(value) is not int or not 800 <= value <= 2200 for value in values):
            raise ProfileError(f"aircraft.payload.{name} values must be integers in [800, 2200]")
    if any(stowed_pwm[index] == release_pwm[index] for index in range(2)):
        raise ProfileError("each payload channel must use different stowed and release PWM")
    if not isinstance(durations, list) or len(durations) != 2:
        raise ProfileError("aircraft.payload.release_duration_s must contain two durations")
    if any(
        type(value) not in (int, float)
        or not math.isfinite(value)
        or not 0.1 <= value <= 3.0
        for value in durations
    ):
        raise ProfileError("payload release durations must be finite seconds in [0.1, 3.0]")
    if _require_bool(payload.get("initialize_stowed"), "payload.initialize_stowed") is not True:
        raise ProfileError("payload.initialize_stowed must remain true")
    if _require_bool(payload.get("return_to_stowed"), "payload.return_to_stowed") is not True:
        raise ProfileError("payload.return_to_stowed must remain true")
    _require_int(
        payload.get("command_ack_timeout_ms"),
        "payload.command_ack_timeout_ms",
        500,
        10_000,
    )

    hard_limits = _require_mapping(
        profile.aircraft.get("hard_limits"), "aircraft.hard_limits"
    )

    def finite_limit(name: str, minimum: float, maximum: float) -> float:
        value = hard_limits.get(name)
        if (
            type(value) not in (int, float)
            or not math.isfinite(value)
            or not minimum <= value <= maximum
        ):
            raise ProfileError(
                f"aircraft.hard_limits.{name} must be finite in [{minimum}, {maximum}]"
            )
        return float(value)

    maximum_horizontal = finite_limit("max_horizontal_speed_m_s", 0.3, 20.0)
    maximum_drop = finite_limit("max_drop_speed_m_s", 0.1, 5.0)
    maximum_altitude = finite_limit("max_mission_altitude_m", 1.0, 120.0)
    maximum_release_tilt = finite_limit("max_release_tilt_deg", 0.1, 30.0)
    if maximum_drop > maximum_horizontal:
        raise ProfileError("aircraft max_drop_speed_m_s cannot exceed horizontal limit")
    _require_string(
        hard_limits.get("verification_status"), "hard_limits.verification_status"
    )

    combined_mission = profile.mission_parameters(1)
    for name in ("transit_speed_m_s", "search_speed_m_s", "recon_speed_m_s"):
        value = combined_mission.get(name)
        if type(value) not in (int, float) or not 0.0 < value <= maximum_horizontal:
            raise ProfileError(f"mission speed {name} exceeds aircraft horizontal limit")
    for name in ("drop_approach_speed_m_s", "release_positioning_speed_m_s"):
        value = combined_mission.get(name)
        if type(value) not in (int, float) or not 0.0 < value <= maximum_drop:
            raise ProfileError(f"mission speed {name} exceeds aircraft drop-speed limit")
    for name in ("takeoff_alt_m", "search_alt_m", "coarse_alt_m", "fine_alt_m", "recon_hover_alt_m"):
        value = combined_mission.get(name)
        if type(value) not in (int, float) or not 0.0 < value <= maximum_altitude:
            raise ProfileError(f"mission altitude {name} exceeds aircraft altitude limit")
    release_tilt = combined_mission.get("direct_release_max_tilt_deg")
    if (
        type(release_tilt) not in (int, float)
        or not 0.0 < release_tilt <= maximum_release_tilt
    ):
        raise ProfileError("direct_release_max_tilt_deg exceeds aircraft tilt limit")

    calibration_payload = _require_mapping(
        profile.calibration.get("payload"), "calibration.payload"
    )
    offsets = calibration_payload.get("release_offsets_body_m")
    if not isinstance(offsets, list) or len(offsets) != 6:
        raise ProfileError("payload release offsets must contain exactly two XYZ vectors")
    if any(type(value) not in (int, float) or not math.isfinite(value) for value in offsets):
        raise ProfileError("payload release offsets must be finite numeric values")

    hazard = _require_mapping(profile.mission.get("hazard_recon"), "mission.hazard_recon")
    if hazard.get("mode") != "waypoint_only" or hazard.get("visual_node_required") is not False:
        raise ProfileError("hazard reconnaissance must remain waypoint-only without visual input")
    lanes = _require_int(hazard.get("lane_count"), "hazard_recon.lane_count", 3, 3)
    if hazard.get("expected_waypoints") != 6:
        raise ProfileError("hazard reconnaissance is frozen at three lanes / six endpoints")
    field_parameters = _require_mapping(
        profile.field.get("mission_parameters"), "field.mission_parameters"
    )
    if field_parameters.get("recon_lane_count") != lanes:
        raise ProfileError("field recon_lane_count must match the three-lane hazard contract")

    # Build every node map now so overlap and missing-key errors are caught by
    # the loader, before any hardware process can be launched.
    merged_mission = profile.mission_parameters(1)
    _require_exact_parameter_keys(
        merged_mission, MISSION_PARAMETER_ALLOWLIST, "mission_node"
    )
    payload_parameters = profile.payload_parameters()
    _require_exact_parameter_keys(
        payload_parameters, PAYLOAD_PARAMETER_ALLOWLIST, "payload_node"
    )
    profile.realsense_parameters()
    safety = profile.safety_parameters()
    _require_exact_parameter_keys(
        safety, SAFETY_PARAMETER_ALLOWLIST, "safety_monitor"
    )
    perception_parameters = profile.perception_parameters(Path("/validated/basket_v3.pt"))
    _require_exact_parameter_keys(
        perception_parameters, PERCEPTION_PARAMETER_ALLOWLIST, "bucket_perception_node"
    )
    safety_timeouts = (
        "mavros_timeout_s",
        "odom_timeout_s",
        "vision_timeout_s",
        "payload_timeout_s",
        "mission_status_timeout_s",
        "mission_timeout_s",
        "startup_grace_s",
    )
    for name in safety_timeouts:
        value = safety.get(name)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0.0:
            raise ProfileError(f"mission.safety_parameters.{name} must be finite and positive")
    minimum_free = safety.get("minimum_free_megabytes")
    if type(minimum_free) is not int or minimum_free < 128:
        raise ProfileError("minimum_free_megabytes must be an integer of at least 128")
    topic = _require_string(safety.get("recon_image_topic"), "safety.recon_image_topic")
    if not topic.startswith("/"):
        raise ProfileError("safety.recon_image_topic must be an absolute ROS topic")
    capture_interval = safety.get("recon_capture_interval_s")
    if (
        type(capture_interval) not in (int, float)
        or not math.isfinite(capture_interval)
        or not 0.1 <= capture_interval <= 60.0
    ):
        raise ProfileError("recon_capture_interval_s must be in [0.1, 60.0]")
    _require_int(safety.get("recon_jpeg_quality"), "safety.recon_jpeg_quality", 1, 100)
    _require_string(
        safety.get("log_directory", ""), "safety.log_directory", nonempty=False
    )
    _require_string(
        safety.get("recon_evidence_directory", ""),
        "safety.recon_evidence_directory",
        nonempty=False,
    )


def load_profile(config_root: str | Path, aircraft: str, mission: str) -> LoadedProfile:
    """Load, validate, and hash one selected five-layer configuration bundle."""

    if aircraft not in SUPPORTED_AIRCRAFT:
        raise ProfileError(
            f"aircraft must be one of {', '.join(SUPPORTED_AIRCRAFT)}"
        )
    if not _SAFE_ID.fullmatch(mission):
        raise ProfileError(f"unsafe mission identifier: {mission!r}")
    configured_root = Path(config_root).expanduser()
    if not configured_root.is_absolute():
        raise ProfileError("config_root must be an absolute path")
    try:
        root = configured_root.resolve(strict=True)
    except OSError as exc:
        raise ProfileError(f"config_root does not exist: {config_root}") from exc
    if not root.is_dir():
        raise ProfileError(f"config_root is not a directory: {root}")

    aircraft_path = _safe_layer_path(root, "aircraft", aircraft)
    mission_path = _safe_layer_path(root, "mission", mission)
    aircraft_layer = _load_yaml(aircraft_path)
    mission_layer = _load_yaml(mission_path)
    _layer_identity(aircraft_layer, "aircraft", aircraft)
    _layer_identity(mission_layer, "mission", mission)

    aircraft_links = _require_mapping(aircraft_layer.get("layers"), "aircraft.layers")
    mission_links = _require_mapping(mission_layer.get("layers"), "mission.layers")
    calibration_id = _require_string(
        aircraft_links.get("calibration"), "aircraft.layers.calibration"
    )
    field_id = _require_string(mission_links.get("field"), "mission.layers.field")
    perception_id = _require_string(
        mission_links.get("perception"), "mission.layers.perception"
    )

    layer_ids = {
        "aircraft": aircraft,
        "calibration": calibration_id,
        "mission": mission,
        "field": field_id,
        "perception": perception_id,
    }
    layer_paths = {
        "aircraft": aircraft_path,
        "calibration": _safe_layer_path(root, "calibration", calibration_id),
        "mission": mission_path,
        "field": _safe_layer_path(root, "field", field_id),
        "perception": _safe_layer_path(root, "perception", perception_id),
    }
    layers: Dict[str, Mapping[str, Any]] = {
        "aircraft": aircraft_layer,
        "mission": mission_layer,
    }
    for kind in ("calibration", "field", "perception"):
        layers[kind] = _load_yaml(layer_paths[kind])
    for kind in LAYER_ORDER:
        _layer_identity(layers[kind], kind, layer_ids[kind])
    if layers["calibration"].get("aircraft") != aircraft:
        raise ProfileError("calibration layer belongs to a different aircraft")

    canonical_document = {
        "schema": BUNDLE_SCHEMA,
        "selection": {"aircraft": aircraft, "mission": mission},
        "layers": [
            {"kind": kind, "identifier": layer_ids[kind], "content": layers[kind]}
            for kind in LAYER_ORDER
        ],
    }
    canonical_json = json.dumps(
        canonical_document,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    digest = hashlib.sha256(canonical_json).hexdigest()
    profile = LoadedProfile(
        config_root=root,
        aircraft_name=aircraft,
        mission_name=mission,
        layers=layers,
        layer_paths=layer_paths,
        bundle_sha256=digest,
        canonical_json=canonical_json,
    )
    _validate_profile(profile)
    return profile


def describe_layers(profile: LoadedProfile) -> Iterable[Tuple[str, Path]]:
    for kind in LAYER_ORDER:
        yield kind, profile.layer_paths[kind]
