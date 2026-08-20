#!/usr/bin/env python3
"""The only CUADC bringup: fail-closed preflight and six real-hardware processes."""

from __future__ import annotations

import secrets
import sys
from pathlib import Path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_BRINGUP_SHARE = Path(get_package_share_directory("cuadc_bringup")).resolve()
_SCRIPTS = _BRINGUP_SHARE / "scripts"
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

from profile_loader import ProfileError, load_profile  # noqa: E402
from preflight import (  # noqa: E402
    CONFIG_BUNDLE_ENV,
    PreflightError,
    RUNTIME_GATE_VALUE,
    resolve_model_path,
    run_preflight,
)


def _parse_bool(value: str, name: str) -> bool:
    normalised = value.strip().lower()
    if normalised in ("true", "1"):
        return True
    if normalised in ("false", "0"):
        return False
    raise RuntimeError(f"{name} must be exactly true or false")


def _real_actions(context: LaunchContext) -> List[object]:
    aircraft = LaunchConfiguration("aircraft").perform(context)
    mission = LaunchConfiguration("mission").perform(context)
    config_root = LaunchConfiguration("config_root").perform(context)
    preflight_only = _parse_bool(
        LaunchConfiguration("preflight_only").perform(context), "preflight_only"
    )

    try:
        profile = load_profile(config_root, aircraft, mission)
        model_path = resolve_model_path(profile)
        report = run_preflight(
            profile,
            model_path,
            require_flight_authorization=not preflight_only,
        )
    except (OSError, ProfileError, PreflightError) as exc:
        raise RuntimeError(f"CUADC preflight refused launch: {exc}") from exc

    actions: List[object] = [LogInfo(msg=line) for line in report.lines()]
    if preflight_only:
        actions.append(
            LogInfo(
                msg=(
                    "READ-ONLY PREFLIGHT PASSED: no FCU/camera/mission/payload/safety "
                    "process was started and no runtime gate was issued"
                )
            )
        )
        return actions

    # These values are created only after the synchronous hardware, graph,
    # CUDA, profile approval, authorization phrase, and approved-hash checks.
    # additional_env confines each gate to its intended child process.
    mission_environment = {
        "CUADC_REAL_RUNTIME_GATE": RUNTIME_GATE_VALUE,
        CONFIG_BUNDLE_ENV: profile.bundle_sha256,
    }
    payload_environment = {
        "CUADC_PAYLOAD_RUNTIME_GATE": RUNTIME_GATE_VALUE,
        CONFIG_BUNDLE_ENV: profile.bundle_sha256,
    }
    safety_environment = {
        "CUADC_REAL_RUNTIME_GATE": RUNTIME_GATE_VALUE,
        CONFIG_BUNDLE_ENV: profile.bundle_sha256,
    }
    # ROS parameters carry signed int64 values.  A cryptographically random,
    # positive 63-bit epoch is generated exactly once for this launch and is
    # never part of the approved static bundle hash.
    mission_epoch = secrets.randbits(63) or 1

    mavros_share = Path(get_package_share_directory("mavros"))
    mavros_plugin_denylist = [
        "actuator_control",
        "ftp",
        "hil",
        "altitude",
        "debug_value",
        "image_pub",
        "px4flow",
        "sim_state",
        "vibration",
        "vision_speed_estimate",
        "wheel_odometry",
    ]
    fcu = profile.fcu
    actions.extend(
        [
            LogInfo(
                msg=(
                    f"STARTING REAL FLIGHT aircraft={profile.aircraft_name} "
                    f"mission={profile.mission_name} bundle={profile.bundle_sha256} "
                    f"mission_epoch={mission_epoch}"
                )
            ),
            Node(
                package="mavros",
                executable="mavros_node",
                namespace="mavros",
                name="mavros",
                output="screen",
                emulate_tty=True,
                respawn=False,
                parameters=[
                    str(mavros_share / "launch" / "apm_pluginlists.yaml"),
                    str(mavros_share / "launch" / "apm_config.yaml"),
                    {
                        "fcu_url": str(fcu["fcu_url"]),
                        "gcs_url": str(fcu.get("gcs_url", "")),
                        "tgt_system": int(fcu.get("target_system", 1)),
                        "tgt_component": int(fcu.get("target_component", 1)),
                        "fcu_protocol": str(fcu.get("protocol", "v2.0")),
                        "plugin_denylist": mavros_plugin_denylist,
                    },
                ],
            ),
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace="camera",
                name="camera",
                output="screen",
                emulate_tty=True,
                respawn=False,
                parameters=[profile.realsense_parameters()],
            ),
            Node(
                package="cuadc_perception",
                executable="bucket_perception_node",
                name="bucket_perception_node",
                output="screen",
                emulate_tty=True,
                respawn=False,
                parameters=[profile.perception_parameters(model_path)],
            ),
            Node(
                package="cuadc_mission",
                executable="mission_node",
                name="mission_node",
                output="screen",
                emulate_tty=True,
                respawn=False,
                parameters=[profile.mission_parameters(mission_epoch)],
                additional_env=mission_environment,
            ),
            Node(
                package="cuadc_payload",
                executable="payload_node",
                name="payload_node",
                output="screen",
                emulate_tty=True,
                respawn=False,
                parameters=[profile.payload_parameters()],
                additional_env=payload_environment,
            ),
            Node(
                package="cuadc_safety",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
                emulate_tty=True,
                respawn=False,
                parameters=[profile.safety_parameters()],
                additional_env=safety_environment,
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "aircraft",
                default_value="v6x_3E0032",
                description="Exact aircraft profile: v6x_3E0032 or v5nano_410035",
            ),
            DeclareLaunchArgument(
                "mission",
                default_value="competition_2026",
                description="Mission layer identifier under config/mission",
            ),
            DeclareLaunchArgument(
                "preflight_only",
                default_value="true",
                description="true performs diagnostics only; false requests gated real flight",
            ),
            DeclareLaunchArgument(
                "config_root",
                default_value=str(_BRINGUP_SHARE / "config"),
                description="Absolute root containing the five approved configuration layers",
            ),
            OpaqueFunction(function=_real_actions),
        ]
    )
