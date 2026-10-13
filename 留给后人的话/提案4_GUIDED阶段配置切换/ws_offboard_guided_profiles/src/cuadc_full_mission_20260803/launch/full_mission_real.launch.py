"""Real-aircraft launch for the CUADC full mission."""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("cuadc_full_mission_20260803")

    params_file = LaunchConfiguration("params_file")
    model_path = LaunchConfiguration("model_path")
    camera_serial = LaunchConfiguration("camera_serial")
    telemetry_csv_path = LaunchConfiguration("telemetry_csv_path")


    # CUADC_RUNTIME_EXPOSURE_OVERRIDE
    vision_overrides = {
        "model_path": model_path,
        "camera_serial": ParameterValue(camera_serial, value_type=str),
    }
    runtime_vision_device = os.environ.get("CUADC_VISION_DEVICE", "").strip().lower()
    if runtime_vision_device:
        if runtime_vision_device not in {"cpu", "intel:gpu"}:
            raise RuntimeError("CUADC_VISION_DEVICE must be cpu or intel:gpu")
        vision_overrides["device"] = runtime_vision_device

    runtime_exposure_text = os.environ.get(
        "CUADC_RUNTIME_EXPOSURE", ""
    ).strip()

    if runtime_exposure_text:
        runtime_exposure = float(runtime_exposure_text)

        if (
            not math.isfinite(runtime_exposure)
            or runtime_exposure <= 0.0
        ):
            raise RuntimeError(
                "CUADC_RUNTIME_EXPOSURE must be finite "
                "and greater than 0"
            )

        vision_overrides["exposure"] = runtime_exposure

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(package_share, "launch", "mission_mavros.launch.py")
        ),
        launch_arguments={
            "fcu_url": LaunchConfiguration("fcu_url"),
            "gcs_url": LaunchConfiguration("gcs_url"),
            "namespace": "mavros",
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_mavros")),
    )
    stream_configurator = Node(
        package="cuadc_full_mission_20260803",
        executable="mavlink_stream_configurator.py",
        name="mavlink_stream_configurator",
        output="screen",
    )
    vision_node = Node(
        package="cuadc_full_mission_20260803",
        executable="basket_vision_ros_node.py",
        name="basket_vision_ros_node",
        parameters=[params_file, vision_overrides],
        condition=IfCondition(LaunchConfiguration("start_vision")),
        output="screen",
        # Hide only the vision node's INFO chatter from the terminal. Its
        # topics, inference, recording, diagnostics and optional visual window
        # remain unchanged; WARN/ERROR messages are still visible.
        additional_env={"RCUTILS_LOGGING_SEVERITY": "WARN"},
    )
    mission_node = Node(
        package="cuadc_full_mission_20260803",
        executable="cuadc_full_mission_node",
        name="cuadc_full_mission_node",
        parameters=[params_file],
        output="screen",
    )
    telemetry_recorder = Node(
        package="cuadc_full_mission_20260803",
        executable="flight_telemetry_recorder.py",
        name="flight_telemetry_recorder",
        arguments=["--output", telemetry_csv_path, "--rate-hz", "2"],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                description="Explicit MAVROS serial URL for the flight controller.",
            ),
            DeclareLaunchArgument("gcs_url", default_value=""),
            DeclareLaunchArgument("start_mavros", default_value="true"),
            DeclareLaunchArgument(
                "camera_serial",
                default_value="",
                description="Exact Intel RealSense D435i serial number.",
            ),
            DeclareLaunchArgument("start_vision", default_value="true"),
            DeclareLaunchArgument(
                "telemetry_csv_path",
                default_value="/tmp/cuadc_flight_telemetry.csv",
                description="Low-rate structured MAVROS telemetry CSV.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    package_share, "config", "mission_real.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "model_path",
                default_value=os.path.join(
                    package_share, "models", "basket_detect.onnx"
                ),
            ),
            LogInfo(
                msg=(
                    "Mission launch shuts down when the mission node exits. "
                    "The wrapper owns MAVROS startup and cleanup when "
                    "start_mavros=false."
                )
            ),
            mavros_launch,
            stream_configurator,
            telemetry_recorder,
            vision_node,
            mission_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=mission_node,
                    on_exit=[
                        LogInfo(msg="Mission node exited; shutting down launch."),
                        EmitEvent(
                            event=Shutdown(reason="mission node exited")
                        ),
                    ],
                )
            ),
        ]
    )
