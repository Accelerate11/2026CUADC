"""从重新组织的 ROS 2 包启动原 CUADC 全任务。"""
import math
import os
from datetime import datetime
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from cuadc_bringup.route_config import read_route_heading
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
    OpaqueFunction, RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    route_file = Path(value("route_file")).expanduser().resolve(strict=True)
    heading = read_route_heading(route_file)
    params_file = str(Path(value("params_file")).expanduser().resolve(strict=True))
    model_path = str(Path(value("model_path")).expanduser().resolve(strict=True))
    if not value("fcu_url").strip():
        raise ValueError("fcu_url must be provided explicitly")

    vision_overrides = {"model_path": model_path}
    camera_serial = value("camera_serial").strip()
    if camera_serial:
        vision_overrides["camera_serial"] = camera_serial
    exposure_text = os.environ.get("CUADC_RUNTIME_EXPOSURE", "").strip()
    if exposure_text:
        exposure = float(exposure_text)
        if not math.isfinite(exposure) or exposure <= 0.0:
            raise ValueError("CUADC_RUNTIME_EXPOSURE must be finite and positive")
        vision_overrides["exposure"] = exposure

    log_dir = Path(value("log_dir")).expanduser().resolve()
    log_dir.mkdir(parents=True, exist_ok=True)
    vision_overrides.update({
        "diagnostic_video_path": str(log_dir / "basket_video.mp4"),
        "diagnostic_detection_csv_path": str(log_dir / "basket_detections.csv"),
        "release_snapshot_dir": str(log_dir / "release_snapshots"),
    })
    mission_overrides = {
        "geographic_heading_deg": heading,
        "bucket_projection_log_path": str(log_dir / "bucket_projection.csv"),
    }
    # 保存实际启动输入，不修改用户的航向和参数文件。
    (log_dir / "route.yaml").write_bytes(route_file.read_bytes())
    (log_dir / "mission_config.yaml").write_bytes(Path(params_file).read_bytes())
    (log_dir / "launch_inputs.yaml").write_text(
        yaml.safe_dump({
            "geographic_heading_deg": heading, "model_path": model_path,
            "fcu_url": value("fcu_url"), "camera_serial_override": camera_serial,
            "mission_overrides": mission_overrides, "vision_overrides": vision_overrides,
        }, sort_keys=False), encoding="utf-8",
    )

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(
            get_package_share_directory("mavros"), "launch", "apm.launch")),
        launch_arguments={"fcu_url": value("fcu_url"), "gcs_url": value("gcs_url"),
                          "namespace": "mavros"}.items(),
        condition=IfCondition(LaunchConfiguration("start_mavros")),
    )
    stream_configurator = Node(
        package="cuadc_tools", executable="mavlink_stream_configurator",
        name="mavlink_stream_configurator", output="screen",
    )
    vision_node = Node(
        package="cuadc_perception", executable="basket_vision_ros_node",
        name="basket_vision_ros_node", parameters=[params_file, vision_overrides],
        condition=IfCondition(LaunchConfiguration("start_vision")), output="screen",
    )
    mission_node = Node(
        package="cuadc_mission", executable="cuadc_full_mission_node",
        name="cuadc_full_mission_node", parameters=[params_file, mission_overrides],
        output="screen",
    )
    telemetry = Node(
        package="cuadc_tools", executable="flight_telemetry_recorder",
        name="flight_telemetry_recorder", output="screen",
        arguments=["--output", str(log_dir / "telemetry.csv"), "--rate-hz", "2"],
    )
    return [
        LogInfo(msg=f"Route: {route_file}; heading: {heading:.8f}; logs: {log_dir}"),
        mavros_launch, stream_configurator, telemetry, vision_node, mission_node,
        RegisterEventHandler(OnProcessExit(
            target_action=mission_node,
            on_exit=[EmitEvent(event=Shutdown(reason="mission node exited"))],
        )),
    ]


def generate_launch_description():
    bringup = get_package_share_directory("cuadc_bringup")
    perception = get_package_share_directory("cuadc_perception")
    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", description="Explicit MAVROS serial/UDP URL."),
        DeclareLaunchArgument("route_file", description="Current calibration YAML with heading_deg."),
        DeclareLaunchArgument("gcs_url", default_value=""),
        DeclareLaunchArgument("start_mavros", default_value="true"),
        DeclareLaunchArgument("start_vision", default_value="true"),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("params_file", default_value=os.path.join(bringup, "config", "mission_real.yaml")),
        DeclareLaunchArgument("model_path", default_value=os.path.join(perception, "models", "basket_detect.onnx")),
        DeclareLaunchArgument("log_dir", default_value=os.path.join(
            os.getcwd(), "log", datetime.now().strftime("flight_%Y%m%d_%H%M%S_%f"))),
        OpaqueFunction(function=launch_nodes),
    ])
