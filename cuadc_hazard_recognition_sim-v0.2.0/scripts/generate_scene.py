#!/usr/bin/env python3
"""Generate randomized CUADC hazard scenes with RTK/ENU truth metadata."""

from __future__ import annotations

import argparse
from pathlib import Path
import math
import random
import textwrap
import yaml


def sample_points(rng, center, size, count, min_sep):
    cx, cy, _ = center
    sx, sy = size
    points = []
    for _ in range(10000):
        x = rng.uniform(cx - sx / 2.0 + 0.35, cx + sx / 2.0 - 0.35)
        y = rng.uniform(cy - sy / 2.0 + 0.35, cy + sy / 2.0 - 0.35)
        if all(math.hypot(x - px, y - py) >= min_sep for px, py in points):
            points.append((round(x, 3), round(y, 3)))
            if len(points) == count:
                return points
    raise RuntimeError("Unable to sample non-overlapping recognition buckets")


def include(name, uri, x, y, z, yaw=0.0):
    return textwrap.dedent(f"""\
        <include>
          <name>{name}</name>
          <uri>model://{uri}</uri>
          <pose>{x} {y} {z} 0 0 {yaw}</pose>
        </include>
        """)


def marker_model(assignment):
    marker = assignment["marker"]
    texture = f"model://hazard_marker/materials/textures/{marker}.png"
    size = assignment["size"]
    return textwrap.dedent(f"""\
        <model name="marker_{marker}_{assignment['bucket_id']}">
          <static>true</static>
          <pose>{assignment['x']} {assignment['y']} 0.180 0 0 {assignment['yaw']}</pose>
          <link name="link">
            <visual name="marker">
              <geometry><box><size>{size} {size} 0.004</size></box></geometry>
              <material>
                <diffuse>{assignment['brightness']} {assignment['brightness']} {assignment['brightness']} 1</diffuse>
                <pbr><metal><albedo_map>{texture}</albedo_map><roughness>0.65</roughness><metalness>0</metalness></metal></pbr>
              </material>
            </visual>
          </link>
        </model>
        """)


def build_world(camera_position, vehicle, buckets, assignments, light, rtk):
    parts = [include("recognition_field", "recognition_field", 0, 0, 0)]
    if camera_position is not None:
        parts.append(include("static_d435i_camera", "static_d435i_camera", *camera_position))
    if vehicle.get("enabled", False):
        pose = vehicle["pose"]
        vehicle_model = vehicle.get("model", "iris_d435i")
        parts.append(include(vehicle_model, vehicle_model, *pose[:3], pose[5]))
    for bucket in buckets:
        parts.append(include(bucket["id"], "recon_bucket", bucket["x"], bucket["y"], 0.0))
    parts.extend(marker_model(assignment) for assignment in assignments)
    body = "\n".join(textwrap.indent(part.strip(), "    ") for part in parts)
    return textwrap.dedent(f"""\
        <?xml version="1.0"?>
        <sdf version="1.9">
          <world name="hazard_recognition_single">
            <physics name="1ms" type="ignore"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
            <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
            <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
            <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
            <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
            <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
            <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
            <spherical_coordinates>
              <latitude_deg>{rtk['latitude_deg']}</latitude_deg>
              <longitude_deg>{rtk['longitude_deg']}</longitude_deg>
              <elevation>{rtk['altitude_m']}</elevation>
              <heading_deg>{rtk.get('world_to_rtk_yaw_deg', 0.0)}</heading_deg>
              <surface_model>EARTH_WGS84</surface_model>
            </spherical_coordinates>
            <scene><ambient>{light['ambient']} {light['ambient']} {light['ambient']}</ambient><background>0.78 0.80 0.82</background><sky/></scene>
            <light type="directional" name="sun"><pose>0 0 20 0 0 0</pose><diffuse>{light['sun']} {light['sun']} {light['sun']} 1</diffuse><specular>0.6 0.6 0.6 1</specular><direction>{light['dx']} {light['dy']} -0.9</direction></light>
        {body}
          </world>
        </sdf>
        """)


def world_to_rtk_enu(x, y, rtk):
    base_x, base_y = rtk["world_position_enu_m"][:2]
    yaw = math.radians(float(rtk.get("world_to_rtk_yaw_deg", 0.0)))
    dx, dy = x - base_x, y - base_y
    return [
        round(math.cos(yaw) * dx - math.sin(yaw) * dy, 4),
        round(math.sin(yaw) * dx + math.cos(yaw) * dy, 4),
        0.0,
    ]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()
    package_dir = Path(__file__).resolve().parents[1]
    scene_path = package_dir / "config" / "scene.yaml"
    truth_path = package_dir / "config" / "generated_scene.yaml"
    world_path = package_dir / "worlds" / "hazard_recognition_single.sdf"
    scene = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    seed = int(scene.get("seed", 2026) if args.seed is None else args.seed)
    rng = random.Random(seed)

    area = scene["recognition_area"]
    bucket_cfg = scene["recon_buckets"]
    marker_cfg = scene["hazard_markers"]
    vehicle = scene.get("vehicle", {"enabled": False})
    rtk = scene["rtk_base"]
    points = sample_points(rng, area["center"], area["size"], bucket_cfg["count"], bucket_cfg["min_separation"])
    buckets = [{"id": f"recon_{i + 1}", "x": x, "y": y} for i, (x, y) in enumerate(points)]

    selected_buckets = rng.sample(buckets, marker_cfg["count"])
    selected_markers = rng.sample(marker_cfg["classes"], marker_cfg["count"])
    assignments = []
    max_offset = float(marker_cfg.get("max_center_offset_m", 0.025))
    size_min, size_max = marker_cfg.get("random_size_range_m", marker_cfg.get("size", [0.12, 0.12]))
    for bucket, marker in zip(selected_buckets, selected_markers):
        radius = rng.uniform(0.0, max_offset)
        angle = rng.uniform(-math.pi, math.pi)
        assignments.append({
            "bucket_id": bucket["id"], "marker": marker,
            "x": round(bucket["x"] + radius * math.cos(angle), 4),
            "y": round(bucket["y"] + radius * math.sin(angle), 4),
            "yaw": round(rng.uniform(-math.radians(8.0), math.radians(8.0)), 6),
            "size": round(rng.uniform(float(size_min), float(size_max)), 4),
            "brightness": round(rng.uniform(0.82, 1.0), 3),
        })

    targets = {}
    for bucket in buckets:
        assignment = next((a for a in assignments if a["bucket_id"] == bucket["id"]), None)
        targets[bucket["id"]] = {
            "x": bucket["x"], "y": bucket["y"], "z": 0.0,
            "rtk_enu": world_to_rtk_enu(bucket["x"], bucket["y"], rtk),
            "marker": assignment["marker"] if assignment else "none",
            "marker_x": assignment["x"] if assignment else None,
            "marker_y": assignment["y"] if assignment else None,
            "yaw": assignment["yaw"] if assignment else 0.0,
            "marker_size": assignment["size"] if assignment else 0.0,
        }

    light = {"ambient": round(rng.uniform(0.72, 1.0), 3), "sun": round(rng.uniform(0.75, 1.0), 3), "dx": round(rng.uniform(-0.6, -0.2), 3), "dy": round(rng.uniform(-0.3, 0.3), 3)}
    truth = {
        "seed": seed, "source": "hazard_recognition_only_randomized",
        "coordinate_frames": {"world": "gazebo_world_enu", "rtk": "rtk_base_enu", "mavros": "map_enu"},
        # Public mission geometry. The flight state machine reads this boundary
        # but deliberately ignores the randomized recon_targets truth below.
        "recognition_area": {
            "center_world": [
                float(area["center"][0]), float(area["center"][1]), 0.0
            ],
            "size": [float(area["size"][0]), float(area["size"][1])],
            "center_rtk_enu": world_to_rtk_enu(
                float(area["center"][0]), float(area["center"][1]), rtk
            ),
            "world_to_rtk_yaw_deg": float(rtk.get("world_to_rtk_yaw_deg", 0.0)),
        },
        "rtk_base": rtk,
        "camera": {"enabled": bool(scene["camera"].get("enabled", False)), "x": scene["camera"]["position"][0], "y": scene["camera"]["position"][1], "z": scene["camera"]["position"][2], "topic_prefix": "/hazard_d435i"},
        "vehicle": {"enabled": bool(vehicle.get("enabled", False)), "model": vehicle.get("model", ""), "pose": vehicle.get("pose", []), "rtk_enu": world_to_rtk_enu(vehicle["pose"][0], vehicle["pose"][1], rtk), "camera_topic_prefix": "/d435i"},
        "visual_randomization": light,
        "recon_targets": targets,
    }
    truth_path.write_text(yaml.safe_dump(truth, sort_keys=False), encoding="utf-8")
    static_camera_position = scene["camera"]["position"] if scene["camera"].get("enabled", False) else None
    world_path.write_text(build_world(static_camera_position, vehicle, buckets, assignments, light, rtk), encoding="utf-8")
    print(f"seed={seed}; markers=" + ",".join(f"{a['bucket_id']}:{a['marker']}" for a in assignments))
    print(f"Wrote {truth_path}")
    print(f"Wrote {world_path}")


if __name__ == "__main__":
    main()

