#!/usr/bin/env python3
"""Generate a standalone hazard-recognition Gazebo world and truth file."""

from __future__ import annotations

from pathlib import Path
import math
import random
import textwrap
import yaml


def sample_points(rng: random.Random, center, size, count, min_sep):
    cx, cy, _ = center
    sx, sy = size
    points = []
    for _ in range(10000):
        x = rng.uniform(cx - sx / 2.0 + 0.35, cx + sx / 2.0 - 0.35)
        y = rng.uniform(cy - sy / 2.0 + 0.35, cy + sy / 2.0 - 0.35)
        if all(math.hypot(x - px, y - py) >= min_sep for px, py in points):
            points.append((round(x, 2), round(y, 2)))
            if len(points) == count:
                return points
    raise RuntimeError("Unable to sample non-overlapping recognition buckets")


def include(name: str, uri: str, x: float, y: float, z: float, yaw: float = 0.0) -> str:
    return textwrap.dedent(
        f"""\
        <include>
          <name>{name}</name>
          <uri>model://{uri}</uri>
          <pose>{x} {y} {z} 0 0 {yaw}</pose>
        </include>
        """
    )


def marker_model(name: str, marker: str, x: float, y: float, yaw: float) -> str:
    texture = f"model://hazard_marker/materials/textures/{marker}.png"
    return textwrap.dedent(
        f"""\
        <model name="{name}">
          <static>true</static>
          <pose>{x} {y} 0.165 0 0 {yaw}</pose>
          <link name="link">
            <visual name="marker">
              <geometry>
                <box>
                  <size>0.12 0.12 0.004</size>
                </box>
              </geometry>
              <material>
                <diffuse>1 1 1 1</diffuse>
                <pbr>
                  <metal>
                    <albedo_map>{texture}</albedo_map>
                    <roughness>0.65</roughness>
                    <metalness>0</metalness>
                  </metal>
                </pbr>
              </material>
            </visual>
          </link>
        </model>
        """
    )


def build_world(camera_position, buckets, assignments) -> str:
    parts = [
        include("recognition_field", "recognition_field", 0, 0, 0),
        include("static_d435i_camera", "static_d435i_camera", *camera_position),
    ]

    for bucket in buckets:
        parts.append(include(bucket["id"], "recon_bucket", bucket["x"], bucket["y"], 0.0))

    for assignment in assignments:
        parts.append(
            marker_model(
                f"marker_{assignment['marker']}_{assignment['bucket_id']}",
                assignment["marker"],
                assignment["x"],
                assignment["y"],
                assignment["yaw"],
            )
        )

    body = "\n".join(textwrap.indent(p.strip(), "    ") for p in parts)
    return textwrap.dedent(
        f"""\
        <?xml version="1.0"?>
        <sdf version="1.9">
          <world name="hazard_recognition_single">
            <physics name="1ms" type="ignore">
              <max_step_size>0.001</max_step_size>
              <real_time_factor>1.0</real_time_factor>
            </physics>

            <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
            <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
              <render_engine>ogre2</render_engine>
            </plugin>
            <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
            <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>

            <scene>
              <ambient>1.0 1.0 1.0</ambient>
              <background>0.78 0.80 0.82</background>
              <sky/>
            </scene>

            <light type="directional" name="sun">
              <pose>0 0 20 0 0 0</pose>
              <diffuse>0.9 0.9 0.85 1</diffuse>
              <specular>0.6 0.6 0.6 1</specular>
              <direction>-0.4 0.2 -0.9</direction>
            </light>

        {body}
          </world>
        </sdf>
        """
    )


def main() -> None:
    package_dir = Path(__file__).resolve().parents[1]
    scene_path = package_dir / "config" / "scene.yaml"
    truth_path = package_dir / "config" / "generated_scene.yaml"
    world_path = package_dir / "worlds" / "hazard_recognition_single.sdf"

    scene = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    rng = random.Random(scene.get("seed", 2026))

    area = scene["recognition_area"]
    bucket_cfg = scene["recon_buckets"]
    marker_cfg = scene["hazard_markers"]
    camera_position = scene["camera"]["position"]

    points = sample_points(
        rng,
        area["center"],
        area["size"],
        bucket_cfg["count"],
        bucket_cfg["min_separation"],
    )
    buckets = [
        {"id": f"recon_{i + 1}", "x": x, "y": y}
        for i, (x, y) in enumerate(points)
    ]

    selected_buckets = rng.sample(buckets, marker_cfg["count"])
    selected_markers = rng.sample(marker_cfg["classes"], marker_cfg["count"])
    assignments = []
    for bucket, marker in zip(selected_buckets, selected_markers):
        assignments.append(
            {
                "bucket_id": bucket["id"],
                "marker": marker,
                "x": bucket["x"],
                "y": bucket["y"],
                "yaw": rng.choice([0.0, 1.57079632679, 3.14159265359, -1.57079632679]),
            }
        )

    truth = {
        "seed": scene.get("seed", 2026),
        "source": "hazard_recognition_only",
        "camera": {
            "x": camera_position[0],
            "y": camera_position[1],
            "z": camera_position[2],
            "topic_prefix": "/hazard_d435i",
        },
        "recon_targets": {
            bucket["id"]: {
                "x": bucket["x"],
                "y": bucket["y"],
                "z": 0.0,
                "marker": next((a["marker"] for a in assignments if a["bucket_id"] == bucket["id"]), "none"),
                "yaw": next((a["yaw"] for a in assignments if a["bucket_id"] == bucket["id"]), 0.0),
            }
            for bucket in buckets
        },
    }

    truth_path.write_text(yaml.safe_dump(truth, sort_keys=False), encoding="utf-8")
    world_path.write_text(build_world(camera_position, buckets, assignments), encoding="utf-8")
    print(f"Wrote {truth_path}")
    print(f"Wrote {world_path}")


if __name__ == "__main__":
    main()
