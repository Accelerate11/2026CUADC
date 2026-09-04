#!/usr/bin/env python3
"""Generate a randomized CUADC rescue scene and matching Gazebo world."""

from __future__ import annotations

from pathlib import Path
import math
import random
import textwrap
import yaml


MARKERS = [
    "explosive",
    "nonflammable_gas",
    "irritant",
    "radioactive",
    "corrosive",
    "biohazard",
    "dangerous_when_wet",
    "toxic",
    "spontaneously_combustible",
    "flammable",
]


def sample_points(rng: random.Random, center, size, count, min_sep):
    cx, cy, _ = center
    sx, sy = size
    points = []
    for _ in range(10000):
        x = rng.uniform(cx - sx / 2.0 + 0.45, cx + sx / 2.0 - 0.45)
        y = rng.uniform(cy - sy / 2.0 + 0.45, cy + sy / 2.0 - 0.45)
        if all(math.hypot(x - px, y - py) >= min_sep for px, py in points):
            points.append((round(x, 2), round(y, 2)))
            if len(points) == count:
                return points
    raise RuntimeError("Unable to sample non-overlapping scene points")


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
              <pose>0 0 0 0 0 0</pose>
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


def build_world(drop_buckets, recon_buckets, marker_assignments) -> str:
    parts = []
    parts.append(include("cuadc_field", "cuadc_field", 0, 0, 0))
    parts.append(include("takeoff_pad", "takeoff_pad", 0, 0, 0.01))

    for bucket in drop_buckets:
        parts.append(include(bucket["id"], bucket["model"], bucket["x"], bucket["y"], 0))

    for bucket in recon_buckets:
        parts.append(include(bucket["id"], "recon_bucket", bucket["x"], bucket["y"], 0))

    for assignment in marker_assignments:
        parts.append(
            marker_model(
                f"hazard_{assignment['marker']}_{assignment['bucket_id']}",
                assignment["marker"],
                assignment["x"],
                assignment["y"],
                assignment["yaw"],
            )
        )

    parts.append('<include><uri>model://iris_d435i</uri><pose degrees="true">0 0 0.195 0 0 90</pose></include>')
    body = "\n".join(textwrap.indent(p.strip(), "    ") for p in parts)

    return textwrap.dedent(
        f"""\
        <?xml version="1.0"?>
        <sdf version="1.9">
          <world name="cuadc_rescue_single">
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
            <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
            <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>

            <scene>
              <ambient>1.0 1.0 1.0</ambient>
              <background>0.78 0.80 0.82</background>
              <sky/>
            </scene>

            <spherical_coordinates>
              <latitude_deg>-35.363262</latitude_deg>
              <longitude_deg>149.165237</longitude_deg>
              <elevation>584</elevation>
              <heading_deg>0</heading_deg>
              <surface_model>EARTH_WGS84</surface_model>
            </spherical_coordinates>

            <light type="directional" name="sun">
              <cast_shadows>true</cast_shadows>
              <pose>0 0 20 0 0 0</pose>
              <diffuse>0.9 0.9 0.85 1</diffuse>
              <specular>0.6 0.6 0.6 1</specular>
              <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
              </attenuation>
              <direction>-0.45 0.15 -0.88</direction>
            </light>

        {body}
          </world>
        </sdf>
        """
    )


def main() -> None:
    package_dir = Path(__file__).resolve().parents[1]
    scene_path = package_dir / "config" / "scene.yaml"
    generated_path = package_dir / "config" / "generated_scene.yaml"
    world_path = package_dir / "worlds" / "cuadc_rescue_single.sdf"

    with scene_path.open("r", encoding="utf-8") as f:
        scene = yaml.safe_load(f)

    rng = random.Random(scene.get("seed", 2026))
    field = scene["field"]

    drop_points = sample_points(
        rng,
        field["drop_area_center"],
        field["drop_area_size"],
        3,
        min_sep=0.75,
    )
    recon_points = sample_points(
        rng,
        field["recon_area_center"],
        field["recon_area_size"],
        5,
        min_sep=0.70,
    )

    drop_specs = [
        ("drop_1", "drop_bucket_15", 0.075, 500),
        ("drop_2", "drop_bucket_20", 0.10, 300),
        ("drop_3", "drop_bucket_25", 0.125, 100),
    ]
    drop_buckets = [
        {"id": bid, "model": model, "x": x, "y": y, "radius": radius, "score": score}
        for (bid, model, radius, score), (x, y) in zip(drop_specs, drop_points)
    ]
    recon_buckets = [
        {"id": f"recon_{i + 1}", "x": x, "y": y}
        for i, (x, y) in enumerate(recon_points)
    ]

    selected_recon = rng.sample(recon_buckets, 3)
    selected_markers = rng.sample(MARKERS, 3)
    marker_assignments = []
    for bucket, marker in zip(selected_recon, selected_markers):
        marker_assignments.append(
            {
                "bucket_id": bucket["id"],
                "marker": marker,
                "x": bucket["x"],
                "y": bucket["y"],
                "yaw": rng.choice([0.0, 1.57079632679, 3.14159265359, -1.57079632679]),
            }
        )

    generated = {
        "seed": scene.get("seed", 2026),
        "source": "randomized_scene_from_scene_yaml",
        "drop_targets": {
            b["id"]: {"x": b["x"], "y": b["y"], "z": 0.0, "radius": b["radius"], "score": b["score"]}
            for b in drop_buckets
        },
        "recon_targets": {
            b["id"]: {
                "x": b["x"],
                "y": b["y"],
                "z": 0.0,
                "marker": next((m["marker"] for m in marker_assignments if m["bucket_id"] == b["id"]), "none"),
            }
            for b in recon_buckets
        },
    }

    with generated_path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(generated, f, sort_keys=False)

    world_path.write_text(build_world(drop_buckets, recon_buckets, marker_assignments), encoding="utf-8")
    print(f"Wrote {generated_path}")
    print(f"Wrote {world_path}")


if __name__ == "__main__":
    main()
