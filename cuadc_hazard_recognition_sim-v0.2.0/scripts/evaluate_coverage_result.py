#!/usr/bin/env python3
"""Post-mission scorer. Ground truth is never provided to the flight state machine."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml


CLASS_MAP = {
    "BaoZhaPin": "explosive",
    "ShengWuWeiHai": "biohazard",
    "YiRan": "flammable",
    "FangSheXingWuPin": "radioactive",
    "BuRanQiTi": "nonflammable_gas",
    "FuShiPin": "corrosive",
    "YouDuPin": "toxic",
    "YuShiYiRanWuPin": "dangerous_when_wet",
    "ZiRanWuPin": "spontaneously_combustible",
    "CiJiXing": "irritant",
}


def canonical(name: str) -> str:
    return CLASS_MAP.get(name, name)


def distance_xy(a, b) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", required=True)
    parser.add_argument("--observations", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--match-radius", type=float, default=0.90)
    args = parser.parse_args()

    scene = yaml.safe_load(Path(args.scene).read_text(encoding="utf-8"))
    raw = json.loads(Path(args.observations).read_text(encoding="utf-8"))
    if raw.get("state_machine_has_ground_truth") is not False:
        raise RuntimeError("raw result must explicitly prove that flight logic had no ground truth")
    if not raw.get("coverage_completed", False):
        raise RuntimeError("specified recognition area was not fully covered")

    targets = []
    for bucket_id, target in scene["recon_targets"].items():
        targets.append(
            {
                "bucket": bucket_id,
                "truth": str(target.get("marker", "none")),
                "position": [float(target["rtk_enu"][0]), float(target["rtk_enu"][1])],
            }
        )

    observations = []
    for index, observation in enumerate(raw.get("observations", [])):
        if "position_rtk_enu" not in observation:
            continue
        observations.append(
            {
                "index": index,
                "predicted": str(observation["class"]),
                "confidence": float(observation["confidence"]),
                "position": [
                    float(observation["position_rtk_enu"][0]),
                    float(observation["position_rtk_enu"][1]),
                ],
            }
        )

    candidate_pairs = []
    for observation_index, observation in enumerate(observations):
        for target_index, target in enumerate(targets):
            separation = distance_xy(observation["position"], target["position"])
            if separation <= args.match_radius:
                candidate_pairs.append(
                    (
                        separation - 0.05 * observation["confidence"],
                        separation,
                        observation_index,
                        target_index,
                    )
                )
    candidate_pairs.sort()

    assigned_observations = set()
    assigned_targets = {}
    for _, separation, observation_index, target_index in candidate_pairs:
        if observation_index in assigned_observations or target_index in assigned_targets:
            continue
        assigned_observations.add(observation_index)
        assigned_targets[target_index] = (observation_index, separation)

    results = []
    correct = 0
    false_positive_count = 0
    for target_index, target in enumerate(targets):
        if target_index in assigned_targets:
            observation_index, separation = assigned_targets[target_index]
            observation = observations[observation_index]
            predicted = observation["predicted"]
            confidence = observation["confidence"]
        else:
            separation = None
            predicted = "none"
            confidence = 0.0
        match = canonical(predicted) == canonical(target["truth"])
        correct += int(match)
        if target["truth"] == "none" and predicted != "none":
            false_positive_count += 1
        results.append(
            {
                "bucket": target["bucket"],
                "predicted": predicted,
                "truth": target["truth"],
                "confidence": round(confidence, 4),
                "separation_m": None if separation is None else round(separation, 4),
                "correct": match,
            }
        )

    unmatched = [
        observations[index]
        for index in range(len(observations))
        if index not in assigned_observations
    ]
    false_positive_count += len(unmatched)
    output = {
        "seed": int(raw.get("seed", scene.get("seed", 0))),
        "mission": "coverage_search_unknown_bucket_positions",
        "state_machine_has_ground_truth": False,
        "localization": raw.get("localization", "unknown"),
        "coverage_waypoints": int(raw.get("coverage_waypoints", 0)),
        "coverage_completed": bool(raw.get("coverage_completed", False)),
        "raw_observations": len(observations),
        "unmatched_observations": len(unmatched),
        "false_positive_count": false_positive_count,
        "results": results,
        "correct": correct,
        "total": len(results),
        "pass": correct == len(results) and false_positive_count == 0,
    }
    Path(args.output).write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        f"coverage_eval seed={output['seed']} correct={correct}/{len(results)} "
        f"observations={len(observations)} false_positives={false_positive_count}"
    )


if __name__ == "__main__":
    main()
