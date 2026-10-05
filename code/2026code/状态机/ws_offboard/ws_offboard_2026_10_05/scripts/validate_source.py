#!/usr/bin/env python3
"""检查发布文件，无需 ROS、相机或飞控。"""
import ast
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate():
    packages = {}
    for path in sorted((ROOT / "src").glob("*/package.xml")):
        tree = ET.parse(path).getroot()
        name = tree.findtext("name")
        if name != path.parent.name or name in packages:
            raise ValueError(f"Invalid/duplicate package name: {path}")
        if tree.findtext("license") != "AGPL-3.0-only":
            raise ValueError(f"Unexpected license: {name}")
        packages[name] = tree
    expected = {"cuadc_mission", "cuadc_perception", "cuadc_tools", "cuadc_bringup"}
    if set(packages) != expected:
        raise ValueError(f"Package set mismatch: {set(packages)}")
    for path in ROOT.rglob("*.py"):
        if any(p in {"build", "install", "log", "__pycache__"} for p in path.relative_to(ROOT).parts):
            continue
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    source = json.loads((ROOT / "SOURCE_MANIFEST.json").read_text(encoding="utf-8"))
    for entry in source["migrated_files"].values():
        if digest(ROOT / entry["destination"]) != entry["sha256"]:
            raise ValueError(f"Source checksum mismatch: {entry['destination']}")
    models = json.loads((ROOT / "MODEL_SHA256.json").read_text(encoding="utf-8"))
    for relative, expected_hash in models.items():
        if digest(ROOT / relative) != expected_hash:
            raise ValueError(f"Model checksum mismatch: {relative}")
    checksum_file = ROOT / "SHA256SUMS"
    if checksum_file.exists():
        for line in checksum_file.read_text(encoding="utf-8").splitlines():
            expected_hash, relative = line.split("  ", 1)
            if digest(ROOT / relative) != expected_hash:
                raise ValueError(f"Release checksum mismatch: {relative}")
    print(f"PASS: {len(packages)} ROS packages, Python syntax, original-source hashes, {len(models)} models and release checksums")


if __name__ == "__main__":
    try:
        validate()
    except (ValueError, OSError, SyntaxError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
