#!/usr/bin/env python3
"""校验提案4完整性、基线保留文件及源码结构，仅依赖Python标准库。"""
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
    package = ROOT / "src/cuadc_full_mission_20260803"
    tree = ET.parse(package / "package.xml").getroot()
    if tree.findtext("name") != "cuadc_full_mission_20260803":
        raise ValueError("ROS包名与原V16不一致")
    if [x.text for x in tree.findall("license")] != ["AGPL-3.0-only"]:
        raise ValueError("包许可声明不一致")
    manifest = json.loads((ROOT / "SOURCE_MANIFEST.json").read_text(encoding="utf-8"))
    preserved = 0
    for item in manifest["files"]:
        path = ROOT / item["destination"]
        if digest(path) != item["packaged_sha256"]:
            raise ValueError("发布源文件变更：" + item["destination"])
        if item["change"] == "unchanged":
            if digest(path) != item["source_sha256"]:
                raise ValueError("V16原样文件不一致：" + item["destination"])
            preserved += 1
    models = json.loads((ROOT / "MODEL_SHA256.json").read_text(encoding="utf-8"))
    for relative, expected in models.items():
        if digest(ROOT / relative) != expected:
            raise ValueError("模型校验失败：" + relative)
    checked = 0
    for line in (ROOT / "SHA256SUMS").read_text(encoding="utf-8").splitlines():
        expected, relative = line.split("  ", 1)
        path = (ROOT / relative).resolve()
        if not path.is_relative_to(ROOT):
            raise ValueError("校验路径越界")
        if digest(path) != expected:
            raise ValueError("发布校验失败：" + relative)
        checked += 1
    for directory in ["src", "scripts", "tools", "tests", "environment", "routes", "servo_test"]:
        for path in (ROOT / directory).rglob("*.py"):
            if "__pycache__" not in path.parts:
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for required in ["src/guided_profile_client.hpp", "src/stage_motion_profile.hpp"]:
        if not (package / required).is_file():
            raise ValueError("缺少阶段切换实现：" + required)
    print(f"PASS：{checked}个提案文件、{preserved}个基线原样文件、{len(models)}个模型资源；源码语法正常")

if __name__ == "__main__":
    try:
        validate()
    except (OSError, ValueError, SyntaxError, KeyError) as error:
        print("FAIL：" + str(error), file=sys.stderr)
        raise SystemExit(1)
