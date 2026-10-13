#!/usr/bin/env python3
"""V16 dependency/model checks; never starts ROS, a camera, or a mission.

files: standard-library-only model inventory and metadata inspection.
onnx: real ONNX Runtime CPU inference on a synthetic tensor.
python: vision dependency imports and recorded version checks.
cpu/intel-gpu: original adapter + real YOLO inference on a synthetic image.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import re
import sys
import tempfile
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET


PACKAGE_NAME = "cuadc_full_mission_20260803"
MODEL_FILES = (
    "basket_detect.onnx", "basket_v3.onnx", "basket_v3.pt",
    "basket_detect_openvino_model/basket_detect_openvino_model.xml",
    "basket_detect_openvino_model/basket_detect_openvino_model.bin",
    "basket_detect_openvino_model/metadata.yaml",
)
RECORDED_VERSIONS = {
    "ultralytics": "8.4.41", "onnx": "1.17.0", "onnxruntime": "1.23.2",
}


class Report:
    def __init__(self, mode: str):
        self.data = {
            "mode": mode, "python": sys.version, "executable": sys.executable,
            "platform": platform.platform(), "checks": [],
            "scope": "No ROS node, camera, serial port, or flight-control action is started.",
        }

    def add(self, name: str, status: str, detail):
        self.data["checks"].append({"name": name, "status": status, "detail": detail})
        print("[{}] {}: {}".format(status, name, detail))

    @property
    def failed(self) -> bool:
        return any(item["status"] == "FAIL" for item in self.data["checks"])


def check_files(package: Path, report: Report) -> None:
    report.data["package_root"] = str(package)
    for relative in MODEL_FILES:
        path = package / "models" / relative
        if not path.is_file() or not path.stat().st_size:
            report.add(relative, "FAIL", "missing or empty model asset")
            continue
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        report.add(relative, "PASS", {
            "bytes": path.stat().st_size, "sha256": digest.hexdigest(),
        })
    xml_path = package / "models" / MODEL_FILES[3]
    if xml_path.is_file():
        try:
            root = ET.parse(xml_path).getroot()
            node = root.find("./rt_info/Runtime_version")
            report.add("OpenVINO IR conversion runtime", "PASS" if node is not None else "WARN",
                       node.attrib.get("value") if node is not None else "not recorded")
        except (ET.ParseError, OSError) as error:
            report.add("OpenVINO XML", "FAIL", str(error))


def load_module(name: str, report: Report, distribution: str | None = None):
    try:
        module = importlib.import_module(name)
        try:
            version = importlib.metadata.version(distribution or name)
        except importlib.metadata.PackageNotFoundError:
            version = str(getattr(module, "__version__", "not recorded"))
        report.add("import " + name, "PASS", {
            "version": version, "path": str(getattr(module, "__file__", "built-in")),
        })
        expected = RECORDED_VERSIONS.get(distribution or name)
        if expected and version != expected:
            report.add("recorded version " + name, "FAIL",
                       "expected {}; found {}".format(expected, version))
        return module
    except Exception as error:
        report.add("import " + name, "FAIL", "{}: {}".format(type(error).__name__, error))
        return None


def check_python(report: Report) -> None:
    if sys.version_info[:2] != (3, 10):
        report.add("ROS Humble Python", "WARN", "target is Ubuntu 22.04 / Python 3.10")
    modules = (
        ("numpy", "numpy"), ("cv2", "opencv-python"), ("yaml", "PyYAML"),
        ("PIL", "Pillow"), ("torch", "torch"), ("torchvision", "torchvision"),
        ("ultralytics", "ultralytics"), ("onnx", "onnx"),
        ("onnxruntime", "onnxruntime"), ("pyrealsense2", "pyrealsense2"),
    )
    for name, distribution in modules:
        module = load_module(name, report, distribution)
        if name == "numpy" and module is not None:
            numbers = tuple(int(v) for v in module.__version__.split(".")[:2])
            if not (1, 24) <= numbers < (2, 0):
                report.add("NumPy source requirement", "FAIL", "requires >=1.24,<2.0")
        if name == "torch" and module is not None and not module.__version__.startswith("2.11."):
            report.add("PyTorch source note", "WARN", "original note records torch 2.11 CPU")
        if name == "pyrealsense2" and module is not None:
            required = (
                hasattr(module.option, "global_time_enabled"),
                hasattr(module.frame_metadata_value, "sensor_timestamp"),
                hasattr(module.frame_metadata_value, "frame_timestamp"),
            )
            report.add("RealSense timestamp API", "PASS" if all(required) else "FAIL",
                       "checks API only; exposure metadata and USB3 require the physical D435i")


def check_onnx(package: Path, report: Report) -> None:
    np = load_module("numpy", report)
    ort = load_module("onnxruntime", report)
    if np is None or ort is None:
        return
    try:
        start = time.perf_counter()
        session = ort.InferenceSession(
            str(package / "models" / "basket_detect.onnx"),
            providers=["CPUExecutionProvider"],
        )
        feeds = {}
        for item in session.get_inputs():
            if item.type != "tensor(float)" or any(not isinstance(d, int) or d < 1 for d in item.shape):
                raise RuntimeError("unexpected model input: {} {} {}".format(item.name, item.type, item.shape))
            feeds[item.name] = np.zeros(item.shape, dtype=np.float32)
        outputs = session.run(None, feeds)
        if not outputs or any(not np.all(np.isfinite(value)) for value in outputs):
            raise RuntimeError("empty or non-finite output")
        report.add("ONNX CPU synthetic inference", "PASS", {
            "inputs": {name: list(value.shape) for name, value in feeds.items()},
            "outputs": [list(value.shape) for value in outputs],
            "providers": session.get_providers(),
            "elapsed_ms": round((time.perf_counter() - start) * 1000, 2),
            "model_metadata": session.get_modelmeta().custom_metadata_map,
        })
    except Exception as error:
        report.add("ONNX CPU synthetic inference", "FAIL", "{}: {}".format(type(error).__name__, error))


def check_adapter(package: Path, backend: str, report: Report) -> None:
    try:
        import numpy as np
        from ultralytics import YOLO
        sys.path.insert(0, str(package / "scripts"))
        from basket_vision_ros_node import BasketVisionRosNode
        if backend == "intel:gpu":
            import openvino as ov
            devices = ov.Core().available_devices
            report.add("OpenVINO devices", "PASS", devices)
            if not any(device.startswith("GPU") for device in devices):
                raise RuntimeError("Intel GPU is not exposed by OpenVINO; check GPU drivers and device permissions")
        logger = SimpleNamespace(
            info=lambda message: print("[adapter] " + message),
            error=lambda message: print("[adapter error] " + message),
        )
        node = BasketVisionRosNode.__new__(BasketVisionRosNode)
        node.model_path = str(package / "models" / "basket_detect.onnx")
        node.cpu_model_path = node.model_path
        node.device = backend
        node.requested_device = backend
        node.target_class = 0
        node.confidence_threshold = 0.08
        node.nms_iou_threshold = 0.01
        node.inference_image_size = 640
        node.half = False
        node.using_tensorrt = False
        node.node = SimpleNamespace(get_logger=lambda: logger)
        node.vision = SimpleNamespace(YOLO=YOLO)
        node._load_model()
        result = node._run_inference(np.zeros((640, 640, 3), dtype=np.uint8))
        if node.device != backend:
            raise RuntimeError("requested {} but adapter fell back to {}; GPU validation failed".format(backend, node.device))
        report.add("original adapter synthetic inference", "PASS", {
            "requested": backend, "active": node.device,
            "model": node.model_path, "boxes": len(result.boxes) if result.boxes is not None else 0,
        })
    except Exception as error:
        report.add("original adapter synthetic inference", "FAIL", "{}: {}".format(type(error).__name__, error))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("files", "python", "onnx", "cpu", "intel-gpu"), default="python")
    parser.add_argument("--package-root", type=Path,
                        default=Path(__file__).resolve().parents[1] / "src" / PACKAGE_NAME)
    parser.add_argument("--require-ros", action="store_true", help="import ROS interfaces; no node is created")
    parser.add_argument("--json", type=Path, help="write this machine's check report")
    args = parser.parse_args()
    report = Report(args.mode)
    sys.dont_write_bytecode = True
    os.environ["YOLO_AUTOINSTALL"] = "false"
    with tempfile.TemporaryDirectory(prefix="cuadc-env-check-") as config_dir:
        os.environ["YOLO_CONFIG_DIR"] = config_dir
        package = args.package_root.resolve()
        check_files(package, report)
        if args.mode in ("python", "cpu", "intel-gpu"):
            check_python(report)
        if args.require_ros:
            for name in ("rclpy", "ament_index_python", "cv_bridge", "geometry_msgs.msg",
                         "diagnostic_msgs.msg", "nav_msgs.msg", "sensor_msgs.msg", "mavros_msgs.msg"):
                load_module(name, report)
        if args.mode == "onnx":
            check_onnx(package, report)
        if args.mode == "intel-gpu":
            ov = load_module("openvino", report)
            if ov is not None and not str(getattr(ov, "__version__", "")).startswith("2026.4.1"):
                report.add("OpenVINO model conversion version", "WARN", "IR records 2026.4.1; validate this different runtime")
        if args.mode in ("cpu", "intel-gpu") and not report.failed:
            check_adapter(package, "cpu" if args.mode == "cpu" else "intel:gpu", report)
    report.data["success"] = not report.failed
    report.data["validation_limit"] = "Synthetic inference validates loading/execution, not detection accuracy, camera timing, Intel GPU performance, or flight safety."
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(report.data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("Result: " + ("FAIL" if report.failed else "PASS") + " (scope: " + args.mode + ")")
    return 1 if report.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
