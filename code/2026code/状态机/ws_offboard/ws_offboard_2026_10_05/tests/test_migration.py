"""回归检查包迁移后的路径和标定航向集成。"""
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest

from cuadc_bringup.route_config import read_route_heading
from cuadc_perception.basket_vision_ros_node import (
    FatalVisionError, PACKAGE_NAME, resolve_default_model_path,
)


class RouteHeadingTests(unittest.TestCase):
    def read(self, content):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "route.yaml"
            path.write_text(content, encoding="utf-8")
            return read_route_heading(path)

    def test_calibration_format(self):
        self.assertEqual(self.read("point_1: {latitude_deg: 0, longitude_deg: 0}\nheading_deg: 38.67452660\n"), 38.67452660)

    def test_boundaries(self):
        self.assertEqual(self.read("heading_deg: 0\n"), 0.0)
        self.assertEqual(self.read("heading_deg: 359.999\n"), 359.999)

    def test_invalid_routes_fail_before_launch(self):
        for content in ["[]", "", "other: 0", "heading_deg: true", "heading_deg: '30'",
                        "heading_deg: -1", "heading_deg: 360", "heading_deg: .nan", "heading_deg: .inf"]:
            with self.subTest(content=content), self.assertRaises(ValueError):
                self.read(content)

    def test_missing_file(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileNotFoundError):
                read_route_heading(Path(directory) / "missing.yaml")


class InstalledModelTests(unittest.TestCase):
    def test_installed_model_uses_new_package_index(self):
        with tempfile.TemporaryDirectory() as directory:
            share = Path(directory) / "share/cuadc_perception"
            model = share / "models/basket_v3.pt"
            model.parent.mkdir(parents=True)
            model.write_bytes(b"model-path-fixture")
            def getter(package):
                self.assertEqual(package, "cuadc_perception")
                return share
            result = resolve_default_model_path(
                script_path=Path(directory) / "lib/python3/site-packages/cuadc_perception/basket_vision_ros_node.py",
                package_share_getter=getter,
            )
            self.assertEqual(result, model)

    def test_source_tree_model_fallback(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / "src/cuadc_perception"
            model = package / "models/basket_v3.pt"
            model.parent.mkdir(parents=True)
            model.write_bytes(b"model-path-fixture")
            def unavailable(_):
                raise LookupError("No installed workspace")
            result = resolve_default_model_path(
                script_path=package / "cuadc_perception/basket_vision_ros_node.py",
                package_share_getter=unavailable,
            )
            self.assertEqual(result, model)

    def test_no_model_reports_checked_locations(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FatalVisionError):
                resolve_default_model_path(
                    script_path=Path(directory) / "module/node.py",
                    package_share_getter=lambda _: Path(directory) / "share",
                )


class LaunchIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / "src/cuadc_bringup/launch/flight.launch.py"
        spec = importlib.util.spec_from_file_location("cuadc_flight_launch", path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def context(self, directory, route="heading_deg: 45.0\n"):
        from launch import LaunchContext
        base = Path(directory)
        (base / "route.yaml").write_text(route, encoding="utf-8")
        (base / "params.yaml").write_text("cuadc_full_mission_node:\n  ros__parameters:\n    flight_enable: true\n", encoding="utf-8")
        (base / "model.onnx").write_bytes(b"path-fixture")
        context = LaunchContext()
        context.launch_configurations.update({
            "route_file": str(base / "route.yaml"), "params_file": str(base / "params.yaml"),
            "model_path": str(base / "model.onnx"), "fcu_url": "udp://:14550@",
            "gcs_url": "", "camera_serial": "", "log_dir": str(base / "logs"),
            "start_mavros": "false", "start_vision": "false",
        })
        return context

    def test_package_executables_and_heading_override(self):
        from launch_ros.actions import Node
        from launch.utilities import normalize_to_list_of_substitutions
        from launch.utilities import perform_substitutions
        with tempfile.TemporaryDirectory() as directory:
            context = self.context(directory)
            actions = self.module.launch_nodes(context)
            nodes = [a for a in actions if isinstance(a, Node)]
            def evaluate(value):
                return perform_substitutions(context, normalize_to_list_of_substitutions(value))
            pairs = {(evaluate(n.node_package), evaluate(n.node_executable)) for n in nodes}
            self.assertEqual(pairs, {
                ("cuadc_tools", "mavlink_stream_configurator"),
                ("cuadc_tools", "flight_telemetry_recorder"),
                ("cuadc_perception", "basket_vision_ros_node"),
                ("cuadc_mission", "cuadc_full_mission_node"),
            })
            import yaml
            inputs = yaml.safe_load((Path(directory) / "logs/launch_inputs.yaml").read_text())
            self.assertEqual(inputs["mission_overrides"]["geographic_heading_deg"], 45.0)
            self.assertNotIn("camera_serial", inputs["vision_overrides"])
            self.assertEqual((Path(directory) / "logs/route.yaml").read_text(), "heading_deg: 45.0\n")

    def test_bad_heading_does_not_create_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            context = self.context(directory, "heading_deg: -1\n")
            with self.assertRaises(ValueError):
                self.module.launch_nodes(context)
            self.assertFalse((Path(directory) / "logs").exists())


if __name__ == "__main__":
    unittest.main()
