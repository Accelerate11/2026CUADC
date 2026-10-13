"""部署回归：不启动 ROS、相机、飞控，不连接 SMTP。"""
import ast
from contextlib import redirect_stderr, redirect_stdout
import importlib.util
import io
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]


def load_module(name, relative):
    spec = importlib.util.spec_from_file_location(name, ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


deployment = load_module("deployment_under_test", "tools/deployment_config.py")
report = load_module("report_under_test", "tools/flight_summary_report.py")


class DeploymentConfigTests(unittest.TestCase):
    def test_explicit_serial_url(self):
        self.assertEqual(
            deployment.parse_serial_url("serial:///dev/serial/by-id/TEST_FCU:115200"),
            ("/dev/serial/by-id/TEST_FCU", 115200),
        )

    def test_invalid_or_missing_connection_rejected(self):
        for value in ("", "udp://:14550@", "serial://relative:115200",
                      "serial:///dev/ttyACM0", "serial:///dev/ttyACM0:0",
                      "serial:///dev/ttyACM0:abc", "serial:///dev/ttyACM0:115200\n"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                deployment.parse_serial_url(value)

    def test_lock_is_portable_and_can_be_shared(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(deployment.serial_lock_path(), ROOT / ".flight_serial.lock")
        with mock.patch.dict(os.environ, {"CUADC_SERIAL_LOCK": "/tmp/cuadc-test.lock"}):
            self.assertEqual(deployment.serial_lock_path(), Path("/tmp/cuadc-test.lock"))


class LocalReportTests(unittest.TestCase):
    def invoke(self, directory, *arguments):
        with mock.patch.object(sys, "argv", ["flight_summary_report.py", str(directory), *arguments]), \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return report.main()

    def test_default_generates_local_report_without_email(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(report, "send_email_with_retry") as sender, \
                mock.patch.object(report.smtplib, "SMTP_SSL") as network:
            self.assertEqual(self.invoke(directory), 0)
            self.assertTrue((Path(directory) / "flight_summary.txt").is_file())
            sender.assert_not_called()
            network.assert_not_called()

    def test_failure_also_stays_local(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(report, "generate_report", side_effect=ValueError("fixture failure")), \
                mock.patch.object(report, "send_email_with_retry") as sender:
            self.assertEqual(self.invoke(directory), 1)
            self.assertIn("fixture failure", (Path(directory) / "flight_summary_error.txt").read_text(encoding="utf-8"))
            sender.assert_not_called()

    def test_manual_send_is_explicit_and_mocked(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(report, "send_email_with_retry") as sender:
            self.assertEqual(self.invoke(directory, "--send"), 0)
            sender.assert_called_once()

    def test_no_email_remains_compatible(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(report, "send_email_with_retry") as sender:
            self.assertEqual(self.invoke(directory, "--no-email"), 0)
            sender.assert_not_called()


class RoutePreflightTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # 只编译原文件中的数学函数及姿态回调，不导入或初始化 ROS。
        tree = ast.parse((ROOT / "tools/route_preflight.py").read_text(encoding="utf-8"))
        functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)
                     and node.name in {"distance_m", "bearing_deg"}]
        stream = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Stream")
        functions.extend(node for node in stream.body if isinstance(node, ast.FunctionDef)
                         and node.name == "attitude_cb")
        cls.namespace = {"math": math, "time": time, "EARTH_M": 6378137.0}
        exec(compile(ast.Module(body=functions, type_ignores=[]), "<preflight-functions>", "exec"), cls.namespace)

    def test_synthetic_gps_baseline(self):
        p1, p2 = (0.0, 0.0), (0.0002, 0.0)
        self.assertAlmostEqual(self.namespace["distance_m"](p1, p2), 22.263898, places=5)
        self.assertEqual(self.namespace["bearing_deg"](p1, p2), 0.0)
        self.assertEqual(self.namespace["bearing_deg"](p2, p1), 180.0)

    def test_attitude_gate_rejects_invalid_quaternion(self):
        node = SimpleNamespace(attitude_count=0, attitude_arrival=None)
        for w, covariance in ((0.0, 0.0), (float("nan"), 0.0), (1.0, -1.0)):
            message = SimpleNamespace(orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=w), orientation_covariance=[covariance])
            self.namespace["attitude_cb"](node, message)
        self.assertEqual(node.attitude_count, 0)
        valid = SimpleNamespace(orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0), orientation_covariance=[0.0])
        self.namespace["attitude_cb"](node, valid)
        self.assertEqual(node.attitude_count, 1)
        self.assertIsNotNone(node.attitude_arrival)


@unittest.skipUnless(os.name == "posix" and shutil.which("bash"), "机载 Bash 参数检查在 Linux 上执行")
class ShellEntryTests(unittest.TestCase):
    def run_entry(self, relative, *arguments):
        environment = dict(os.environ, CUADC_FCU_URL="", PYTHONDONTWRITEBYTECODE="1")
        return subprocess.run(["bash", str(ROOT / relative), *arguments], cwd=ROOT,
                              env=environment, capture_output=True, text=True, timeout=10)

    def test_help_never_needs_ros_or_hardware(self):
        for relative in ("full_mission/run_full_mission.sh", "drop_only/run_drop_only.sh",
                         "fixed_release_test/run_fixed_release_test.sh", "servo_test/run_servo_test.sh",
                         "indoor_projection/preflight_projection.sh"):
            with self.subTest(entry=relative):
                result = self.run_entry(relative, "--help")
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("用法", result.stdout)

    def test_missing_fcu_is_rejected_before_hardware_setup(self):
        for relative in ("full_mission/run_full_mission.sh", "drop_only/run_drop_only.sh",
                         "fixed_release_test/run_fixed_release_test.sh"):
            with self.subTest(entry=relative):
                result = self.run_entry(relative, "example")
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("串口 URL", result.stderr)


if __name__ == "__main__":
    unittest.main()
