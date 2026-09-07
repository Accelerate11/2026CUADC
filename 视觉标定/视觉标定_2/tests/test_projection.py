import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "vision_servo_calibration"
sys.path.insert(0, str(PKG))

import numpy as np

from vision_servo_calibration.projection import (
    body_to_camera,
    camera_to_body,
    project_camera_point,
    unpack_payload_offsets,
    validate_rotation_matrix,
)


def test_round_trip_identity():
    r = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    t = [0.1, -0.2, 0.3]
    camera = np.array([0.2, 0.4, 2.0])
    body = camera_to_body(camera, r, t)
    recovered = body_to_camera(body, r, t)
    assert np.allclose(recovered, camera)


def test_projection_center():
    uv = project_camera_point([0.0, 0.0, 2.0], 600.0, 600.0, 640.0, 360.0)
    assert uv == (640.0, 360.0)


def test_projection_known_offset():
    uv = project_camera_point([1.0, -0.5, 2.0], 600.0, 400.0, 640.0, 360.0)
    assert math.isclose(uv[0], 940.0)
    assert math.isclose(uv[1], 260.0)


def test_reject_reflection():
    bad = [-1, 0, 0, 0, 1, 0, 0, 0, 1]
    try:
        validate_rotation_matrix(bad)
    except ValueError:
        pass
    else:
        raise AssertionError("reflection should be rejected")


def test_payload_unpack():
    offsets = unpack_payload_offsets([1, 2, 3, 4, 5, 6])
    assert len(offsets) == 2
    assert np.allclose(offsets[1], [4, 5, 6])
