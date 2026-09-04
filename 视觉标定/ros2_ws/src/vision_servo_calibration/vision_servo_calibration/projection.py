"""Pure geometry helpers shared by the ROS viewer and unit tests."""
from __future__ import annotations

import math
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np


def validate_rotation_matrix(values: Sequence[float], tolerance: float = 1.0e-3) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.size != 9:
        raise ValueError("rotation must contain exactly 9 values")
    array = array.reshape(3, 3)
    if not np.all(np.isfinite(array)):
        raise ValueError("rotation contains non-finite values")
    error = float(np.linalg.norm(array.T @ array - np.eye(3), ord=np.inf))
    determinant = float(np.linalg.det(array))
    if error > tolerance:
        raise ValueError(f"rotation is not orthogonal: error={error:.6g}")
    if abs(determinant - 1.0) > tolerance:
        raise ValueError(f"rotation determinant must be +1, got {determinant:.6g}")
    return array


def finite_vector(values: Sequence[float], size: int, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=np.float64)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must be a finite {size}-vector")
    return array


def camera_to_body(
    camera_xyz_m: Sequence[float],
    rotation_camera_to_body: Sequence[float],
    translation_body_m: Sequence[float],
) -> np.ndarray:
    rotation = validate_rotation_matrix(rotation_camera_to_body)
    point = finite_vector(camera_xyz_m, 3, "camera_xyz_m")
    translation = finite_vector(translation_body_m, 3, "translation_body_m")
    return rotation @ point + translation


def body_to_camera(
    body_xyz_m: Sequence[float],
    rotation_camera_to_body: Sequence[float],
    translation_body_m: Sequence[float],
) -> np.ndarray:
    rotation = validate_rotation_matrix(rotation_camera_to_body)
    point = finite_vector(body_xyz_m, 3, "body_xyz_m")
    translation = finite_vector(translation_body_m, 3, "translation_body_m")
    return rotation.T @ (point - translation)


def project_camera_point(
    camera_xyz_m: Sequence[float],
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    min_z_m: float = 0.05,
) -> Optional[Tuple[float, float]]:
    point = finite_vector(camera_xyz_m, 3, "camera_xyz_m")
    values = (fx, fy, cx, cy, min_z_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("intrinsics must be finite")
    if fx <= 0.0 or fy <= 0.0 or min_z_m <= 0.0:
        raise ValueError("fx, fy and min_z_m must be positive")
    if point[2] <= min_z_m:
        return None
    return (
        float(fx * point[0] / point[2] + cx),
        float(fy * point[1] / point[2] + cy),
    )


def project_body_point(
    body_xyz_m: Sequence[float],
    rotation_camera_to_body: Sequence[float],
    translation_body_m: Sequence[float],
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    min_z_m: float = 0.05,
) -> Optional[Tuple[float, float]]:
    return project_camera_point(
        body_to_camera(body_xyz_m, rotation_camera_to_body, translation_body_m),
        fx,
        fy,
        cx,
        cy,
        min_z_m,
    )


def unpack_payload_offsets(flat: Iterable[float]) -> list[np.ndarray]:
    values = np.asarray(list(flat), dtype=np.float64)
    if values.size == 0 or values.size % 3 != 0 or not np.all(np.isfinite(values)):
        raise ValueError("payload offsets must contain 3*N finite values")
    return [values[index:index + 3].copy() for index in range(0, values.size, 3)]
