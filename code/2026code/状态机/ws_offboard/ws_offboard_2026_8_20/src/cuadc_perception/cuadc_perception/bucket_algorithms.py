"""Camera-transport-independent helpers extracted from the supplied detector.

This module intentionally has no RealSense SDK import or device-opening code;
the production ROS node receives all images from the RealSense ROS driver.
"""

from __future__ import annotations

from typing import Any, Mapping, Sequence

import cv2
import numpy as np


DUPLICATE_MASK_IOU_THRESHOLD = 0.20


def fit_ellipse_from_mask(mask: np.ndarray):
    binary_mask = (mask > 0).astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    usable = [
        contour
        for contour in contours
        if len(contour) >= 5 and cv2.contourArea(contour) > 30
    ]
    if not usable:
        return None
    contour = max(usable, key=cv2.contourArea)
    return cv2.fitEllipse(contour), contour


def valid_depth_values(
    depth_raw: np.ndarray, sample_mask: np.ndarray, depth_scale: float
) -> np.ndarray:
    values = depth_raw[sample_mask > 0].astype(np.float64) * depth_scale
    return values[np.isfinite(values) & (values > 0.10) & (values < 10.0)]


def summarize_depth(values: np.ndarray, sample_count: int) -> dict[str, Any]:
    if values.size == 0:
        return {
            "valid_count": 0,
            "valid_ratio": 0.0,
            "p15": None,
            "median": None,
            "iqr": None,
            "std": None,
        }
    p15, p25, median, p75 = np.percentile(values, [15, 25, 50, 75])
    return {
        "valid_count": int(values.size),
        "valid_ratio": float(values.size / max(1, sample_count)),
        "p15": float(p15),
        "median": float(median),
        "iqr": float(p75 - p25),
        "std": float(np.std(values)),
    }


def depth_summary(
    depth_raw: np.ndarray, sample_mask: np.ndarray, depth_scale: float
) -> dict[str, Any]:
    sample_count = int(np.count_nonzero(sample_mask))
    return summarize_depth(
        valid_depth_values(depth_raw, sample_mask, depth_scale), sample_count
    )


def filtered_depth_meters(
    depth_raw: np.ndarray,
    u: int,
    v: int,
    depth_scale: float,
    filter_size: int,
) -> float | None:
    if filter_size < 1 or filter_size % 2 == 0:
        raise ValueError("depth filter size must be a positive odd integer")
    half = filter_size // 2
    y0, y1 = max(0, v - half), min(depth_raw.shape[0], v + half + 1)
    x0, x1 = max(0, u - half), min(depth_raw.shape[1], u + half + 1)
    values = depth_raw[y0:y1, x0:x1].astype(np.float64) * depth_scale
    values = values[np.isfinite(values) & (values > 0.10) & (values < 10.0)]
    return float(np.median(values)) if values.size else None


def ellipse_band_mask(
    shape: Sequence[int], ellipse: tuple[Any, Any, Any], thickness: int
) -> np.ndarray:
    if thickness < 1:
        raise ValueError("ellipse band thickness must be positive")
    band = np.zeros(tuple(shape), dtype=np.uint8)
    cv2.ellipse(band, ellipse, 255, thickness)
    return band


def mask_iou(first: np.ndarray, second: np.ndarray) -> float:
    first_binary = first > 0
    second_binary = second > 0
    union = np.count_nonzero(first_binary | second_binary)
    if union == 0:
        return 0.0
    return float(np.count_nonzero(first_binary & second_binary) / union)


def boxes_overlap(first: np.ndarray, second: np.ndarray) -> bool:
    left = max(float(first[0]), float(second[0]))
    top = max(float(first[1]), float(second[1]))
    right = min(float(first[2]), float(second[2]))
    bottom = min(float(first[3]), float(second[3]))
    return right > left and bottom > top


def remove_duplicate_masks(
    candidates: Sequence[Mapping[str, Any]],
    iou_threshold: float = DUPLICATE_MASK_IOU_THRESHOLD,
) -> list[Mapping[str, Any]]:
    if not 0.0 <= iou_threshold <= 1.0:
        raise ValueError("duplicate mask IoU threshold must be in [0, 1]")
    selected: list[Mapping[str, Any]] = []
    for candidate in sorted(
        candidates, key=lambda item: float(item["confidence"]), reverse=True
    ):
        if any(
            boxes_overlap(np.asarray(candidate["box"]), np.asarray(kept["box"]))
            or mask_iou(
                np.asarray(candidate["mask"]), np.asarray(kept["mask"])
            )
            >= iou_threshold
            for kept in selected
        ):
            continue
        selected.append(candidate)
    return selected
