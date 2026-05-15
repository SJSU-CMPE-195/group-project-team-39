"""TPS remap maps and preprocess identical to main.py (uses calibration_config)."""

from __future__ import annotations

import cv2
import numpy as np
from scipy.interpolate import RBFInterpolator

from calibration_config import OUTPUT_H, OUTPUT_W, SRC_POINTS


def build_remap_maps():
    dst = np.array(
        [
            [0, 0],
            [OUTPUT_W // 2, 0],
            [OUTPUT_W - 1, 0],
            [OUTPUT_W - 1, OUTPUT_H - 1],
            [OUTPUT_W // 2, OUTPUT_H - 1],
            [0, OUTPUT_H - 1],
        ],
        dtype=np.float64,
    )
    src = np.array(SRC_POINTS, dtype=np.float64)
    rbf_x = RBFInterpolator(dst, src[:, 0], kernel="thin_plate_spline")
    rbf_y = RBFInterpolator(dst, src[:, 1], kernel="thin_plate_spline")
    gx, gy = np.meshgrid(np.arange(OUTPUT_W), np.arange(OUTPUT_H))
    grid = np.stack([gx.ravel(), gy.ravel()], axis=1).astype(np.float64)
    map_x = rbf_x(grid).reshape(OUTPUT_H, OUTPUT_W).astype(np.float32)
    map_y = rbf_y(grid).reshape(OUTPUT_H, OUTPUT_W).astype(np.float32)
    return map_x, map_y


def preprocess_frame(raw_bgr: np.ndarray, map_x: np.ndarray, map_y: np.ndarray):
    """TPS-remap then rotate 90 CW to match main.py portrait space."""
    remapped = cv2.remap(raw_bgr, map_x, map_y, cv2.INTER_LINEAR)
    return cv2.rotate(remapped, cv2.ROTATE_90_CLOCKWISE)
