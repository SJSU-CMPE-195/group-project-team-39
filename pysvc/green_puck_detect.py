"""Green puck HSV + contour centroid (aligned with main.py)."""

from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

MIN_AREA = 75

MORPH_KERNEL = np.ones((5, 5), np.uint8)
HSV_LOWER    = np.array([25,  40,  40], dtype=np.uint8)
HSV_UPPER    = np.array([95, 255, 255], dtype=np.uint8)

# CLAHE applied to the V channel before inRange so dim frames are normalised.
_clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))


def green_mask_hsv(frame_bgr: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    v = _clahe.apply(v)
    hsv = cv2.merge([h, s, v])
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  MORPH_KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=1)
    return mask


def detect_green_puck_center(
    frame_bgr: np.ndarray,
) -> Optional[Tuple[int, int]]:
    fh, fw = frame_bgr.shape[:2]
    mask = green_mask_hsv(frame_bgr)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None
    cx = int(np.clip(M["m10"] / M["m00"], 0, fw - 1))
    cy = int(np.clip(M["m01"] / M["m00"], 0, fh - 1))
    return (cx, cy)
