"""
camera.py  --  ZED 2i stereo capture with horizontal-shift binocular fusion.

The ZED 2i outputs a side-by-side (SBS) frame at 720p mode:
    Raw frame  : 2560 x 720 px  @ up to 60 FPS
    Per eye    : 1280 x 720 px

The two cameras are factory-aligned horizontally, so the parallax between
them is almost entirely a horizontal translation.  At startup we estimate
that shift using phase correlation on the central region of the frame,
then every frame we translate the right eye by the inverse of that shift
before blending with the left eye 50/50.

Pipeline:
    raw 2560x720 SBS  ->  split left / right (1280x720 each)
                      ->  shift right eye horizontally to align with left
                      ->  50/50 blend  ->  1280x720 fused frame
"""

from __future__ import annotations

import glob
import sys
from typing import Optional, Tuple

import cv2
import numpy as np

# ── ZED 720p stereo dimensions ───────────────────────────────────────────────
CAPTURE_WIDTH  = 2560
CAPTURE_HEIGHT = 720
EYE_WIDTH      = CAPTURE_WIDTH // 2   # 1280 px per eye
TARGET_FPS     = 60


class StereoCamera:
    """ZED 2i camera wrapper -- returns a shift-aligned binocular fused frame."""

    def __init__(self, target_fps: int = TARGET_FPS) -> None:
        self._cap = self._open_device(target_fps)
        self.output_width:  int = EYE_WIDTH
        self.output_height: int = CAPTURE_HEIGHT

        # Horizontal shift (pixels) to translate right eye onto left eye
        self._shift_x: float = 0.0
        self._shift_y: float = 0.0
        self._calibrate_shift()

    # ── Public API ───────────────────────────────────────────────────────────

    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Capture one frame and return the fused image (BGR).

        Returns (True, frame) on success, (False, None) on read error.
        The returned frame is EYE_WIDTH x CAPTURE_HEIGHT (1280 x 720).
        """
        ret, raw = self._cap.read()
        if not ret or raw is None:
            return False, None

        w = raw.shape[1]
        if w < CAPTURE_WIDTH:
            return True, raw

        left  = raw[:, :EYE_WIDTH]
        right = raw[:, EYE_WIDTH:EYE_WIDTH * 2]

        if self._shift_x != 0.0 or self._shift_y != 0.0:
            M = np.float32([[1, 0, -self._shift_x],
                            [0, 1, -self._shift_y]])
            right = cv2.warpAffine(right, M, (EYE_WIDTH, CAPTURE_HEIGHT),
                                   borderMode=cv2.BORDER_REPLICATE)

        fused = cv2.addWeighted(left, 0.5, right, 0.5, 0)
        return True, fused

    def release(self) -> None:
        self._cap.release()

    # ── Internal helpers ─────────────────────────────────────────────────────

    def _calibrate_shift(self) -> None:
        """Estimate the horizontal parallax shift between left and right eyes.

        Uses phase correlation on the central 60% of the frame (avoids edges
        where there may be non-overlapping content).  Averages over several
        frames for stability.
        """
        print("[StereoCamera] Estimating stereo parallax shift...", flush=True)

        shifts_x = []
        shifts_y = []

        for _ in range(20):
            ret, raw = self._cap.read()
            if not ret or raw is None:
                continue
            if raw.shape[1] < CAPTURE_WIDTH:
                continue

            left  = raw[:, :EYE_WIDTH]
            right = raw[:, EYE_WIDTH:EYE_WIDTH * 2]

            gray_l = cv2.cvtColor(left,  cv2.COLOR_BGR2GRAY).astype(np.float64)
            gray_r = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY).astype(np.float64)

            # Use the central 60% to avoid non-overlapping borders
            margin_x = int(EYE_WIDTH * 0.20)
            margin_y = int(CAPTURE_HEIGHT * 0.20)
            roi_l = gray_l[margin_y:-margin_y, margin_x:-margin_x]
            roi_r = gray_r[margin_y:-margin_y, margin_x:-margin_x]

            # Apply a Hanning window to reduce edge artifacts in FFT
            hann = cv2.createHanningWindow(
                (roi_l.shape[1], roi_l.shape[0]), cv2.CV_64F)
            roi_l = roi_l * hann
            roi_r = roi_r * hann

            shift, response = cv2.phaseCorrelate(roi_l, roi_r)

            if response > 0.15:
                shifts_x.append(shift[0])
                shifts_y.append(shift[1])

        if shifts_x:
            self._shift_x = float(np.median(shifts_x))
            self._shift_y = float(np.median(shifts_y))
            print(f"[StereoCamera] Parallax shift: dx={self._shift_x:.2f} px, "
                  f"dy={self._shift_y:.2f} px  "
                  f"({len(shifts_x)} samples). Fusion enabled.", flush=True)
        else:
            self._shift_x = 0.0
            self._shift_y = 0.0
            print("[StereoCamera] WARNING: Could not estimate shift. "
                  "Falling back to left eye only.", flush=True)

    def _open_device(self, target_fps: int) -> cv2.VideoCapture:
        """Find and open the ZED device.

        On Linux (Apalis) iterates V4L2 paths (/dev/video*).
        On macOS falls back to VideoCapture(0) for development.
        """
        if sys.platform == "darwin":
            print("[StereoCamera] macOS -- using VideoCapture(0)", flush=True)
            cap = cv2.VideoCapture(0)
            self._configure(cap, target_fps)
            return cap

        devices = sorted(glob.glob("/dev/video*"))
        print(f"[StereoCamera] Found video devices: {devices}", flush=True)
        for dev in devices:
            cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            if not cap.isOpened():
                continue
            self._configure(cap, target_fps)
            ret, _ = cap.read()
            if ret:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(f"[StereoCamera] ZED at {dev} ({w}x{h} @ {target_fps} FPS)",
                      flush=True)
                return cap
            cap.release()

        raise RuntimeError(
            f"No working camera found. V4L2 devices tried: {devices}"
        )

    @staticmethod
    def _configure(cap: cv2.VideoCapture, target_fps: int) -> None:
        cap.set(cv2.CAP_PROP_FOURCC,       cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS,          target_fps)
