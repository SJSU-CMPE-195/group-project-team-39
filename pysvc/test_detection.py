"""
test_detection.py  --  Visual test for green-puck detection on the calibrated view.

Uses the same StereoCamera fusion, TPS remap, rotation, HSV thresholds, and
calibration overlays as main.py (via calibration_config, vision_preprocess,
green_puck_detect).  No IPC / Kalman — optional simple velocity from last two
accepted detections for arrow display.

Usage:
    python test_detection.py

Controls:
    SPACE  -- pause / resume the live feed
    m      -- toggle the HSV mask preview window
    r      -- reset smoothed velocity state
    ESC    -- quit
"""

from __future__ import annotations

import time

import cv2
import numpy as np

from calibration_config import (
    CRIT_BR_PX,
    CRIT_TL_PX,
    GANTRY_BR_PX,
    GANTRY_TL_PX,
    MALLET_ORIGIN_PX,
    PLAY_BR_PX,
    PLAY_TL_PX,
)
from camera import StereoCamera
from green_puck_detect import detect_green_puck_center, green_mask_hsv
from vision_preprocess import build_remap_maps, preprocess_frame

GREEN  = (0, 255, 0)
RED    = (0, 0, 255)
YELLOW = (0, 255, 255)
WHITE  = (255, 255, 255)
ORANGE = (0, 200, 255)
CYAN   = (255, 200, 0)


def draw_calibration_overlay(frame: np.ndarray) -> None:
    overlay = frame.copy()
    cv2.rectangle(overlay, CRIT_TL_PX, CRIT_BR_PX, CYAN, -1)
    cv2.addWeighted(overlay, 0.10, frame, 0.90, 0, frame)
    cv2.rectangle(frame, PLAY_TL_PX,   PLAY_BR_PX,   GREEN,  1)
    cv2.rectangle(frame, GANTRY_TL_PX, GANTRY_BR_PX, YELLOW, 1)
    cv2.rectangle(frame, CRIT_TL_PX,   CRIT_BR_PX,   CYAN,   2)
    cv2.putText(frame, "PLAY",
                (PLAY_TL_PX[0] + 4, PLAY_TL_PX[1] + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, GREEN, 1)
    cv2.putText(frame, "GANTRY",
                (GANTRY_TL_PX[0] + 4, GANTRY_TL_PX[1] + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, YELLOW, 1)
    cv2.putText(frame, "CRITICAL ZONE",
                (CRIT_TL_PX[0] + 6, CRIT_TL_PX[1] + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, CYAN, 1)
    mx, my = MALLET_ORIGIN_PX
    cv2.drawMarker(frame, (mx, my), ORANGE, cv2.MARKER_CROSS, 30, 2)
    cv2.circle(frame, (mx, my), 6, ORANGE, -1)
    cv2.putText(frame, "0,0", (mx + 10, my - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, ORANGE, 1)


def main() -> None:
    print("[INFO] Building TPS remap (same as main.py)...", flush=True)
    map_x, map_y = build_remap_maps()
    print("[INFO] TPS remap ready.", flush=True)

    cam = StereoCamera()

    print(
        f"Camera (fused): {cam.output_width}x{cam.output_height} px  "
        "|  calibration_config.py drives remap (same as main.py)",
        flush=True,
    )
    print("\nControls:")
    print("  SPACE -- pause / resume")
    print("  m     -- toggle HSV mask window")
    print("  r     -- reset last position (velocity arrow)")
    print("  ESC   -- quit\n")

    paused = False
    show_mask = False

    fps_t0 = time.time()
    frame_count = 0
    fps_display = 0.0
    last_frame: np.ndarray | None = None

    prev_pos: tuple[int, int] | None = None

    cv2.namedWindow("Detection Test (calibrated)", cv2.WINDOW_NORMAL)

    try:
        while True:
            if not paused:
                ret, raw = cam.read()
                if not ret or raw is None:
                    print("Failed to read frame.", flush=True)
                    break

                frame = preprocess_frame(raw, map_x, map_y)
                last_frame = frame.copy()

                center = detect_green_puck_center(frame)

                frame_count += 1
                elapsed = time.time() - fps_t0
                if elapsed >= 1.0:
                    fps_display = frame_count / elapsed
                    frame_count = 0
                    fps_t0 = time.time()
            else:
                if last_frame is None:
                    continue
                frame = last_frame.copy()
                center = detect_green_puck_center(frame)

            disp = frame.copy()
            h, w = disp.shape[:2]

            cv2.rectangle(disp, (0, 0), (w - 1, h - 1), WHITE, 1)
            cv2.line(disp, (0, h - 1), (w - 1, h - 1), ORANGE, 3)
            draw_calibration_overlay(disp)

            vx = vy = 0.0
            label = "NO PUCK"

            if center is not None:
                cx, cy = center
                if prev_pos is not None:
                    vx = float(cx - prev_pos[0])
                    vy = float(cy - prev_pos[1])
                prev_pos = (cx, cy)

                speed = (vx * vx + vy * vy) ** 0.5
                label = "STILL" if speed < 1.0 else "MOVING"

                cv2.circle(disp, (cx, cy), 8, GREEN, -1)
                cv2.circle(disp, (cx, cy), 14, GREEN, 2)
                cv2.putText(
                    disp,
                    label,
                    (cx + 18, cy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    GREEN,
                    2,
                )
                cv2.putText(
                    disp,
                    f"({cx},{cy})",
                    (cx + 18, cy + 14),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    WHITE,
                    1,
                )
                if label == "MOVING" and speed >= 1.0:
                    tip = (int(cx + 4 * vx), int(cy + 4 * vy))
                    cv2.arrowedLine(disp, (cx, cy), tip, YELLOW, 2, tipLength=0.3)
            else:
                prev_pos = None
                cv2.putText(disp, "NO PUCK", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, RED, 2)

            cv2.putText(disp, f"FPS: {fps_display:.1f}", (10, h - 36),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, WHITE, 1)
            cv2.putText(disp, f"{w}x{h}", (10, h - 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, WHITE, 1)
            if paused:
                cv2.putText(disp, "[PAUSED]", (w - 110, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, YELLOW, 1)

            cv2.imshow("Detection Test (calibrated)", disp)

            if show_mask and last_frame is not None:
                mask = green_mask_hsv(last_frame)
                cv2.imshow("HSV Green Mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            elif key == ord(" "):
                paused = not paused
                print(f"{'Paused' if paused else 'Resumed'}.", flush=True)
            elif key == ord("m"):
                show_mask = not show_mask
                if not show_mask:
                    cv2.destroyWindow("HSV Green Mask")
                print(f"Mask window {'on' if show_mask else 'off'}.", flush=True)
            elif key == ord("r"):
                prev_pos = None
                print("Velocity state reset.", flush=True)

    finally:
        cam.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
