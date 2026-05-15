import math
import os
import signal
import sys
import time

import cv2
import numpy as np

from calibration_config import (
    CRIT_BR_PX,
    CRIT_TL_PX,
    GANTRY_BR_PX,
    GANTRY_TL_PX,
    MALLET_ORIGIN_PX,
    OUTPUT_H,
    OUTPUT_W,
    PLAY_BR_PX,
    PLAY_TL_PX,
    PX_PER_CM_X,
    PX_PER_CM_Y,
)
from camera import StereoCamera
from green_puck_detect import detect_green_puck_center
from ipc_handler import IDLE, INTERCEPT, MOVING, STATE_IDLE, IPCHandler
from vision_preprocess import build_remap_maps, preprocess_frame

HEADLESS = os.environ.get("HEADLESS", "0") == "1"

# Calibration lives in calibration_config.py (paste crop_calibrate.py output there).

# ==============================================================================
# GANTRY LIMITS (mm)
# ==============================================================================
GANTRY_X_MAX_MM = 869.0
GANTRY_Y_MAX_MM = 901.0
DEFAULT_X_MM    = 434.0   # idle X -- gantry centre
DEFAULT_Y_MM    =   0.0   # idle Y -- south wall

GOAL_LINE_PX_Y  = PLAY_BR_PX[1]  # robot-side goal line (pixel Y in portrait frame)

# ==============================================================================
# CALIBRATION / STARTUP CHECKS
# ==============================================================================
CALIBRATION_TOL_MM     = 2.0    # max mm error allowed at GANTRY_TL/BR corners
CORNER_CHECK_TIMEOUT_S = 30.0   # per-corner wait for cppsvc ready=1
SKIP_CALIBRATION_CHECK = os.environ.get("SKIP_CALIBRATION_CHECK", "0") == "1"
SKIP_CORNER_CHECK      = os.environ.get("SKIP_CORNER_CHECK",      "0") == "1"

# ==============================================================================
# FRAME DIMENSIONS (derived from camera output)
# ==============================================================================
# Camera returns 1280x720 fused frame; TPS remap produces OUTPUT_W x OUTPUT_H;
# a 90-CW rotation yields portrait OUTPUT_H wide x OUTPUT_W tall.
FRAME_W = OUTPUT_H   # 400 px  (portrait width)
FRAME_H = OUTPUT_W   # 510 px  (portrait height)

# ==============================================================================
# TRACKING SETTINGS
# ==============================================================================
MIN_AREA                 = 75
MAX_JUMP_PX              = 200
RELAX_JUMP_AFTER_MISSING = 5
MAX_MISSING_FRAMES       = 15
STILL_THRESH_PX          = 1.0
STILL_CONFIRM_FRAMES     = 1
MAX_BOUNCES              = 2
MIN_SPEED_PX             = 4.0

# ==============================================================================
# PREDICTION BUFFER / CONVERGENCE
# ==============================================================================
BUFFER_WINDOW         = 0.15   # seconds
PRINT_INTERVAL        = 0.02   # seconds -- evaluate at ~50 Hz
MIN_BUFFER_FOR_COMMIT = 3
RECENT_N              = 3
CONVERGE_SPREAD_PX    = 18
ANCHOR_DEADBAND_PX    = 12
REFINE_DEADBAND_PX    = 30
DISPLAY_EMA_ALPHA     = 0.40

# ==============================================================================
# COLOURS (BGR)
# ==============================================================================
GREEN  = (0, 255, 0)
BLUE   = (255, 0, 0)
RED    = (0, 0, 255)
WHITE  = (255, 255, 255)
YELLOW = (0, 255, 255)
ORANGE = (0, 200, 255)
CYAN   = (255, 200, 0)

FPS_LOG_INTERVAL = 1.0


# ==============================================================================
# COORDINATE CONVERSION
# ==============================================================================
def px_to_gantry_mm_raw(px_x: float, px_y: float):
    """Same conversion as px_to_gantry_mm, but unclamped.

    Used by verify_gantry_calibration() so that an out-of-range pixel maps to
    out-of-range mm instead of being silently clipped to (0,0)..(869,901).
    """
    cm_x = (px_x - MALLET_ORIGIN_PX[0]) / PX_PER_CM_X
    cm_y = (MALLET_ORIGIN_PX[1] - px_y) / PX_PER_CM_Y
    return float(cm_x * 10.0), float(cm_y * 10.0)


def px_to_gantry_mm(px_x: float, px_y: float):
    """Convert remapped+rotated pixel coords to absolute gantry mm.

    MALLET_ORIGIN_PX is where the gantry (0, 0) appears in the portrait frame.
    X grows right, Y grows upward (toward the opponent) in the portrait frame,
    matching the gantry convention where Y=0 is the south (robot) wall.
    """
    raw_x, raw_y = px_to_gantry_mm_raw(px_x, px_y)
    gx = float(np.clip(raw_x, 0.0, GANTRY_X_MAX_MM))
    gy = float(np.clip(raw_y, 0.0, GANTRY_Y_MAX_MM))
    return gx, gy


def verify_gantry_calibration(tol_mm: float = CALIBRATION_TOL_MM) -> bool:
    """Assert GANTRY_TL_PX -> (0, 901) mm and GANTRY_BR_PX -> (869, 0) mm.

    Always logs the implied mm + delta for both corners.  Returns True iff
    every component is within `tol_mm`.
    """
    tl_mm = px_to_gantry_mm_raw(*GANTRY_TL_PX)
    br_mm = px_to_gantry_mm_raw(*GANTRY_BR_PX)

    exp_tl = (0.0,             GANTRY_Y_MAX_MM)
    exp_br = (GANTRY_X_MAX_MM, 0.0)

    dtl = (tl_mm[0] - exp_tl[0], tl_mm[1] - exp_tl[1])
    dbr = (br_mm[0] - exp_br[0], br_mm[1] - exp_br[1])

    print(
        f"[CALIB] GANTRY_TL_PX {GANTRY_TL_PX} -> "
        f"({tl_mm[0]:8.2f}, {tl_mm[1]:8.2f}) mm  "
        f"expected ({exp_tl[0]:.1f}, {exp_tl[1]:.1f})  "
        f"delta=({dtl[0]:+7.2f}, {dtl[1]:+7.2f})",
        flush=True,
    )
    print(
        f"[CALIB] GANTRY_BR_PX {GANTRY_BR_PX} -> "
        f"({br_mm[0]:8.2f}, {br_mm[1]:8.2f}) mm  "
        f"expected ({exp_br[0]:.1f}, {exp_br[1]:.1f})  "
        f"delta=({dbr[0]:+7.2f}, {dbr[1]:+7.2f})",
        flush=True,
    )

    ok = (abs(dtl[0]) <= tol_mm and abs(dtl[1]) <= tol_mm
          and abs(dbr[0]) <= tol_mm and abs(dbr[1]) <= tol_mm)

    if ok:
        print(f"[CALIB] OK: both gantry corners within {tol_mm:.1f} mm.",
              flush=True)
    else:
        print(
            f"[CALIB] FAIL: gantry corners exceed {tol_mm:.1f} mm tolerance.\n"
            f"        Re-run crop_calibrate.py to fix MALLET_ORIGIN_PX /\n"
            f"        PX_PER_CM_X / PX_PER_CM_Y, or set SKIP_CALIBRATION_CHECK=1\n"
            f"        to bypass (development only).",
            flush=True,
        )
    return ok


def run_four_corner_check(ipc, timeout_s: float = CORNER_CHECK_TIMEOUT_S) -> bool:
    """Drive the gantry to the four mm corners and verify each move completes.

    Sends raw mm coordinates (not via MALLET_ORIGIN_PX).  For each corner,
    waits for cppsvc to transition MOVING -> STATE_IDLE+ready=1 with
    status_seq advance.  Returns True iff every corner completes within
    `timeout_s`.  No operator confirmation: this verifies the IPC + motor
    actuation only, not physical reach accuracy.
    """
    corners = [
        ("SW", (0.0,             0.0)),
        ("SE", (GANTRY_X_MAX_MM, 0.0)),
        ("NE", (GANTRY_X_MAX_MM, GANTRY_Y_MAX_MM)),
        ("NW", (0.0,             GANTRY_Y_MAX_MM)),
    ]

    print("\n[CORNER] Starting 4-corner movement verification.", flush=True)
    print("[CORNER] Set SKIP_CORNER_CHECK=1 to bypass once trusted.", flush=True)

    # Wait for cppsvc to finish its own startup move (HOMING -> STATE_IDLE,
    # ready=1) before reading any pre_seq.  Without this, an early read can
    # latch onto cppsvc's startup STATE_IDLE write and false-pass the first
    # corner before cppsvc has even dequeued the SW command.
    print("[CORNER] Waiting for cppsvc startup (STATE_IDLE + ready=1)...",
          flush=True)
    t0 = time.time()
    ready_seen = False
    while time.time() - t0 < timeout_s:
        st = ipc.read_status()
        if st.state == STATE_IDLE and st.ready == 1 and st.status_seq >= 1:
            ready_seen = True
            break
        time.sleep(0.05)
    if not ready_seen:
        print(
            f"[CORNER] cppsvc never reached STATE_IDLE+ready=1 within "
            f"{timeout_s:.0f}s; aborting.",
            flush=True,
        )
        return False
    print("[CORNER] cppsvc ready.", flush=True)

    results = []
    for label, (x_mm, y_mm) in corners:
        pre_seq = ipc.read_status().status_seq
        print(
            f"\n[CORNER] {label}: sending INTERCEPT -> "
            f"({x_mm:.1f}, {y_mm:.1f}) mm",
            flush=True,
        )
        ipc.send_command(INTERCEPT, x_mm, y_mm)

        # Wait for the MOVING -> STATE_IDLE transition with status_seq
        # advance.  Requires observing MOVING first, so we cannot latch
        # onto a stale STATE_IDLE/ready=1 from the previous command.
        t_start = time.time()
        completed = False
        saw_moving = False
        while time.time() - t_start < timeout_s:
            st = ipc.read_status()
            if st.state == MOVING:
                saw_moving = True
            if (saw_moving and st.state == STATE_IDLE
                    and st.ready == 1 and st.status_seq > pre_seq):
                completed = True
                break
            time.sleep(0.05)

        if not completed:
            print(
                f"[CORNER] {label}: TIMEOUT after {timeout_s:.0f} s "
                f"(status_seq stuck at {pre_seq}). Marking FAIL.",
                flush=True,
            )
        else:
            print(f"[CORNER] {label}: completed.", flush=True)
        results.append((label, x_mm, y_mm, completed))

    print("\n[CORNER] Summary:", flush=True)
    all_ok = True
    for label, x_mm, y_mm, ok in results:
        tag = "OK  " if ok else "FAIL"
        print(f"  {tag}  {label}  ({x_mm:.0f}, {y_mm:.0f}) mm", flush=True)
        all_ok = all_ok and ok

    # Park at default rest position regardless of outcome.
    ipc.send_command(IDLE, DEFAULT_X_MM, DEFAULT_Y_MM)
    return all_ok


def in_critical_zone(px_x: float) -> bool:
    """True if the predicted bottom-target X falls inside the mallet's defend band."""
    return CRIT_TL_PX[0] <= px_x <= CRIT_BR_PX[0]


# ==============================================================================
# KALMAN FILTER
# ==============================================================================
def create_kalman():
    # processNoiseCov velocity terms (vx, vy) are kept low (1e-3) so the filter
    # trusts its smoothed velocity estimate rather than raw jitter.  Under poor
    # lighting the centroid can jump 1-3 px per frame even when the puck is
    # stationary; without this the Kalman velocity never settles near zero and
    # the puck oscillates STILL<->MOVING.  measurementNoiseCov is raised to 0.1
    # (from 0.05) for the same reason -- tell the filter measurement are noisier.
    # If the system feels sluggish to react to a real fast shot, lower the
    # velocity processNoiseCov back toward 0.05 or drop measurementNoiseCov.
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix   = np.eye(2, 4, dtype=np.float32)
    kf.transitionMatrix    = np.array(
        [[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
    kf.processNoiseCov     = np.diag([1e-2, 1e-2, 1e-3, 1e-3]).astype(np.float32)
    kf.measurementNoiseCov = (1e-1 * np.eye(2)).astype(np.float32)
    kf.errorCovPost        = np.eye(4, dtype=np.float32)
    return kf


# ==============================================================================
# GEOMETRY / PHYSICS
# ==============================================================================
def normalize(vx: float, vy: float, eps: float = 1e-9):
    mag = math.hypot(vx, vy)
    return (vx / mag, vy / mag) if mag > eps else (0.0, 0.0)


def point_distance(a, b) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def reflect(v, wall):
    vx, vy = float(v[0]), float(v[1])
    if wall in ("left", "right"):
        return (-vx,  vy)
    if wall in ("top",  "bottom"):
        return ( vx, -vy)
    return vx, vy


def first_border_intersection(p0, v, w: int, bottom_y: int):
    """Return the first (point, wall) the ray (p0, v) hits inside the play box.

    The play box spans x in [0, w-1] and y in [0, bottom_y].  Only the south
    wall uses bottom_y; north, west, and east walls are unchanged.
    """
    x0, y0     = float(p0[0]), float(p0[1])
    vx, vy     = float(v[0]),  float(v[1])
    eps        = 1e-9
    candidates = []

    if abs(vx) > eps:
        for t, wall in [((0 - x0) / vx, "left"), (((w - 1) - x0) / vx, "right")]:
            if t > 0:
                y = y0 + t * vy
                if 0 <= y <= bottom_y:
                    candidates.append(
                        (t, (0 if wall == "left" else w - 1, int(round(y))), wall))

    if abs(vy) > eps:
        for t, wall in [((0 - y0) / vy, "top"), ((bottom_y - y0) / vy, "bottom")]:
            if t > 0:
                x = x0 + t * vx
                if 0 <= x <= w - 1:
                    candidates.append(
                        (t, (int(round(x)), 0 if wall == "top" else bottom_y), wall))

    if not candidates:
        return None, None
    candidates.sort(key=lambda c: c[0])
    return candidates[0][1], candidates[0][2]


def trace_path_to_bottom(start_pt, unit_v, w: int, bottom_y: int,
                         max_bounces: int = MAX_BOUNCES):
    """Cast a bounce-aware ray and return (path_points, bottom_target).

    'bottom' is the goal line at pixel row bottom_y (PLAY_BR_PX[1]).
    Returns (None, None) if the ray exits through the top or loops.
    """
    points     = [(int(np.clip(round(start_pt[0]), 0, w - 1)),
                   int(np.clip(round(start_pt[1]), 0, bottom_y)))]
    current_pt = (float(start_pt[0]), float(start_pt[1]))
    current_v  = (float(unit_v[0]),   float(unit_v[1]))
    eps        = 1e-3

    for _ in range(max_bounces + 1):
        hit_pt, hit_wall = first_border_intersection(current_pt, current_v, w, bottom_y)
        if hit_pt is None:
            return None, None
        hit_pt = (int(np.clip(hit_pt[0], 0, w - 1)),
                  int(np.clip(hit_pt[1], 0, bottom_y)))
        if hit_pt == points[-1]:
            return None, None
        points.append(hit_pt)

        if hit_wall == "bottom":
            return points, hit_pt
        if hit_wall not in ("left", "right"):
            return None, None

        current_v  = reflect(current_v, hit_wall)
        current_pt = (hit_pt[0] + current_v[0] * eps,
                      hit_pt[1] + current_v[1] * eps)

    return None, None


# ==============================================================================
# DISPLAY HELPERS
# ==============================================================================
def draw_calibration_overlay(frame):
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


# ==============================================================================
# MAIN
# ==============================================================================
def main() -> None:
    # ── Task 1: coord-system alignment check ─────────────────────────────────
    if SKIP_CALIBRATION_CHECK:
        print("[CALIB] Skipped (SKIP_CALIBRATION_CHECK=1).", flush=True)
    elif not verify_gantry_calibration(CALIBRATION_TOL_MM):
        sys.exit(1)

    print("[INFO] Building TPS remap...", flush=True)
    map_x, map_y = build_remap_maps()
    print("[INFO] TPS remap ready.", flush=True)

    cam = StereoCamera()
    ipc = IPCHandler()

    # ── Task 2: four-corner physical-reach check ─────────────────────────────
    if SKIP_CORNER_CHECK:
        print("[CORNER] Skipped (SKIP_CORNER_CHECK=1).", flush=True)
    elif not run_four_corner_check(ipc, CORNER_CHECK_TIMEOUT_S):
        print("[CORNER] One or more corners failed. Exiting.", flush=True)
        ipc.close()
        cam.release()
        sys.exit(1)

    if not HEADLESS:
        cv2.namedWindow("RoboMallet", cv2.WINDOW_NORMAL)

    running = [True]

    def _stop(sig, _frame):
        running[0] = False

    signal.signal(signal.SIGINT,  _stop)
    signal.signal(signal.SIGTERM, _stop)

    kalman             = create_kalman()
    kalman_initialized = False

    frame_count     = 0
    fps_display     = 0.0
    fps_t0          = time.time()
    missing_count   = 0
    still_count     = 0

    prediction_buffer     = []
    last_print_time       = 0.0
    last_committed_target = None   # last px target committed to IPC

    display_ema        = None
    last_path_points   = []
    last_bottom_target = None
    window_sized       = False

    print(f"[INFO] Mallet origin (px): {MALLET_ORIGIN_PX}", flush=True)
    print(f"[INFO] Play area   (px): {PLAY_TL_PX} -> {PLAY_BR_PX}", flush=True)
    print(f"[INFO] Gantry range(px): {GANTRY_TL_PX} -> {GANTRY_BR_PX}", flush=True)
    print(f"[INFO] Critical zone(px): {CRIT_TL_PX} -> {CRIT_BR_PX}", flush=True)
    print(f"[INFO] PX_PER_CM: x={PX_PER_CM_X:.4f}  y={PX_PER_CM_Y:.4f}", flush=True)
    print("pysvc main loop started.", flush=True)

    try:
        while running[0]:
            ret, raw_frame = cam.read()
            if not ret or raw_frame is None:
                print("Failed to read frame.", flush=True)
                break

            frame    = preprocess_frame(raw_frame, map_x, map_y)
            h, w     = frame.shape[:2]
            now      = time.time()

            if not HEADLESS and not window_sized:
                cv2.resizeWindow("RoboMallet", w, h)
                window_sized = True

            center = detect_green_puck_center(frame)

            # ── Reset tracker after too many missed frames ─────────────────
            if missing_count >= MAX_MISSING_FRAMES and kalman_initialized:
                print(f"[INFO] Lost track after {missing_count} frames", flush=True)
                kalman, kalman_initialized, still_count = create_kalman(), False, 0
                prediction_buffer.clear()
                display_ema = None
                last_committed_target = None
                ipc.send_command(IDLE, DEFAULT_X_MM, DEFAULT_Y_MM)
                continue

            predicted      = kalman.predict()
            pred_x, pred_y = float(predicted[0][0]), float(predicted[1][0])

            accepted_measurement = None
            if center is not None:
                if not kalman_initialized:
                    kalman.statePost = np.array(
                        [[np.float32(center[0])], [np.float32(center[1])],
                         [0.], [0.]], np.float32)
                    kalman_initialized   = True
                    accepted_measurement = center
                    missing_count        = 0
                else:
                    dx, dy = center[0] - pred_x, center[1] - pred_y
                    ej = MAX_JUMP_PX * (3 if missing_count >= RELAX_JUMP_AFTER_MISSING else 1)
                    if dx * dx + dy * dy <= ej * ej:
                        accepted_measurement = center
                        missing_count        = 0
                    else:
                        missing_count += 1
            else:
                missing_count += 1

            if kalman_initialized and accepted_measurement is not None:
                kalman.correct(np.array(
                    [[np.float32(accepted_measurement[0])],
                     [np.float32(accepted_measurement[1])]], np.float32))

            filtered_pos = filtered_vel = dir_tip = None
            if kalman_initialized:
                filtered_pos = (
                    int(np.clip(round(float(kalman.statePost[0][0])), 0, w - 1)),
                    int(np.clip(round(float(kalman.statePost[1][0])), 0, h - 1)),
                )
                filtered_vel = (float(kalman.statePost[2][0]),
                                float(kalman.statePost[3][0]))

            # ── Draw border ────────────────────────────────────────────────
            if not HEADLESS:
                cv2.rectangle(frame, (0, 0), (w - 1, h - 1), WHITE, 1)
                cv2.line(frame, (0, GOAL_LINE_PX_Y), (w - 1, GOAL_LINE_PX_Y), ORANGE, 3)
                draw_calibration_overlay(frame)

            have_fresh_track = (kalman_initialized and filtered_pos is not None
                                and filtered_vel is not None
                                and accepted_measurement is not None)

            if have_fresh_track:
                vx, vy    = filtered_vel
                still_count = (still_count + 1) if (abs(vx) <= STILL_THRESH_PX
                                                     and abs(vy) <= STILL_THRESH_PX) else 0

                if still_count < STILL_CONFIRM_FRAMES:
                    ux, uy   = normalize(vx, vy)
                    speed_px = math.hypot(vx, vy)

                    if not HEADLESS:
                        dir_tip = (
                            int(np.clip(round(filtered_pos[0] + 60 * ux), 0, w - 1)),
                            int(np.clip(round(filtered_pos[1] + 60 * uy), 0, h - 1)),
                        )

                    if speed_px >= MIN_SPEED_PX and (abs(ux) > 0.0 or abs(uy) > 0.0):
                        path_pts, bot_tgt = trace_path_to_bottom(
                            filtered_pos, (ux, uy), w, GOAL_LINE_PX_Y, MAX_BOUNCES)

                        if path_pts is not None and bot_tgt is not None:
                            last_path_points   = path_pts
                            last_bottom_target = bot_tgt
                            prediction_buffer.append(
                                (float(bot_tgt[0]), float(bot_tgt[1]), now))

                            if display_ema is None:
                                display_ema = (float(bot_tgt[0]), float(bot_tgt[1]))
                            else:
                                display_ema = (
                                    DISPLAY_EMA_ALPHA * bot_tgt[0]
                                    + (1.0 - DISPLAY_EMA_ALPHA) * display_ema[0],
                                    DISPLAY_EMA_ALPHA * bot_tgt[1]
                                    + (1.0 - DISPLAY_EMA_ALPHA) * display_ema[1],
                                )
                    else:
                        prediction_buffer.clear()
                        display_ema = None

            # Purge stale buffer entries
            cutoff           = now - BUFFER_WINDOW
            prediction_buffer = [e for e in prediction_buffer if e[2] >= cutoff]

            # ── Return to idle when buffer empties ─────────────────────────
            if (now - last_print_time >= PRINT_INTERVAL
                    and len(prediction_buffer) == 0
                    and last_committed_target is not None):
                print("[TARGET] return-to-idle (no active trajectory)", flush=True)
                ipc.send_command(IDLE, DEFAULT_X_MM, DEFAULT_Y_MM)
                last_committed_target = None
                last_print_time       = now

            # ── Convergence-based commit ───────────────────────────────────
            if (now - last_print_time >= PRINT_INTERVAL
                    and len(prediction_buffer) >= MIN_BUFFER_FOR_COMMIT):

                recent = (prediction_buffer[-RECENT_N:]
                          if len(prediction_buffer) >= RECENT_N
                          else prediction_buffer)
                xs = [e[0] for e in recent]
                ys = [e[1] for e in recent]

                spread    = float(max(xs) - min(xs))
                converged = spread <= CONVERGE_SPREAD_PX

                mean_x  = int(round(float(np.mean(xs))))
                mean_y  = int(round(float(np.mean(ys))))
                mean_pt = (mean_x, mean_y)

                in_zone = in_critical_zone(mean_x)

                deadband = (ANCHOR_DEADBAND_PX if last_committed_target is None
                            else REFINE_DEADBAND_PX)
                moved_enough = (last_committed_target is None
                                or point_distance(mean_pt, last_committed_target) > deadband)

                ready_to_commit = (in_zone and converged) or (not in_zone)

                if ready_to_commit and moved_enough:
                    tag = "DEFEND" if in_zone else "IGNORE"
                    gx, gy = px_to_gantry_mm(mean_x, mean_y)

                    print(
                        f"[TARGET] px=({mean_x},{mean_y})  "
                        f"gantry=({gx:.1f},{gy:.1f}) mm  zone={tag}  "
                        f"(spread={int(spread)}px, n={len(recent)})",
                        flush=True,
                    )

                    if in_zone:
                        ipc.send_command(INTERCEPT, gx, gy)
                        last_committed_target = mean_pt
                    else:
                        ipc.send_command(IDLE, DEFAULT_X_MM, DEFAULT_Y_MM)
                        last_committed_target = None

                last_print_time = now

            # ── FPS counter ────────────────────────────────────────────────
            frame_count += 1
            elapsed      = now - fps_t0
            if elapsed >= FPS_LOG_INTERVAL:
                fps_display = frame_count / elapsed
                frame_count = 0
                fps_t0      = now
                print(f"[FPS] {fps_display:.1f}", flush=True)

            # ── Display ────────────────────────────────────────────────────
            if not HEADLESS:
                if filtered_pos is not None:
                    cv2.circle(frame, filtered_pos, 7, RED, -1)
                if filtered_pos is not None and dir_tip is not None:
                    cv2.arrowedLine(frame, filtered_pos, dir_tip, GREEN, 2,
                                    tipLength=0.25)
                if (filtered_pos is not None and display_ema is not None
                        and len(last_path_points) >= 2):
                    smooth_end = (int(round(display_ema[0])),
                                  int(round(display_ema[1])))
                    draw_pts   = list(last_path_points[:-1]) + [smooth_end]
                    for i in range(len(draw_pts) - 1):
                        cv2.line(frame, draw_pts[i], draw_pts[i + 1], BLUE, 2)
                    for pt in draw_pts[1:-1]:
                        cv2.circle(frame, pt, 5, BLUE, -1)
                    cv2.circle(frame, smooth_end, 8, BLUE, -1)

                if last_committed_target is not None:
                    cx, cy  = last_committed_target
                    color   = GREEN if in_critical_zone(cx) else (100, 100, 100)
                    cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 24, 2)

                cv2.putText(frame, f"FPS: {fps_display:.1f}",
                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)
                if filtered_vel is not None:
                    spd = math.hypot(filtered_vel[0], filtered_vel[1])
                    state_lbl = "STILL" if still_count >= STILL_CONFIRM_FRAMES else "MOVING"
                    cv2.putText(frame, f"|v|={spd:.1f}px  still={still_count}  {state_lbl}",
                                (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.45, WHITE, 1)
                if missing_count > 0:
                    cv2.putText(frame, f"missing={missing_count}",
                                (10, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.45, GREEN, 1)

                cv2.imshow("RoboMallet", frame)
                if cv2.waitKey(1) == 27:
                    break

    finally:
        ipc.close()
        cam.release()
        if not HEADLESS:
            cv2.destroyAllWindows()
        print("pysvc shut down.", flush=True)


if __name__ == "__main__":
    main()
