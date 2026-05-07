import cv2
import math
import numpy as np
import time
import glob
import sys
from scipy.interpolate import RBFInterpolator
import atexit
import signal
from ipc import Publisher, KIND_DEFEND, KIND_IGNORE, KIND_NO_TRACK

# ==============================================================================
# CAMERA + FRAME SETTINGS
# ==============================================================================
TARGET_FPS   = 100
FRAME_WIDTH  = 1920
FRAME_HEIGHT = 1080

# ==============================================================================
# OUTPUT dimensions — PRE-rotation (landscape).
# After ROTATE_90_COUNTERCLOCKWISE: OUTPUT_H wide x OUTPUT_W tall (portrait).
# ==============================================================================
OUTPUT_W, OUTPUT_H = 1020, 800

# ==============================================================================
# SRC_POINTS — paste from calibrate_roi.py
# Order: TL, T-mid, TR, R-mid, BR, B-mid, BL, L-mid (clockwise)
# ==============================================================================
SRC_POINTS = [
    (197.0, 41.0),   # TL
    (556.0, 43.0),   # T-mid
    (937.0, 50.0),   # TR
    (929.0, 360.0),   # R-mid
    (914.0, 653.0),   # BR
    (536.0, 649.0),   # B-mid
    (195.0, 640.0),   # BL
    (192.0, 334.0),   # L-mid
]

# ==============================================================================
# CALIBRATION OUTPUTS — paste from calibrate_roi.py
# All coords are in PREVIEW (rotated) pixel space — same space as everything below.
# ==============================================================================
MALLET_ORIGIN_PX = (397, 882)         # mallet 0,0 position in pixels
CRIT_TL_PX       = (215, 750)         # critical zone top-left
CRIT_BR_PX       = (586, 956)         # critical zone bottom-right

# Real-world conversion derived from a 55 x 35 cm critical zone
PX_PER_CM_X = 6.7455
PX_PER_CM_Y = 5.8857

_DST_POINTS = np.array([
    [0,              0             ],
    [OUTPUT_W // 2,  0             ],
    [OUTPUT_W - 1,   0             ],
    [OUTPUT_W - 1,   OUTPUT_H // 2 ],
    [OUTPUT_W - 1,   OUTPUT_H - 1  ],
    [OUTPUT_W // 2,  OUTPUT_H - 1  ],
    [0,              OUTPUT_H - 1  ],
    [0,              OUTPUT_H // 2 ],
], dtype=np.float64)


def _build_remap():
    src   = np.array(SRC_POINTS, dtype=np.float64)
    rbf_x = RBFInterpolator(_DST_POINTS, src[:, 0], kernel="thin_plate_spline")
    rbf_y = RBFInterpolator(_DST_POINTS, src[:, 1], kernel="thin_plate_spline")
    gx, gy = np.meshgrid(np.arange(OUTPUT_W), np.arange(OUTPUT_H))
    grid   = np.stack([gx.ravel(), gy.ravel()], axis=1).astype(np.float64)
    map_x  = rbf_x(grid).reshape(OUTPUT_H, OUTPUT_W).astype(np.float32)
    map_y  = rbf_y(grid).reshape(OUTPUT_H, OUTPUT_W).astype(np.float32)
    return map_x, map_y


print("[INFO] Building TPS remap...", flush=True)
_MAP_X, _MAP_Y = _build_remap()
print("[INFO] TPS remap ready.", flush=True)


# ==============================================================================
# COORDINATE CONVERSION
# Pixel space → centimeters, with mallet origin as 0,0.
# X grows right, Y grows toward the goal (downward in image).
# ==============================================================================
def px_to_cm(px_x, px_y):
    """Convert remapped+rotated pixel coords to mallet-relative cm coords."""
    cm_x = (px_x - MALLET_ORIGIN_PX[0]) / PX_PER_CM_X
    cm_y = (px_y - MALLET_ORIGIN_PX[1]) / PX_PER_CM_Y
    return round(cm_x, 1), round(cm_y, 1)


def in_critical_zone(px_x, px_y):
    """True if the predicted bottom-target X falls within the mallet's defend range.
    Y is not checked: trace_path_to_bottom always returns y = h-1 (frame bottom),
    which is below CRIT_BR_PX[1] and would permanently fail the y-check."""
    return CRIT_TL_PX[0] <= px_x <= CRIT_BR_PX[0]


# ==============================================================================
# TRACKING SETTINGS
# ==============================================================================
MIN_AREA                 = 300
MAX_JUMP_PX              = 400
RELAX_JUMP_AFTER_MISSING = 5
MAX_MISSING_FRAMES       = 15
STILL_THRESH_PX          = 2.0
STILL_CONFIRM_FRAMES     = 1
DIR_ARROW_LEN            = 120
UPDATE_INTERVAL          = 1.0
MOVE_START_THRESH_PX     = 1
TRACK_PAUSE_ON_MOVE_SEC  = 0.0
MAX_BOUNCES              = 1
MIN_SPEED_PX             = 8.0

# ==============================================================================
# TARGET OUTPUT — time-window median averaging
# ==============================================================================
BUFFER_WINDOW    = 0.4    # seconds — averaging window
PRINT_INTERVAL   = 0.5    # seconds — minimum time between prints
TARGET_DEADBAND  = 40     # px — suppress print if hasn't moved this much
DISPLAY_EMA_ALPHA = 0.12  # smoothing for the on-screen blue line

# ==============================================================================
# COLORS
# ==============================================================================
GREEN  = (0, 255, 0)
BLUE   = (255, 0, 0)
RED    = (0, 0, 255)
WHITE  = (255, 255, 255)
ORANGE = (0, 200, 255)
CYAN   = (255, 200, 0)

# ==============================================================================
# HSV GREEN MASK
# ==============================================================================
MORPH_KERNEL = np.ones((5, 5), np.uint8)
HSV_LOWER    = np.array([35,  60,  60], dtype=np.uint8)
HSV_UPPER    = np.array([85, 255, 255], dtype=np.uint8)


# ==============================================================================
# CAMERA
# ==============================================================================
def open_camera():
    if sys.platform == "darwin":
        print("macOS detected — using VideoCapture(0)", flush=True)
        return cv2.VideoCapture(0)
    devices = sorted(glob.glob("/dev/video*"))
    print(f"Found: {devices}", flush=True)
    for dev in devices:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"Camera at {dev}", flush=True)
                return cap
            cap.release()
    raise RuntimeError(f"No camera found. Tried: {devices}")

publisher = Publisher()

def _cleanup(*_):
    publisher.close()
    cap.release()
    cv2.destroyAllWindows()
    sys.exit(0)

atexit.register(publisher.close)
signal.signal(signal.SIGINT,  _cleanup)
signal.signal(signal.SIGTERM, _cleanup)

# ==============================================================================
# VELOCITY CONVERSION
# ==============================================================================
def kalman_vel_to_cm_s(vx_pxpf, vy_pxpf, fps):
    if fps <= 0:
        fps = TARGET_FPS
    return (vx_pxpf * fps / PX_PER_CM_X,
            vy_pxpf * fps / PX_PER_CM_Y)

def configure_camera(cap):
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          TARGET_FPS)
    w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or TARGET_FPS
    print(f"[INFO] Camera {w}x{h} @ {fps} fps", flush=True)
    return w, h, fps


# ==============================================================================
# IMAGE PROCESSING
# ==============================================================================
def preprocess_frame(raw):
    remapped = cv2.remap(raw, _MAP_X, _MAP_Y, cv2.INTER_LINEAR)
    rotated  = cv2.rotate(remapped, cv2.ROTATE_90_CLOCKWISE)
    return rotated


def green_mask_hsv(frame_bgr):
    hsv  = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  MORPH_KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=2)
    return mask


def detect_green_puck_center(frame_bgr):
    fh, fw = frame_bgr.shape[:2]
    mask   = green_mask_hsv(frame_bgr)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < MIN_AREA:
        return None
    M = cv2.moments(contour)
    if M["m00"] == 0:
        return None
    cx = int(np.clip(M["m10"] / M["m00"], 0, fw - 1))
    cy = int(np.clip(M["m01"] / M["m00"], 0, fh - 1))
    return (cx, cy)


# ==============================================================================
# GEOMETRY / PHYSICS
# ==============================================================================
def normalize(vx, vy, eps=1e-9):
    mag = math.hypot(vx, vy)
    return (vx / mag, vy / mag) if mag > eps else (0.0, 0.0)


def point_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def reflect(v, wall):
    vx, vy = float(v[0]), float(v[1])
    if wall in ("left", "right"):  return (-vx,  vy)
    if wall in ("top",  "bottom"): return ( vx, -vy)
    return vx, vy


def first_border_intersection(p0, v, w, h):
    x0, y0     = float(p0[0]), float(p0[1])
    vx, vy     = float(v[0]),  float(v[1])
    eps        = 1e-9
    candidates = []

    if abs(vx) > eps:
        for t, wall in [((0 - x0) / vx, "left"), (((w-1) - x0) / vx, "right")]:
            if t > 0:
                y = y0 + t * vy
                if 0 <= y <= h - 1:
                    candidates.append((t, (0 if wall == "left" else w-1, int(round(y))), wall))

    if abs(vy) > eps:
        for t, wall in [((0 - y0) / vy, "top"), (((h-1) - y0) / vy, "bottom")]:
            if t > 0:
                x = x0 + t * vx
                if 0 <= x <= w - 1:
                    candidates.append((t, (int(round(x)), 0 if wall == "top" else h-1), wall))

    if not candidates:
        return None, None
    candidates.sort(key=lambda c: c[0])
    return candidates[0][1], candidates[0][2]


def trace_path_to_bottom(start_pt, unit_v, w, h, max_bounces=MAX_BOUNCES):
    points     = [(int(np.clip(round(start_pt[0]), 0, w-1)),
                   int(np.clip(round(start_pt[1]), 0, h-1)))]
    current_pt = (float(start_pt[0]), float(start_pt[1]))
    current_v  = (float(unit_v[0]),   float(unit_v[1]))
    eps        = 1e-3

    for _ in range(max_bounces + 1):
        hit_pt, hit_wall = first_border_intersection(current_pt, current_v, w, h)
        if hit_pt is None:
            return None, None
        hit_pt = (int(np.clip(hit_pt[0], 0, w-1)), int(np.clip(hit_pt[1], 0, h-1)))
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
# KALMAN
# ==============================================================================
def create_kalman():
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix   = np.eye(2, 4, dtype=np.float32)
    kf.transitionMatrix    = np.array(
        [[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
    kf.processNoiseCov     = np.diag([1e-2, 1e-2, 0.1, 0.1]).astype(np.float32)
    kf.measurementNoiseCov = (5e-2 * np.eye(2)).astype(np.float32)
    kf.errorCovPost        = np.eye(4, dtype=np.float32)
    return kf


# ==============================================================================
# DRAWING — calibration overlays
# ==============================================================================
def draw_calibration_overlay(frame):
    """Draws mallet origin + critical zone on every frame for visual reference."""
    # Critical zone rectangle (semi-transparent fill)
    overlay = frame.copy()
    cv2.rectangle(overlay, CRIT_TL_PX, CRIT_BR_PX, CYAN, -1)
    cv2.addWeighted(overlay, 0.10, frame, 0.90, 0, frame)
    cv2.rectangle(frame, CRIT_TL_PX, CRIT_BR_PX, CYAN, 2)
    cv2.putText(frame, "CRITICAL ZONE",
                (CRIT_TL_PX[0] + 6, CRIT_TL_PX[1] + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, CYAN, 1)

    # Mallet origin marker
    mx, my = MALLET_ORIGIN_PX
    cv2.drawMarker(frame, (mx, my), ORANGE, cv2.MARKER_CROSS, 30, 2)
    cv2.circle(frame, (mx, my), 6, ORANGE, -1)
    cv2.putText(frame, "0,0", (mx + 10, my - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, ORANGE, 1)


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    tracking_paused_until = 0.0
    was_moving            = False

    cap = open_camera()
    configure_camera(cap)

    kalman             = create_kalman()
    kalman_initialized = False

    frame_count     = 0
    fps_display     = 0.0
    fps_timer_start = time.time()
    missing_count   = 0
    still_count     = 0

    prediction_buffer     = []
    last_print_time       = 0.0
    last_committed_target = None     # last printed target (px tuple)

    display_ema        = None
    last_path_points   = []
    last_bottom_target = None

    cv2.namedWindow("RoboMallet", cv2.WINDOW_NORMAL)
    window_sized = False

    print(f"[INFO] Mallet origin (px): {MALLET_ORIGIN_PX}", flush=True)
    print(f"[INFO] Critical zone (px): {CRIT_TL_PX} → {CRIT_BR_PX}", flush=True)
    print(f"[INFO] PX_PER_CM: x={PX_PER_CM_X:.3f}  y={PX_PER_CM_Y:.3f}", flush=True)

    while True:
        ret, raw_frame = cap.read()
        if not ret:
            break

        frame = preprocess_frame(raw_frame)
        h, w  = frame.shape[:2]

        if not window_sized:
            cv2.resizeWindow("RoboMallet", w, h)
            window_sized = True

        center = detect_green_puck_center(frame)
        now    = time.time()

        if missing_count >= MAX_MISSING_FRAMES and kalman_initialized:
            print(f"[INFO] Lost track after {missing_count} frames", flush=True)
            kalman, kalman_initialized, still_count = create_kalman(), False, 0
            prediction_buffer.clear()
            display_ema = None
            publisher.publish(KIND_NO_TRACK)
            continue

        predicted      = kalman.predict()
        pred_x, pred_y = float(predicted[0][0]), float(predicted[1][0])

        accepted_measurement = None
        if center is not None:
            if not kalman_initialized:
                kalman.statePost = np.array(
                    [[np.float32(center[0])], [np.float32(center[1])],
                     [0.], [0.]], np.float32)
                kalman_initialized, accepted_measurement, missing_count = True, center, 0
            else:
                dx, dy = center[0] - pred_x, center[1] - pred_y
                ej = MAX_JUMP_PX * (3 if missing_count >= RELAX_JUMP_AFTER_MISSING else 1)
                if dx*dx + dy*dy <= ej*ej:
                    accepted_measurement, missing_count = center, 0
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
                int(np.clip(round(float(kalman.statePost[0][0])), 0, w-1)),
                int(np.clip(round(float(kalman.statePost[1][0])), 0, h-1)),
            )
            filtered_vel = (float(kalman.statePost[2][0]), float(kalman.statePost[3][0]))

        # ── Border + goal highlight ────────────────────────────────────────
        cv2.rectangle(frame, (0, 0), (w-1, h-1), WHITE, 1)
        cv2.line(frame, (0, h-1), (w-1, h-1), ORANGE, 3)

        # Draw calibration overlays (mallet origin + critical zone)
        draw_calibration_overlay(frame)

        have_fresh_track = (kalman_initialized and filtered_pos is not None
                            and filtered_vel is not None and accepted_measurement is not None)

        if have_fresh_track:
            vx, vy     = filtered_vel
            moving_now = abs(vx) > MOVE_START_THRESH_PX or abs(vy) > MOVE_START_THRESH_PX

            if moving_now and not was_moving:
                tracking_paused_until = now + TRACK_PAUSE_ON_MOVE_SEC
            was_moving = moving_now

            still_count = (still_count + 1) if (abs(vx) <= STILL_THRESH_PX
                                                  and abs(vy) <= STILL_THRESH_PX) else 0

            if now >= tracking_paused_until and still_count < STILL_CONFIRM_FRAMES:
                ux, uy   = normalize(vx, vy)
                speed_px = math.hypot(vx, vy)

                dir_tip = (
                    int(np.clip(round(filtered_pos[0] + DIR_ARROW_LEN * ux), 0, w-1)),
                    int(np.clip(round(filtered_pos[1] + DIR_ARROW_LEN * uy), 0, h-1)),
                )

                if speed_px >= MIN_SPEED_PX and (abs(ux) > 0.0 or abs(uy) > 0.0):
                    path_pts, bot_tgt = trace_path_to_bottom(
                        filtered_pos, (ux, uy), w, h, MAX_BOUNCES)

                    if path_pts is not None and bot_tgt is not None:
                        last_path_points   = path_pts
                        last_bottom_target = bot_tgt

                        prediction_buffer.append((float(bot_tgt[0]),
                                                   float(bot_tgt[1]),
                                                   now))

                        if display_ema is None:
                            display_ema = (float(bot_tgt[0]), float(bot_tgt[1]))
                        else:
                            display_ema = (
                                DISPLAY_EMA_ALPHA * bot_tgt[0] + (1.0 - DISPLAY_EMA_ALPHA) * display_ema[0],
                                DISPLAY_EMA_ALPHA * bot_tgt[1] + (1.0 - DISPLAY_EMA_ALPHA) * display_ema[1],
                            )

                else:
                    prediction_buffer.clear()
                    display_ema = None

        # Purge old buffer entries
        cutoff = now - BUFFER_WINDOW
        prediction_buffer = [e for e in prediction_buffer if e[2] >= cutoff]

        # ── Print median once every PRINT_INTERVAL ─────────────────────────
        if (now - last_print_time >= PRINT_INTERVAL
                and len(prediction_buffer) >= 3):

            xs = [e[0] for e in prediction_buffer]
            ys = [e[1] for e in prediction_buffer]
            median_x = int(round(float(np.median(xs))))
            median_y = int(round(float(np.median(ys))))
            median_pt = (median_x, median_y)

            in_zone = in_critical_zone(median_x, median_y)
            cm_x, cm_y = px_to_cm(median_x, median_y)

            if (last_committed_target is None or
                    point_distance(median_pt, last_committed_target) > TARGET_DEADBAND):
                tag = "DEFEND" if in_zone else "ignore"

                print(f"[TARGET] px=({median_x},{median_y})  "
                    f"cm=({cm_x:+.1f},{cm_y:+.1f})  zone={tag}",
                flush=True)

            # NEW: publish to shared memory
            if filtered_vel is not None:
                vx_cms, vy_cms = kalman_vel_to_cm_s(
                    filtered_vel[0], filtered_vel[1], fps_display)
            else:
                vx_cms, vy_cms = 0.0, 0.0

            publisher.publish(
                KIND_DEFEND if in_zone else KIND_IGNORE,
                x_mm=cm_x * 10, y_mm=cm_y * 10,
                vx_mm_s=vx_cms * 10, vy_mm_s=vy_cms * 10,
                confidence=1.0,
            )

            last_committed_target = median_pt
            last_print_time = now

        # ── Draw puck ──────────────────────────────────────────────────────
        if filtered_pos is not None:
            cv2.circle(frame, filtered_pos, 7, RED, -1)
            cm_x, cm_y = px_to_cm(*filtered_pos)
            cv2.putText(frame,
                        f"{filtered_pos}  ({cm_x:+.1f},{cm_y:+.1f})cm",
                        (filtered_pos[0]+10, filtered_pos[1]+20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, RED, 1)

        # Direction arrow
        if filtered_pos is not None and dir_tip is not None:
            cv2.arrowedLine(frame, filtered_pos, dir_tip, GREEN, 2, tipLength=0.25)

        # Bounce path with smoothed endpoint
        if filtered_pos is not None and display_ema is not None and len(last_path_points) >= 2:
            smooth_end = (int(round(display_ema[0])), int(round(display_ema[1])))
            draw_pts = list(last_path_points[:-1]) + [smooth_end]

            for i in range(len(draw_pts) - 1):
                cv2.line(frame, draw_pts[i], draw_pts[i+1], BLUE, 2)
            for pt in draw_pts[1:-1]:
                cv2.circle(frame, pt, 5, BLUE, -1)

            cv2.circle(frame, smooth_end, 8, BLUE, -1)
            cm_x, cm_y = px_to_cm(*smooth_end)
            cv2.putText(frame, f"({cm_x:+.1f},{cm_y:+.1f})cm",
                        (smooth_end[0]+8, smooth_end[1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, BLUE, 1)

        # Last committed target crosshair
        if last_committed_target is not None:
            cx, cy = last_committed_target
            in_zone = in_critical_zone(cx, cy)
            color   = (0, 255, 0) if in_zone else (100, 100, 100)
            cv2.drawMarker(frame, (cx, cy), color, cv2.MARKER_CROSS, 24, 2)
            cm_x, cm_y = px_to_cm(cx, cy)
            tag = "DEFEND" if in_zone else "ignore"
            cv2.putText(frame, f"CMD ({cm_x:+.1f},{cm_y:+.1f}) {tag}",
                        (cx + 8, cy - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # Buffer fill bar
        buf_count    = len(prediction_buffer)
        bar_max      = max(1, int(BUFFER_WINDOW * 60))
        bar_val      = min(buf_count, bar_max)
        bar_w, bar_h = 100, 8
        bar_x, bar_y = 10, h - 30
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), WHITE, 1)
        if bar_val > 0:
            fill  = int(bar_w * bar_val / bar_max)
            color = ORANGE if buf_count >= 3 else GREEN
            cv2.rectangle(frame, (bar_x, bar_y), (bar_x + fill, bar_y + bar_h), color, -1)
        cv2.putText(frame, f"buf={buf_count}",
                    (bar_x + bar_w + 4, bar_y + bar_h),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, WHITE, 1)

        # FPS
        frame_count += 1
        elapsed = now - fps_timer_start
        if elapsed >= UPDATE_INTERVAL:
            fps_display     = frame_count / elapsed
            frame_count     = 0
            fps_timer_start = now

        cv2.putText(frame, f"FPS: {fps_display:.1f}",
                    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)
        if missing_count > 0:
            cv2.putText(frame, f"missing={missing_count}",
                        (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, GREEN, 1)

        cv2.imshow("RoboMallet", frame)
        if cv2.waitKey(1) == 27:
            break

    print("\nStream terminated.")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()