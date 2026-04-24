import cv2
import math
import numpy as np
import time
import glob
import sys

# ==============================================================================
# CAMERA + FRAME SETTINGS
# ==============================================================================
TARGET_FPS = 100
FRAME_WIDTH = 2560
FRAME_HEIGHT = 720

# Run calibrate_roi_apalis.py to get these values
ROI_X, ROI_Y, ROI_W, ROI_H = 425, 21, 779, 683
OUTPUT_W, OUTPUT_H = 600, 765

# ==============================================================================
# TRACKING / PREDICTION SETTINGS
# ==============================================================================
MIN_AREA = 300
MAX_JUMP_PX = 400
RELAX_JUMP_AFTER_MISSING = 5
MAX_MISSING_FRAMES = 15

STILL_THRESH_PX = 2.0
STILL_CONFIRM_FRAMES = 5
DIR_ARROW_LEN = 120

INTERCEPT_CHANGE_THRESH_PX = 20
INTERCEPT_ARRIVAL_TOLERANCE_PX = 12

UPDATE_INTERVAL = 1.0

# ==============================================================================
# COLORS
# ==============================================================================
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)

# ==============================================================================
# HSV GREEN MASK SETTINGS
# ==============================================================================
MORPH_KERNEL = np.ones((5, 5), np.uint8)
HSV_LOWER = np.array([35, 60, 60], dtype=np.uint8)
HSV_UPPER = np.array([85, 255, 255], dtype=np.uint8)


MOVE_START_THRESH_PX = 10
TRACK_PAUSE_ON_MOVE_SEC = 0.4




# ==============================================================================
# CAMERA
# ==============================================================================
def open_camera():
    if sys.platform == "darwin":
        print("macOS detected. Using default cv2.VideoCapture(0)...", flush=True)
        return cv2.VideoCapture(0)

    devices = sorted(glob.glob("/dev/video*"))
    print(f"Found video devices: {devices}", flush=True)

    for dev in devices:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"Camera found at {dev}", flush=True)
                return cap
            cap.release()

    raise RuntimeError(f"No working camera found. Tried: {devices}")


def configure_camera(cap):
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS) or TARGET_FPS

    print(f"[INFO] Camera fully opened at: {actual_w}x{actual_h}", flush=True)
    return actual_w, actual_h, actual_fps


# ==============================================================================
# IMAGE PROCESSING
# ==============================================================================
def preprocess_frame(frame):
    frame = frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.resize(frame, (OUTPUT_W, OUTPUT_H))
    return frame


def green_mask_hsv(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=2)
    return mask


def detect_green_puck_center(frame_bgr):
    h, w = frame_bgr.shape[:2]
    mask = green_mask_hsv(frame_bgr)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < MIN_AREA:
        return None

    m = cv2.moments(contour)
    if m["m00"] == 0:
        return None

    cx = int(m["m10"] / m["m00"])
    cy = int(m["m01"] / m["m00"])

    cx = int(np.clip(cx, 0, w - 1))
    cy = int(np.clip(cy, 0, h - 1))
    return (cx, cy)


# ==============================================================================
# GEOMETRY / PHYSICS
# ==============================================================================
def normalize(vx, vy, eps=1e-9):
    mag = math.hypot(vx, vy)
    if mag < eps:
        return 0.0, 0.0
    return vx / mag, vy / mag


def point_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def reflect(v, wall):
    vx, vy = float(v[0]), float(v[1])

    if wall in ("left", "right"):
        return -vx, vy
    if wall in ("top", "bottom"):
        return vx, -vy
    return vx, vy


def first_border_intersection(p0, v, w, h):
    x0, y0 = float(p0[0]), float(p0[1])
    vx, vy = float(v[0]), float(v[1])

    eps = 1e-9
    candidates = []

    if abs(vx) > eps:
        t = (0 - x0) / vx
        if t > 0:
            y = y0 + t * vy
            if 0 <= y <= (h - 1):
                candidates.append((t, (0, int(round(y))), "left"))

        t = ((w - 1) - x0) / vx
        if t > 0:
            y = y0 + t * vy
            if 0 <= y <= (h - 1):
                candidates.append((t, (w - 1, int(round(y))), "right"))

    if abs(vy) > eps:
        t = (0 - y0) / vy
        if t > 0:
            x = x0 + t * vx
            if 0 <= x <= (w - 1):
                candidates.append((t, (int(round(x)), 0), "top"))

        t = ((h - 1) - y0) / vy
        if t > 0:
            x = x0 + t * vx
            if 0 <= x <= (w - 1):
                candidates.append((t, (int(round(x)), h - 1), "bottom"))

    if not candidates:
        return None, None

    candidates.sort(key=lambda item: item[0])
    _, pt, wall = candidates[0]
    return pt, wall


def estimate_eta_seconds(current_pt, target_pt, velocity_xy, fps_now, fps_fallback):
    vx, vy = velocity_xy
    speed_px_per_frame = math.hypot(vx, vy)
    if speed_px_per_frame < 1e-6:
        return None

    fps_used = fps_now if fps_now > 1.0 else fps_fallback
    if fps_used <= 0:
        return None

    distance_px = point_distance(current_pt, target_pt)
    speed_px_per_sec = speed_px_per_frame * fps_used
    return distance_px / speed_px_per_sec


# ==============================================================================
# KALMAN
# ==============================================================================
def create_kalman():
    kf = cv2.KalmanFilter(4, 2)
    kf.measurementMatrix = np.eye(2, 4, dtype=np.float32)
    kf.transitionMatrix = np.array(
        [
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ],
        dtype=np.float32,
    )
    kf.processNoiseCov = np.diag([1e-2, 1e-2, 0.1, 0.1]).astype(np.float32)
    kf.measurementNoiseCov = (5e-2 * np.eye(2)).astype(np.float32)
    kf.errorCovPost = np.eye(4, dtype=np.float32)
    return kf


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    
    tracking_paused_until = 0.0
    was_moving = False

    cap = open_camera()
    _, _, actual_camera_fps = configure_camera(cap)

    kalman = create_kalman()
    kalman_initialized = False

    frame_count = 0
    fps_display = 0.0
    fps_timer_start = time.time()

    missing_count = 0
    still_count = 0

    last_intercept_pt = None
    last_bounce_end = None

    active_intercept_target = None
    active_intercept_started_at = None
    active_intercept_arrived = False

    cv2.namedWindow("RoboMallet Apalis", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("RoboMallet Apalis", OUTPUT_W, OUTPUT_H)

    while True:
        ret, raw_frame = cap.read()
        if not ret:
            break

        frame = preprocess_frame(raw_frame)
        h, w = frame.shape[:2]

        center = detect_green_puck_center(frame)

        if missing_count >= MAX_MISSING_FRAMES and kalman_initialized:
            print(f"[INFO] Lost track after {missing_count} frames", flush=True)
            kalman = create_kalman()
            kalman_initialized = False
            still_count = 0
            active_intercept_target = None
            active_intercept_started_at = None
            active_intercept_arrived = False

        predicted = kalman.predict()
        pred_x = float(predicted[0][0])
        pred_y = float(predicted[1][0])

        accepted_measurement = None

        if center is not None:
            if not kalman_initialized:
                kalman.statePost = np.array(
                    [
                        [np.float32(center[0])],
                        [np.float32(center[1])],
                        [0.0],
                        [0.0],
                    ],
                    dtype=np.float32,
                )
                kalman_initialized = True
                accepted_measurement = center
                missing_count = 0
            else:
                dx = center[0] - pred_x
                dy = center[1] - pred_y
                dist2 = dx * dx + dy * dy

                effective_jump = MAX_JUMP_PX
                if missing_count >= RELAX_JUMP_AFTER_MISSING:
                    effective_jump = MAX_JUMP_PX * 3

                if dist2 <= (effective_jump * effective_jump):
                    accepted_measurement = center
                    missing_count = 0
                else:
                    missing_count += 1
        else:
            missing_count += 1

        if kalman_initialized and accepted_measurement is not None:
            measurement = np.array(
                [
                    [np.float32(accepted_measurement[0])],
                    [np.float32(accepted_measurement[1])],
                ],
                dtype=np.float32,
            )
            kalman.correct(measurement)

        filtered_pos = None
        filtered_vel = None

        if kalman_initialized:
            filtered_x = float(kalman.statePost[0][0])
            filtered_y = float(kalman.statePost[1][0])
            filtered_vx = float(kalman.statePost[2][0])
            filtered_vy = float(kalman.statePost[3][0])

            filtered_pos = (
                int(np.clip(round(filtered_x), 0, w - 1)),
                int(np.clip(round(filtered_y), 0, h - 1)),
            )
            filtered_vel = (filtered_vx, filtered_vy)

        intercept_pt = last_intercept_pt
        bounce_end = last_bounce_end
        dir_tip = None

        # Debug border
        cv2.line(frame, (0, 0), (w - 1, 0), WHITE, 1)
        cv2.line(frame, (0, h - 1), (w - 1, h - 1), WHITE, 1)
        cv2.line(frame, (0, 0), (0, h - 1), WHITE, 1)
        cv2.line(frame, (w - 1, 0), (w - 1, h - 1), WHITE, 1)

        have_fresh_track = (
            kalman_initialized
            and filtered_pos is not None
            and filtered_vel is not None
            and accepted_measurement is not None
        )

        if have_fresh_track:
            vx, vy = filtered_vel

            moving_now = abs(vx) > MOVE_START_THRESH_PX or abs(vy) > MOVE_START_THRESH_PX

            # Detect movement start once, then pause intercept tracking for a short time
            if moving_now and not was_moving:
                tracking_paused_until = time.time() + TRACK_PAUSE_ON_MOVE_SEC
                active_intercept_target = None
                active_intercept_started_at = None
                active_intercept_arrived = False
                last_intercept_pt = None
                last_bounce_end = None

            was_moving = moving_now

            if abs(vx) <= STILL_THRESH_PX and abs(vy) <= STILL_THRESH_PX:
                still_count += 1
            else:
                still_count = 0

            # While paused, do not calculate intercepts and do not print intercept logs
            if time.time() < tracking_paused_until:
                intercept_pt = None
                bounce_end = None
                last_intercept_pt = None
                last_bounce_end = None

            elif still_count >= STILL_CONFIRM_FRAMES:
                intercept_pt = None
                bounce_end = None
                last_intercept_pt = None
                last_bounce_end = None

                active_intercept_target = None
                active_intercept_started_at = None
                active_intercept_arrived = False
            else:
                ux, uy = normalize(vx, vy)

                tip_x = int(round(filtered_pos[0] + DIR_ARROW_LEN * ux))
                tip_y = int(round(filtered_pos[1] + DIR_ARROW_LEN * uy))
                dir_tip = (
                    int(np.clip(tip_x, 0, w - 1)),
                    int(np.clip(tip_y, 0, h - 1)),
                )

                intercept_pt, hit_wall = first_border_intersection(filtered_pos, (ux, uy), w, h)
                bounce_end = None

                if intercept_pt is not None and hit_wall in ("left", "right"):
                    reflected_v = reflect((ux, uy), hit_wall)
                    bounce_end, _ = first_border_intersection(intercept_pt, reflected_v, w, h)

                last_intercept_pt = intercept_pt
                last_bounce_end = bounce_end

                if intercept_pt is not None:
                    is_new_target = (
                        active_intercept_target is None
                        or point_distance(intercept_pt, active_intercept_target) > INTERCEPT_CHANGE_THRESH_PX
                    )

                    if is_new_target:
                        eta_s = estimate_eta_seconds(
                            filtered_pos,
                            intercept_pt,
                            filtered_vel,
                            fps_display,
                            actual_camera_fps,
                        )

                        active_intercept_target = intercept_pt
                        active_intercept_started_at = time.time()
                        active_intercept_arrived = False

                        if eta_s is None:
                            print(f"[INTERCEPT START] target={intercept_pt}", flush=True)
                        else:
                            print(f"[INTERCEPT START] target={intercept_pt} eta={eta_s:.3f}s", flush=True)
                else:
                    active_intercept_target = None
                    active_intercept_started_at = None
                    active_intercept_arrived = False

        if (
            time.time() >= tracking_paused_until
            and filtered_pos is not None
            and active_intercept_target is not None
            and not active_intercept_arrived
        ):
            if point_distance(filtered_pos, active_intercept_target) <= INTERCEPT_ARRIVAL_TOLERANCE_PX:
                elapsed = None
                if active_intercept_started_at is not None:
                    elapsed = time.time() - active_intercept_started_at

                if elapsed is None:
                    print(
                        f"[INTERCEPT ARRIVED] target={active_intercept_target} puck={filtered_pos}",
                        flush=True,
                    )
                else:
                    print(
                        f"[INTERCEPT ARRIVED] target={active_intercept_target} puck={filtered_pos} elapsed={elapsed:.3f}s",
                        flush=True,
                    )

                active_intercept_arrived = True

        if filtered_pos is not None:
            cv2.circle(frame, filtered_pos, 7, RED, -1)
            cv2.putText(
                frame,
                f"kalman {filtered_pos}",
                (filtered_pos[0] + 10, filtered_pos[1] + 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                RED,
                1,
            )

        if filtered_pos is not None and dir_tip is not None:
            cv2.arrowedLine(frame, filtered_pos, dir_tip, GREEN, 2, tipLength=0.25)

        tracking_pause_active = time.time() < tracking_paused_until

        if not tracking_pause_active and filtered_pos is not None and intercept_pt is not None:
            cv2.line(frame, filtered_pos, intercept_pt, BLUE, 2)
            cv2.circle(frame, intercept_pt, 8, BLUE, -1)
            if bounce_end is None:
                cv2.putText(
                    frame,
                    f"{intercept_pt}",
                    (intercept_pt[0] + 5, intercept_pt[1] - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    BLUE,
                    1,
                )

        if not tracking_pause_active and intercept_pt is not None and bounce_end is not None:
            cv2.line(frame, intercept_pt, bounce_end, BLUE, 2)
            cv2.circle(frame, bounce_end, 6, BLUE, -1)
            cv2.putText(
                frame,
                f"{bounce_end}",
                (bounce_end[0] + 5, bounce_end[1] + 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                BLUE,
                1,
            )

        frame_count += 1
        now = time.time()
        if now - fps_timer_start >= UPDATE_INTERVAL:
            fps_display = frame_count / (now - fps_timer_start)
            frame_count = 0
            fps_timer_start = now

            # Console FPS / PUCK status print disabled
            # if filtered_pos is not None:
            #     puck_str = f"PUCK: X={filtered_pos[0]:04d}, Y={filtered_pos[1]:04d}"
            # else:
            #     puck_str = f"PUCK: LOST (Missing: {missing_count:02d})"
            #
            # print(f"Live FPS: {fps_display:.1f}  |  {puck_str}", flush=True)

        cv2.putText(
            frame,
            f"FPS: {fps_display:.1f}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            GREEN,
            2,
        )

        if missing_count > 0:
            cv2.putText(
                frame,
                f"missing={missing_count}",
                (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                GREEN,
                1,
            )

        cv2.imshow("RoboMallet Apalis", frame)

        if cv2.waitKey(1) == 27:
            break

    print("\n\nStream terminated.")
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()