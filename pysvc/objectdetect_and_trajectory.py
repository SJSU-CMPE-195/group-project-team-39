import cv2
import numpy as np
import time

# ── AI Feature Flags ──────────────────────────────────────────────────────────
# Each module is independent; flip any flag to False to revert to the original
# classical-CV behaviour for that component.
USE_KALMAN_TRACKER  = True   # Kalman filter: smooth position + velocity from state
USE_AI_TRAJECTORY   = True   # Polynomial fit: better trajectory for curved/spin paths
USE_YOLO_DETECTOR   = False  # Neural-net detection — set True after training a model
YOLO_MODEL_PATH     = "puck_model.pt"  # path to trained YOLOv8 weights (see yolo_detector.py)
# ─────────────────────────────────────────────────────────────────────────────

from kalman_tracker      import KalmanPuckTracker
from trajectory_predictor import TrajectoryPredictor
from yolo_detector       import PuckDetector

cap = cv2.VideoCapture(0)  # 0 default, 1 extra camera
cap.set(cv2.CAP_PROP_FPS, 120)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

# display fps
frame_count = 0
start_time = time.time()
fps_display = 0
UPDATE_INTERVAL = 1  # seconds

# Tracking / detection
MIN_AREA = 300  # ignore tiny green areas
MAX_JUMP_PX = 200  # reject sudden jumps (noise and outliers)

# If abs(dx) <= STILL_THRESH_PX AND abs(dy) <= STILL_THRESH_PX for STILL_CONFIRM_FRAMES, treat as STILL (no trajectory line)
STILL_THRESH_PX = 5
STILL_CONFIRM_FRAMES = 2

# Direction visual line
DIR_ARROW_LEN = 120    # px, shows direction arrow only

# print coordinates
PRINT_INTERVAL = 0.05  # seconds
last_print_t = 0.0

# tracking
missing_count = 0
MAX_MISSING_TO_FORCE_ACCEPT = 3
AREA_GAIN_THRESHOLD = 1.3

# Colors
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)

def green_mask_hsv(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    lower1 = np.array([35, 60, 60], dtype=np.uint8)
    upper1 = np.array([85, 255, 255], dtype=np.uint8)

    mask = cv2.inRange(hsv, lower1, upper1)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask

def detect_green_puck_center(frame_bgr):

    # Returns puck center (cx, cy) from largest green contour and returns the area of that largest contour

    h, w = frame_bgr.shape[:2]
    mask = green_mask_hsv(frame_bgr)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask, 0

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA:
        return None, mask, 0

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, mask, 0

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    cx = int(np.clip(cx, 0, w - 1))
    cy = int(np.clip(cy, 0, h - 1))

    return (cx, cy), mask, area

def first_border_intersection(p0, v, w, h):
    # From point p0 moving along direction v=(vx,vy), finds the first intersection with the frame borders (x=0, x=w-1, y=0, y=h-1).
   
    x0, y0 = float(p0[0]), float(p0[1])
    vx, vy = float(v[0]), float(v[1])
    eps = 1e-9
    candidates = []

    # Left wall x=0
    if abs(vx) > eps:
        t = (0 - x0) / vx
        if t > 0:
            y = y0 + t * vy
            if 0 <= y <= (h - 1):
                candidates.append((t, (0, int(round(y))), "left"))

    # Right wall x=w-1
    if abs(vx) > eps:
        t = ((w - 1) - x0) / vx
        if t > 0:
            y = y0 + t * vy
            if 0 <= y <= (h - 1):
                candidates.append((t, (w - 1, int(round(y))), "right"))

    # Top wall y=0
    if abs(vy) > eps:
        t = (0 - y0) / vy
        if t > 0:
            x = x0 + t * vx
            if 0 <= x <= (w - 1):
                candidates.append((t, (int(round(x)), 0), "top"))

    # Bottom wall y=h-1
    if abs(vy) > eps:
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

def reflect(v, wall):
    vx, vy = float(v[0]), float(v[1])
    if wall in ("left", "right"):
        return (-vx, vy)
    if wall in ("top", "bottom"):
        return (vx, -vy)
    return (vx, vy)

def normalize(vx, vy, eps=1e-9):
    mag = (vx * vx + vy * vy) ** 0.5
    if mag < eps:
        return 0.0, 0.0
    return vx / mag, vy / mag

# Track last two points
p0 = None
p1 = None

# Track the area of the currently tracked object
tracked_area = None

# Stillness counter
still_count = 0

# ── AI component init ─────────────────────────────────────────────────────────
_detector  = PuckDetector(
    model_path=YOLO_MODEL_PATH if USE_YOLO_DETECTOR else None,
    min_area=MIN_AREA,
)
_kalman    = KalmanPuckTracker(process_noise=1.0, meas_noise=10.0) if USE_KALMAN_TRACKER else None
_predictor = TrajectoryPredictor(history_len=12, poly_degree=2)    if USE_AI_TRAJECTORY  else None
# ─────────────────────────────────────────────────────────────────────────────

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]

    # ── Detection ─────────────────────────────────────────────────────────────
    center, area, _det_mode = _detector.detect(frame)

    # Update p0 and p1 using consecutive accepted detections
    if center is not None:
        missing_count = 0

        if p1 is None:
            p1 = center
            tracked_area = area
            if _kalman:
                _kalman.init(*center)
            if _predictor:
                _predictor.update(*center)
        else:
            dx = center[0] - p1[0]
            dy = center[1] - p1[1]
            dist2 = (dx * dx + dy * dy)

            cond_normal_accept = (dist2 <= (MAX_JUMP_PX * MAX_JUMP_PX))

            if tracked_area is None:
                cond_area_bigger = True
            else:
                cond_area_bigger = (area > tracked_area * AREA_GAIN_THRESHOLD)

            cond_missing_relax = (missing_count >= MAX_MISSING_TO_FORCE_ACCEPT)

            if cond_normal_accept or cond_area_bigger or cond_missing_relax:
                p0 = p1

                # ── Kalman: update with raw detection, use smoothed position ──
                if _kalman:
                    smooth = _kalman.update(*center)
                    p1 = smooth if smooth else center
                else:
                    p1 = center

                tracked_area = area

                if _predictor:
                    _predictor.update(*p1)
    else:
        missing_count += 1
        # Kalman advances its state during brief occlusions
        if _kalman:
            predicted = _kalman.predict()
            if predicted and p1 is not None:
                p1 = predicted

    # Borders
    cv2.line(frame, (0, 0), (w - 1, 0), (255, 255, 255), 1)           # top
    cv2.line(frame, (0, h - 1), (w - 1, h - 1), (255, 255, 255), 1)   # bottom
    cv2.line(frame, (0, 0), (0, h - 1), (255, 255, 255), 1)           # left
    cv2.line(frame, (w - 1, 0), (w - 1, h - 1), (255, 255, 255), 1)   # right

    # Intercept and bounce using only direction
    intercept_pt = None
    bounce_end = None
    predicted_pt = None
    predicted_label = None  # "INTERCEPT", "BOUNCE", or "STILL"
    dir_tip = None # arrow tip for direction

    if p0 is not None and p1 is not None:
        # ── Velocity estimate ─────────────────────────────────────────────────
        # Kalman gives a smoothed (vx, vy) from its state — far less noisy than
        # the raw p1-p0 pixel difference.
        if _kalman and _kalman.initialized:
            vx_raw, vy_raw = _kalman.velocity
        else:
            vx_raw = p1[0] - p0[0]
            vy_raw = p1[1] - p0[1]

        # Override with polynomial derivative when predictor has enough history
        if _predictor and _predictor.n >= 3:
            vx_raw, vy_raw = _predictor.predicted_velocity()

        # Count consecutive "small motion" frames to avoid jitter triggering BOUNCE
        if abs(vx_raw) <= STILL_THRESH_PX and abs(vy_raw) <= STILL_THRESH_PX:
            still_count += 1
        else:
            still_count = 0

        stationary = (still_count >= STILL_CONFIRM_FRAMES)

        if stationary:
            # STILL: print STILL and draw no trajectory
            predicted_label = "STILL"
            predicted_pt = p1  # prints x/y, but no trajectory is drawn
            dir_tip = None
            intercept_pt = None
            bounce_end = None
        else:
            # Normalize direction and draw a fixed-length arrow from p1
            ux, uy = normalize(float(vx_raw), float(vy_raw))
            tip_x = int(round(p1[0] + DIR_ARROW_LEN * ux))
            tip_y = int(round(p1[1] + DIR_ARROW_LEN * uy))
            dir_tip = (int(np.clip(tip_x, 0, w - 1)), int(np.clip(tip_y, 0, h - 1)))

            # ── Trajectory / intercept prediction ─────────────────────────────
            # With enough history the polynomial predictor walks along the curve
            # to find the wall hit (handles spin/arc).  Falls back to linear ray
            # when history is short.
            if _predictor and _predictor.n >= 3:
                intercept_pt, hit_wall = _predictor.predict_border_hit(w, h)
            else:
                v_dir = (ux, uy)
                intercept_pt, hit_wall = first_border_intersection(p1, v_dir, w, h)

            predicted_pt = None
            predicted_label = None

            if intercept_pt is not None:
                predicted_pt = intercept_pt
                predicted_label = "INTERCEPT"

                # Bounce: post-bounce history isn't available yet, so always use
                # linear reflection regardless of AI mode.
                if hit_wall in ("top", "bottom"):
                    v_dir = (ux, uy)
                    v_out = reflect(v_dir, hit_wall)
                    bounce_end, _ = first_border_intersection(intercept_pt, v_out, w, h)
                    if bounce_end is not None:
                        predicted_pt = bounce_end
                        predicted_label = "BOUNCE"

        # Printing (labelled)
        now_t = time.time()
        if predicted_pt is not None and (PRINT_INTERVAL <= 0 or (now_t - last_print_t) >= PRINT_INTERVAL):
            print(f"[PREDICTED {predicted_label}] x={predicted_pt[0]} y={predicted_pt[1]}", flush=True)
            last_print_t = now_t

    # Drawing p0/p1
    if p1 is not None:
        cv2.circle(frame, p1, 6, GREEN, -1)
        cv2.putText(frame, f"p1 {p1}", (p1[0] + 10, p1[1] + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

    if p0 is not None:
        cv2.circle(frame, p0, 6, GREEN, -1)
        cv2.putText(frame, f"p0 {p0}", (p0[0] + 10, p0[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

    # Drawing direction arrow only (green)
    if p1 is not None and dir_tip is not None:
        cv2.arrowedLine(frame, p1, dir_tip, GREEN, 2, tipLength=0.25)
        cv2.putText(frame, "dir", (dir_tip[0] + 10, dir_tip[1] + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

    # Drawing intercept + bounce (blue)
    if p1 is not None and intercept_pt is not None:
        cv2.line(frame, p1, intercept_pt, BLUE, 2)
        cv2.circle(frame, intercept_pt, 10, BLUE, -1)
        cv2.putText(frame, f"intercept {intercept_pt}", (intercept_pt[0] + 10, intercept_pt[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2)

    if intercept_pt is not None and bounce_end is not None:
        cv2.line(frame, intercept_pt, bounce_end, BLUE, 2)
        cv2.circle(frame, bounce_end, 8, BLUE, -1)
        cv2.putText(frame, f"bounce {bounce_end}", (bounce_end[0] + 10, bounce_end[1] + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2)

    # FPS
    frame_count += 1
    now = time.time()
    if now - start_time >= UPDATE_INTERVAL:
        fps_display = frame_count / (now - start_time)
        frame_count = 0
        start_time = now

    cv2.putText(frame, f"FPS: {fps_display:.1f}", (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1, GREEN, 2)

    # AI mode indicator
    ai_label = f"[{_detector.mode}]"
    if _kalman:  ai_label += "[KF]"
    if _predictor: ai_label += "[POLY]"
    cv2.putText(frame, ai_label, (20, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 0), 1)

    cv2.imshow("RoboMallet", frame)

    if cv2.waitKey(1) == 27:  # ESC
        break

cap.release()
cv2.destroyAllWindows()