# import urllib.request
# import sys
import cv2
import numpy as np
import time
import glob

def open_camera():
    devices = sorted(glob.glob('/dev/video*'))
    print(f"Found video devices: {devices}", flush=True)

    for dev in devices:
        c = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if c.isOpened():
            ret, _ = c.read()
            if ret:
                print(f"Camera found at {dev}", flush=True)
                return c
            c.release()

    raise RuntimeError(f"No working camera found. Tried: {devices}")

cap = open_camera()
cap.set(cv2.CAP_PROP_FPS, 120)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

# FPS tracking
frame_count = 0
start_time = time.time()
fps_display = 0
UPDATE_INTERVAL = 1  # seconds

# Tracking / detection
MIN_AREA = 300          # ignore tiny green areas
MAX_JUMP_PX = 200       # reject sudden jumps (noise and outliers)

# If abs(dx) <= STILL_THRESH_PX and abs(dy) <= STILL_THRESH_PX
# for STILL_CONFIRM_FRAMES, treat as STILL
STILL_THRESH_PX = 5
STILL_CONFIRM_FRAMES = 2

# Print coordinates
PRINT_INTERVAL = 0.05   # seconds
last_print_t = 0.0

# Tracking state
missing_count = 0
MAX_MISSING_TO_FORCE_ACCEPT = 3
AREA_GAIN_THRESHOLD = 1.3

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
    # Returns puck center (cx, cy) from the largest green contour
    # and returns the area of that contour
    h, w = frame_bgr.shape[:2]
    mask = green_mask_hsv(frame_bgr)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < MIN_AREA:
        return None, 0

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, 0

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])

    cx = int(np.clip(cx, 0, w - 1))
    cy = int(np.clip(cy, 0, h - 1))

    return (cx, cy), area

def first_border_intersection(p0, v, w, h):
    # From point p0 moving along direction v=(vx, vy),
    # find the first intersection with the frame borders
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

# Last two accepted points
p0 = None
p1 = None

# Area of the currently tracked object
tracked_area = None

# Consecutive low-motion frame count
still_count = 0

# Track disappearance state
prev_detected = False

print("Camera started.", flush=True)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from camera.", flush=True)
            break

        h, w = frame.shape[:2]

        # Detection returns center and contour area
        center, area = detect_green_puck_center(frame)

        # Update p0 and p1 using consecutive accepted detections
        if center is not None:
            cond_missing_relax = (missing_count >= MAX_MISSING_TO_FORCE_ACCEPT)
            missing_count = 0

            if p1 is None:
                p1 = center
                tracked_area = area
            else:
                dx = center[0] - p1[0]
                dy = center[1] - p1[1]
                dist2 = dx * dx + dy * dy

                cond_normal_accept = (dist2 <= (MAX_JUMP_PX * MAX_JUMP_PX))

                if tracked_area is None:
                    cond_area_bigger = True
                else:
                    cond_area_bigger = (area > tracked_area * AREA_GAIN_THRESHOLD)

                if cond_normal_accept or cond_area_bigger or cond_missing_relax:
                    p0 = p1
                    p1 = center
                    tracked_area = area
        else:
            missing_count += 1

            # Reset tracking when object is gone too long
            if missing_count > MAX_MISSING_TO_FORCE_ACCEPT:
                if prev_detected:
                    print("Object Disappeared", flush=True)

                p0 = None
                p1 = None
                tracked_area = None
                still_count = 0

        # Update detection state flag
        prev_detected = (p1 is not None)

        # Predicted results
        intercept_pt = None
        bounce_end = None
        predicted_pt = None
        predicted_label = None

        if p0 is not None and p1 is not None:
            vx_raw = p1[0] - p0[0]
            vy_raw = p1[1] - p0[1]

            # Count consecutive small-motion frames to avoid jitter
            if abs(vx_raw) <= STILL_THRESH_PX and abs(vy_raw) <= STILL_THRESH_PX:
                still_count += 1
            else:
                still_count = 0

            stationary = (still_count >= STILL_CONFIRM_FRAMES)

            if stationary:
                predicted_label = "STILL"
                predicted_pt = p1
            else:
                # Use normalized motion direction for intercept / bounce prediction
                ux, uy = normalize(float(vx_raw), float(vy_raw))
                v_dir = (ux, uy)

                intercept_pt, hit_wall = first_border_intersection(p1, v_dir, w, h)

                if intercept_pt is not None:
                    predicted_pt = intercept_pt
                    predicted_label = "INTERCEPT"

                    # Only bounce if first hit is top or bottom
                    if hit_wall in ("top", "bottom"):
                        v_out = reflect(v_dir, hit_wall)
                        bounce_end, _ = first_border_intersection(intercept_pt, v_out, w, h)
                        if bounce_end is not None:
                            predicted_pt = bounce_end
                            predicted_label = "BOUNCE"

            # Print predicted point at a limited rate
            now_t = time.time()
            if predicted_pt is not None and (PRINT_INTERVAL <= 0 or (now_t - last_print_t) >= PRINT_INTERVAL):
                print(f"[PREDICTED {predicted_label}] x={predicted_pt[0]} y={predicted_pt[1]}", flush=True)
                last_print_t = now_t

        # FPS update
        frame_count += 1
        now = time.time()
        if now - start_time >= UPDATE_INTERVAL:
            fps_display = frame_count / (now - start_time)
            print(f"[FPS] {fps_display:.1f}", flush=True)
            frame_count = 0
            start_time = now

finally:
    cap.release()
    cv2.destroyAllWindows()

# URL = "http://127.0.0.1:8080/ping"  # use 127.0.0.1 if network_mode: host

# try:
#     with urllib.request.urlopen(URL, timeout=3) as r:
#         body = r.read().decode("utf-8", errors="replace").strip()
#         print(f"[pysvc] GET /ping -> {r.status}, body='{body}'")
#     sys.exit(0)
# except Exception as e:
#     print(f"[pysvc] ping failed: {e}")
#     sys.exit(1)
