import cv2
import math
import numpy as np
import time

# Must match calibrate_roi.py CAPTURE resolution so ROI coords line up
CAPTURE_W, CAPTURE_H = 720, 1080
OUTPUT_W,  OUTPUT_H  = 540, 960   # vertical (portrait) — width x height after rotate

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FPS, 120)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] Camera resolution: {actual_w}x{actual_h}", flush=True)

ROI_X, ROI_Y, ROI_W, ROI_H = 251, 1, 889, 717

OUTPUT_W, OUTPUT_H = 600, 765   # portrait space after rotate: width x height

frame_count = 0
start_time = time.time()
fps_display = 0
UPDATE_INTERVAL = 1

# All thresholds are relative to OUTPUT size, not raw capture
SCALE = OUTPUT_W / 640.0

MIN_AREA = int(300 * SCALE * SCALE)
MAX_JUMP_PX = int(200 * SCALE)
STILL_THRESH_PX = 8.0
STILL_CONFIRM_FRAMES = 5
DIR_ARROW_LEN = int(120 * SCALE)

PRINT_INTERVAL = 1
last_print_t = 0.0
last_vel_display_t = 0.0
displayed_vx = 0.0
displayed_vy = 0.0
VEL_DISPLAY_INTERVAL = 0.0   # seconds between on-screen updates

# If we miss this many frames in a row, throw the Kalman track away and
# re-initialize on the next good detection.
MAX_MISSING_FRAMES = 15

# If we've been missing a few frames, relax the outlier gate so we can
# re-acquire after fast motion or brief occlusion.
RELAX_JUMP_AFTER_MISSING = 5

GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
YELLOW = (0, 255, 255)
RED = (0, 0, 255)

# Reuse the morphology kernel instead of re-creating it every frame
MORPH_KERNEL = np.ones((5, 5), np.uint8)

# Reuse HSV bounds arrays — no need to rebuild every call
HSV_LOWER = np.array([35, 60, 60], dtype=np.uint8)
HSV_UPPER = np.array([85, 255, 255], dtype=np.uint8)

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

def reflect(v, wall):
    vx, vy = float(v[0]), float(v[1])
    if wall in ("left", "right"):
        return (-vx, vy)
    if wall in ("top", "bottom"):
        return (vx, -vy)
    return (vx, vy)

def normalize(vx, vy, eps=1e-9):
    mag = math.hypot(vx, vy)
    if mag < eps:
        return 0.0, 0.0
    return vx / mag, vy / mag

def create_kalman():
    kf = cv2.KalmanFilter(4, 2)

    # Measurement extracts x, y from state [x, y, vx, vy]
    kf.measurementMatrix = np.eye(2,callable 4, dtype=np.float32)

    # Constant-velocity model: new_pos = old_pos + velocity
    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=np.float32)

    # Trust physics on position, let velocity adapt quickly, VX, VY updating time kf.processNoiseCov = np.diag([1e-2, 1e-2, UPDATING_TIME, UPDATING_TIME]).
    kf.processNoiseCov = np.diag([1e-2, 1e-2, 0.1, 0.1]).astype(np.float32)

    # Camera is reasonably reliable
    kf.measurementNoiseCov = (5e-2 * np.eye(2)).astype(np.float32)

    kf.errorCovPost = np.eye(4, dtype=np.float32)
    return kf

kalman = create_kalman()
kalman_initialized = False

missing_count = 0
still_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Crop → resize → rotate to portrait (600x765)
    frame = frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]
    frame = cv2.resize(frame, (OUTPUT_H, OUTPUT_W))
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    h, w = frame.shape[:2]

    center, mask, area = detect_green_puck_center(frame)

    # If we've been missing too long, throw the track away entirely so the
    # next good detection can re-seed the filter cleanly.
    if missing_count >= MAX_MISSING_FRAMES and kalman_initialized:
        print(f"[INFO] Lost track after {missing_count} missing frames - resetting Kalman", flush=True)
        kalman = create_kalman()
        kalman_initialized = False
        still_count = 0

    predicted = kalman.predict()
    pred_x = float(predicted[0][0])
    pred_y = float(predicted[1][0])

    accepted_measurement = None

    if center is not None:
        if not kalman_initialized:
            kalman.statePost = np.array([
                [np.float32(center[0])],
                [np.float32(center[1])],
                [0.0],
                [0.0]
            ], np.float32)
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
                # Debug output: if you see this spam, the jump gate is the problem
                if missing_count % 5 == 1:
                    print(f"[WARN] Rejected measurement at {center}, "
                          f"pred=({pred_x:.0f},{pred_y:.0f}), "
                          f"jump={dist2**0.5:.0f}px > {effective_jump}px",
                          flush=True)
    else:
        missing_count += 1

    if kalman_initialized and accepted_measurement is not None:
        measurement = np.array([
            [np.float32(accepted_measurement[0])],
            [np.float32(accepted_measurement[1])]
        ], np.float32)
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
            int(np.clip(round(filtered_y), 0, h - 1))
        )
        filtered_vel = (filtered_vx, filtered_vy)

    intercept_pt = None
    bounce_end = None
    predicted_pt = None
    predicted_label = None
    dir_tip = None

    cv2.line(frame, (0, 0), (w - 1, 0), (255, 255, 255), 1)
    cv2.line(frame, (0, h - 1), (w - 1, h - 1), (255, 255, 255), 1)
    cv2.line(frame, (0, 0), (0, h - 1), (255, 255, 255), 1)
    cv2.line(frame, (w - 1, 0), (w - 1, h - 1), (255, 255, 255), 1)

    have_fresh_track = (
        kalman_initialized
        and filtered_pos is not None
        and filtered_vel is not None
        and accepted_measurement is not None
    )

    if have_fresh_track:
        vx, vy = filtered_vel

        if abs(vx) <= STILL_THRESH_PX and abs(vy) <= STILL_THRESH_PX:
            still_count += 1
        else:
            still_count = 0

        if still_count >= STILL_CONFIRM_FRAMES:
            predicted_label = "STILL"
            predicted_pt = filtered_pos
        else:
            ux, uy = normalize(vx, vy)

            tip_x = int(round(filtered_pos[0] + DIR_ARROW_LEN * ux))
            tip_y = int(round(filtered_pos[1] + DIR_ARROW_LEN * uy))
            dir_tip = (
                int(np.clip(tip_x, 0, w - 1)),
                int(np.clip(tip_y, 0, h - 1))
            )

            intercept_pt, hit_wall = first_border_intersection(filtered_pos, (ux, uy), w, h)

            if intercept_pt is not None:
                predicted_pt = intercept_pt
                predicted_label = "INTERCEPT"

                if hit_wall in ("top", "bottom"):
                    v_out = reflect((ux, uy), hit_wall)
                    bounce_end, _ = first_border_intersection(intercept_pt, v_out, w, h)
                    if bounce_end is not None:
                        predicted_pt = bounce_end
                        predicted_label = "BOUNCE"

        now_t = time.time()
        if predicted_pt is not None and (PRINT_INTERVAL <= 0 or (now_t - last_print_t) >= PRINT_INTERVAL):
            print(f"[PREDICTED {predicted_label}] x={predicted_pt[0]} y={predicted_pt[1]}", flush=True)
            last_print_t = now_t

    # --- DEBUG VISUALIZATIONS (commented out) ----------------------------

    # if center is not None:
    #     cv2.circle(frame, center, 6, GREEN, -1)
    #     cv2.putText(frame, f"meas {center}", (center[0] + 10, center[1] - 10),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)
    #
    # if kalman_initialized:
    #     pred_pt = (
    #         int(np.clip(round(pred_x), 0, w - 1)),
    #         int(np.clip(round(pred_y), 0, h - 1))
    #     )
    #     cv2.circle(frame, pred_pt, 6, YELLOW, 2)
    #     cv2.putText(frame, f"pred {pred_pt}", (pred_pt[0] + 10, pred_pt[1] + 20),
    #                 cv2.FONT_HERSHEY_SIMPLEX, 0.5, YELLOW, 2)
    # ---------------------------------------------------------------------

    # Kalman filtered position (red dot) - the main tracking output
    if filtered_pos is not None:
        cv2.circle(frame, filtered_pos, 7, RED, -1)
        cv2.putText(frame, f"kalman {filtered_pos}", (filtered_pos[0] + 10, filtered_pos[1] + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, RED, 2)

    if filtered_pos is not None and dir_tip is not None:
        cv2.arrowedLine(frame, filtered_pos, dir_tip, GREEN, 2, tipLength=0.25)
        cv2.putText(frame, "dir", (dir_tip[0] + 10, dir_tip[1] + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

    if filtered_pos is not None and intercept_pt is not None:
        cv2.line(frame, filtered_pos, intercept_pt, BLUE, 2)
        cv2.circle(frame, intercept_pt, 10, BLUE, -1)
        cv2.putText(frame, f"intercept {intercept_pt}", (intercept_pt[0] + 10, intercept_pt[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2)

    if intercept_pt is not None and bounce_end is not None:
        cv2.line(frame, intercept_pt, bounce_end, BLUE, 2)
        cv2.circle(frame, bounce_end, 8, BLUE, -1)
        cv2.putText(frame, f"bounce {bounce_end}", (bounce_end[0] + 10, bounce_end[1] + 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2)

    frame_count += 1
    now = time.time()
    if now - start_time >= UPDATE_INTERVAL:
        fps_display = frame_count / (now - start_time)
        frame_count = 0
        start_time = now

    cv2.putText(frame, f"FPS: {fps_display:.1f}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, GREEN, 2)

    if kalman_initialized and filtered_vel is not None:
        now_t = time.time()
        if now_t - last_vel_display_t >= VEL_DISPLAY_INTERVAL:
            displayed_vx = filtered_vel[0]
            displayed_vy = filtered_vel[1]
            last_vel_display_t = now_t
            if abs(displayed_vx) > 2 or abs(displayed_vy) > 2:
                cv2.putText(frame, f"vx={displayed_vx:.2f} vy={displayed_vy:.2f}", (20, 75),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2)

    if missing_count > 0:
        cv2.putText(frame, f"missing={missing_count}", (20, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, GREEN, 2)

    cv2.imshow("RoboMallet", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()









# import cv2
# import numpy as np
# import time

# cap = cv2.VideoCapture(0)  # 0 default, 1 extra camera
# cap.set(cv2.CAP_PROP_FPS, 120)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

# # display fps
# frame_count = 0
# start_time = time.time()
# fps_display = 0
# UPDATE_INTERVAL = 1  # seconds

# # Tracking / detection
# MIN_AREA = 300  # ignore tiny green areas
# MAX_JUMP_PX = 200  # reject sudden jumps (noise and outliers)

# # If abs(dx) <= STILL_THRESH_PX AND abs(dy) <= STILL_THRESH_PX for STILL_CONFIRM_FRAMES, treat as STILL (no trajectory line)
# STILL_THRESH_PX = 5
# STILL_CONFIRM_FRAMES = 2

# # Direction visual line
# DIR_ARROW_LEN = 120    # px, shows direction arrow only

# # print coordinates
# PRINT_INTERVAL = 0.05  # seconds
# last_print_t = 0.0

# # tracking
# missing_count = 0
# MAX_MISSING_TO_FORCE_ACCEPT = 3
# AREA_GAIN_THRESHOLD = 1.3

# # Colors
# GREEN = (0, 255, 0)
# BLUE = (255, 0, 0)

# def green_mask_hsv(frame_bgr):
#     hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

#     lower1 = np.array([35, 60, 60], dtype=np.uint8)
#     upper1 = np.array([85, 255, 255], dtype=np.uint8)

#     mask = cv2.inRange(hsv, lower1, upper1)

#     kernel = np.ones((5, 5), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
#     return mask

# def detect_green_puck_center(frame_bgr):

#     # Returns puck center (cx, cy) from largest green contour and returns the area of that largest contour

#     h, w = frame_bgr.shape[:2]
#     mask = green_mask_hsv(frame_bgr)

#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     if not contours:
#         return None, mask, 0

#     c = max(contours, key=cv2.contourArea)
#     area = cv2.contourArea(c)
#     if area < MIN_AREA:
#         return None, mask, 0

#     M = cv2.moments(c)
#     if M["m00"] == 0:
#         return None, mask, 0

#     cx = int(M["m10"] / M["m00"])
#     cy = int(M["m01"] / M["m00"])

#     cx = int(np.clip(cx, 0, w - 1))
#     cy = int(np.clip(cy, 0, h - 1))

#     return (cx, cy), mask, area

# def first_border_intersection(p0, v, w, h):
#     # From point p0 moving along direction v=(vx,vy), finds the first intersection with the frame borders (x=0, x=w-1, y=0, y=h-1).
   
#     x0, y0 = float(p0[0]), float(p0[1])
#     vx, vy = float(v[0]), float(v[1])
#     eps = 1e-9
#     candidates = []

#     # Left wall x=0
#     if abs(vx) > eps:
#         t = (0 - x0) / vx
#         if t > 0:
#             y = y0 + t * vy
#             if 0 <= y <= (h - 1):
#                 candidates.append((t, (0, int(round(y))), "left"))

#     # Right wall x=w-1
#     if abs(vx) > eps:
#         t = ((w - 1) - x0) / vx
#         if t > 0:
#             y = y0 + t * vy
#             if 0 <= y <= (h - 1):
#                 candidates.append((t, (w - 1, int(round(y))), "right"))

#     # Top wall y=0
#     if abs(vy) > eps:
#         t = (0 - y0) / vy
#         if t > 0:
#             x = x0 + t * vx
#             if 0 <= x <= (w - 1):
#                 candidates.append((t, (int(round(x)), 0), "top"))

#     # Bottom wall y=h-1
#     if abs(vy) > eps:
#         t = ((h - 1) - y0) / vy
#         if t > 0:
#             x = x0 + t * vx
#             if 0 <= x <= (w - 1):
#                 candidates.append((t, (int(round(x)), h - 1), "bottom"))

#     if not candidates:
#         return None, None

#     candidates.sort(key=lambda item: item[0])
#     _, pt, wall = candidates[0]
#     return pt, wall

# def reflect(v, wall):
#     vx, vy = float(v[0]), float(v[1])
#     if wall in ("left", "right"):
#         return (-vx, vy)
#     if wall in ("top", "bottom"):
#         return (vx, -vy)
#     return (vx, vy)

# def normalize(vx, vy, eps=1e-9):
#     mag = (vx * vx + vy * vy) ** 0.5
#     if mag < eps:
#         return 0.0, 0.0
#     return vx / mag, vy / mag

# # Track last two points
# p0 = None
# p1 = None

# # Track the area of the currently tracked object
# tracked_area = None

# # Stillness counter
# still_count = 0

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         break

#     h, w = frame.shape[:2]

#     # detect returns center, mask, and area (area = 0 if nothing)
#     center, mask, area = detect_green_puck_center(frame)

#     # Update p0 and p1 using consecutive accepted detections
#     if center is not None:
#         missing_count = 0

#         if p1 is None:
#             p1 = center
#             tracked_area = area
#         else:
#             dx = center[0] - p1[0]
#             dy = center[1] - p1[1]
#             dist2 = (dx * dx + dy * dy)

#             cond_normal_accept = (dist2 <= (MAX_JUMP_PX * MAX_JUMP_PX))

#             if tracked_area is None:
#                 cond_area_bigger = True
#             else:
#                 cond_area_bigger = (area > tracked_area * AREA_GAIN_THRESHOLD)

#             cond_missing_relax = (missing_count >= MAX_MISSING_TO_FORCE_ACCEPT)

#             if cond_normal_accept or cond_area_bigger or cond_missing_relax:
#                 p0 = p1
#                 p1 = center
#                 tracked_area = area
#     else:
#         missing_count += 1

#     # Borders
#     cv2.line(frame, (0, 0), (w - 1, 0), (255, 255, 255), 1)           # top
#     cv2.line(frame, (0, h - 1), (w - 1, h - 1), (255, 255, 255), 1)   # bottom
#     cv2.line(frame, (0, 0), (0, h - 1), (255, 255, 255), 1)           # left
#     cv2.line(frame, (w - 1, 0), (w - 1, h - 1), (255, 255, 255), 1)   # right

#     # Intercept and bounce using only direction
#     intercept_pt = None
#     bounce_end = None
#     predicted_pt = None
#     predicted_label = None  # "INTERCEPT", "BOUNCE", or "STILL"
#     dir_tip = None # arrow tip for direction

#     if p0 is not None and p1 is not None:
#         vx_raw = p1[0] - p0[0]
#         vy_raw = p1[1] - p0[1]

#         # Count consecutive "small motion" frames to avoid jitter triggering BOUNCE
#         if abs(vx_raw) <= STILL_THRESH_PX and abs(vy_raw) <= STILL_THRESH_PX:
#             still_count += 1
#         else:
#             still_count = 0

#         stationary = (still_count >= STILL_CONFIRM_FRAMES)

#         if stationary:
#             # STILL: print STILL and draw no trajectory
#             predicted_label = "STILL"
#             predicted_pt = p1  # prints x/y, but no trajectory is drawn
#             dir_tip = None
#             intercept_pt = None
#             bounce_end = None
#         else:
#             # Normalize direction and draw a fixed-length arrow from p1
#             ux, uy = normalize(float(vx_raw), float(vy_raw))
#             tip_x = int(round(p1[0] + DIR_ARROW_LEN * ux))
#             tip_y = int(round(p1[1] + DIR_ARROW_LEN * uy))
#             dir_tip = (int(np.clip(tip_x, 0, w - 1)), int(np.clip(tip_y, 0, h - 1)))

#             # Use direction vector for intercept/bounce ray
#             v_dir = (ux, uy)
#             intercept_pt, hit_wall = first_border_intersection(p1, v_dir, w, h)

#             predicted_pt = None
#             predicted_label = None

#             if intercept_pt is not None:
#                 predicted_pt = intercept_pt
#                 predicted_label = "INTERCEPT"

#                 # Only bounce if first hit is top/bottom (no bounce for left/right)
#                 if hit_wall in ("top", "bottom"):
#                     v_out = reflect(v_dir, hit_wall)
#                     bounce_end, _ = first_border_intersection(intercept_pt, v_out, w, h)
#                     if bounce_end is not None:
#                         predicted_pt = bounce_end
#                         predicted_label = "BOUNCE"

#         # Printing (labelled)
#         now_t = time.time()
#         if predicted_pt is not None and (PRINT_INTERVAL <= 0 or (now_t - last_print_t) >= PRINT_INTERVAL):
#             print(f"[PREDICTED {predicted_label}] x={predicted_pt[0]} y={predicted_pt[1]}", flush=True)
#             last_print_t = now_t

#     # Drawing p0/p1
#     if p1 is not None:
#         cv2.circle(frame, p1, 6, GREEN, -1)
#         cv2.putText(frame, f"p1 {p1}", (p1[0] + 10, p1[1] + 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

#     if p0 is not None:
#         cv2.circle(frame, p0, 6, GREEN, -1)
#         cv2.putText(frame, f"p0 {p0}", (p0[0] + 10, p0[1] - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

#     # Drawing direction arrow only (green)
#     if p1 is not None and dir_tip is not None:
#         cv2.arrowedLine(frame, p1, dir_tip, GREEN, 2, tipLength=0.25)
#         cv2.putText(frame, "dir", (dir_tip[0] + 10, dir_tip[1] + 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, GREEN, 2)

#     # Drawing intercept + bounce (blue)
#     if p1 is not None and intercept_pt is not None:
#         cv2.line(frame, p1, intercept_pt, BLUE, 2)
#         cv2.circle(frame, intercept_pt, 10, BLUE, -1)
#         cv2.putText(frame, f"intercept {intercept_pt}", (intercept_pt[0] + 10, intercept_pt[1] - 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2)

#     if intercept_pt is not None and bounce_end is not None:
#         cv2.line(frame, intercept_pt, bounce_end, BLUE, 2)
#         cv2.circle(frame, bounce_end, 8, BLUE, -1)
#         cv2.putText(frame, f"bounce {bounce_end}", (bounce_end[0] + 10, bounce_end[1] + 10),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.6, BLUE, 2)

#     # FPS
#     frame_count += 1
#     now = time.time()
#     if now - start_time >= UPDATE_INTERVAL:
#         fps_display = frame_count / (now - start_time)
#         frame_count = 0
#         start_time = now

#     cv2.putText(frame, f"FPS: {fps_display:.1f}", (20, 50),
#                 cv2.FONT_HERSHEY_SIMPLEX, 1, GREEN, 2)

#     cv2.imshow("RoboMallet", frame)

#     if cv2.waitKey(1) == 27:  # ESC
#         break

# cap.release()
# cv2.destroyAllWindows()