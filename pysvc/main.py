import cv2
import math
import numpy as np
import time
import glob
import os
from multiprocessing import shared_memory
import struct
import atexit


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

# Read the ACTUAL resolution the camera gave us
actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] Camera resolution: {actual_w}x{actual_h}", flush=True)

frame_count = 0
start_time = time.time()
fps_display = 0
UPDATE_INTERVAL = 1

SCALE = actual_w / 640.0

MIN_AREA = int(300 * SCALE * SCALE)
MAX_JUMP_PX = int(200 * SCALE)
STILL_THRESH_PX = 8.0
STILL_CONFIRM_FRAMES = 5

PRINT_INTERVAL = 1
last_print_t = 0.0

VEL_PRINT_INTERVAL = 0.25
last_vel_print_t = 0.0

MM_PER_PX = 3.0

# Shared memory layout:
# seq          uint32 offset 0
# ready        uint32 offset 4
# request_next uint32 offset 8
# kind         uint32 offset 12
# x_mm         double offset 16
# y_mm         double offset 24
SHM_NAME = "puck_xy_mm"
SHM_PATH = f"/dev/shm/{SHM_NAME}"
SHM_SIZE = 32

U32 = struct.Struct("<I")
F64 = struct.Struct("<d")

KIND_NONE = 0
KIND_STILL = 1
KIND_INTERCEPT = 2
KIND_BOUNCE = 3

# If we miss this many frames in a row, throw the Kalman track away and
# re-initialize on the next good detection.
MAX_MISSING_FRAMES = 15

# If we've been missing a few frames, relax the outlier gate so we can
# re-acquire after fast motion or brief occlusion.
RELAX_JUMP_AFTER_MISSING = 5

# Reuse the morphology kernel instead of re-creating it every frame
MORPH_KERNEL = np.ones((5, 5), np.uint8)

# Reuse HSV bounds arrays
HSV_LOWER = np.array([35, 60, 60], dtype=np.uint8)
HSV_UPPER = np.array([85, 255, 255], dtype=np.uint8)


def chmod_shm():
    try:
        os.chmod(SHM_PATH, 0o666)
        st = os.stat(SHM_PATH)
        print(f"[INFO] shm permissions set to {oct(st.st_mode & 0o777)} on {SHM_PATH}", flush=True)
    except Exception as e:
        print(f"[WARN] Failed to chmod {SHM_PATH}: {e}", flush=True)


try:
    shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    shm.buf[:] = b"\x00" * SHM_SIZE

    # Initial state: let C++ request the first coordinate
    U32.pack_into(shm.buf, 0, 0)   # seq
    U32.pack_into(shm.buf, 4, 0)   # ready
    U32.pack_into(shm.buf, 8, 1)   # request_next
    U32.pack_into(shm.buf, 12, 0)  # kind
    F64.pack_into(shm.buf, 16, 0.0)
    F64.pack_into(shm.buf, 24, 0.0)

    chmod_shm()

except FileExistsError:
    shm = shared_memory.SharedMemory(name=SHM_NAME, create=False, size=SHM_SIZE)
    chmod_shm()

py_seq = 0


def close_shm():
    try:
        shm.close()
    except Exception:
        pass
    try:
        shm.unlink()
    except Exception:
        pass


atexit.register(close_shm)


def shm_get_u32(offset):
    return U32.unpack_from(shm.buf, offset)[0]


def shm_set_u32(offset, value):
    U32.pack_into(shm.buf, offset, value)


def shm_set_f64(offset, value):
    F64.pack_into(shm.buf, offset, value)


def kind_from_label(label):
    if label == "STILL":
        return KIND_STILL
    if label == "INTERCEPT":
        return KIND_INTERCEPT
    if label == "BOUNCE":
        return KIND_BOUNCE
    return KIND_NONE


def cxx_requested_next():
    return shm_get_u32(8) == 1


def publish_coordinate(kind, x_mm, y_mm):
    global py_seq

    py_seq += 1
    if py_seq % 2 == 0:
        py_seq += 1

    # Mark write in progress with odd seq
    shm_set_u32(0, py_seq)

    # Write payload
    shm_set_u32(12, kind)
    shm_set_f64(16, x_mm)
    shm_set_f64(24, y_mm)
    shm_set_u32(4, 1)   # ready
    shm_set_u32(8, 0)   # consume request_next

    # Publish stable data with even seq
    py_seq += 1
    shm_set_u32(0, py_seq)


def px_to_mm(pt, mm_per_px=MM_PER_PX):
    if pt is None:
        return None
    return (pt[0] * mm_per_px, pt[1] * mm_per_px)


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
    kf.measurementMatrix = np.eye(2, 4, dtype=np.float32)

    # Constant-velocity model: new_pos = old_pos + velocity
    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=np.float32)

    # Trust position dynamics fairly strongly, but allow velocity to adapt
    kf.processNoiseCov = np.diag([1e-2, 1e-2, 0.1, 0.1]).astype(np.float32)

    # Camera is reasonably reliable
    kf.measurementNoiseCov = (5e-2 * np.eye(2)).astype(np.float32)

    kf.errorCovPost = np.eye(4, dtype=np.float32)
    return kf


kalman = create_kalman()
kalman_initialized = False

missing_count = 0
still_count = 0

print("Camera started.", flush=True)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from camera.", flush=True)
            break

        h, w = frame.shape[:2]

        center, area = detect_green_puck_center(frame)

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
                    if missing_count % 5 == 1:
                        print(
                            f"[WARN] Rejected measurement at {center}, "
                            f"pred=({pred_x:.0f},{pred_y:.0f}), "
                            f"jump={dist2**0.5:.0f}px > {effective_jump}px",
                            flush=True
                        )
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

                intercept_pt, hit_wall = first_border_intersection(filtered_pos, (ux, uy), w, h)

                if intercept_pt is not None:
                    predicted_pt = intercept_pt
                    predicted_label = "INTERCEPT"

                    # Only left/right walls create a bounce now.
                    # Top/bottom walls stop at intercept only.
                    if hit_wall in ("left", "right"):
                        v_out = reflect((ux, uy), hit_wall)
                        bounce_end, _ = first_border_intersection(intercept_pt, v_out, w, h)
                        if bounce_end is not None:
                            predicted_pt = bounce_end
                            predicted_label = "BOUNCE"

            now_t = time.time()
            if predicted_pt is not None and (PRINT_INTERVAL <= 0 or (now_t - last_print_t) >= PRINT_INTERVAL):
                predicted_mm = px_to_mm(predicted_pt)

                print(
                    f"[PREDICTED {predicted_label}] "
                    f"x={predicted_pt[0]} y={predicted_pt[1]} "
                    f"x_mm={predicted_mm[0]:.1f} y_mm={predicted_mm[1]:.1f}",
                    flush=True
                )

                if cxx_requested_next():
                    publish_coordinate(
                        kind_from_label(predicted_label),
                        predicted_mm[0],
                        predicted_mm[1]
                    )

                last_print_t = now_t

        frame_count += 1
        now = time.time()
        if now - start_time >= UPDATE_INTERVAL:
            fps_display = frame_count / (now - start_time)
            print(f"[FPS] {fps_display:.1f}", flush=True)
            frame_count = 0
            start_time = now

        if kalman_initialized and filtered_vel is not None:
            now_t = time.time()
            if now_t - last_vel_print_t >= VEL_PRINT_INTERVAL:
                vx, vy = filtered_vel
                speed = math.hypot(vx, vy)
                # print(f"[VELOCITY] vx={vx:.2f} vy={vy:.2f} speed={speed:.2f}", flush=True)
                last_vel_print_t = now_t

finally:
    cap.release()


# # # import cv2
# # # import numpy as np
# # # import time
# # # import glob

# # # def open_camera():
# # #     devices = sorted(glob.glob('/dev/video*'))
# # #     print(f"Found video devices: {devices}", flush=True)

# # #     for dev in devices:
# # #         c = cv2.VideoCapture(dev, cv2.CAP_V4L2)
# # #         if c.isOpened():
# # #             ret, _ = c.read()
# # #             if ret:
# # #                 print(f"Camera found at {dev}", flush=True)
# # #                 return c
# # #             c.release()

# # #     raise RuntimeError(f"No working camera found. Tried: {devices}")

# # # cap = open_camera()
# # # cap.set(cv2.CAP_PROP_FPS, 120)
# # # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# # # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

# # # # FPS tracking
# # # frame_count = 0
# # # start_time = time.time()
# # # fps_display = 0
# # # UPDATE_INTERVAL = 1  # seconds

# # # # Tracking / detection
# # # MIN_AREA = 300          # ignore tiny green areas
# # # MAX_JUMP_PX = 200       # reject sudden jumps (noise and outliers)

# # # # If abs(dx) <= STILL_THRESH_PX and abs(dy) <= STILL_THRESH_PX
# # # # for STILL_CONFIRM_FRAMES, treat as STILL
# # # STILL_THRESH_PX = 5
# # # STILL_CONFIRM_FRAMES = 2

# # # # Print coordinates
# # # PRINT_INTERVAL = 0.05   # seconds
# # # last_print_t = 0.0

# # # # Tracking state
# # # missing_count = 0
# # # MAX_MISSING_TO_FORCE_ACCEPT = 3
# # # AREA_GAIN_THRESHOLD = 1.3

# # # def green_mask_hsv(frame_bgr):
# # #     hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

# # #     lower1 = np.array([35, 60, 60], dtype=np.uint8)
# # #     upper1 = np.array([85, 255, 255], dtype=np.uint8)

# # #     mask = cv2.inRange(hsv, lower1, upper1)

# # #     kernel = np.ones((5, 5), np.uint8)
# # #     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
# # #     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
# # #     return mask

# # # def detect_green_puck_center(frame_bgr):
# # #     # Returns puck center (cx, cy) from the largest green contour
# # #     # and returns the area of that contour
# # #     h, w = frame_bgr.shape[:2]
# # #     mask = green_mask_hsv(frame_bgr)

# # #     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# # #     if not contours:
# # #         return None, 0

# # #     c = max(contours, key=cv2.contourArea)
# # #     area = cv2.contourArea(c)
# # #     if area < MIN_AREA:
# # #         return None, 0

# # #     M = cv2.moments(c)
# # #     if M["m00"] == 0:
# # #         return None, 0

# # #     cx = int(M["m10"] / M["m00"])
# # #     cy = int(M["m01"] / M["m00"])

# # #     cx = int(np.clip(cx, 0, w - 1))
# # #     cy = int(np.clip(cy, 0, h - 1))

# # #     return (cx, cy), area

# # # def first_border_intersection(p0, v, w, h):
# # #     # From point p0 moving along direction v=(vx, vy),
# # #     # find the first intersection with the frame borders
# # #     x0, y0 = float(p0[0]), float(p0[1])
# # #     vx, vy = float(v[0]), float(v[1])
# # #     eps = 1e-9
# # #     candidates = []

# # #     # Left wall x=0
# # #     if abs(vx) > eps:
# # #         t = (0 - x0) / vx
# # #         if t > 0:
# # #             y = y0 + t * vy
# # #             if 0 <= y <= (h - 1):
# # #                 candidates.append((t, (0, int(round(y))), "left"))

# # #     # Right wall x=w-1
# # #     if abs(vx) > eps:
# # #         t = ((w - 1) - x0) / vx
# # #         if t > 0:
# # #             y = y0 + t * vy
# # #             if 0 <= y <= (h - 1):
# # #                 candidates.append((t, (w - 1, int(round(y))), "right"))

# # #     # Top wall y=0
# # #     if abs(vy) > eps:
# # #         t = (0 - y0) / vy
# # #         if t > 0:
# # #             x = x0 + t * vx
# # #             if 0 <= x <= (w - 1):
# # #                 candidates.append((t, (int(round(x)), 0), "top"))

# # #     # Bottom wall y=h-1
# # #     if abs(vy) > eps:
# # #         t = ((h - 1) - y0) / vy
# # #         if t > 0:
# # #             x = x0 + t * vx
# # #             if 0 <= x <= (w - 1):
# # #                 candidates.append((t, (int(round(x)), h - 1), "bottom"))

# # #     if not candidates:
# # #         return None, None

# # #     candidates.sort(key=lambda item: item[0])
# # #     _, pt, wall = candidates[0]
# # #     return pt, wall

# # # def reflect(v, wall):
# # #     vx, vy = float(v[0]), float(v[1])
# # #     if wall in ("left", "right"):
# # #         return (-vx, vy)
# # #     if wall in ("top", "bottom"):
# # #         return (vx, -vy)
# # #     return (vx, vy)

# # # def normalize(vx, vy, eps=1e-9):
# # #     mag = (vx * vx + vy * vy) ** 0.5
# # #     if mag < eps:
# # #         return 0.0, 0.0
# # #     return vx / mag, vy / mag

# # # # Last two accepted points
# # # p0 = None
# # # p1 = None

# # # # Area of the currently tracked object
# # # tracked_area = None

# # # # Consecutive low-motion frame count
# # # still_count = 0

# # # # Track disappearance state
# # # prev_detected = False

# # # print("Camera started.", flush=True)

# # # try:
# # #     while True:
# # #         ret, frame = cap.read()
# # #         if not ret:
# # #             print("Failed to read frame from camera.", flush=True)
# # #             break

# # #         h, w = frame.shape[:2]

# # #         # Detection returns center and contour area
# # #         center, area = detect_green_puck_center(frame)

# # #         # Update p0 and p1 using consecutive accepted detections
# # #         if center is not None:
# # #             cond_missing_relax = (missing_count >= MAX_MISSING_TO_FORCE_ACCEPT)
# # #             missing_count = 0

# # #             if p1 is None:
# # #                 p1 = center
# # #                 tracked_area = area
# # #             else:
# # #                 dx = center[0] - p1[0]
# # #                 dy = center[1] - p1[1]
# # #                 dist2 = dx * dx + dy * dy

# # #                 cond_normal_accept = (dist2 <= (MAX_JUMP_PX * MAX_JUMP_PX))

# # #                 if tracked_area is None:
# # #                     cond_area_bigger = True
# # #                 else:
# # #                     cond_area_bigger = (area > tracked_area * AREA_GAIN_THRESHOLD)

# # #                 if cond_normal_accept or cond_area_bigger or cond_missing_relax:
# # #                     p0 = p1
# # #                     p1 = center
# # #                     tracked_area = area
# # #         else:
# # #             missing_count += 1

# # #             # Reset tracking when object is gone too long
# # #             if missing_count > MAX_MISSING_TO_FORCE_ACCEPT:
# # #                 if prev_detected:
# # #                     print("Object Disappeared", flush=True)

# # #                 p0 = None
# # #                 p1 = None
# # #                 tracked_area = None
# # #                 still_count = 0

# # #         # Update detection state flag
# # #         prev_detected = (p1 is not None)

# # #         # Predicted results
# # #         intercept_pt = None
# # #         bounce_end = None
# # #         predicted_pt = None
# # #         predicted_label = None

# # #         if p0 is not None and p1 is not None:
# # #             vx_raw = p1[0] - p0[0]
# # #             vy_raw = p1[1] - p0[1]

# # #             # Count consecutive small-motion frames to avoid jitter
# # #             if abs(vx_raw) <= STILL_THRESH_PX and abs(vy_raw) <= STILL_THRESH_PX:
# # #                 still_count += 1
# # #             else:
# # #                 still_count = 0

# # #             stationary = (still_count >= STILL_CONFIRM_FRAMES)

# # #             if stationary:
# # #                 predicted_label = "STILL"
# # #                 predicted_pt = p1
# # #             else:
# # #                 # Use normalized motion direction for intercept / bounce prediction
# # #                 ux, uy = normalize(float(vx_raw), float(vy_raw))
# # #                 v_dir = (ux, uy)

# # #                 intercept_pt, hit_wall = first_border_intersection(p1, v_dir, w, h)

# # #                 if intercept_pt is not None:
# # #                     predicted_pt = intercept_pt
# # #                     predicted_label = "INTERCEPT"

# # #                     # Only bounce if first hit is top or bottom
# # #                     if hit_wall in ("top", "bottom"):
# # #                         v_out = reflect(v_dir, hit_wall)
# # #                         bounce_end, _ = first_border_intersection(intercept_pt, v_out, w, h)
# # #                         if bounce_end is not None:
# # #                             predicted_pt = bounce_end
# # #                             predicted_label = "BOUNCE"

# # #             # Print predicted point at a limited rate
# # #             now_t = time.time()
# # #             if predicted_pt is not None and (PRINT_INTERVAL <= 0 or (now_t - last_print_t) >= PRINT_INTERVAL):
# # #                 print(f"[PREDICTED {predicted_label}] x={predicted_pt[0]} y={predicted_pt[1]}", flush=True)
# # #                 last_print_t = now_t

# # #         # FPS update
# # #         frame_count += 1
# # #         now = time.time()
# # #         if now - start_time >= UPDATE_INTERVAL:
# # #             fps_display = frame_count / (now - start_time)
# # #             print(f"[FPS] {fps_display:.1f}", flush=True)
# # #             frame_count = 0
# # #             start_time = now

# # # finally:
# # #     cap.release()
# # #     cv2.destroyAllWindows()
