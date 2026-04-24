import cv2
import math
import numpy as np
import time
import glob
import os
from multiprocessing import shared_memory
import struct
import atexit
import signal
import sys

# Requested camera resolution
REQUEST_W = 640
REQUEST_H = 480

# Physical table dimensions in millimeters
TABLE_W_MM = 1066.8
TABLE_H_MM = 990.6

# Allowed robot-side Y range in millimeters
# If an INTERCEPT or BOUNCE target falls outside this range,
# Python will label it as OUT_OF_RANGE so C++ will not move.
ROBOT_Y_MIN_MM = 247.75
ROBOT_Y_MAX_MM = 743.25


def open_camera():
    # Search all Linux video devices and return the first one that
    # successfully opens and produces a frame.
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


# Open the camera and request the desired frame rate / resolution
cap = open_camera()
cap.set(cv2.CAP_PROP_FPS, 120)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUEST_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUEST_H)

# Read the actual resolution returned by the camera
actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] Camera resolution: {actual_w}x{actual_h}", flush=True)

# Fixed pixel-to-mm conversion for 640x480
# x-axis: 1066.8 / 640 = 1.667 mm/px
# y-axis: 990.6 / 480 = 2.064 mm/px
MM_PER_PX_X = 1066.8 / 640.0
MM_PER_PX_Y = 990.6 / 480.0
print(f"[INFO] MM_PER_PX_X={MM_PER_PX_X:.6f} MM_PER_PX_Y={MM_PER_PX_Y:.6f}", flush=True)

# FPS tracking variables
frame_count = 0
start_time = time.time()
fps_display = 0
UPDATE_INTERVAL = 1

# Scale some thresholds using actual delivered resolution
SCALE_X = actual_w / REQUEST_W
SCALE_Y = actual_h / REQUEST_H
SCALE_AREA = SCALE_X * SCALE_Y
SCALE_JUMP = max(SCALE_X, SCALE_Y)

# Detection / tracking thresholds
MIN_AREA = max(1, int(300 * SCALE_AREA))
MAX_JUMP_PX = max(1, int(200 * SCALE_JUMP))
STILL_THRESH_PX = 8.0 * SCALE_JUMP
STILL_CONFIRM_FRAMES = 5

# Console print timing
PRINT_INTERVAL = 1
last_print_t = 0.0

VEL_PRINT_INTERVAL = 0.25
last_vel_print_t = 0.0

# Shared memory configuration
SHM_NAME = "puck_xy_mm"
SHM_PATH = f"/dev/shm/{SHM_NAME}"
SHM_SIZE = 32

# Struct helpers for writing binary data into shared memory
U32 = struct.Struct("<I")
F64 = struct.Struct("<d")

# Labels / kinds written to shared memory
KIND_NONE = 0
KIND_STILL = 1
KIND_INTERCEPT = 2
KIND_BOUNCE = 3
KIND_RIGHT_SIDE = 4
KIND_OUT_OF_RANGE = 5

# Tracking tolerances
MAX_MISSING_FRAMES = 15
RELAX_JUMP_AFTER_MISSING = 5

# Morphology kernel for mask cleanup
MORPH_KERNEL = np.ones((5, 5), np.uint8)

# HSV range for green puck detection
HSV_LOWER = np.array([35, 60, 60], dtype=np.uint8)
HSV_UPPER = np.array([85, 255, 255], dtype=np.uint8)


def chmod_shm():
    # Make the shared memory object read/write for both containers
    try:
        os.chmod(SHM_PATH, 0o666)
        st = os.stat(SHM_PATH)
        print(f"[INFO] shm permissions set to {oct(st.st_mode & 0o777)} on {SHM_PATH}", flush=True)
    except Exception as e:
        print(f"[WARN] Failed to chmod {SHM_PATH}: {e}", flush=True)


# Create or attach to shared memory
try:
    shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=SHM_SIZE)
    shm.buf[:] = b"\x00" * SHM_SIZE

    # Initial shared memory state
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

# Sequence counter used to mark stable writes
py_seq = 0

py_seq = 0


def close_shm():
    # Cleanly close and unlink shared memory on exit
    try:
        shm.close()
    except Exception:
        pass
    try:
        shm.unlink()
    except Exception:
        pass


def handle_exit(signum, frame):
    print(f"[INFO] Caught signal {signum}, cleaning up...", flush=True)
    try:
        close_shm()
    except Exception:
        pass
    try:
        cap.release()
    except Exception:
        pass
    sys.exit(0)

atexit.register(close_shm)
signal.signal(signal.SIGINT, handle_exit)
signal.signal(signal.SIGTERM, handle_exit)


def shm_get_u32(offset):
    # Read a uint32 from shared memory at the given offset
    return U32.unpack_from(shm.buf, offset)[0]


def shm_set_u32(offset, value):
    # Write a uint32 to shared memory at the given offset
    U32.pack_into(shm.buf, offset, value)


def shm_set_f64(offset, value):
    # Write a float64 to shared memory at the given offset
    F64.pack_into(shm.buf, offset, value)


def kind_from_label(label):
    # Convert string labels into numeric shared-memory values
    if label == "STILL":
        return KIND_STILL
    if label == "INTERCEPT":
        return KIND_INTERCEPT
    if label == "BOUNCE":
        return KIND_BOUNCE
    if label == "RIGHT_SIDE":
        return KIND_RIGHT_SIDE
    if label == "OUT_OF_RANGE":
        return KIND_OUT_OF_RANGE
    return KIND_NONE


def cxx_requested_next():
    # C++ sets request_next = 1 when it wants a fresh coordinate
    return shm_get_u32(8) == 1


def publish_coordinate(kind, x_mm, y_mm):
    # Publish one stable coordinate packet into shared memory
    global py_seq

    py_seq += 1
    if py_seq % 2 == 0:
        py_seq += 1

    # Odd seq means "write in progress"
    shm_set_u32(0, py_seq)

    # Write payload
    shm_set_u32(12, kind)
    shm_set_f64(16, x_mm)
    shm_set_f64(24, y_mm)
    shm_set_u32(4, 1)  # ready
    shm_set_u32(8, 0)  # consume request_next

    # Even seq means "stable completed write"
    py_seq += 1
    shm_set_u32(0, py_seq)


def px_to_mm(pt):
    # Convert a pixel coordinate into millimeters
    if pt is None:
        return None
    return (pt[0] * MM_PER_PX_X, pt[1] * MM_PER_PX_Y)


def green_mask_hsv(frame_bgr):
    # Build a binary mask for green pixels in the frame
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

    # Remove noise and fill holes
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=2)
    return mask


def detect_green_puck_center(frame_bgr):
    # Find the largest green contour and return its center and area
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
    # Find the first border the ray from p0 in direction v hits
    x0, y0 = float(p0[0]), float(p0[1])
    vx, vy = float(v[0]), float(v[1])
    eps = 1e-9
    candidates = []

    # Left / right walls
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

    # Top / bottom walls
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
    # Reflect a direction vector across a border
    vx, vy = float(v[0]), float(v[1])
    if wall in ("left", "right"):
        return (-vx, vy)
    if wall in ("top", "bottom"):
        return (vx, -vy)
    return (vx, vy)


def normalize(vx, vy, eps=1e-9):
    # Normalize a 2D vector
    mag = math.hypot(vx, vy)
    if mag < eps:
        return 0.0, 0.0
    return vx / mag, vy / mag


def create_kalman():
    # Create a constant-velocity Kalman filter with state:
    # [x, y, vx, vy]
    kf = cv2.KalmanFilter(4, 2)

    # Measurement is just position [x, y]
    kf.measurementMatrix = np.eye(2, 4, dtype=np.float32)

    # State transition: position += velocity
    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=np.float32)

    # Process and measurement noise
    kf.processNoiseCov = np.diag([1e-2, 1e-2, 0.1, 0.1]).astype(np.float32)
    kf.measurementNoiseCov = (5e-2 * np.eye(2)).astype(np.float32)

    # Initial state covariance
    kf.errorCovPost = np.eye(4, dtype=np.float32)
    return kf


# Tracking state
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

        # Detect puck center from current frame
        center, area = detect_green_puck_center(frame)

        # If too many frames were missed, reset the Kalman tracker
        if missing_count >= MAX_MISSING_FRAMES and kalman_initialized:
            print(f"[INFO] Lost track after {missing_count} missing frames - resetting Kalman", flush=True)
            kalman = create_kalman()
            kalman_initialized = False
            still_count = 0

        # Predict next state
        predicted = kalman.predict()
        pred_x = float(predicted[0][0])
        pred_y = float(predicted[1][0])

        accepted_measurement = None

        # Accept or reject the current detection
        if center is not None:
            if not kalman_initialized:
                # First valid detection initializes the filter
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

                # Gate large jumps to reject bad detections
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

        # Correct Kalman state with accepted measurement
        if kalman_initialized and accepted_measurement is not None:
            measurement = np.array([
                [np.float32(accepted_measurement[0])],
                [np.float32(accepted_measurement[1])]
            ], np.float32)
            kalman.correct(measurement)

        filtered_pos = None
        filtered_vel = None

        # Extract filtered position and velocity
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

        # Prediction outputs
        intercept_pt = None
        bounce_end = None
        predicted_pt = None
        predicted_label = None
        predicted_mm = None

        # Only generate targets when we have a fresh accepted measurement
        have_fresh_track = (
            kalman_initialized
            and filtered_pos is not None
            and filtered_vel is not None
            and accepted_measurement is not None
        )

        if have_fresh_track:
            vx, vy = filtered_vel

            # Detect whether puck is still
            if abs(vx) <= STILL_THRESH_PX and abs(vy) <= STILL_THRESH_PX:
                still_count += 1
            else:
                still_count = 0

            if still_count >= STILL_CONFIRM_FRAMES:
                # STILL means use puck position directly
                predicted_label = "STILL"
                predicted_pt = filtered_pos
            else:
                # Otherwise use motion direction to predict intercept/bounce
                ux, uy = normalize(vx, vy)

                intercept_pt, hit_wall = first_border_intersection(filtered_pos, (ux, uy), w, h)

                if intercept_pt is not None:
                    predicted_pt = intercept_pt
                    predicted_label = "INTERCEPT"

                    # Only top/bottom walls generate a bounce
                    if hit_wall in ("top", "bottom"):
                        v_out = reflect((ux, uy), hit_wall)
                        bounce_end, _ = first_border_intersection(intercept_pt, v_out, w, h)
                        if bounce_end is not None:
                            predicted_pt = bounce_end
                            predicted_label = "BOUNCE"

            # Convert chosen pixel point into millimeters
            if predicted_pt is not None:
                predicted_mm = px_to_mm(predicted_pt)

                # If point is on right border, label as opponent side
                if predicted_pt[0] == (w - 1):
                    predicted_label = "RIGHT_SIDE"
                # If intercept/bounce is outside allowed robot Y range, label out of range
                elif predicted_label in ("INTERCEPT", "BOUNCE"):
                    if not (ROBOT_Y_MIN_MM <= predicted_mm[1] <= ROBOT_Y_MAX_MM):
                        predicted_label = "OUT_OF_RANGE"

            # Periodically print and publish the result
            now_t = time.time()
            if predicted_pt is not None and (PRINT_INTERVAL <= 0 or (now_t - last_print_t) >= PRINT_INTERVAL):
                print(
                    f"[PREDICTED {predicted_label}] "
                    f"x={predicted_pt[0]} y={predicted_pt[1]} "
                    f"x_mm={predicted_mm[0]:.1f} y_mm={predicted_mm[1]:.1f}",
                    flush=True
                )

                # Only write if C++ requested a new coordinate
                if cxx_requested_next():
                    publish_coordinate(
                        kind_from_label(predicted_label),
                        predicted_mm[0],
                        predicted_mm[1]
                    )

                last_print_t = now_t

        # FPS update
        frame_count += 1
        now = time.time()
        if now - start_time >= UPDATE_INTERVAL:
            fps_display = frame_count / (now - start_time)
            print(f"[FPS] {fps_display:.1f}", flush=True)
            frame_count = 0
            start_time = now

        # Optional velocity timing block
        if kalman_initialized and filtered_vel is not None:
            now_t = time.time()
            if now_t - last_vel_print_t >= VEL_PRINT_INTERVAL:
                vx, vy = filtered_vel
                speed = math.hypot(vx, vy)
                # print(f"[VELOCITY] vx={vx:.2f} vy={vy:.2f} speed={speed:.2f}", flush=True)
                last_vel_print_t = now_t

finally:
    cap.release()