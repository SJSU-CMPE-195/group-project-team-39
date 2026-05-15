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
REQUEST_W = 2560
REQUEST_H = 1440

ROI_X, ROI_Y, ROI_W, ROI_H = 274, 19, 629, 672
#155, 21, 651, 686
#154, 11, 622, 691

# X calibration bounds for edge clamping
X_LEFT_BOUND = 0
X_RIGHT_BOUND = 672
EDGE_RANGE = 45

# Only allow INTERCEPT targets inside this pixel X range
INTERCEPT_X_MIN_PX = 230
INTERCEPT_X_MAX_PX = 430

INTERCEPT_Y_MIN_PX = 0
INTERCEPT_Y_MAX_PX = 95

STILL_Y_MAX_PX = ROI_W
STILL_Y_DIFF = 100

# Rotate cropped ROI 90 degrees clockwise
ROTATE_CAMERA_CW = True

# Processed frame size after crop/rotation
if ROTATE_CAMERA_CW:
    PROC_W = ROI_H
    PROC_H = ROI_W
else:
    PROC_W = ROI_W
    PROC_H = ROI_H

# Physical table dimensions in millimeters
TABLE_W_MM = 869
TABLE_H_MM = 901


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


def preprocess_frame(frame):
    # Crop first using the original raw-camera ROI
    frame = frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]

    # Then rotate the cropped ROI 90 degrees clockwise.
    # All x/y coordinates after this point use the rotated frame.
    if ROTATE_CAMERA_CW:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    return frame


# Open the camera and request the desired frame rate / resolution
cap = open_camera()
cap.set(cv2.CAP_PROP_FPS, 120)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUEST_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUEST_H)

# Read the actual resolution returned by the camera
actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"[INFO] Camera source resolution: {actual_w}x{actual_h}", flush=True)
print(f"[INFO] Raw ROI: x={ROI_X} y={ROI_Y} w={ROI_W} h={ROI_H}", flush=True)
print(f"[INFO] Rotate CW: {ROTATE_CAMERA_CW}", flush=True)
print(f"[INFO] Processed frame resolution: {PROC_W}x{PROC_H}", flush=True)

# Pixel-to-mm conversion for processed frame
# After rotation, PROC_W/PROC_H are used instead of ROI_W/ROI_H.
MM_PER_PX_X = TABLE_W_MM / PROC_W
MM_PER_PX_Y = TABLE_H_MM / PROC_H

# Scale some thresholds using actual delivered resolution
SCALE_X = actual_w / REQUEST_W
SCALE_Y = actual_h / REQUEST_H
SCALE_AREA = SCALE_X * SCALE_Y
SCALE_JUMP = max(SCALE_X, SCALE_Y)

# Detection / tracking thresholds
MIN_AREA = max(1, int(300 * SCALE_AREA))
MAX_JUMP_PX = max(1, int(200 * SCALE_JUMP))
STILL_THRESH_PX = 8.0 * SCALE_JUMP
STILL_CONFIRM_FRAMES = 20

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
KIND_TOP_SIDE = 3
KIND_NO_TRACK = 4

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
    except Exception:
        pass


def clear_shm():
    # Clear shared memory safely using the seq protocol.
    # Odd seq means "write/reset in progress".
    U32.pack_into(shm.buf, 0, 1)

    # Reset shared-memory fields
    U32.pack_into(shm.buf, 4, 0)           # ready
    U32.pack_into(shm.buf, 8, 1)           # request_next
    U32.pack_into(shm.buf, 12, KIND_NONE)  # kind
    F64.pack_into(shm.buf, 16, 0.0)        # x_mm
    F64.pack_into(shm.buf, 24, 0.0)        # y_mm

    # Even seq means "stable completed reset".
    U32.pack_into(shm.buf, 0, 2)


# Create or attach to shared memory
try:
    shm = shared_memory.SharedMemory(
        name=SHM_NAME,
        create=True,
        size=SHM_SIZE
    )

    clear_shm()
    chmod_shm()

except FileExistsError:
    shm = shared_memory.SharedMemory(
        name=SHM_NAME,
        create=False,
        size=SHM_SIZE
    )

    clear_shm()
    chmod_shm()


# Sequence counter used to mark stable writes.
# Start from 2 because clear_shm() ends with seq = 2.
py_seq = 2


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
    # Handle Ctrl+C / container stop more cleanly
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

    if label == "TOP_SIDE":
        return KIND_TOP_SIDE

    if label == "NO_TRACK":
        return KIND_NO_TRACK

    return KIND_NONE


def cxx_requested_next():
    # C++ sets request_next = 1 when it wants a fresh coordinate
    return shm_get_u32(8) == 1


def publish_coordinate(kind, x_mm, y_mm):
    # Publish one stable coordinate packet into shared memory
    global py_seq

    t0 = time.perf_counter()

    py_seq += 1
    if py_seq % 2 == 0:
        py_seq += 1

    # Odd seq means "write in progress"
    shm_set_u32(0, py_seq)

    if x_mm > 230.0:
        x_mm = x_mm + x_mm * 0.07

    if x_mm >= 869.0:
        x_mm = 869.0

    # Write payload
    shm_set_u32(12, kind)
    shm_set_f64(16, x_mm)
    shm_set_f64(24, y_mm)
    shm_set_u32(4, 1)  # ready
    shm_set_u32(8, 0)  # consume request_next

    # Even seq means "stable completed write"
    py_seq += 1
    shm_set_u32(0, py_seq)

    return (time.perf_counter() - t0) * 1000.0

def px_to_mm(pt):
    # Convert a pixel coordinate into millimeters.
    # OpenCV image coordinates use top-left origin:
    #   y_px = 0 is near the top of the image.
    #   y_phys_px = 0 is physical bottom / your side.
    # So we flip y once and use y_phys_px for both:
    #   1. normal y_mm conversion
    #   2. y edge clamping
    if pt is None:
        return None

    x_px = pt[0]
    y_px = pt[1]

    # Convert OpenCV top-origin y into physical bottom-origin y
    y_phys_px = PROC_H - 1 - y_px

    # Normal pixel-to-mm conversion
    x_mm = x_px * MM_PER_PX_X
    y_mm = y_phys_px * MM_PER_PX_Y

    # Clamp x near physical left edge
    if x_px >= X_LEFT_BOUND and x_px <= X_LEFT_BOUND + EDGE_RANGE:
        x_mm = 0.0

    # Clamp x near physical right edge
    elif x_px >= X_RIGHT_BOUND - EDGE_RANGE and x_px <= X_RIGHT_BOUND:
        x_mm = 869.0

    # Clamp y near physical bottom edge
    # Use y_phys_px, not raw y_px.
    if y_phys_px >= INTERCEPT_Y_MIN_PX and y_phys_px <= INTERCEPT_Y_MAX_PX:
        y_mm = 0.0

    # Clamp y near physical top edge
    # Use y_phys_px, not raw y_px.
    if y_phys_px <= STILL_Y_MAX_PX and y_phys_px >= STILL_Y_MAX_PX - STILL_Y_DIFF:
        y_mm = 901.0

    return x_mm, y_mm

# def px_to_mm(pt):
#     # Convert a pixel coordinate into millimeters using bottom-left
#     # as the physical origin of the processed rotated frame.
#     if pt is None:
#         return None

#     x_px = pt[0]
#     y_px = pt[1]

#     x_mm = x_px * MM_PER_PX_X
#     y_mm = (PROC_H - 1 - y_px) * MM_PER_PX_Y

#     if x_px >= X_LEFT_BOUND and x_px <= X_LEFT_BOUND + EDGE_RANGE:
#         x_mm = 0.0

#     elif x_px >= X_RIGHT_BOUND - EDGE_RANGE and x_px <= X_RIGHT_BOUND:
#         x_mm = 869.0

#     if y_px >= INTERCEPT_Y_MIN_PX and y_px <= INTERCEPT_Y_MAX_PX:
#         y_mm = 0.0

#     return x_mm, y_mm

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

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

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
        return -vx, vy

    if wall in ("top", "bottom"):
        return vx, -vy

    return vx, vy


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
    kf.processNoiseCov = np.diag([
        1e-2,
        1e-2,
        0.1,
        0.1
    ]).astype(np.float32)

    kf.measurementNoiseCov = (
        5e-2 * np.eye(2)
    ).astype(np.float32)

    # Initial state covariance
    kf.errorCovPost = np.eye(4, dtype=np.float32)

    return kf


# Tracking state
kalman = create_kalman()
kalman_initialized = False

missing_count = 0
still_count = 0

try:
    while True:
        ret, raw_frame = cap.read()

        if not ret:
            break

        # Crop ROI, then rotate 90 degrees clockwise.
        # Every coordinate after this is in the rotated frame.
        frame = preprocess_frame(raw_frame)
        h, w = frame.shape[:2]

        # Detect puck center from current processed frame
        center, area = detect_green_puck_center(frame)

        # If too many frames were missed, reset the Kalman tracker
        # and notify C++ that tracking was lost so it can move to rest point.
        if missing_count >= MAX_MISSING_FRAMES and kalman_initialized:
            kalman = create_kalman()
            kalman_initialized = False
            still_count = 0

            if cxx_requested_next():
                dt_ms = publish_coordinate(
                    kind_from_label("NO_TRACK"),
                    0.0,
                    0.0
                )

                print(
                    f"[PY->C++] kind=NO_TRACK x=0 y=0 "
                    f"x_mm=0.0 y_mm=0.0 "
                    f"send_time_ms={dt_ms:.3f}",
                    flush=True
                )

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

                intercept_pt, hit_wall = first_border_intersection(
                    filtered_pos,
                    (ux, uy),
                    w,
                    h
                )

                if intercept_pt is not None:
                    predicted_pt = intercept_pt
                    predicted_label = "INTERCEPT"

                    # Left/right walls can bounce
                    if hit_wall in ("left", "right"):
                        v_out = reflect((ux, uy), hit_wall)

                        bounce_end, _ = first_border_intersection(
                            intercept_pt,
                            v_out,
                            w,
                            h
                        )

                        if bounce_end is not None:
                            # Send the post-bounce intercept to C++
                            predicted_pt = bounce_end
                            predicted_label = "INTERCEPT"

            # Convert chosen pixel point into millimeters
            if predicted_pt is not None:
                predicted_mm = px_to_mm(predicted_pt)

                # Only bottom-wall targets are usable intercepts.
                # Top wall is opponent side, and left/right wall hits are not move targets.
                if predicted_label == "STILL":
                    pass
                elif predicted_pt[1] == (h - 1):
                    predicted_label = "INTERCEPT"
                elif predicted_pt[1] == 0:
                    predicted_label = "TOP_SIDE"
                else:
                    predicted_label = "TOP_SIDE"

                # Only send INTERCEPT if its x pixel is inside the allowed range.
                # STILL, TOP_SIDE, and NO_TRACK behavior stay unchanged.
                should_publish = True

                if predicted_label == "INTERCEPT":
                    if not (
                        INTERCEPT_X_MIN_PX
                        <= predicted_pt[0]
                        <= INTERCEPT_X_MAX_PX
                    ):
                        should_publish = False

                # Only write if C++ requested a new coordinate and the target is allowed
                if cxx_requested_next() and should_publish:
                    dt_ms = publish_coordinate(
                        kind_from_label(predicted_label),
                        predicted_mm[0],
                        predicted_mm[1]
                    )

                    # Print y as bottom-left coordinate in pixels
                    y_bottom_left_px = PROC_H - 1 - predicted_pt[1]

                    print(
                        f"[PY->C++] kind={predicted_label} "
                        f"x={predicted_pt[0]} y={y_bottom_left_px} "
                        f"x_mm={predicted_mm[0]:.1f} "
                        f"y_mm={predicted_mm[1]:.1f} "
                        f"send_time_ms={dt_ms:.3f}",
                        flush=True
                    )

                elif cxx_requested_next() and not should_publish:
                    y_bottom_left_px = PROC_H - 1 - predicted_pt[1]

                    print(
                        f"[PY] skipped INTERCEPT outside range "
                        f"x={predicted_pt[0]} y={y_bottom_left_px} "
                        f"allowed_x={INTERCEPT_X_MIN_PX}-{INTERCEPT_X_MAX_PX}",
                        flush=True
                    )

finally:
    cap.release()