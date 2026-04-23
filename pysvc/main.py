import cv2
import math
import numpy as np
import time
import glob
import os
from multiprocessing import shared_memory
import struct
import atexit

REQUEST_W = 1067
REQUEST_H = 991

TABLE_W_MM = 1066.8
TABLE_H_MM = 990.6

ROBOT_Y_MIN_MM = 247.75
ROBOT_Y_MAX_MM = 743.25


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
cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQUEST_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQUEST_H)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] Camera resolution: {actual_w}x{actual_h}", flush=True)

MM_PER_PX_X = TABLE_W_MM / actual_w
MM_PER_PX_Y = TABLE_H_MM / actual_h
print(f"[INFO] MM_PER_PX_X={MM_PER_PX_X:.6f} MM_PER_PX_Y={MM_PER_PX_Y:.6f}", flush=True)

frame_count = 0
start_time = time.time()
fps_display = 0
UPDATE_INTERVAL = 1

SCALE_X = actual_w / REQUEST_W
SCALE_Y = actual_h / REQUEST_H
SCALE_AREA = SCALE_X * SCALE_Y
SCALE_JUMP = max(SCALE_X, SCALE_Y)

MIN_AREA = max(1, int(300 * SCALE_AREA))
MAX_JUMP_PX = max(1, int(200 * SCALE_JUMP))
STILL_THRESH_PX = 8.0 * SCALE_JUMP
STILL_CONFIRM_FRAMES = 5

PRINT_INTERVAL = 1
last_print_t = 0.0

VEL_PRINT_INTERVAL = 0.25
last_vel_print_t = 0.0

SHM_NAME = "puck_xy_mm"
SHM_PATH = f"/dev/shm/{SHM_NAME}"
SHM_SIZE = 32

U32 = struct.Struct("<I")
F64 = struct.Struct("<d")

KIND_NONE = 0
KIND_STILL = 1
KIND_INTERCEPT = 2
KIND_BOUNCE = 3
KIND_RIGHT_SIDE = 4
KIND_OUT_OF_RANGE = 5

MAX_MISSING_FRAMES = 15
RELAX_JUMP_AFTER_MISSING = 5

MORPH_KERNEL = np.ones((5, 5), np.uint8)

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

    U32.pack_into(shm.buf, 0, 0)
    U32.pack_into(shm.buf, 4, 0)
    U32.pack_into(shm.buf, 8, 1)
    U32.pack_into(shm.buf, 12, 0)
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
    if label == "RIGHT_SIDE":
        return KIND_RIGHT_SIDE
    if label == "OUT_OF_RANGE":
        return KIND_OUT_OF_RANGE
    return KIND_NONE


def cxx_requested_next():
    return shm_get_u32(8) == 1


def publish_coordinate(kind, x_mm, y_mm):
    global py_seq

    py_seq += 1
    if py_seq % 2 == 0:
        py_seq += 1

    shm_set_u32(0, py_seq)

    shm_set_u32(12, kind)
    shm_set_f64(16, x_mm)
    shm_set_f64(24, y_mm)
    shm_set_u32(4, 1)
    shm_set_u32(8, 0)

    py_seq += 1
    shm_set_u32(0, py_seq)


def px_to_mm(pt):
    if pt is None:
        return None
    return (pt[0] * MM_PER_PX_X, pt[1] * MM_PER_PX_Y)


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
    kf.measurementMatrix = np.eye(2, 4, dtype=np.float32)
    kf.transitionMatrix = np.array([
        [1, 0, 1, 0],
        [0, 1, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ], dtype=np.float32)
    kf.processNoiseCov = np.diag([1e-2, 1e-2, 0.1, 0.1]).astype(np.float32)
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
        predicted_mm = None

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

                    if hit_wall in ("top", "bottom"):
                        v_out = reflect((ux, uy), hit_wall)
                        bounce_end, _ = first_border_intersection(intercept_pt, v_out, w, h)
                        if bounce_end is not None:
                            predicted_pt = bounce_end
                            predicted_label = "BOUNCE"

            if predicted_pt is not None:
                predicted_mm = px_to_mm(predicted_pt)

                if predicted_pt[0] == (w - 1):
                    predicted_label = "RIGHT_SIDE"
                elif predicted_label in ("INTERCEPT", "BOUNCE"):
                    if not (ROBOT_Y_MIN_MM <= predicted_mm[1] <= ROBOT_Y_MAX_MM):
                        predicted_label = "OUT_OF_RANGE"

            now_t = time.time()
            if predicted_pt is not None and (PRINT_INTERVAL <= 0 or (now_t - last_print_t) >= PRINT_INTERVAL):
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