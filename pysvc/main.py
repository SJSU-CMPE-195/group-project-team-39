# IPC Test — POSIX shared memory (mmap) + named semaphores
# Both containers share /dev/shm via ipc:host in docker-compose.yml.
# Three receive methods are demonstrated sequentially on the main thread
# while a writer thread sends a counter byte every 500 ms.

import mmap
import posix_ipc
import signal
import threading
import time

# ─────────────────────────────────────────────────────────────────────────────
# Shared memory layout  (64 bytes total, extensible via _reserved)
# This layout must stay byte-for-byte identical to IPCBlock in cppsvc/main.cpp.
#
#   offset 0: py_ready   — pysvc writes 1 when py_data is valid
#   offset 1: py_data    — payload byte: pysvc → cppsvc
#   offset 2: cpp_ready  — cppsvc writes 1 when cpp_data is valid
#   offset 3: cpp_data   — payload byte: cppsvc → pysvc
#   offset 4–63: reserved for future fields (coordinates, status, etc.)
# ─────────────────────────────────────────────────────────────────────────────

SHM_NAME   = "/ipc_block"
SEM_PY_CPP = "/sem_py_to_cpp"
SEM_CPP_PY = "/sem_cpp_to_py"
SHM_SIZE   = 64

g_running = True

def _handle_signal(sig, frame):
    global g_running
    g_running = False

signal.signal(signal.SIGINT,  _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)

# ── Open / create shared memory ──────────────────────────────────────────────
# O_CREAT creates the region if absent, opens it if already present.
# size= is only applied when creating; the existing size is kept otherwise.
shm = posix_ipc.SharedMemory(SHM_NAME, posix_ipc.O_CREAT, mode=0o666, size=SHM_SIZE)
mm  = mmap.mmap(shm.fd, SHM_SIZE)
shm.close_fd()  # fd is no longer needed once the mapping is established

# ── Open / create named semaphores ────────────────────────────────────────────
# initial_value=0 means "not ready"; O_CREAT is ignored for existing semaphores.
sem_py_cpp = posix_ipc.Semaphore(SEM_PY_CPP, posix_ipc.O_CREAT, mode=0o666, initial_value=0)
sem_cpp_py = posix_ipc.Semaphore(SEM_CPP_PY, posix_ipc.O_CREAT, mode=0o666, initial_value=0)

print(f"[pysvc] IPC ready  shm={SHM_NAME}", flush=True)

# ─────────────────────────────────────────────────────────────────────────────
# Low-level accessors
# mm[offset] returns an int in Python 3; avoids the seek-position race that
# would affect the paired seek() + read_byte() approach under threading.
# ─────────────────────────────────────────────────────────────────────────────

def _rb(offset: int) -> int:
    return mm[offset]

def _wb(offset: int, value: int) -> None:
    mm[offset] = value & 0xFF

def _send_py(data: int) -> None:
    """Write py_data (offset 1) then set py_ready flag (offset 0)."""
    _wb(1, data)  # write data before flag — mirrors __sync_synchronize() in C++
    _wb(0, 1)

def _consume_cpp() -> int:
    """Read cpp_data (offset 3) then clear cpp_ready flag (offset 2)."""
    val = _rb(3)
    _wb(2, 0)
    return val

def _cpp_ready() -> bool:
    return _rb(2) != 0

# ─────────────────────────────────────────────────────────────────────────────
# Receive Method 1 — Polling
# Sleeps 1 ms between flag checks. Low CPU cost; ~1 ms latency floor.
# ─────────────────────────────────────────────────────────────────────────────
def poll_receive() -> int:
    while g_running:
        if _cpp_ready():
            return _consume_cpp()
        time.sleep(0.001)
    return 0

# ─────────────────────────────────────────────────────────────────────────────
# Receive Method 2 — Spin (busy-wait)
# Loops with no sleep. Lowest possible latency; consumes a full CPU core.
# ─────────────────────────────────────────────────────────────────────────────
def spin_receive() -> int:
    while g_running and not _cpp_ready():
        pass
    return _consume_cpp()

# ─────────────────────────────────────────────────────────────────────────────
# Receive Method 3 — Non-blocking semaphore (trywait + yield)
# acquire(timeout=0) returns immediately — raises BusyError if count is 0.
# Equivalent to sem_trywait in C++.
# ─────────────────────────────────────────────────────────────────────────────
def semaphore_nonblocking_receive() -> int:
    while g_running:
        try:
            sem_cpp_py.acquire(timeout=0)   # non-blocking; raises BusyError if not ready
            return _consume_cpp()
        except posix_ipc.BusyError:
            pass                            # not ready — yield implicitly and retry
    return 0

# ── Writer thread: send a counter byte to cppsvc every 500 ms ─────────────────
def _writer() -> None:
    counter = 0
    while g_running:
        _send_py(counter)
        sem_py_cpp.release()
        print(f"[pysvc] SENT        byte={counter}", flush=True)
        counter = (counter + 1) % 256
        time.sleep(0.5)

writer_thread = threading.Thread(target=_writer, daemon=True)
writer_thread.start()

# ── Reader: demonstrate all three receive methods in sequence ─────────────────
print("[pysvc] Waiting for cppsvc data (POLL)...", flush=True)
v1 = poll_receive()
print(f"[pysvc] POLL        received byte={v1}", flush=True)

print("[pysvc] Waiting for cppsvc data (SPIN)...", flush=True)
v2 = spin_receive()
print(f"[pysvc] SPIN        received byte={v2}", flush=True)

print("[pysvc] Waiting for cppsvc data (SEM NON-BLOCK)...", flush=True)
v3 = semaphore_nonblocking_receive()
print(f"[pysvc] SEM         received byte={v3}", flush=True)

print("[pysvc] All three receive methods completed. Exiting.", flush=True)
g_running = False
writer_thread.join(timeout=1)
mm.close()
# Note: shm and semaphores are intentionally not unlinked here so the other
# container can finish accessing them after this one exits.

# ─────────────────────────────────────────────────────────────────────────────
# Original camera / vision code — preserved below, commented out
# ─────────────────────────────────────────────────────────────────────────────

# import cv2
# import numpy as np
# import glob

# def open_camera():
#     devices = sorted(glob.glob('/dev/video*'))
#     print(f"Found video devices: {devices}", flush=True)
#     for dev in devices:
#         c = cv2.VideoCapture(dev, cv2.CAP_V4L2)
#         if c.isOpened():
#             ret, _ = c.read()
#             if ret:
#                 print(f"Camera found at {dev}", flush=True)
#                 return c
#             c.release()
#     raise RuntimeError(f"No working camera found. Tried: {devices}")

# cap = open_camera()
# cap.set(cv2.CAP_PROP_FPS, 120)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

# frame_count = 0
# start_time = time.time()
# fps_display = 0
# UPDATE_INTERVAL = 1
# MIN_AREA = 300
# MAX_JUMP_PX = 200
# STILL_THRESH_PX = 5
# STILL_CONFIRM_FRAMES = 2
# PRINT_INTERVAL = 0.05
# last_print_t = 0.0
# missing_count = 0
# MAX_MISSING_TO_FORCE_ACCEPT = 3
# AREA_GAIN_THRESHOLD = 1.3

# def green_mask_hsv(frame_bgr):
#     hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
#     lower1 = np.array([35, 60, 60], dtype=np.uint8)
#     upper1 = np.array([85, 255, 255], dtype=np.uint8)
#     mask = cv2.inRange(hsv, lower1, upper1)
#     kernel = np.ones((5, 5), np.uint8)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
#     return mask

# def detect_green_puck_center(frame_bgr):
#     h, w = frame_bgr.shape[:2]
#     mask = green_mask_hsv(frame_bgr)
#     contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     if not contours:
#         return None, 0
#     c = max(contours, key=cv2.contourArea)
#     area = cv2.contourArea(c)
#     if area < MIN_AREA:
#         return None, 0
#     M = cv2.moments(c)
#     if M["m00"] == 0:
#         return None, 0
#     cx = int(np.clip(int(M["m10"] / M["m00"]), 0, w - 1))
#     cy = int(np.clip(int(M["m01"] / M["m00"]), 0, h - 1))
#     return (cx, cy), area

# def first_border_intersection(p0, v, w, h):
#     x0, y0 = float(p0[0]), float(p0[1])
#     vx, vy = float(v[0]), float(v[1])
#     eps = 1e-9
#     candidates = []
#     if abs(vx) > eps:
#         for tx, lbl in [((0 - x0) / vx, "left"), (((w-1) - x0) / vx, "right")]:
#             if tx > 0:
#                 y = y0 + tx * vy
#                 if 0 <= y <= h - 1:
#                     candidates.append((tx, (0 if lbl == "left" else w-1, int(round(y))), lbl))
#     if abs(vy) > eps:
#         for ty, lbl in [((0 - y0) / vy, "top"), (((h-1) - y0) / vy, "bottom")]:
#             if ty > 0:
#                 x = x0 + ty * vx
#                 if 0 <= x <= w - 1:
#                     candidates.append((ty, (int(round(x)), 0 if lbl == "top" else h-1), lbl))
#     if not candidates:
#         return None, None
#     candidates.sort(key=lambda item: item[0])
#     _, pt, wall = candidates[0]
#     return pt, wall

# def reflect(v, wall):
#     vx, vy = float(v[0]), float(v[1])
#     return (-vx, vy) if wall in ("left", "right") else (vx, -vy)

# def normalize(vx, vy, eps=1e-9):
#     mag = (vx*vx + vy*vy) ** 0.5
#     return (0.0, 0.0) if mag < eps else (vx/mag, vy/mag)

# p0 = p1 = tracked_area = None
# still_count = 0
# prev_detected = False
# print("Camera started.", flush=True)

# try:
#     while True:
#         ret, frame = cap.read()
#         if not ret:
#             print("Failed to read frame from camera.", flush=True)
#             break
#         h, w = frame.shape[:2]
#         center, area = detect_green_puck_center(frame)
#         if center is not None:
#             cond_missing_relax = (missing_count >= MAX_MISSING_TO_FORCE_ACCEPT)
#             missing_count = 0
#             if p1 is None:
#                 p1 = center; tracked_area = area
#             else:
#                 dx, dy = center[0]-p1[0], center[1]-p1[1]
#                 if (dx*dx+dy*dy <= MAX_JUMP_PX**2) or \
#                    (tracked_area and area > tracked_area*AREA_GAIN_THRESHOLD) or \
#                    cond_missing_relax:
#                     p0 = p1; p1 = center; tracked_area = area
#         else:
#             missing_count += 1
#             if missing_count > MAX_MISSING_TO_FORCE_ACCEPT:
#                 if prev_detected:
#                     print("Object Disappeared", flush=True)
#                 p0 = p1 = tracked_area = None; still_count = 0
#         prev_detected = (p1 is not None)
#         predicted_pt = predicted_label = None
#         if p0 is not None and p1 is not None:
#             vx_raw, vy_raw = p1[0]-p0[0], p1[1]-p0[1]
#             if abs(vx_raw) <= STILL_THRESH_PX and abs(vy_raw) <= STILL_THRESH_PX:
#                 still_count += 1
#             else:
#                 still_count = 0
#             if still_count >= STILL_CONFIRM_FRAMES:
#                 predicted_label = "STILL"; predicted_pt = p1
#             else:
#                 ux, uy = normalize(float(vx_raw), float(vy_raw))
#                 intercept_pt, hit_wall = first_border_intersection(p1, (ux, uy), w, h)
#                 if intercept_pt is not None:
#                     predicted_pt = intercept_pt; predicted_label = "INTERCEPT"
#                     if hit_wall in ("top", "bottom"):
#                         v_out = reflect((ux, uy), hit_wall)
#                         bounce_end, _ = first_border_intersection(intercept_pt, v_out, w, h)
#                         if bounce_end is not None:
#                             predicted_pt = bounce_end; predicted_label = "BOUNCE"
#             now_t = time.time()
#             if predicted_pt is not None and (now_t - last_print_t) >= PRINT_INTERVAL:
#                 print(f"[PREDICTED {predicted_label}] x={predicted_pt[0]} y={predicted_pt[1]}", flush=True)
#                 last_print_t = now_t
#         frame_count += 1
#         now = time.time()
#         if now - start_time >= UPDATE_INTERVAL:
#             print(f"[FPS] {frame_count/(now-start_time):.1f}", flush=True)
#             frame_count = 0; start_time = now
# finally:
#     cap.release()
#     cv2.destroyAllWindows()

# import urllib.request, sys
# URL = "http://127.0.0.1:8080/ping"
# try:
#     with urllib.request.urlopen(URL, timeout=3) as r:
#         body = r.read().decode("utf-8", errors="replace").strip()
#         print(f"[pysvc] GET /ping -> {r.status}, body='{body}'")
#     sys.exit(0)
# except Exception as e:
#     print(f"[pysvc] ping failed: {e}")
#     sys.exit(1)
