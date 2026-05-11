import cv2
import glob
import sys
import numpy as np
from scipy.interpolate import RBFInterpolator

# ==============================================================================
# SETTINGS — must match nam.py
# ==============================================================================
CAPTURE_WIDTH  = 2560        # native stereo output: 2x(1280x720)
CAPTURE_HEIGHT = 720
FRAME_WIDTH    = CAPTURE_WIDTH // 2   # 1280, after cropping to left eye
FRAME_HEIGHT   = CAPTURE_HEIGHT       # 720
GRAB_R         = 22

# Remap output (matches nam.py — pre-rotation, landscape)
PREVIEW_W = 510
PREVIEW_H = 400

# Real-world critical zone size (centimeters)
CRITICAL_ZONE_W_CM = 55.0
CRITICAL_ZONE_H_CM = 35.0


# ==============================================================================
# HANDLE GROUPS
# ==============================================================================
# 6 remap points (TL, T-mid, TR, BR, B-mid, BL).
# Positions are filled in once the camera reports its real frame size, so
# the handles are guaranteed to land inside the visible image.
remap_pts    = [[0, 0] for _ in range(6)]
REMAP_LABELS = ["TL", "T-mid", "TR", "BR", "B-mid", "BL"]

def _default_remap_pts(w, h):
    """Place all 6 handles well inside a (w x h) frame."""
    return [
        [int(w * 0.25), int(h * 0.15)],   # 0 TL
        [int(w * 0.50), int(h * 0.10)],   # 1 T-mid
        [int(w * 0.75), int(h * 0.15)],   # 2 TR
        [int(w * 0.75), int(h * 0.85)],   # 3 BR
        [int(w * 0.50), int(h * 0.90)],   # 4 B-mid
        [int(w * 0.25), int(h * 0.85)],   # 5 BL
    ]
REMAP_COLOR  = (0, 220, 180)        # teal

# These three handles work in PREVIEW (rotated) coordinate space.
# Mallet origin = where mallet sits at motor 0,0.
# Critical zone TL/BR define the rectangle the mallet defends.
# After rotation: width=PREVIEW_H, height=PREVIEW_W.
mallet_origin = [PREVIEW_H // 2, PREVIEW_W - 80]                # near bottom centre
crit_tl       = [PREVIEW_H // 4,            PREVIEW_W - 200]
crit_br       = [PREVIEW_H - PREVIEW_H // 4, PREVIEW_W - 60]

MALLET_COLOR = (0, 200, 255)        # orange
CRIT_COLOR   = (255, 100, 0)        # blue-ish

drag_target = None    # ("remap", idx) or ("mallet", None) or ("crit_tl"/"crit_br", None)
remap_dirty = True
_map_x = _map_y = None

# Real displayed frame size — set once the first frame is captured.
disp_w = FRAME_WIDTH
disp_h = FRAME_HEIGHT


# ==============================================================================
# DST points for the remap (matches main.py)
# ==============================================================================
def _dst_points():
    return np.array([
        [0,               0              ],   # TL
        [PREVIEW_W // 2,  0              ],   # T-mid
        [PREVIEW_W - 1,   0              ],   # TR
        [PREVIEW_W - 1,   PREVIEW_H - 1  ],   # BR
        [PREVIEW_W // 2,  PREVIEW_H - 1  ],   # B-mid
        [0,               PREVIEW_H - 1  ],   # BL
    ], dtype=np.float64)


def _build_remap():
    global _map_x, _map_y, remap_dirty
    src   = np.array(remap_pts, dtype=np.float64)
    dst   = _dst_points()
    rbf_x = RBFInterpolator(dst, src[:, 0], kernel="thin_plate_spline")
    rbf_y = RBFInterpolator(dst, src[:, 1], kernel="thin_plate_spline")
    gx, gy = np.meshgrid(np.arange(PREVIEW_W), np.arange(PREVIEW_H))
    grid   = np.stack([gx.ravel(), gy.ravel()], axis=1).astype(np.float64)
    _map_x = rbf_x(grid).reshape(PREVIEW_H, PREVIEW_W).astype(np.float32)
    _map_y = rbf_y(grid).reshape(PREVIEW_H, PREVIEW_W).astype(np.float32)
    remap_dirty = False


# ==============================================================================
# CAMERA
# ==============================================================================
def open_camera():
    if sys.platform == "darwin":
        return cv2.VideoCapture(0)
    for dev in sorted(glob.glob("/dev/video*")):
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                return cap
            cap.release()
    raise RuntimeError("No camera found")


# ==============================================================================
# MOUSE HELPERS
# ==============================================================================
def _hit(x, y, px, py):
    return abs(x - px) < GRAB_R and abs(y - py) < GRAB_R


def mouse_raw(event, x, y, flags, param):
    """Drag handles in the RAW (left) window — only the 8 remap points live here."""
    global drag_target, remap_dirty
    if event == cv2.EVENT_LBUTTONDOWN:
        for i, (px, py) in enumerate(remap_pts):
            if _hit(x, y, px, py):
                drag_target = ("remap", i)
                return
    elif event == cv2.EVENT_MOUSEMOVE and drag_target and drag_target[0] == "remap":
        idx = drag_target[1]
        remap_pts[idx][0] = int(np.clip(x, 0, disp_w - 1))
        remap_pts[idx][1] = int(np.clip(y, 0, disp_h - 1))
    elif event == cv2.EVENT_LBUTTONUP and drag_target and drag_target[0] == "remap":
        drag_target = None
        remap_dirty = True


def mouse_preview(event, x, y, flags, param):
    """Drag mallet origin + critical zone corners in the PREVIEW (right) window."""
    global drag_target
    pw, ph = PREVIEW_H, PREVIEW_W   # rotated dims

    if event == cv2.EVENT_LBUTTONDOWN:
        for tag, pt in (("mallet", mallet_origin),
                        ("crit_tl", crit_tl),
                        ("crit_br", crit_br)):
            if _hit(x, y, pt[0], pt[1]):
                drag_target = (tag, None)
                return
    elif event == cv2.EVENT_MOUSEMOVE and drag_target:
        tag = drag_target[0]
        nx, ny = int(np.clip(x, 0, pw - 1)), int(np.clip(y, 0, ph - 1))
        if   tag == "mallet":  mallet_origin[0], mallet_origin[1] = nx, ny
        elif tag == "crit_tl": crit_tl[0],       crit_tl[1]       = nx, ny
        elif tag == "crit_br": crit_br[0],       crit_br[1]       = nx, ny
    elif event == cv2.EVENT_LBUTTONUP:
        drag_target = None


# ==============================================================================
# DRAW HELPERS
# ==============================================================================
def draw_handle(img, x, y, color, label):
    cv2.circle(img, (x, y), GRAB_R, color, 2)
    cv2.circle(img, (x, y), 4,      color, -1)
    cv2.putText(img, label, (x + GRAB_R + 4, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def draw_raw(frame):
    disp = frame.copy()
    pts  = np.array(remap_pts, dtype=np.int32)

    overlay = disp.copy()
    cv2.fillPoly(overlay, [pts], (200, 200, 100))
    cv2.addWeighted(overlay, 0.12, disp, 0.88, 0, disp)
    cv2.polylines(disp, [pts], isClosed=True, color=(255, 255, 255), thickness=1)

    for i, (px, py) in enumerate(remap_pts):
        draw_handle(disp, px, py, REMAP_COLOR, REMAP_LABELS[i])

    cv2.putText(disp, "RAW — drag 8 remap handles to table edges",
                (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return disp


def draw_preview(frame):
    if remap_dirty or _map_x is None:
        ph = np.zeros((PREVIEW_W, PREVIEW_H, 3), dtype=np.uint8)
        cv2.putText(ph, "Building preview...",
                    (20, PREVIEW_W // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 1)
        return ph

    remapped = cv2.remap(frame, _map_x, _map_y, cv2.INTER_LINEAR)
    rotated  = cv2.rotate(remapped, cv2.ROTATE_90_CLOCKWISE)
    pw, ph   = rotated.shape[1], rotated.shape[0]

    cv2.rectangle(rotated, (0, 0), (pw - 1, ph - 1), (255, 255, 255), 1)
    cv2.line(rotated, (0, ph - 1), (pw - 1, ph - 1), (0, 200, 255), 3)
    cv2.putText(rotated, "GOAL", (pw // 2 - 25, ph - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    # Critical zone rectangle
    x0, y0 = min(crit_tl[0], crit_br[0]), min(crit_tl[1], crit_br[1])
    x1, y1 = max(crit_tl[0], crit_br[0]), max(crit_tl[1], crit_br[1])
    cv2.rectangle(rotated, (x0, y0), (x1, y1), CRIT_COLOR, 2)

    # Mallet origin marker (axis cross)
    mx, my = mallet_origin
    cv2.drawMarker(rotated, (mx, my), MALLET_COLOR, cv2.MARKER_CROSS, 30, 2)
    cv2.circle(rotated, (mx, my), 6, MALLET_COLOR, -1)

    # Handles
    draw_handle(rotated, mx, my, MALLET_COLOR, "MALLET 0,0")
    draw_handle(rotated, *crit_tl, CRIT_COLOR, "CRIT-TL")
    draw_handle(rotated, *crit_br, CRIT_COLOR, "CRIT-BR")

    # Live measurements
    crit_px_w = abs(crit_br[0] - crit_tl[0])
    crit_px_h = abs(crit_br[1] - crit_tl[1])
    px_per_cm_x = crit_px_w / CRITICAL_ZONE_W_CM if crit_px_w > 0 else 0
    px_per_cm_y = crit_px_h / CRITICAL_ZONE_H_CM if crit_px_h > 0 else 0

    info = [
        f"Critical zone: {crit_px_w}px x {crit_px_h}px  ({CRITICAL_ZONE_W_CM:.0f}cm x {CRITICAL_ZONE_H_CM:.0f}cm)",
        f"px/cm  X: {px_per_cm_x:.2f}   Y: {px_per_cm_y:.2f}",
        f"Mallet origin (px): ({mx}, {my})",
        "S = save  |  ESC = quit",
    ]
    for i, line in enumerate(info):
        cv2.putText(rotated, line, (8, 22 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return rotated


def main():
    global remap_dirty

    cap = open_camera()
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 100)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[INFO] Camera: {actual_w}x{actual_h}", flush=True)

    win_raw  = "RAW — drag remap handles"
    win_prev = "PREVIEW — drag mallet origin + critical zone"

    cv2.namedWindow(win_raw,  cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_prev, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_raw,  FRAME_WIDTH // 2,  FRAME_HEIGHT // 2)
    cv2.resizeWindow(win_prev, PREVIEW_H,     PREVIEW_W)
    cv2.setMouseCallback(win_raw,  mouse_raw)
    cv2.setMouseCallback(win_prev, mouse_preview)

    print("\n=== Calibration ===")
    print("LEFT  (raw)     — drag 8 teal handles to your table edges")
    print("RIGHT (preview) — drag the orange MALLET 0,0 cross to where mallet rests")
    print("                  drag the two blue CRIT corners to mark a 55cm × 35cm zone")
    print("                  (use your physical ruler on the table to size it)")
    print("Press  S  to save full config    ESC  to quit\n")

    # Grab one frame so we know the ACTUAL displayed dimensions, then place
    # the 6 remap handles inside it.  This avoids the off-screen-default bug
    # when the camera reports a smaller frame than FRAME_WIDTH/FRAME_HEIGHT.
    global disp_w, disp_h
    ret, first_frame = cap.read()
    if not ret:
        raise RuntimeError("Couldn't read a frame from the camera.")
    if first_frame.shape[1] >= CAPTURE_WIDTH:
        first_frame = first_frame[:, :FRAME_WIDTH]
    disp_h, disp_w = first_frame.shape[:2]
    for i, pt in enumerate(_default_remap_pts(disp_w, disp_h)):
        remap_pts[i][0] = pt[0]
        remap_pts[i][1] = pt[1]
    print(f"[INFO] Displayed frame size: {disp_w}x{disp_h}", flush=True)
    print(f"[INFO] Handles placed inside frame.", flush=True)

    # Size the RAW window to match the displayed frame so mouse coords
    # always align with the image (OpenCV maps clicks to image space).
    cv2.resizeWindow(win_raw, disp_w, disp_h)

    print("[INFO] Building initial preview...", flush=True)
    _build_remap()
    print("[INFO] Ready.", flush=True)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Stereo cam outputs 2560x720 side-by-side; keep only the left eye
        # (matches what nam.py does in preprocess_frame).
        if frame.shape[1] >= CAPTURE_WIDTH:
            frame = frame[:, :FRAME_WIDTH]

        if remap_dirty:
            print("[INFO] Rebuilding remap...", flush=True)
            _build_remap()
            print("[INFO] Preview updated.", flush=True)

        cv2.imshow(win_raw,  draw_raw(frame))
        cv2.imshow(win_prev, draw_preview(frame))

        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            crit_px_w = abs(crit_br[0] - crit_tl[0])
            crit_px_h = abs(crit_br[1] - crit_tl[1])
            if crit_px_w < 10 or crit_px_h < 10:
                print("⚠️  Critical zone too small — drag the corners apart first.")
                continue

            px_per_cm_x = crit_px_w / CRITICAL_ZONE_W_CM
            px_per_cm_y = crit_px_h / CRITICAL_ZONE_H_CM

            x0, y0 = min(crit_tl[0], crit_br[0]), min(crit_tl[1], crit_br[1])
            x1, y1 = max(crit_tl[0], crit_br[0]), max(crit_tl[1], crit_br[1])

            print("\n✅ Paste into main.py:\n")
            print(f"OUTPUT_W, OUTPUT_H = {PREVIEW_W}, {PREVIEW_H}\n")

            print("SRC_POINTS = [")
            for i, (px, py) in enumerate(remap_pts):
                print(f"    ({float(px)}, {float(py)}),   # {REMAP_LABELS[i]}")
            print("]\n")

            print(f"# Mallet origin in remapped+rotated pixel space")
            print(f"MALLET_ORIGIN_PX = ({mallet_origin[0]}, {mallet_origin[1]})\n")

            print(f"# Critical zone in remapped+rotated pixel space")
            print(f"CRIT_TL_PX = ({x0}, {y0})")
            print(f"CRIT_BR_PX = ({x1}, {y1})\n")

            print(f"# Real-world conversion (derived from {CRITICAL_ZONE_W_CM:.0f} x {CRITICAL_ZONE_H_CM:.0f} cm zone)")
            print(f"PX_PER_CM_X = {px_per_cm_x:.4f}")
            print(f"PX_PER_CM_Y = {px_per_cm_y:.4f}\n")

        if key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()