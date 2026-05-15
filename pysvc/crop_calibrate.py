"""
crop_calibrate.py  --  Interactive calibration tool for the TPS remap.

Run this once (on a machine with a display) before starting main.py.
After saving with **S**, paste the printed block into calibration_config.py.

    python crop_calibrate.py

TWO windows open side-by-side:
    LEFT  (RAW)     -- drag the 6 teal handles to the four corners and
                       two edge midpoints of the table.

                       IMPORTANT: drag them WIDER than the playing surface
                       (include some of the table rails / margin). The puck
                       gets clipped at the bottom otherwise. The actual play
                       area is defined separately on the PREVIEW window.

    RIGHT (PREVIEW) -- the TPS-remapped + 90-CW-rotated view. Four nested
                       regions, each with two draggable corner handles:

                         GREEN  PLAY    -- the actual playing surface
                         YELLOW GANTRY  -- the gantry's reachable rectangle
                         BLUE   CRIT    -- the critical zone (55x35 cm)
                         ORANGE MALLET 0,0 cross -- gantry rest position

                       Containment is enforced (hard clamping):
                       PLAY in preview, GANTRY in PLAY, CRIT in GANTRY,
                       MALLET in GANTRY.

Press  S  to print the config block -- paste it into calibration_config.py
(and keep main.py importing from there unchanged).

Press  ESC to quit without saving.

The camera is opened with the same StereoCamera fusion as main.py.
"""

import cv2
import numpy as np
from scipy.interpolate import RBFInterpolator

from calibration_config import (
    CRIT_BR_PX,
    CRIT_TL_PX,
    GANTRY_BR_PX,
    GANTRY_TL_PX,
    MALLET_ORIGIN_PX,
    OUTPUT_H as PREVIEW_H,
    OUTPUT_W as PREVIEW_W,
    PLAY_BR_PX,
    PLAY_TL_PX,
)
from camera import StereoCamera

# ==============================================================================
# SETTINGS
# ==============================================================================
CAPTURE_WIDTH  = 2560
CAPTURE_HEIGHT = 720
FRAME_WIDTH    = CAPTURE_WIDTH // 2   # 1280 px after fusion

# PREVIEW_W / PREVIEW_H are aliases of OUTPUT_W / OUTPUT_H from calibration_config.

CRITICAL_ZONE_W_CM = 55.0
CRITICAL_ZONE_H_CM = 35.0

# Gantry physical extents (mm) -- must match cppsvc/include/gantry.hpp and
# the GANTRY_*_MAX_MM constants in pysvc/main.py.  The yellow GANTRY corners
# in the PREVIEW window must imply these values when converted through
# MALLET_ORIGIN_PX + PX_PER_CM_X/Y, otherwise main.py's startup assertion
# will refuse to launch.
GANTRY_X_MAX_MM   = 869.0
GANTRY_Y_MAX_MM   = 901.0
CALIB_TOL_MM      = 2.0

GRAB_R = 22   # hit radius for handle dragging (px)

# ==============================================================================
# HANDLE STATE
# ==============================================================================
# 6 remap points in raw 1280x720 pixel space:  TL, T-mid, TR, BR, B-mid, BL
remap_pts    = [[0, 0] for _ in range(6)]
REMAP_LABELS = ["TL", "T-mid", "TR", "BR", "B-mid", "BL"]

REMAP_COLOR  = (0, 220, 180)   # teal
MALLET_COLOR = (0, 200, 255)   # orange
CRIT_COLOR   = (255, 100, 0)   # blue
PLAY_COLOR   = (0, 220, 0)     # green
GANTRY_COLOR = (0, 255, 255)   # yellow


def _default_remap_pts(w, h):
    return [
        [int(w * 0.25), int(h * 0.15)],
        [int(w * 0.50), int(h * 0.10)],
        [int(w * 0.75), int(h * 0.15)],
        [int(w * 0.75), int(h * 0.85)],
        [int(w * 0.50), int(h * 0.90)],
        [int(w * 0.25), int(h * 0.85)],
    ]


# Preview handles live in PREVIEW (rotated) space: width=PREVIEW_H, height=PREVIEW_W.
# Seed from calibration_config so a re-run starts where the last save left off.
mallet_origin = list(MALLET_ORIGIN_PX)
play_tl       = list(PLAY_TL_PX)
play_br       = list(PLAY_BR_PX)
gantry_tl     = list(GANTRY_TL_PX)
gantry_br     = list(GANTRY_BR_PX)
crit_tl       = list(CRIT_TL_PX)
crit_br       = list(CRIT_BR_PX)

drag_target = None
remap_dirty = True
_map_x = _map_y = None

disp_w = FRAME_WIDTH
disp_h = CAPTURE_HEIGHT


# ==============================================================================
# TPS REMAP BUILD
# ==============================================================================
def _dst_points():
    return np.array([
        [0,               0              ],
        [PREVIEW_W // 2,  0              ],
        [PREVIEW_W - 1,   0              ],
        [PREVIEW_W - 1,   PREVIEW_H - 1  ],
        [PREVIEW_W // 2,  PREVIEW_H - 1  ],
        [0,               PREVIEW_H - 1  ],
    ], dtype=np.float64)


def _build_remap():
    global _map_x, _map_y, remap_dirty
    src    = np.array(remap_pts, dtype=np.float64)
    dst    = _dst_points()
    rbf_x  = RBFInterpolator(dst, src[:, 0], kernel="thin_plate_spline")
    rbf_y  = RBFInterpolator(dst, src[:, 1], kernel="thin_plate_spline")
    gx, gy = np.meshgrid(np.arange(PREVIEW_W), np.arange(PREVIEW_H))
    grid   = np.stack([gx.ravel(), gy.ravel()], axis=1).astype(np.float64)
    _map_x = rbf_x(grid).reshape(PREVIEW_H, PREVIEW_W).astype(np.float32)
    _map_y = rbf_y(grid).reshape(PREVIEW_H, PREVIEW_W).astype(np.float32)
    remap_dirty = False


# ==============================================================================
# MOUSE CALLBACKS
# ==============================================================================
def _hit(x, y, px, py):
    return abs(x - px) < GRAB_R and abs(y - py) < GRAB_R


def mouse_raw(event, x, y, flags, param):
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


def _rect_bounds(tl, br):
    """Return (xmin, ymin, xmax, ymax) regardless of TL/BR orientation."""
    return (min(tl[0], br[0]), min(tl[1], br[1]),
            max(tl[0], br[0]), max(tl[1], br[1]))


def _clamp_to_rect(nx, ny, tl, br):
    xmin, ymin, xmax, ymax = _rect_bounds(tl, br)
    return int(np.clip(nx, xmin, xmax)), int(np.clip(ny, ymin, ymax))


def mouse_preview(event, x, y, flags, param):
    global drag_target
    pw, ph = PREVIEW_H, PREVIEW_W   # rotated dims

    handles = (
        ("play_tl",   play_tl),
        ("play_br",   play_br),
        ("gantry_tl", gantry_tl),
        ("gantry_br", gantry_br),
        ("crit_tl",   crit_tl),
        ("crit_br",   crit_br),
        ("mallet",    mallet_origin),
    )

    if event == cv2.EVENT_LBUTTONDOWN:
        for tag, pt in handles:
            if _hit(x, y, pt[0], pt[1]):
                drag_target = (tag, None)
                return
    elif event == cv2.EVENT_MOUSEMOVE and drag_target:
        tag = drag_target[0]
        nx  = int(np.clip(x, 0, pw - 1))
        ny  = int(np.clip(y, 0, ph - 1))

        # Hierarchy clamp: each child stays inside its parent's current rect.
        if tag in ("play_tl", "play_br"):
            pass  # parent is the full preview; nx/ny already clamped above
        elif tag in ("gantry_tl", "gantry_br"):
            nx, ny = _clamp_to_rect(nx, ny, play_tl, play_br)
        elif tag in ("crit_tl", "crit_br", "mallet"):
            nx, ny = _clamp_to_rect(nx, ny, gantry_tl, gantry_br)

        if   tag == "play_tl":   play_tl[0],       play_tl[1]       = nx, ny
        elif tag == "play_br":   play_br[0],       play_br[1]       = nx, ny
        elif tag == "gantry_tl": gantry_tl[0],     gantry_tl[1]     = nx, ny
        elif tag == "gantry_br": gantry_br[0],     gantry_br[1]     = nx, ny
        elif tag == "crit_tl":   crit_tl[0],       crit_tl[1]       = nx, ny
        elif tag == "crit_br":   crit_br[0],       crit_br[1]       = nx, ny
        elif tag == "mallet":    mallet_origin[0], mallet_origin[1] = nx, ny

        # After moving a parent, re-clamp every descendant so the hierarchy
        # never goes out of sync (e.g. shrinking PLAY also tightens GANTRY).
        _enforce_hierarchy()

    elif event == cv2.EVENT_LBUTTONUP:
        drag_target = None


def _enforce_hierarchy():
    """Clamp gantry inside play, then crit + mallet inside gantry."""
    gantry_tl[:] = _clamp_to_rect(gantry_tl[0], gantry_tl[1], play_tl,   play_br)
    gantry_br[:] = _clamp_to_rect(gantry_br[0], gantry_br[1], play_tl,   play_br)
    crit_tl[:]   = _clamp_to_rect(crit_tl[0],   crit_tl[1],   gantry_tl, gantry_br)
    crit_br[:]   = _clamp_to_rect(crit_br[0],   crit_br[1],   gantry_tl, gantry_br)
    mallet_origin[:] = _clamp_to_rect(
        mallet_origin[0], mallet_origin[1], gantry_tl, gantry_br
    )


# ==============================================================================
# DRAWING
# ==============================================================================
def draw_handle(img, x, y, color, label):
    cv2.circle(img, (x, y), GRAB_R, color, 2)
    cv2.circle(img, (x, y), 4,      color, -1)
    cv2.putText(img, label, (x + GRAB_R + 4, y + 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def draw_raw(frame):
    disp    = frame.copy()
    pts     = np.array(remap_pts, dtype=np.int32)
    overlay = disp.copy()
    cv2.fillPoly(overlay, [pts], (200, 200, 100))
    cv2.addWeighted(overlay, 0.12, disp, 0.88, 0, disp)
    cv2.polylines(disp, [pts], isClosed=True, color=(255, 255, 255), thickness=1)
    for i, (px, py) in enumerate(remap_pts):
        draw_handle(disp, px, py, REMAP_COLOR, REMAP_LABELS[i])
    cv2.putText(disp, "RAW -- drag 6 remap handles to table edges",
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

    px0, py0, px1, py1 = _rect_bounds(play_tl,   play_br)
    gx0, gy0, gx1, gy1 = _rect_bounds(gantry_tl, gantry_br)
    cx0, cy0, cx1, cy1 = _rect_bounds(crit_tl,   crit_br)

    cv2.rectangle(rotated, (px0, py0), (px1, py1), PLAY_COLOR,   2)
    cv2.rectangle(rotated, (gx0, gy0), (gx1, gy1), GANTRY_COLOR, 2)
    cv2.rectangle(rotated, (cx0, cy0), (cx1, cy1), CRIT_COLOR,   2)

    mx, my = mallet_origin
    cv2.drawMarker(rotated, (mx, my), MALLET_COLOR, cv2.MARKER_CROSS, 30, 2)
    cv2.circle(rotated, (mx, my), 6, MALLET_COLOR, -1)
    draw_handle(rotated, mx, my,         MALLET_COLOR, "MALLET 0,0")
    draw_handle(rotated, *play_tl,       PLAY_COLOR,   "PLAY-TL")
    draw_handle(rotated, *play_br,       PLAY_COLOR,   "PLAY-BR")
    draw_handle(rotated, *gantry_tl,     GANTRY_COLOR, "GANTRY-TL")
    draw_handle(rotated, *gantry_br,     GANTRY_COLOR, "GANTRY-BR")
    draw_handle(rotated, *crit_tl,       CRIT_COLOR,   "CRIT-TL")
    draw_handle(rotated, *crit_br,       CRIT_COLOR,   "CRIT-BR")

    crit_px_w   = cx1 - cx0
    crit_px_h   = cy1 - cy0
    gantry_px_w = gx1 - gx0
    gantry_px_h = gy1 - gy0

    # px/cm is now derived from the GANTRY rectangle + known physical extents.
    # CRIT is no longer the calibration reference -- it just defines the defend zone.
    px_per_cm_x = gantry_px_w / (GANTRY_X_MAX_MM / 10.0) if gantry_px_w > 0 else 0
    px_per_cm_y = gantry_px_h / (GANTRY_Y_MAX_MM / 10.0) if gantry_px_h > 0 else 0

    # Implied CRIT physical size under the gantry-derived scale (sanity check).
    crit_cm_w = crit_px_w / px_per_cm_x if px_per_cm_x > 0 else 0.0
    crit_cm_h = crit_px_h / px_per_cm_y if px_per_cm_y > 0 else 0.0

    # Implied gantry mm at the current handle positions.  In the portrait
    # frame Y is inverted (y=0 is top, y=h-1 is bottom = robot side), so the
    # yellow GANTRY-TL handle should imply (0, 901) mm and GANTRY-BR should
    # imply (869, 0) mm.  Mirrors px_to_gantry_mm_raw() in pysvc/main.py.
    if px_per_cm_x > 0 and px_per_cm_y > 0:
        tl_mm_x = (gx0 - mx) / px_per_cm_x * 10.0
        tl_mm_y = (my - gy0) / px_per_cm_y * 10.0
        br_mm_x = (gx1 - mx) / px_per_cm_x * 10.0
        br_mm_y = (my - gy1) / px_per_cm_y * 10.0
    else:
        tl_mm_x = tl_mm_y = br_mm_x = br_mm_y = 0.0

    tl_ok = (abs(tl_mm_x) <= CALIB_TOL_MM
             and abs(tl_mm_y - GANTRY_Y_MAX_MM) <= CALIB_TOL_MM)
    br_ok = (abs(br_mm_x - GANTRY_X_MAX_MM) <= CALIB_TOL_MM
             and abs(br_mm_y) <= CALIB_TOL_MM)

    info = [
        f"PLAY:   {px1 - px0}px x {py1 - py0}px",
        f"GANTRY: {gantry_px_w}px x {gantry_px_h}px  "
        f"({GANTRY_X_MAX_MM:.0f}mm x {GANTRY_Y_MAX_MM:.0f}mm physical)",
        f"CRIT:   {crit_px_w}px x {crit_px_h}px  "
        f"(implied {crit_cm_w:.1f}cm x {crit_cm_h:.1f}cm, "
        f"expect ~{CRITICAL_ZONE_W_CM:.0f}cm x {CRITICAL_ZONE_H_CM:.0f}cm)",
        f"px/cm  X: {px_per_cm_x:.4f}   Y: {px_per_cm_y:.4f}",
        f"Mallet origin (px): ({mx}, {my})",
        "",
        f"GANTRY-TL mm: ({tl_mm_x:+7.1f}, {tl_mm_y:+7.1f})  "
        f"expect (0.0, {GANTRY_Y_MAX_MM:.0f})  "
        f"{'OK' if tl_ok else 'BAD'}",
        f"GANTRY-BR mm: ({br_mm_x:+7.1f}, {br_mm_y:+7.1f})  "
        f"expect ({GANTRY_X_MAX_MM:.0f}, 0.0)  "
        f"{'OK' if br_ok else 'BAD'}",
        "",
        "S = save  |  ESC = quit",
    ]
    bad_color  = (60, 60, 255)   # red
    good_color = (255, 255, 255)
    for i, line in enumerate(info):
        is_tl_line = line.startswith("GANTRY-TL mm")
        is_br_line = line.startswith("GANTRY-BR mm")
        color = good_color
        if (is_tl_line and not tl_ok) or (is_br_line and not br_ok):
            color = bad_color
        cv2.putText(rotated, line, (8, 22 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return rotated


# ==============================================================================
# MAIN
# ==============================================================================
def main():
    global remap_dirty, disp_w, disp_h

    cam = StereoCamera()

    win_raw  = "RAW -- drag remap handles"
    win_prev = "PREVIEW -- drag mallet origin + critical zone"

    cv2.namedWindow(win_raw,  cv2.WINDOW_NORMAL)
    cv2.namedWindow(win_prev, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_raw,  FRAME_WIDTH // 2, CAPTURE_HEIGHT // 2)
    cv2.resizeWindow(win_prev, PREVIEW_H,        PREVIEW_W)
    cv2.setMouseCallback(win_raw,  mouse_raw)
    cv2.setMouseCallback(win_prev, mouse_preview)

    print("\n=== Calibration ===")
    print("LEFT  (raw)     -- drag 6 teal handles WIDER than the play surface")
    print("                   (include some table rail / margin)")
    print("RIGHT (preview) -- drag green  PLAY    corners to playing-surface edges")
    print("                   drag yellow GANTRY  corners to the gantry's reach")
    print("                   drag blue   CRIT    corners to the "
          f"{CRITICAL_ZONE_W_CM:.0f}x{CRITICAL_ZONE_H_CM:.0f} cm critical zone")
    print("                   drag orange MALLET 0,0 cross to gantry rest position")
    print("Press  S  to print config   ESC  to quit\n")

    ret, first_frame = cam.read()
    if not ret or first_frame is None:
        raise RuntimeError("Could not read a frame from the camera.")

    disp_h, disp_w = first_frame.shape[:2]
    for i, pt in enumerate(_default_remap_pts(disp_w, disp_h)):
        remap_pts[i][0] = pt[0]
        remap_pts[i][1] = pt[1]

    print(f"[INFO] Fused frame size: {disp_w}x{disp_h}", flush=True)
    cv2.resizeWindow(win_raw, disp_w, disp_h)

    print("[INFO] Building initial preview...", flush=True)
    _build_remap()
    print("[INFO] Ready.", flush=True)

    while True:
        ret, frame = cam.read()
        if not ret or frame is None:
            break

        if remap_dirty:
            print("[INFO] Rebuilding remap...", flush=True)
            _build_remap()
            print("[INFO] Preview updated.", flush=True)

        cv2.imshow(win_raw,  draw_raw(frame))
        cv2.imshow(win_prev, draw_preview(frame))

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            px0, py0, px1, py1 = _rect_bounds(play_tl,   play_br)
            gx0, gy0, gx1, gy1 = _rect_bounds(gantry_tl, gantry_br)
            cx0, cy0, cx1, cy1 = _rect_bounds(crit_tl,   crit_br)
            crit_px_w   = cx1 - cx0
            crit_px_h   = cy1 - cy0
            gantry_px_w = gx1 - gx0
            gantry_px_h = gy1 - gy0
            if gantry_px_w < 10 or gantry_px_h < 10:
                print("WARNING: gantry zone too small -- drag the GANTRY corners apart first.")
                continue

            px_per_cm_x = gantry_px_w / (GANTRY_X_MAX_MM / 10.0)
            px_per_cm_y = gantry_px_h / (GANTRY_Y_MAX_MM / 10.0)

            mx, my   = mallet_origin
            tl_mm_x  = (gx0 - mx) / px_per_cm_x * 10.0
            tl_mm_y  = (my - gy0) / px_per_cm_y * 10.0
            br_mm_x  = (gx1 - mx) / px_per_cm_x * 10.0
            br_mm_y  = (my - gy1) / px_per_cm_y * 10.0
            tl_ok    = (abs(tl_mm_x) <= CALIB_TOL_MM
                        and abs(tl_mm_y - GANTRY_Y_MAX_MM) <= CALIB_TOL_MM)
            br_ok    = (abs(br_mm_x - GANTRY_X_MAX_MM) <= CALIB_TOL_MM
                        and abs(br_mm_y) <= CALIB_TOL_MM)
            if not (tl_ok and br_ok):
                print("\n### WARNING ##################################################")
                print("# GANTRY corners do not imply the expected mm values.")
                print(f"#   GANTRY-TL -> ({tl_mm_x:+.1f}, {tl_mm_y:+.1f}) mm  "
                      f"expected (0.0, {GANTRY_Y_MAX_MM:.0f})  "
                      f"{'OK' if tl_ok else 'BAD'}")
                print(f"#   GANTRY-BR -> ({br_mm_x:+.1f}, {br_mm_y:+.1f}) mm  "
                      f"expected ({GANTRY_X_MAX_MM:.0f}, 0.0)  "
                      f"{'OK' if br_ok else 'BAD'}")
                print(f"# pysvc/main.py startup will REFUSE to launch with these")
                print(f"# values (tolerance {CALIB_TOL_MM:.1f} mm).  Drag GANTRY /")
                print(f"# MALLET / CRIT until both lines read OK, or set")
                print(f"# SKIP_CALIBRATION_CHECK=1 to bypass (dev only).")
                print("###############################################################\n")

            print("\n# ── Paste into pysvc/calibration_config.py ──────────────────────")
            print(f"OUTPUT_W, OUTPUT_H = {PREVIEW_W}, {PREVIEW_H}\n")
            print("SRC_POINTS = [")
            for i, (px, py) in enumerate(remap_pts):
                print(f"    ({float(px)}, {float(py)}),   # {REMAP_LABELS[i]}")
            print("]\n")
            print(f"MALLET_ORIGIN_PX = ({mallet_origin[0]}, {mallet_origin[1]})\n")
            print(f"PLAY_TL_PX   = ({px0}, {py0})")
            print(f"PLAY_BR_PX   = ({px1}, {py1})\n")
            print(f"GANTRY_TL_PX = ({gx0}, {gy0})")
            print(f"GANTRY_BR_PX = ({gx1}, {gy1})\n")
            print(f"CRIT_TL_PX   = ({cx0}, {cy0})")
            print(f"CRIT_BR_PX   = ({cx1}, {cy1})\n")
            print(f"PX_PER_CM_X = {px_per_cm_x:.4f}")
            print(f"PX_PER_CM_Y = {px_per_cm_y:.4f}")
            print("# ────────────────────────────────────────────────────────────────\n")

        if key == 27:
            break

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
