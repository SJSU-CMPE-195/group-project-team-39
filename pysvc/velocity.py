import cv2
import sys

# ==============================================================================
# MODE SELECTION — change MODE to switch
# ==============================================================================
# Each entry is (width, height, fps) for the side-by-side stereo output.
MODES = {
    "HD2K":   (4416, 1242, 15),
    "HD1080": (3840, 1080, 30),   # also supports 15
    "HD720":  (2560,  720, 60),   # also supports 30, 15
    "VGA":    (1344,  376, 100),  # also supports 60, 30, 15
}

MODE = "HD720"
WIDTH, HEIGHT, FPS = MODES[MODE]


def open_camera():
    # macOS: default backend works fine for UVC
    if sys.platform == "darwin":
        cap = cv2.VideoCapture(0)
    else:
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

    if not cap.isOpened():
        raise RuntimeError("Could not open camera")

    # MJPG is required to hit high resolutions / frame rates over USB
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          FPS)

    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[INFO] Requested {MODE}: {WIDTH}x{HEIGHT} @ {FPS}", flush=True)
    print(f"[INFO] Got:           {actual_w}x{actual_h} @ {actual_fps:.1f}", flush=True)
    return cap


def main():
    cap = open_camera()

    window = "ZED 2i"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 480)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame grab failed", flush=True)
            break

        cv2.imshow(window, frame)
        if cv2.waitKey(1) == 27:   # ESC
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()