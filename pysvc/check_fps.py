import cv2
import sys
import glob
import time

FRAME_WIDTH  = 2560
FRAME_HEIGHT = 720
TARGET_FPS   = 100
UPDATE_INTERVAL = 1.0

def open_camera():
    """Finds and opens a camera using the same logic as nam.py."""
    if sys.platform == "darwin":
        print("macOS detected — using VideoCapture(0)", flush=True)
        return cv2.VideoCapture(0)
    
    devices = sorted(glob.glob("/dev/video*"))
    print(f"Found devices: {devices}", flush=True)
    for dev in devices:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"Camera found at {dev}", flush=True)
                return cap
            cap.release()
    raise RuntimeError("No camera found")

def main():
    try:
        cap = open_camera()
    except Exception as e:
        print(f"Error opening camera: {e}")
        return

    # Configure camera properties (from nam.py and calibrate.py)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    print(f"[INFO] Camera configured: {actual_w}x{actual_h} @ requested {actual_fps} fps", flush=True)

    cv2.namedWindow("FPS Meter", cv2.WINDOW_NORMAL)

    frame_count = 0
    fps_display = 0.0
    fps_timer_start = time.time()

    print("Press ESC to exit.", flush=True)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame from camera.")
            break

        now = time.time()

        # FPS tracking logic
        frame_count += 1
        elapsed = now - fps_timer_start
        if elapsed >= UPDATE_INTERVAL:
            fps_display = frame_count / elapsed
            frame_count = 0
            fps_timer_start = now

        # Draw the FPS on the frame
        cv2.putText(frame, f"Measured FPS: {fps_display:.1f}", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

        cv2.imshow("FPS Meter", frame)

        # Exit on ESC key
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
