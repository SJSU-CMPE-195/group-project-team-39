import cv2
import glob
import sys

CAPTURE_W, CAPTURE_H = 2560, 720

def open_camera():
    if sys.platform == 'darwin':
        print("macOS detected. Using default cv2.VideoCapture(0)...", flush=True)
        return cv2.VideoCapture(0)

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
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)
cap.set(cv2.CAP_PROP_FPS, 100)

ret, frame = cap.read()
cap.release()

if not ret:
    print("Could not read frame from camera")
    exit(1)

actual_h, actual_w = frame.shape[:2]
print(f"Actual camera resolution: {actual_w}x{actual_h}")

roi = cv2.selectROI("Draw box around table, press ENTER", frame, fromCenter=False, showCrosshair=True)
cv2.destroyAllWindows()

x, y, w, h = int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3])

x = min(x, actual_w - 1)
y = min(y, actual_h - 1)
w = min(w, actual_w - x)
h = min(h, actual_h - y)

preview = frame[y:y+h, x:x+w]
cv2.imshow(f"Preview — press any key", preview)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(f"\nCopy these into python_apalis.py:")
print(f"ROI_X, ROI_Y, ROI_W, ROI_H = {x}, {y}, {w}, {h}")
