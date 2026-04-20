import cv2

CAPTURE_W, CAPTURE_H = 1280, 720
OUTPUT_W,  OUTPUT_H  = 960, 540

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAPTURE_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)

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

# Clamp to actual frame bounds
x = min(x, actual_w - 1)
y = min(y, actual_h - 1)
w = min(w, actual_w - x)
h = min(h, actual_h - y)

# Preview: cropped region resized to 960x540
preview = cv2.resize(frame[y:y+h, x:x+w], (OUTPUT_W, OUTPUT_H))
cv2.imshow(f"Preview at {OUTPUT_W}x{OUTPUT_H} — press any key", preview)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(f"\nCopy these into objectdetect_and_trajectory.py:")
print(f"ROI_X, ROI_Y, ROI_W, ROI_H = {x}, {y}, {w}, {h}")
