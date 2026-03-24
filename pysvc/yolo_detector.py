"""
YOLO-based puck detector with HSV fallback.

Usage modes
-----------
1. No model path  → uses the original HSV color-range detection (no extra deps).
2. Custom model   → loads a trained YOLOv8 .pt file for neural-net detection
                    (robust to lighting changes, works with non-green pucks, etc.).

Training a custom model
-----------------------
Step 1 – Collect labeled frames while the system is running:

    from yolo_detector import TrainingDataCollector
    collector = TrainingDataCollector("puck_dataset/")
    # inside your main loop, after detecting center:
    collector.collect(frame, center, frame_w, frame_h)
    # at exit:
    collector.save_yaml()

Step 2 – Train YOLOv8 nano (fastest model):

    pip install ultralytics
    yolo train data=puck_dataset/dataset.yaml model=yolov8n.pt epochs=50 imgsz=640

Step 3 – Point the detector at the trained weights:

    detector = PuckDetector(model_path="runs/detect/train/weights/best.pt")
"""

import cv2
import numpy as np
from pathlib import Path

try:
    from ultralytics import YOLO as _YOLO
    _YOLO_AVAILABLE = True
except ImportError:
    _YOLO_AVAILABLE = False


# ── HSV fallback (mirrors original objectdetect_and_trajectory.py logic) ──────

def _hsv_detect(frame_bgr: np.ndarray, min_area: int = 300):
    """Returns (center, area) using green HSV filtering."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower = np.array([35, 60,  60], dtype=np.uint8)
    upper = np.array([85, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0

    c = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(c)
    if area < min_area:
        return None, 0

    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, 0

    h, w = frame_bgr.shape[:2]
    cx = int(np.clip(M["m10"] / M["m00"], 0, w - 1))
    cy = int(np.clip(M["m01"] / M["m00"], 0, h - 1))
    return (cx, cy), float(area)


# ── Main detector class ───────────────────────────────────────────────────────

class PuckDetector:
    """
    Unified puck detector: wraps YOLO inference or HSV detection.

    Args:
        model_path:      path to a trained YOLOv8 .pt file, or None → HSV mode
        conf_threshold:  minimum confidence for YOLO detections
        min_area:        minimum contour area for HSV detections
    """

    def __init__(
        self,
        model_path: str | None = None,
        conf_threshold: float = 0.4,
        min_area: int = 300,
    ):
        self.conf_threshold = conf_threshold
        self.min_area = min_area
        self._model = None
        self._using_yolo = False

        if model_path:
            if not _YOLO_AVAILABLE:
                print("[PuckDetector] ultralytics not installed — using HSV fallback.")
                print("               pip install ultralytics")
            else:
                p = Path(model_path)
                if p.exists():
                    print(f"[PuckDetector] Loading YOLO model: {model_path}")
                    self._model = _YOLO(model_path)
                    self._using_yolo = True
                    print("[PuckDetector] YOLO model ready.")
                else:
                    print(f"[PuckDetector] Model not found at '{model_path}' — using HSV fallback.")
        else:
            print("[PuckDetector] No model specified — using HSV detection.")

    @property
    def mode(self) -> str:
        return "YOLO" if self._using_yolo else "HSV"

    def detect(self, frame_bgr: np.ndarray) -> tuple:
        """
        Detect puck center in a frame.

        Returns:
            center: (cx, cy) or None
            area:   bounding area in pixels (0 if not detected)
            mode:   "YOLO" or "HSV"
        """
        if self._using_yolo:
            return self._yolo_detect(frame_bgr)
        center, area = _hsv_detect(frame_bgr, self.min_area)
        return center, area, "HSV"

    def _yolo_detect(self, frame_bgr: np.ndarray) -> tuple:
        results = self._model(frame_bgr, verbose=False, conf=self.conf_threshold)[0]
        if not results.boxes or len(results.boxes) == 0:
            return None, 0.0, "YOLO"

        boxes = results.boxes
        best_idx = int(boxes.conf.argmax())
        x1, y1, x2, y2 = boxes.xyxy[best_idx].cpu().numpy()
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)
        area = float((x2 - x1) * (y2 - y1))
        return (cx, cy), area, "YOLO"


# ── Training data collector ───────────────────────────────────────────────────

class TrainingDataCollector:
    """
    Auto-collects YOLO-format training data using HSV auto-labeling.

    Run this while manually moving the puck around the table for ~5 minutes
    to build a dataset, then train a YOLOv8 model on it.

    Example
    -------
        collector = TrainingDataCollector("puck_dataset/")
        # inside main loop (only when puck is detected by HSV):
        collector.collect(frame, center, frame.shape[1], frame.shape[0])
        # on exit:
        collector.save_yaml()
        print(collector.train_command())
    """

    def __init__(self, output_dir: str = "puck_dataset", every_n_frames: int = 5):
        self.output_dir = Path(output_dir)
        self.every_n = every_n_frames
        self._frame_count = 0
        self._saved_count = 0

        (self.output_dir / "images" / "train").mkdir(parents=True, exist_ok=True)
        (self.output_dir / "labels" / "train").mkdir(parents=True, exist_ok=True)
        print(f"[TrainingDataCollector] Saving dataset to: {self.output_dir.resolve()}/")

    def collect(
        self,
        frame_bgr: np.ndarray,
        center: tuple | None,
        frame_w: int,
        frame_h: int,
        puck_radius_px: int = 20,
    ) -> None:
        """
        Call every frame.  Saves image + YOLO label when puck is detected.

        Args:
            frame_bgr:      current video frame
            center:         (cx, cy) from HSV detection, or None
            frame_w/h:      frame dimensions
            puck_radius_px: approximate puck radius in pixels (used for bbox size)
        """
        self._frame_count += 1
        if self._frame_count % self.every_n != 0 or center is None:
            return

        name = f"puck_{self._saved_count:06d}"
        img_path = self.output_dir / "images" / "train" / f"{name}.jpg"
        lbl_path = self.output_dir / "labels" / "train" / f"{name}.txt"

        cv2.imwrite(str(img_path), frame_bgr)

        # YOLO format: class  cx  cy  w  h  (all normalized 0-1)
        cx_n = center[0] / frame_w
        cy_n = center[1] / frame_h
        w_n  = (puck_radius_px * 2) / frame_w
        h_n  = (puck_radius_px * 2) / frame_h
        with open(lbl_path, "w") as f:
            f.write(f"0 {cx_n:.6f} {cy_n:.6f} {w_n:.6f} {h_n:.6f}\n")

        self._saved_count += 1

    @property
    def saved_count(self) -> int:
        return self._saved_count

    def save_yaml(self) -> Path:
        """Write dataset.yaml required by yolo train."""
        yaml_path = self.output_dir / "dataset.yaml"
        abs_path = self.output_dir.resolve()
        yaml_path.write_text(
            f"path: {abs_path}\n"
            f"train: images/train\n"
            f"val:   images/train\n\n"
            f"nc: 1\n"
            f"names: ['puck']\n"
        )
        print(f"[TrainingDataCollector] Saved {yaml_path}  ({self._saved_count} images)")
        return yaml_path

    def train_command(self) -> str:
        yaml = (self.output_dir / "dataset.yaml").resolve()
        return (
            f"pip install ultralytics\n"
            f"yolo train data={yaml} model=yolov8n.pt epochs=50 imgsz=640"
        )
