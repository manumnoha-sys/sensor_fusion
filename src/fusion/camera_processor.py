"""Camera frame processor for Kria KV260.

Captures frames from V4L2 device (e.g. MIPI CSI camera)
and extracts basic visual features for sensor fusion.
"""

import cv2
import numpy as np


class CameraProcessor:
    def __init__(self, device: str = "/dev/video0", width: int = 640, height: int = 480):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera device: {device}")

    def read_frame(self) -> np.ndarray:
        """Read one frame from the camera."""
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Failed to read frame from camera")
        return frame

    def detect_motion(self, frame1: np.ndarray, frame2: np.ndarray,
                      threshold: int = 25) -> dict:
        """Compute frame-difference motion estimate.

        Returns:
            dict with motion_score and bounding box of motion region (or None).
        """
        gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
        gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
        diff = cv2.absdiff(gray1, gray2)
        _, mask = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        motion_score = float(np.sum(mask)) / mask.size

        bbox = None
        if contours:
            x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
            bbox = (x, y, w, h)

        return {"motion_score": motion_score, "bbox": bbox}

    def release(self):
        self.cap.release()
