"""Camera frame processor for Kria KV260.

Captures frames from V4L2 device and extracts basic visual features
for sensor fusion. Supports multiple simultaneous camera instances.
"""

import logging
import cv2
import numpy as np

log = logging.getLogger(__name__)


class CameraProcessor:
    def __init__(self, device: str = "/dev/video1", width: int = 1280, height: int = 720,
                 fourcc: str = "MJPG"):
        self.device = device
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera device: {device}")
        # Confirm actual resolution negotiated
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        log.info("Opened %s at %dx%d", device, self.width, self.height)

    @classmethod
    def try_open(cls, device: str, **kwargs):
        """Return a CameraProcessor or None if the device can't be opened."""
        try:
            return cls(device, **kwargs)
        except RuntimeError as e:
            log.warning("Skipping %s: %s", device, e)
            return None

    def read_frame(self) -> np.ndarray:
        """Read one frame from the camera."""
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError(f"Failed to read frame from {self.device}")
        return frame

    def detect_motion(self, frame1: np.ndarray, frame2: np.ndarray,
                      threshold: int = 25) -> dict:
        """Compute frame-difference motion estimate.

        Returns:
            dict with motion_score and bounding box of largest motion region (or None).
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
