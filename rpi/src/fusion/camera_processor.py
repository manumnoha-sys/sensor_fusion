"""Camera frame processor for Raspberry Pi with IMX219 (cam0).

Uses picamera2 (libcamera) instead of raw V4L2 since the IMX219
requires the libcamera stack on modern Raspberry Pi OS.
"""

import logging
import cv2
import numpy as np
from picamera2 import Picamera2

log = logging.getLogger(__name__)


class CameraProcessor:
    def __init__(self, camera_num: int = 0, width: int = 1920, height: int = 1080):
        self.device = f"imx219:cam{camera_num}"
        self._picam2 = Picamera2(camera_num)
        cfg = self._picam2.create_preview_configuration(
            main={"format": "RGB888", "size": (width, height)}
        )
        self._picam2.configure(cfg)
        self._picam2.start()
        self.width = width
        self.height = height
        log.info("Opened %s at %dx%d via libcamera/picamera2", self.device, width, height)

    @classmethod
    def try_open(cls, camera_num: int = 0, **kwargs):
        """Return a CameraProcessor or None if the camera can't be opened."""
        try:
            return cls(camera_num, **kwargs)
        except Exception as e:
            log.warning("Skipping cam%d: %s", camera_num, e)
            return None

    def read_frame(self) -> np.ndarray:
        """Read one frame as a BGR numpy array."""
        # picamera2 returns RGB888; convert to BGR for OpenCV compatibility
        rgb = self._picam2.capture_array("main")
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    def detect_motion(self, frame1: np.ndarray, frame2: np.ndarray,
                      threshold: int = 25) -> dict:
        """Compute frame-difference motion estimate."""
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
        self._picam2.stop()
