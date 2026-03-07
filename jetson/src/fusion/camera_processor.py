"""
Camera processor for NVIDIA Jetson Nano.

Supports:
  1. Jetson CSI camera (IMX219 / OV5647) via GStreamer + nvarguscamerasrc
  2. USB camera fallback via V4L2

Provides:
  - Frame capture (BGR numpy array)
  - Visual odometry via Lucas-Kanade sparse optical flow
  - Feature-based delta pose estimation (dx, dy, dtheta in image/body frame)

Usage:
    cam = CameraProcessor(source="csi")   # or "usb"
    cam.start()
    while True:
        frame = cam.latest_frame
        vo = cam.visual_odometry_delta()   # dict: dx, dy, dtheta
"""

import cv2
import numpy as np
import math
import logging
import threading
import time
from typing import Optional, Tuple

logger = logging.getLogger(__name__)

# GStreamer pipeline for Jetson CSI camera (IMX219, 1280x720 @ 30fps)
CSI_PIPELINE = (
    "nvarguscamerasrc ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw, width=640, height=360, format=BGRx ! "
    "videoconvert ! "
    "video/x-raw, format=BGR ! "
    "appsink drop=true max-buffers=2"
)

# Optical flow parameters
LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)
FEATURE_PARAMS = dict(maxCorners=150, qualityLevel=0.01, minDistance=10, blockSize=7)

# Minimum good features to attempt VO
MIN_FEATURES = 15
# Pixels per metre (calibrate for your setup)
PIXELS_PER_METRE = 320.0


class CameraProcessor:
    def __init__(self, source: str = "csi", device_id: int = 0, width: int = 640, height: int = 360):
        """
        Args:
            source: "csi" for Jetson CSI camera, "usb" for USB/V4L2 camera
            device_id: V4L2 device index (used when source="usb")
            width, height: capture resolution
        """
        self._source = source
        self._device_id = device_id
        self._width = width
        self._height = height
        self._cap: Optional[cv2.VideoCapture] = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

        self.latest_frame: Optional[np.ndarray] = None
        self._prev_gray: Optional[np.ndarray] = None
        self._prev_pts: Optional[np.ndarray] = None
        self._frame_lock = threading.Lock()

        # VO output
        self._vo_dx = 0.0
        self._vo_dy = 0.0
        self._vo_dtheta = 0.0
        self._last_capture_time = 0.0
        self.fps = 0.0

    # ------------------------------------------------------------------
    def start(self):
        """Open camera and start capture thread."""
        if self._source == "csi":
            self._cap = cv2.VideoCapture(CSI_PIPELINE, cv2.CAP_GSTREAMER)
            if not self._cap.isOpened():
                logger.warning("CSI GStreamer pipeline failed, falling back to V4L2")
                self._cap = cv2.VideoCapture(self._device_id)
        else:
            self._cap = cv2.VideoCapture(self._device_id)

        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera source='{self._source}' device={self._device_id}")

        if self._source == "usb":
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        logger.info("CameraProcessor started (source=%s)", self._source)

    # ------------------------------------------------------------------
    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        logger.info("CameraProcessor stopped")

    # ------------------------------------------------------------------
    def _capture_loop(self):
        prev_time = time.monotonic()
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.005)
                continue

            now = time.monotonic()
            elapsed = now - prev_time
            if elapsed > 0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / elapsed)
            prev_time = now
            self._last_capture_time = now

            with self._frame_lock:
                self.latest_frame = frame
                self._update_visual_odometry(frame)

    # ------------------------------------------------------------------
    def _update_visual_odometry(self, frame: np.ndarray):
        """Compute sparse optical flow between consecutive frames."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._prev_gray is None or self._prev_pts is None or len(self._prev_pts) < MIN_FEATURES:
            self._prev_pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
            self._prev_gray = gray
            self._vo_dx = self._vo_dy = self._vo_dtheta = 0.0
            return

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **LK_PARAMS
        )

        if curr_pts is None or status is None:
            self._prev_pts = None
            return

        good_prev = self._prev_pts[status == 1]
        good_curr = curr_pts[status == 1]

        if len(good_curr) < MIN_FEATURES:
            self._prev_pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
            self._prev_gray = gray
            return

        # Mean displacement (pixels → metres approximation)
        flow = good_curr - good_prev
        mean_flow = np.mean(flow, axis=0)
        self._vo_dx = float(mean_flow[0]) / PIXELS_PER_METRE
        self._vo_dy = float(mean_flow[1]) / PIXELS_PER_METRE

        # Rotation estimate via affine ransac
        if len(good_curr) >= 4:
            M, inliers = cv2.estimateAffinePartial2D(
                good_prev.reshape(-1, 1, 2),
                good_curr.reshape(-1, 1, 2),
                method=cv2.RANSAC,
            )
            if M is not None:
                self._vo_dtheta = float(math.atan2(M[1, 0], M[0, 0]))
        else:
            self._vo_dtheta = 0.0

        # Refresh features if too few remain
        if len(good_curr) < MIN_FEATURES * 2:
            self._prev_pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
        else:
            self._prev_pts = good_curr.reshape(-1, 1, 2)

        self._prev_gray = gray

    # ------------------------------------------------------------------
    def visual_odometry_delta(self) -> dict:
        """Return latest visual odometry delta and reset."""
        with self._frame_lock:
            d = {"dx": self._vo_dx, "dy": self._vo_dy, "dtheta": self._vo_dtheta}
            self._vo_dx = self._vo_dy = self._vo_dtheta = 0.0
        return d

    # ------------------------------------------------------------------
    @property
    def is_open(self) -> bool:
        return self._running and self._cap is not None and self._cap.isOpened()
