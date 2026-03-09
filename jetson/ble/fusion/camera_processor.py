"""
Camera processor for Jetson BLE sensor fusion pipeline.
Captures from IMX219/IMX477 via nvarguscamerasrc and writes
frames + VO deltas into FrameBuffer.
"""
import cv2
import numpy as np
import math
import threading
import time
import logging

logger = logging.getLogger(__name__)

CSI_PIPELINE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
    "nvvidconv flip-method=0 ! "
    "video/x-raw,width=640,height=360,format=BGRx ! "
    "videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=2"
)

USB_DEVICE = "/dev/video0"

LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
)
FEATURE_PARAMS = dict(maxCorners=150, qualityLevel=0.01, minDistance=10, blockSize=7)
MIN_FEATURES = 15
PIXELS_PER_METRE = 320.0


class CameraProcessor(object):
    def __init__(self, frame_buffer, source="csi"):
        self._buf = frame_buffer
        self._source = source
        self._cap = None
        self._thread = None
        self._running = False
        self._prev_gray = None
        self._prev_pts = None
        self.fps = 0.0

    def start(self):
        if self._source == "csi":
            self._cap = cv2.VideoCapture(CSI_PIPELINE, cv2.CAP_GSTREAMER)
            if not self._cap.isOpened():
                logger.warning("CSI pipeline failed, falling back to USB")
                self._cap = cv2.VideoCapture(USB_DEVICE)
        else:
            self._cap = cv2.VideoCapture(USB_DEVICE)

        if not self._cap.isOpened():
            raise RuntimeError("Cannot open camera")

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("Camera started (source=%s)", self._source)

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()

    def _loop(self):
        prev_time = time.monotonic()
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.005)
                continue

            now = time.monotonic()
            dt = now - prev_time
            if dt > 0:
                self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
            prev_time = now

            dx, dy, dtheta = self._optical_flow(frame)
            self._buf.update(frame, dx, dy, dtheta)

    def _optical_flow(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._prev_gray is None or self._prev_pts is None \
                or len(self._prev_pts) < MIN_FEATURES:
            self._prev_pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
            self._prev_gray = gray
            return 0.0, 0.0, 0.0

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._prev_pts, None, **LK_PARAMS)

        if curr_pts is None or status is None:
            self._prev_pts = None
            return 0.0, 0.0, 0.0

        good_prev = self._prev_pts[status == 1]
        good_curr = curr_pts[status == 1]

        if len(good_curr) < MIN_FEATURES:
            self._prev_pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
            self._prev_gray = gray
            return 0.0, 0.0, 0.0

        flow = good_curr - good_prev
        mean_flow = np.mean(flow, axis=0)
        dx = float(mean_flow[0]) / PIXELS_PER_METRE
        dy = float(mean_flow[1]) / PIXELS_PER_METRE

        dtheta = 0.0
        if len(good_curr) >= 4:
            M, _ = cv2.estimateAffinePartial2D(
                good_prev.reshape(-1, 1, 2),
                good_curr.reshape(-1, 1, 2),
                method=cv2.RANSAC,
            )
            if M is not None:
                dtheta = float(math.atan2(M[1, 0], M[0, 0]))

        if len(good_curr) < MIN_FEATURES * 2:
            self._prev_pts = cv2.goodFeaturesToTrack(gray, mask=None, **FEATURE_PARAMS)
        else:
            self._prev_pts = good_curr.reshape(-1, 1, 2)

        self._prev_gray = gray
        return dx, dy, dtheta
