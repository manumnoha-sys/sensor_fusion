"""MJPEG streaming server for Kria KV260.

Streams all available cameras over HTTP as MJPEG.
Run on the Kria board:
    python3 camera_stream.py

Endpoints:
    GET /           - camera index (JSON)
    GET /stream/0   - MJPEG stream from camera 0
    GET /stream/1   - MJPEG stream from camera 1
    GET /snapshot/0 - single JPEG frame from camera 0
"""

import argparse
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Optional
import json

import cv2
import numpy as np

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger(__name__)

# Cameras to try in order: (device, width, height, fourcc, label)
CAMERA_CONFIGS = [
    {"device": "/dev/video1", "width": 1280, "height": 720,  "fourcc": "MJPG", "label": "Logitech C925e"},
    # "/dev/video0" (Xilinx ISP VCAP) excluded — AP1302 probe failure causes open() to block.
    # Re-add once FFC cable on IAS J2 is reseated.
]

BOUNDARY = b"--frame"


class Camera:
    """Thread-safe camera capture with latest-frame caching."""

    def __init__(self, device: str, width: int, height: int, fourcc: str, label: str):
        self.device = device
        self.label = label
        self._lock = threading.Lock()
        self._frame: Optional[bytes] = None
        self._running = False

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open {device}")
        self.cap = cap
        self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        log.info("Camera ready: %s at %dx%d", label, self.width, self.height)

    def start(self):
        self._running = True
        threading.Thread(target=self._capture_loop, daemon=True).start()

    def _capture_loop(self):
        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                log.warning("Frame read failed on %s", self.device)
                time.sleep(0.1)
                continue
            _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self._lock:
                self._frame = jpeg.tobytes()

    def get_jpeg(self) -> Optional[bytes]:
        with self._lock:
            return self._frame

    def stop(self):
        self._running = False
        self.cap.release()


def load_cameras() -> list[Camera]:
    cameras = []
    for cfg in CAMERA_CONFIGS:
        try:
            cam = Camera(**cfg)
            cam.start()
            cameras.append(cam)
        except RuntimeError as e:
            log.warning("Skipping camera: %s", e)
    return cameras


class StreamHandler(BaseHTTPRequestHandler):
    cameras: list[Camera] = []

    def log_message(self, fmt, *args):
        pass  # suppress per-request noise

    def _send_json(self, data: dict):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream(self, cam: Camera):
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                jpeg = cam.get_jpeg()
                if jpeg is None:
                    time.sleep(0.01)
                    continue
                self.wfile.write(
                    BOUNDARY + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    + jpeg + b"\r\n"
                )
                self.wfile.flush()
                time.sleep(0.033)  # ~30 fps cap
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _snapshot(self, cam: Camera):
        jpeg = cam.get_jpeg()
        if jpeg is None:
            self.send_response(503)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg)))
        self.end_headers()
        self.wfile.write(jpeg)

    def do_GET(self):
        path = self.path.rstrip("/")

        if path == "" or path == "/":
            self._send_json({
                "cameras": [
                    {"index": i, "label": c.label, "device": c.device,
                     "width": c.width, "height": c.height,
                     "stream": f"/stream/{i}", "snapshot": f"/snapshot/{i}"}
                    for i, c in enumerate(self.cameras)
                ]
            })
            return

        parts = path.split("/")
        if len(parts) == 3 and parts[1] in ("stream", "snapshot"):
            try:
                idx = int(parts[2])
                cam = self.cameras[idx]
            except (ValueError, IndexError):
                self.send_response(404)
                self.end_headers()
                return
            if parts[1] == "stream":
                self._stream(cam)
            else:
                self._snapshot(cam)
            return

        self.send_response(404)
        self.end_headers()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()

    cameras = load_cameras()
    if not cameras:
        log.error("No cameras available — exiting")
        return

    StreamHandler.cameras = cameras
    server = HTTPServer((args.host, args.port), StreamHandler)
    log.info("Streaming %d camera(s) on http://%s:%d", len(cameras), args.host, args.port)
    for i, c in enumerate(cameras):
        log.info("  /stream/%d  -> %s (%s)", i, c.label, c.device)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log.info("Shutting down")
    finally:
        for cam in cameras:
            cam.stop()


if __name__ == "__main__":
    main()
