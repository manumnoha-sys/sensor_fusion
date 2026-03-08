#!/usr/bin/env python3
"""
IMX477 MJPEG camera stream server for Jetson Nano.

Usage:
    python3 stream_camera.py [--port 8080] [--width 1280] [--height 720]

Open in browser:
    http://<jetson-ip>:8080
"""

import argparse
import time
import cv2
from flask import Flask, Response, render_template_string

app = Flask(__name__)

# ---- Config (overridden by CLI args) ----
WIDTH   = 1280
HEIGHT  = 720
FPS     = 30
PORT    = 8080
QUALITY = 80   # JPEG quality 0-100

# ---- GStreamer pipeline for IMX477 via nvarguscamerasrc ----
def make_pipeline(width, height, fps):
    return (
        f"nvarguscamerasrc sensor-id=0 ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},framerate={fps}/1 ! "
        f"nvvidconv flip-method=0 ! "
        f"video/x-raw,format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw,format=BGR ! "
        f"appsink drop=1 max-buffers=2 sync=false"
    )

# ---- Frame generator ----
def gen_frames(cap):
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, QUALITY]
    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue
        _, buf = cv2.imencode('.jpg', frame, encode_params)
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buf.tobytes() +
            b'\r\n'
        )

# ---- HTML page ----
PAGE = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>IMX477 Live Stream</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { background: #111; color: #eee; font-family: monospace;
           display: flex; flex-direction: column; align-items: center;
           min-height: 100vh; padding: 20px; }
    h1  { font-size: 1.1rem; margin-bottom: 12px; color: #76b900; }
    img { max-width: 100%; border: 1px solid #333; border-radius: 4px; }
    .info { font-size: 0.75rem; color: #666; margin-top: 8px; }
  </style>
</head>
<body>
  <h1>&#9679; IMX477 — Jetson Nano Live</h1>
  <img src="/stream" alt="camera stream">
  <p class="info">{{ width }}x{{ height }} @ {{ fps }}fps &nbsp;|&nbsp; MJPEG</p>
  <script>
    // reload img on error (reconnect if stream drops)
    const img = document.querySelector('img');
    img.onerror = () => setTimeout(() => img.src = '/stream?' + Date.now(), 2000);
  </script>
</body>
</html>"""

@app.route('/')
def index():
    return render_template_string(PAGE, width=WIDTH, height=HEIGHT, fps=FPS)

@app.route('/stream')
def stream():
    return Response(
        gen_frames(app.camera),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',   type=int, default=8080)
    parser.add_argument('--width',  type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parser.add_argument('--fps',    type=int, default=30)
    parser.add_argument('--quality',type=int, default=80)
    args = parser.parse_args()

    WIDTH   = args.width
    HEIGHT  = args.height
    FPS     = args.fps
    QUALITY = args.quality
    PORT    = args.port

    pipeline = make_pipeline(WIDTH, HEIGHT, FPS)
    print(f"Opening camera: {WIDTH}x{HEIGHT} @ {FPS}fps")
    print(f"GStreamer: {pipeline}")

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print("ERROR: Could not open camera via GStreamer.")
        raise SystemExit(1)

    print(f"Camera open. Streaming at http://0.0.0.0:{PORT}")
    app.camera = cap

    try:
        app.run(host='0.0.0.0', port=PORT, threaded=True)
    finally:
        cap.release()
