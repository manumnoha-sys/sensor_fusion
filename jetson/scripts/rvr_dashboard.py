#!/usr/bin/env python3
"""
RVR Dashboard — camera stream + random-drive control.

Usage:
    sudo python3 rvr_dashboard.py [--port 5000] [--cam 1]

Open in browser:
    http://<jetson-ip>:5000
"""

import argparse
import random
import threading
import time

import cv2
from flask import Flask, Response, jsonify, render_template_string, request

from rvr_control import RVR

app = Flask(__name__)

# ------------------------------------------------------------------ #
# Shared state
# ------------------------------------------------------------------ #
state = {
    "connected":   False,
    "driving":     False,
    "obstacle_nav": False,
    "battery":     None,
    "heading":     None,
    "speed":       80,
    "min_dur":     0.8,
    "max_dur":     2.5,
    "pause":       0.3,
    "color":       [0, 0, 0],
    "legs":        0,
    "avoids":      0,
    "error":       None,
}
state_lock    = threading.Lock()
drive_event   = threading.Event()   # set = stop driving
obstacle_event = threading.Event()  # set = stop obstacle nav
rvr = None  # type: RVR
rvr_lock = threading.Lock()

COLORS = [
    (255, 0,   0),
    (255, 128, 0),
    (255, 255, 0),
    (0,   255, 0),
    (0,   255, 255),
    (0,   0,   255),
    (128, 0,   255),
    (255, 0,   255),
]

# ------------------------------------------------------------------ #
# RVR connection
# ------------------------------------------------------------------ #
def connect_rvr():
    global rvr
    with rvr_lock:
        try:
            rvr = RVR()
            rvr.wake()
            batt = rvr.get_battery()
            with state_lock:
                state["connected"] = True
                state["battery"]   = batt
                state["error"]     = None
        except Exception as e:
            with state_lock:
                state["connected"] = False
                state["error"]     = str(e)


def disconnect_rvr():
    global rvr
    with rvr_lock:
        if rvr:
            try:
                rvr.stop()
                rvr.set_leds(0, 0, 0)
                rvr.disconnect()
            except Exception:
                pass
            rvr = None
    with state_lock:
        state["connected"] = False
        state["driving"]   = False
        state["color"]     = [0, 0, 0]


# ------------------------------------------------------------------ #
# Drive loop (runs in background thread)
# ------------------------------------------------------------------ #
def drive_loop():
    global rvr
    with state_lock:
        state["driving"] = True
        state["legs"]    = 0
    drive_event.clear()

    try:
        while not drive_event.is_set():
            with state_lock:
                speed   = state["speed"]
                min_dur = state["min_dur"]
                max_dur = state["max_dur"]
                pause   = state["pause"]

            heading  = random.randint(0, 359)
            duration = random.uniform(min_dur, max_dur)
            r, g, b  = random.choice(COLORS)

            with state_lock:
                state["heading"] = heading
                state["color"]   = [r, g, b]

            with rvr_lock:
                if rvr is None:
                    break
                rvr.set_leds(r, g, b)
                rvr.drive(speed, heading)

            drive_event.wait(timeout=duration)
            if drive_event.is_set():
                break

            with rvr_lock:
                if rvr:
                    rvr.stop()
            drive_event.wait(timeout=pause)

            with state_lock:
                state["legs"] += 1

    finally:
        with rvr_lock:
            if rvr:
                try:
                    rvr.stop()
                    rvr.set_leds(0, 0, 0)
                except Exception:
                    pass
        with state_lock:
            state["driving"] = False
            state["heading"] = None
            state["color"]   = [0, 0, 0]


# ------------------------------------------------------------------ #
# Obstacle navigation loop
# ------------------------------------------------------------------ #
def obstacle_nav_loop():
    global rvr
    with state_lock:
        state["obstacle_nav"] = True
        state["avoids"]       = 0
        speed = state["speed"]

    heading = 0
    with rvr_lock:
        if rvr:
            rvr.reset_yaw()
    obstacle_event.clear()

    try:
        while not obstacle_event.is_set():
            with state_lock:
                speed = state["speed"]

            r, g, b = random.choice(COLORS)
            with state_lock:
                state["heading"] = heading
                state["color"]   = [r, g, b]

            with rvr_lock:
                if rvr is None:
                    break
                rvr.set_leds(r, g, b)
                stalled = rvr.drive_watch(speed, heading, 0.35)

            if stalled:
                # obstacle hit — back up and turn
                with state_lock:
                    state["avoids"] += 1
                    state["color"]  = [255, 0, 0]  # flash red

                with rvr_lock:
                    if rvr:
                        rvr.set_leds(255, 0, 0)
                        rvr.backup(speed, 0.7)

                turn = random.randint(110, 160) * random.choice([-1, 1])
                heading = (heading + turn) % 360

                with rvr_lock:
                    if rvr:
                        rvr.reset_yaw()
                heading = 0
            else:
                # no obstacle — small heading drift to avoid looping
                heading = (heading + random.randint(-20, 20)) % 360

            with state_lock:
                state["legs"] += 1

            obstacle_event.wait(timeout=0.05)

    finally:
        with rvr_lock:
            if rvr:
                try:
                    rvr.stop()
                    rvr.set_leds(0, 0, 0)
                except Exception:
                    pass
        with state_lock:
            state["obstacle_nav"] = False
            state["heading"]      = None
            state["color"]        = [0, 0, 0]


# ------------------------------------------------------------------ #
# Camera stream
# ------------------------------------------------------------------ #
camera = None
camera_lock = threading.Lock()


def open_camera(index: int):
    global camera
    pipeline = (
        "nvarguscamerasrc sensor-id={} "
        "exposuretimerange=\"34000 358733000\" gainrange=\"1 16\" ispdigitalgainrange=\"1 8\" ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=2 sync=false"
    ).format(index)
    with camera_lock:
        camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not camera.isOpened():
            # fallback to direct V4L2
            print("GStreamer failed, falling back to V4L2 index {}".format(index))
            camera = cv2.VideoCapture(index)


def gen_frames():
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 75]
    while True:
        with camera_lock:
            if camera is None or not camera.isOpened():
                time.sleep(0.1)
                continue
            ret, frame = camera.read()
        if not ret:
            time.sleep(0.05)
            continue
        _, buf = cv2.imencode('.jpg', frame, encode_params)
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n'
            + buf.tobytes()
            + b'\r\n'
        )


# ------------------------------------------------------------------ #
# HTML dashboard
# ------------------------------------------------------------------ #
PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RVR Dashboard</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0d0d0d; color: #e0e0e0;
    font-family: 'Segoe UI', system-ui, sans-serif;
    min-height: 100vh;
    display: grid;
    grid-template-columns: 1fr 340px;
    grid-template-rows: auto 1fr;
    gap: 0;
  }

  /* Header */
  header {
    grid-column: 1 / -1;
    background: #161616;
    border-bottom: 1px solid #2a2a2a;
    padding: 14px 24px;
    display: flex; align-items: center; gap: 16px;
  }
  header h1 { font-size: 1.1rem; font-weight: 600; letter-spacing: 0.05em; color: #fff; }
  .badge {
    font-size: 0.72rem; padding: 3px 10px; border-radius: 20px;
    font-weight: 600; letter-spacing: 0.04em;
  }
  .badge.on  { background: #1a3a1a; color: #4caf50; border: 1px solid #4caf50; }
  .badge.off { background: #3a1a1a; color: #f44336; border: 1px solid #f44336; }
  .badge.driving { background: #1a2a3a; color: #2196f3; border: 1px solid #2196f3; }

  /* Camera pane */
  .camera-pane {
    background: #000;
    display: flex; align-items: center; justify-content: center;
    overflow: hidden;
    position: relative;
  }
  .camera-pane img {
    width: 100%; height: 100%;
    object-fit: contain;
  }
  .cam-overlay {
    position: absolute; bottom: 12px; left: 12px;
    background: rgba(0,0,0,0.55); padding: 4px 10px;
    border-radius: 4px; font-size: 0.7rem; color: #aaa;
  }

  /* Control panel */
  .panel {
    background: #161616;
    border-left: 1px solid #2a2a2a;
    padding: 20px;
    overflow-y: auto;
    display: flex; flex-direction: column; gap: 20px;
  }

  .card {
    background: #1e1e1e;
    border: 1px solid #2a2a2a;
    border-radius: 8px;
    padding: 16px;
  }
  .card h2 {
    font-size: 0.75rem; font-weight: 600; letter-spacing: 0.08em;
    color: #666; text-transform: uppercase; margin-bottom: 14px;
  }

  /* Status rows */
  .stat-row {
    display: flex; justify-content: space-between; align-items: center;
    padding: 5px 0; border-bottom: 1px solid #252525; font-size: 0.85rem;
  }
  .stat-row:last-child { border-bottom: none; }
  .stat-val { font-weight: 600; color: #fff; }

  /* Color swatch */
  .swatch {
    width: 18px; height: 18px; border-radius: 4px;
    display: inline-block; border: 1px solid #333; vertical-align: middle;
  }

  /* Sliders */
  .slider-group { display: flex; flex-direction: column; gap: 12px; }
  .slider-row label {
    display: flex; justify-content: space-between;
    font-size: 0.8rem; color: #aaa; margin-bottom: 5px;
  }
  .slider-row label span { color: #fff; font-weight: 600; }
  input[type=range] {
    width: 100%; accent-color: #2196f3; cursor: pointer;
  }

  /* Buttons */
  .btn {
    width: 100%; padding: 11px;
    border: none; border-radius: 6px;
    font-size: 0.9rem; font-weight: 600; cursor: pointer;
    transition: filter 0.15s, opacity 0.15s;
  }
  .btn:hover { filter: brightness(1.15); }
  .btn:active { filter: brightness(0.9); }
  .btn:disabled { opacity: 0.35; cursor: not-allowed; }
  .btn-start { background: #2196f3; color: #fff; }
  .btn-stop  { background: #f44336; color: #fff; }
  .btn-conn  { background: #4caf50; color: #fff; }
  .btn-disc  { background: #555;    color: #fff; }

  .btn-row { display: flex; gap: 8px; }
  .btn-row .btn { flex: 1; }

  /* Error */
  .error-box {
    background: #2a1010; border: 1px solid #f44336;
    border-radius: 6px; padding: 10px 12px;
    font-size: 0.78rem; color: #f44336; display: none;
  }
</style>
</head>
<body>

<header>
  <h1>&#9679; RVR Dashboard</h1>
  <span id="badge-conn" class="badge off">DISCONNECTED</span>
  <span id="badge-drive" class="badge off" style="display:none">DRIVING</span>
  <span id="badge-obstacle" class="badge off" style="display:none;background:#2a1f00;color:#ff9800;border:1px solid #ff9800">OBSTACLE NAV</span>
</header>

<!-- Camera -->
<div class="camera-pane">
  <img id="stream" src="/stream" alt="camera feed"
       onerror="setTimeout(()=>{ this.src='/stream?t='+Date.now() },2000)">
  <div class="cam-overlay">Arducam UC-376 IMX219 &nbsp;|&nbsp; 1280×720 MJPEG</div>
</div>

<!-- Controls -->
<div class="panel">

  <!-- Status -->
  <div class="card">
    <h2>Status</h2>
    <div class="stat-row">
      <span>Battery</span>
      <span class="stat-val" id="stat-batt">—</span>
    </div>
    <div class="stat-row">
      <span>Heading</span>
      <span class="stat-val" id="stat-heading">—</span>
    </div>
    <div class="stat-row">
      <span>LED color</span>
      <span class="stat-val">
        <span class="swatch" id="swatch"></span>
        <span id="stat-color">—</span>
      </span>
    </div>
    <div class="stat-row">
      <span>Legs driven</span>
      <span class="stat-val" id="stat-legs">0</span>
    </div>
    <div class="stat-row">
      <span>Obstacles avoided</span>
      <span class="stat-val" id="stat-avoids">0</span>
    </div>
  </div>

  <!-- Parameters -->
  <div class="card">
    <h2>Parameters</h2>
    <div class="slider-group">
      <div class="slider-row">
        <label>Speed <span id="lbl-speed">80</span></label>
        <input type="range" id="sl-speed" min="10" max="255" value="80"
               oninput="document.getElementById('lbl-speed').textContent=this.value">
      </div>
      <div class="slider-row">
        <label>Min duration (s) <span id="lbl-min">0.8</span></label>
        <input type="range" id="sl-min" min="0.2" max="5" step="0.1" value="0.8"
               oninput="document.getElementById('lbl-min').textContent=parseFloat(this.value).toFixed(1)">
      </div>
      <div class="slider-row">
        <label>Max duration (s) <span id="lbl-max">2.5</span></label>
        <input type="range" id="sl-max" min="0.5" max="10" step="0.1" value="2.5"
               oninput="document.getElementById('lbl-max').textContent=parseFloat(this.value).toFixed(1)">
      </div>
      <div class="slider-row">
        <label>Pause between legs (s) <span id="lbl-pause">0.3</span></label>
        <input type="range" id="sl-pause" min="0" max="3" step="0.1" value="0.3"
               oninput="document.getElementById('lbl-pause').textContent=parseFloat(this.value).toFixed(1)">
      </div>
    </div>
  </div>

  <!-- Drive control -->
  <div class="card">
    <h2>Drive</h2>
    <div class="btn-row" style="margin-bottom:10px">
      <button class="btn btn-start" id="btn-start" onclick="startDrive()">&#9654; Random</button>
      <button class="btn btn-stop"  id="btn-stop"  onclick="stopDrive()" disabled>&#9632; Stop</button>
    </div>
    <div class="btn-row">
      <button class="btn btn-obstacle" id="btn-obstacle" onclick="startObstacle()" style="background:#ff9800;color:#fff">
        &#9888; Obstacle Nav
      </button>
      <button class="btn btn-stop" id="btn-obstacle-stop" onclick="stopObstacle()" disabled>&#9632; Stop</button>
    </div>
  </div>

  <!-- Connection -->
  <div class="card">
    <h2>Connection</h2>
    <div class="btn-row">
      <button class="btn btn-conn" id="btn-conn" onclick="connectRVR()">Connect</button>
      <button class="btn btn-disc" id="btn-disc" onclick="disconnectRVR()" disabled>Disconnect</button>
    </div>
    <div class="error-box" id="error-box"></div>
  </div>

</div><!-- /panel -->

<script>
// ---- API helpers ----
async function api(path, body) {
  const opts = body
    ? { method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body) }
    : { method:'GET' };
  const r = await fetch(path, opts);
  return r.json();
}

function showError(msg) {
  const b = document.getElementById('error-box');
  if (msg) { b.textContent = msg; b.style.display = 'block'; }
  else      { b.style.display = 'none'; }
}

// ---- Actions ----
async function connectRVR() {
  showError(null);
  document.getElementById('btn-conn').disabled = true;
  await api('/api/connect', {});
}

async function disconnectRVR() {
  await api('/api/disconnect', {});
}

async function startDrive() {
  const speed   = parseFloat(document.getElementById('sl-speed').value);
  const min_dur = parseFloat(document.getElementById('sl-min').value);
  const max_dur = parseFloat(document.getElementById('sl-max').value);
  const pause   = parseFloat(document.getElementById('sl-pause').value);
  await api('/api/drive/start', { speed, min_dur, max_dur, pause });
}

async function stopDrive() {
  await api('/api/drive/stop', {});
}

// ---- Status polling ----
function applyStatus(s) {
  const connected = s.connected;
  const driving   = s.driving;
  const obstacle  = s.obstacle_nav;
  const busy      = driving || obstacle;

  document.getElementById('badge-conn').className  = 'badge ' + (connected ? 'on' : 'off');
  document.getElementById('badge-conn').textContent = connected ? 'CONNECTED' : 'DISCONNECTED';
  document.getElementById('badge-drive').style.display    = driving  ? '' : 'none';
  document.getElementById('badge-obstacle').style.display = obstacle ? '' : 'none';

  document.getElementById('stat-batt').textContent   = s.battery != null ? s.battery + '%' : '—';
  document.getElementById('stat-heading').textContent = s.heading != null ? s.heading + '°' : '—';
  document.getElementById('stat-legs').textContent   = s.legs;
  document.getElementById('stat-avoids').textContent = s.avoids || 0;

  const [r,g,b] = s.color || [0,0,0];
  document.getElementById('swatch').style.background = `rgb(${r},${g},${b})`;
  document.getElementById('stat-color').textContent  = (r||g||b) ? `rgb(${r},${g},${b})` : '—';

  document.getElementById('btn-conn').disabled          = connected;
  document.getElementById('btn-disc').disabled          = !connected;
  document.getElementById('btn-start').disabled         = !connected || busy;
  document.getElementById('btn-stop').disabled          = !connected || !driving;
  document.getElementById('btn-obstacle').disabled      = !connected || busy;
  document.getElementById('btn-obstacle-stop').disabled = !connected || !obstacle;

  if (s.error) showError(s.error);
  else showError(null);
}

async function startObstacle() {
  const speed   = parseFloat(document.getElementById('sl-speed').value);
  await api('/api/obstacle/start', { speed });
}

async function stopObstacle() {
  await api('/api/obstacle/stop', {});
}

async function pollStatus() {
  try {
    const s = await api('/api/status');
    applyStatus(s);
  } catch(e) {}
}

setInterval(pollStatus, 800);
pollStatus();
</script>
</body>
</html>"""


# ------------------------------------------------------------------ #
# Routes
# ------------------------------------------------------------------ #
@app.route('/')
def index():
    return render_template_string(PAGE)


@app.route('/stream')
def stream():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/api/status')
def api_status():
    with state_lock:
        return jsonify(dict(state))


@app.route('/api/connect', methods=['POST'])
def api_connect():
    t = threading.Thread(target=connect_rvr, daemon=True)
    t.start()
    return jsonify({"ok": True})


@app.route('/api/disconnect', methods=['POST'])
def api_disconnect():
    drive_event.set()
    obstacle_event.set()
    t = threading.Thread(target=disconnect_rvr, daemon=True)
    t.start()
    return jsonify({"ok": True})


@app.route('/api/drive/start', methods=['POST'])
def api_drive_start():
    with state_lock:
        if not state["connected"]:
            return jsonify({"ok": False, "error": "Not connected"}), 400
        if state["driving"]:
            return jsonify({"ok": False, "error": "Already driving"}), 400
        data = request.get_json(silent=True) or {}
        state["speed"]   = int(data.get("speed",   state["speed"]))
        state["min_dur"] = float(data.get("min_dur", state["min_dur"]))
        state["max_dur"] = float(data.get("max_dur", state["max_dur"]))
        state["pause"]   = float(data.get("pause",  state["pause"]))

    t = threading.Thread(target=drive_loop, daemon=True)
    t.start()
    return jsonify({"ok": True})


@app.route('/api/drive/stop', methods=['POST'])
def api_drive_stop():
    drive_event.set()
    return jsonify({"ok": True})


@app.route('/api/obstacle/start', methods=['POST'])
def api_obstacle_start():
    with state_lock:
        if not state["connected"]:
            return jsonify({"ok": False, "error": "Not connected"}), 400
        if state["obstacle_nav"] or state["driving"]:
            return jsonify({"ok": False, "error": "Already running"}), 400
        data = request.get_json(silent=True) or {}
        state["speed"] = int(data.get("speed", state["speed"]))

    t = threading.Thread(target=obstacle_nav_loop, daemon=True)
    t.start()
    return jsonify({"ok": True})


@app.route('/api/obstacle/stop', methods=['POST'])
def api_obstacle_stop():
    obstacle_event.set()
    return jsonify({"ok": True})


# ------------------------------------------------------------------ #
# Main
# ------------------------------------------------------------------ #
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5000)
    parser.add_argument('--cam',  type=int, default=1, help='/dev/videoN index')
    args = parser.parse_args()

    print(f"Opening camera /dev/video{args.cam}…")
    open_camera(args.cam)

    print(f"Dashboard at http://0.0.0.0:{args.port}")
    app.run(host='0.0.0.0', port=args.port, threaded=True)
