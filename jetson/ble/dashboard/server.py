"""
Flask dashboard server for RVR sensor fusion.
Routes:
  GET /              → live dashboard HTML
  GET /stream        → MJPEG camera stream
  GET /api/state     → JSON pose + IMU snapshot
  GET /api/trajectory → JSON trajectory points
  POST /api/reset    → reset trajectory + filter
"""
import cv2
import time
import json
from flask import Flask, Response, request

DASHBOARD_HTML = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>RVR Sensor Fusion</title>
  <style>
    *{margin:0;padding:0;box-sizing:border-box;}
    body{background:#0d1117;color:#e6edf3;font-family:monospace;padding:12px;}
    h1{font-size:1rem;color:#58a6ff;margin-bottom:12px;letter-spacing:.05em;}
    .grid{display:grid;grid-template-columns:1fr 1fr;grid-template-rows:auto auto;gap:12px;}
    .panel{background:#161b22;border:1px solid #30363d;border-radius:6px;padding:10px;}
    .panel h2{font-size:.75rem;color:#8b949e;margin-bottom:8px;text-transform:uppercase;letter-spacing:.08em;}
    #cam-feed{width:100%;display:block;border-radius:4px;}
    #traj-canvas{width:100%;background:#0d1117;border-radius:4px;display:block;}
    .kv-grid{display:grid;grid-template-columns:1fr 1fr;gap:4px 12px;font-size:.8rem;}
    .kv-grid .k{color:#8b949e;}
    .kv-grid .v{color:#79c0ff;text-align:right;}
    .dot{display:inline-block;width:8px;height:8px;border-radius:50%;margin-right:5px;}
    .dot.on{background:#3fb950;box-shadow:0 0 6px #3fb950;}
    .dot.off{background:#f85149;}
    #status-bar{display:flex;align-items:center;font-size:.75rem;color:#8b949e;margin-bottom:10px;gap:16px;}
    #chart-accel,#chart-gyro{width:100%!important;height:140px!important;}
    .chart-wrap{margin-top:6px;}
    .chart-label{font-size:.7rem;color:#8b949e;margin-bottom:2px;}
    button{background:#21262d;border:1px solid #30363d;color:#e6edf3;padding:4px 10px;
           border-radius:4px;cursor:pointer;font-size:.75rem;font-family:monospace;}
    button:hover{background:#30363d;}
  </style>
</head>
<body>
  <h1>&#9679; RVR Sensor Fusion — Jetson Nano</h1>
  <div id="status-bar">
    <span><span class="dot off" id="ble-dot"></span><span id="ble-txt">BLE disconnected</span></span>
    <span>VO: <span id="vo-fps">0</span> fps</span>
    <span><button onclick="resetTraj()">Reset Trajectory</button></span>
  </div>

  <div class="grid">
    <!-- Camera -->
    <div class="panel">
      <h2>Camera Feed</h2>
      <img id="cam-feed" src="/stream" alt="camera">
    </div>

    <!-- Trajectory -->
    <div class="panel">
      <h2>2D Trajectory Map</h2>
      <canvas id="traj-canvas" width="480" height="360"></canvas>
    </div>

    <!-- IMU -->
    <div class="panel">
      <h2>IMU</h2>
      <div class="chart-wrap">
        <div class="chart-label">Accelerometer (G) — X:red Y:green Z:blue</div>
        <canvas id="chart-accel"></canvas>
      </div>
      <div class="chart-wrap" style="margin-top:10px;">
        <div class="chart-label">Gyroscope (deg/s) — Roll:red Pitch:green Yaw:blue</div>
        <canvas id="chart-gyro"></canvas>
      </div>
    </div>

    <!-- Pose readout -->
    <div class="panel">
      <h2>Pose &amp; Status</h2>
      <div class="kv-grid">
        <span class="k">X (m)</span><span class="v" id="v-x">0.0000</span>
        <span class="k">Y (m)</span><span class="v" id="v-y">0.0000</span>
        <span class="k">Heading (°)</span><span class="v" id="v-hdg">0.00</span>
        <span class="k">Pitch (°)</span><span class="v" id="v-pitch">0.00</span>
        <span class="k">Roll (°)</span><span class="v" id="v-roll">0.00</span>
        <span class="k">Accel X (G)</span><span class="v" id="v-ax">0.0000</span>
        <span class="k">Accel Y (G)</span><span class="v" id="v-ay">0.0000</span>
        <span class="k">Accel Z (G)</span><span class="v" id="v-az">0.0000</span>
        <span class="k">Gyro Roll</span><span class="v" id="v-gr">0.000</span>
        <span class="k">Gyro Pitch</span><span class="v" id="v-gp">0.000</span>
        <span class="k">Gyro Yaw</span><span class="v" id="v-gy">0.000</span>
      </div>
    </div>
  </div>

  <script src="https://cdn.jsdelivr.net/npm/chart.js@3.9.1/dist/chart.min.js"></script>
  <script>
  // ---- rolling buffer helpers ----
  var WINDOW = 200;
  function makeBuffer() { return []; }
  function push(buf, v) { buf.push(v); if(buf.length > WINDOW) buf.shift(); }

  // ---- chart setup ----
  function makeChart(id, labels, colors) {
    var ctx = document.getElementById(id).getContext('2d');
    var datasets = labels.map(function(l, i) {
      return {label: l, data: [], borderColor: colors[i],
              borderWidth: 1.5, pointRadius: 0, tension: 0.1};
    });
    return new Chart(ctx, {
      type: 'line',
      data: {labels: Array(WINDOW).fill(''), datasets: datasets},
      options: {
        animation: false,
        plugins: {legend: {display: false}},
        scales: {
          x: {display: false},
          y: {ticks: {color:'#8b949e', font:{size:9}},
              grid: {color:'#21262d'}}
        }
      }
    });
  }

  var accelChart = makeChart('chart-accel',
    ['ax','ay','az'], ['#f85149','#3fb950','#58a6ff']);
  var gyroChart  = makeChart('chart-gyro',
    ['gr','gp','gy'], ['#f85149','#3fb950','#58a6ff']);

  // ---- buffers ----
  var bufs = {ax:makeBuffer(),ay:makeBuffer(),az:makeBuffer(),
              gr:makeBuffer(),gp:makeBuffer(),gy:makeBuffer()};

  // ---- trajectory canvas ----
  var trajCanvas = document.getElementById('traj-canvas');
  var trajCtx    = trajCanvas.getContext('2d');
  var trajData   = [];

  function drawTrajectory() {
    var W = trajCanvas.width, H = trajCanvas.height;
    trajCtx.fillStyle = '#0d1117';
    trajCtx.fillRect(0,0,W,H);
    if(trajData.length < 2) return;

    var xs = trajData.map(function(p){return p[0];});
    var ys = trajData.map(function(p){return p[1];});
    var minX = Math.min.apply(null,xs), maxX = Math.max.apply(null,xs);
    var minY = Math.min.apply(null,ys), maxY = Math.max.apply(null,ys);
    var pad = 0.2;
    var rx = maxX-minX || 1, ry = maxY-minY || 1;
    minX -= rx*pad; maxX += rx*pad;
    minY -= ry*pad; maxY += ry*pad;
    var scaleX = W/(maxX-minX), scaleY = H/(maxY-minY);

    function tx(x){return (x-minX)*scaleX;}
    function ty(y){return H-(y-minY)*scaleY;}

    // grid lines
    trajCtx.strokeStyle='#21262d'; trajCtx.lineWidth=0.5;
    for(var gx=Math.ceil(minX*10)/10; gx<maxX; gx+=0.1){
      trajCtx.beginPath(); trajCtx.moveTo(tx(gx),0); trajCtx.lineTo(tx(gx),H); trajCtx.stroke();
    }
    for(var gy=Math.ceil(minY*10)/10; gy<maxY; gy+=0.1){
      trajCtx.beginPath(); trajCtx.moveTo(0,ty(gy)); trajCtx.lineTo(W,ty(gy)); trajCtx.stroke();
    }

    // path
    trajCtx.beginPath();
    trajCtx.strokeStyle='#388bfd'; trajCtx.lineWidth=1.5;
    trajCtx.moveTo(tx(trajData[0][0]), ty(trajData[0][1]));
    for(var i=1;i<trajData.length;i++){
      trajCtx.lineTo(tx(trajData[i][0]), ty(trajData[i][1]));
    }
    trajCtx.stroke();

    // current position
    var last = trajData[trajData.length-1];
    trajCtx.fillStyle='#58a6ff';
    trajCtx.beginPath();
    trajCtx.arc(tx(last[0]),ty(last[1]),5,0,2*Math.PI);
    trajCtx.fill();
  }

  // ---- poll state ----
  function updateState(d) {
    document.getElementById('v-x').textContent    = d.x.toFixed(4);
    document.getElementById('v-y').textContent    = d.y.toFixed(4);
    document.getElementById('v-hdg').textContent  = d.heading.toFixed(2);
    document.getElementById('v-pitch').textContent= d.pitch.toFixed(2);
    document.getElementById('v-roll').textContent = d.roll.toFixed(2);
    document.getElementById('v-ax').textContent   = d.accel[0].toFixed(4);
    document.getElementById('v-ay').textContent   = d.accel[1].toFixed(4);
    document.getElementById('v-az').textContent   = d.accel[2].toFixed(4);
    document.getElementById('v-gr').textContent   = d.gyro[0].toFixed(3);
    document.getElementById('v-gp').textContent   = d.gyro[1].toFixed(3);
    document.getElementById('v-gy').textContent   = d.gyro[2].toFixed(3);
    document.getElementById('vo-fps').textContent = d.vo_fps;

    var dot = document.getElementById('ble-dot');
    var txt = document.getElementById('ble-txt');
    if(d.ble_connected){ dot.className='dot on'; txt.textContent='BLE connected'; }
    else               { dot.className='dot off'; txt.textContent='BLE disconnected'; }

    push(bufs.ax, d.accel[0]); push(bufs.ay, d.accel[1]); push(bufs.az, d.accel[2]);
    push(bufs.gr, d.gyro[0]);  push(bufs.gp, d.gyro[1]);  push(bufs.gy, d.gyro[2]);

    accelChart.data.datasets[0].data = bufs.ax.slice();
    accelChart.data.datasets[1].data = bufs.ay.slice();
    accelChart.data.datasets[2].data = bufs.az.slice();
    accelChart.update('none');

    gyroChart.data.datasets[0].data = bufs.gr.slice();
    gyroChart.data.datasets[1].data = bufs.gp.slice();
    gyroChart.data.datasets[2].data = bufs.gy.slice();
    gyroChart.update('none');
  }

  setInterval(function(){
    fetch('/api/state').then(function(r){return r.json();}).then(updateState)
      .catch(function(){});
  }, 100);

  setInterval(function(){
    fetch('/api/trajectory').then(function(r){return r.json();}).then(function(pts){
      trajData = pts; drawTrajectory();
    }).catch(function(){});
  }, 500);

  function resetTraj(){
    fetch('/api/reset', {method:'POST'}).catch(function(){});
    trajData=[];
  }

  // reconnect cam on error
  var cam = document.getElementById('cam-feed');
  cam.onerror = function(){ setTimeout(function(){ cam.src='/stream?'+Date.now(); }, 2000); };
  </script>
</body>
</html>"""


def gen_frames(frame_buf):
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 75]
    while True:
        frame = frame_buf.get_frame()
        if frame is None:
            time.sleep(0.05)
            continue
        _, buf = cv2.imencode('.jpg', frame, encode_params)
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' +
            buf.tobytes() +
            b'\r\n'
        )


def create_app(pose_state, frame_buf, fusion_filter=None):
    app = Flask(__name__)

    @app.route('/')
    def index():
        return DASHBOARD_HTML

    @app.route('/stream')
    def stream():
        return Response(
            gen_frames(frame_buf),
            mimetype='multipart/x-mixed-replace; boundary=frame'
        )

    @app.route('/api/state')
    def api_state():
        return Response(
            json.dumps(pose_state.to_dict()),
            mimetype='application/json'
        )

    @app.route('/api/trajectory')
    def api_trajectory():
        return Response(
            json.dumps(pose_state.trajectory_list()),
            mimetype='application/json'
        )

    @app.route('/api/reset', methods=['POST'])
    def api_reset():
        pose_state.reset_trajectory()
        if fusion_filter:
            fusion_filter.reset()
        return Response('{"ok":true}', mimetype='application/json')

    return app
