# Sensor Fusion — Multi-Platform

Real-time sensor fusion across two embedded hardware platforms, each with its own fusion pipeline, Docker container, and bring-up guide.

---

## Platforms

| Platform | Board | Sensors | Fusion Method | Status |
|----------|-------|---------|---------------|--------|
| [Kria KV260](kria/README.md) | AMD/Xilinx KV260 (ARM Cortex-A53, FPGA) | AR1335 MIPI camera + 6-DOF IMU | Complementary filter | Camera bring-up in progress |
| [Jetson Nano](jetson/README.md) | NVIDIA Jetson Nano 4GB (Maxwell GPU) | CSI/USB camera + Sphero RVR (IMU + encoders) | Extended Kalman Filter | Active development |

---

## Repository Structure

```
sensor_fusion/
│
├── kria/                        # AMD/Xilinx Kria KV260 platform
│   └── README.md                # Hardware specs, quick start, links to src/ below
│
├── jetson/                      # NVIDIA Jetson Nano + Sphero RVR platform
│   ├── README.md
│   ├── requirements.txt
│   ├── src/
│   │   ├── main.py              # Asyncio EKF loop (50 Hz)
│   │   └── fusion/
│   │       ├── ekf.py           # Extended Kalman Filter — state [x, y, θ, v, ω]
│   │       ├── sphero_processor.py  # Sphero RVR UART telemetry (IMU + encoders)
│   │       ├── imu_processor.py     # Complementary filter + optional MPU-6050
│   │       └── camera_processor.py  # CSI/USB capture + Lucas-Kanade optical flow
│   ├── docker/
│   │   ├── Dockerfile           # L4T r32.7.1 (JetPack 4.6) base
│   │   ├── build.sh
│   │   └── run.sh               # --runtime nvidia + /dev passthrough
│   └── docs/
│       ├── ARCHITECTURE.md      # Full Jetson + Sphero pipeline diagram
│       └── BRINGUP.md           # Step-by-step Jetson + Sphero bring-up
│
├── src/                         # Kria canonical source
│   ├── main.py                  # Fusion loop (~30 Hz)
│   └── fusion/
│       ├── camera_processor.py  # V4L2 capture + motion detection
│       └── imu_processor.py     # Complementary filter → roll/pitch
├── Dockerfile                   # Kria arm64 dev container
├── requirements.txt
├── gazebo/                      # Ignition Fortress + ROS2 Humble simulation
│   ├── worlds/kv260_world.sdf
│   ├── models/kv260_robot/
│   ├── launch/simulate.launch.py
│   ├── config/ar1335_params.yaml
│   ├── scripts/sim_bridge.py
│   └── docker/Dockerfile        # x86 sim container
├── docker/
│   ├── build.sh
│   └── run.sh
└── docs/
    ├── ARCHITECTURE.md          # Multi-platform overview + Kria pipeline diagram
    └── BRINGUP.md               # Kria KV260 camera bring-up guide
```

---

## Kria KV260

**Board:** AMD/Xilinx KV260 Vision AI Starter Kit — ARM Cortex-A53 + FPGA fabric
**IP:** `100.101.58.66`

### Pipeline

```
ON Semi AR1335 (MIPI CSI-2)
  └─► AP1302 ISP (I2C 0x3C)
       └─► FPGA kv260-smartcam bitstream
            └─► /dev/video0
                 └─► CameraProcessor (OpenCV / V4L2)
                      └─► IMUProcessor (complementary filter)
                           └─► roll / pitch / yaw
```

### Quick Start

```bash
ssh ubuntu@100.101.58.66

# Load FPGA bitstream + camera driver
sudo xmutil loadapp kv260-smartcam
sudo modprobe ap1302

# Run fusion
bash docker/run.sh
# or: python3 src/main.py
```

**Docs:** [kria/README.md](kria/README.md) · [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) · [docs/BRINGUP.md](docs/BRINGUP.md)

---

## Jetson Nano + Sphero RVR

**Board:** NVIDIA Jetson Nano 4GB — ARM Cortex-A57 + 128-core Maxwell GPU
**IP:** `100.112.94.123` · Ubuntu 18.04 LTS (JetPack 4.6) · CUDA 10.2

### Pipeline

```
Sphero RVR (UART /dev/ttyUSB0)          Camera (/dev/video0)
  ├─ LSM6DS3 IMU    20 Hz                 └─ GStreamer nvarguscamerasrc / V4L2
  └─ Wheel encoders 20 Hz                      └─ Lucas-Kanade optical flow (10 Hz)
          │                                              │
          └──────────────┬───────────────────────────────┘
                         ▼
             Extended Kalman Filter (50 Hz)
             State: [x, y, θ, v, ω]
                         │
                         └─► Pose estimate {x, y, heading, velocity}
```

### Quick Start

```bash
ssh jetson4gb@100.112.94.123

# 1. Connect Sphero RVR via USB-C → verify /dev/ttyUSB0
# 2. Attach CSI camera ribbon or USB webcam

# Install deps
pip3 install -r jetson/requirements.txt

# Run fusion
cd jetson/src
python3 main.py --port /dev/ttyUSB0 --camera csi

# Or via Docker
bash jetson/docker/build.sh
SPHERO_PORT=/dev/ttyUSB0 CAMERA_SOURCE=csi bash jetson/docker/run.sh
```

**Docs:** [jetson/README.md](jetson/README.md) · [jetson/docs/ARCHITECTURE.md](jetson/docs/ARCHITECTURE.md) · [jetson/docs/BRINGUP.md](jetson/docs/BRINGUP.md)

---

## Gazebo Simulation (x86)

Ignition Fortress + ROS2 Humble simulation for Kria — AR1335-spec camera + 6-DOF IMU model.

```bash
docker build -f gazebo/docker/Dockerfile -t sensor-fusion-sim .

# Run with GUI
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix sensor-fusion-sim

# Run headless
docker run -it --rm sensor-fusion-sim \
  ros2 launch gazebo/launch/simulate.launch.py headless:=true
```

| ROS2 Topic | Type | Source |
|------------|------|--------|
| `/kv260/camera/image_raw` | `sensor_msgs/Image` | AR1335-spec camera |
| `/kv260/imu/data_raw` | `sensor_msgs/Imu` | 6-DOF IMU @ 200 Hz |
| `/kv260/odom` | `nav_msgs/Odometry` | Wheel odometry |

---

## Docs Index

| Document | Description |
|----------|-------------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Multi-platform overview + Kria hardware pipeline |
| [docs/BRINGUP.md](docs/BRINGUP.md) | Kria KV260 camera bring-up from scratch |
| [kria/README.md](kria/README.md) | Kria platform quick reference |
| [jetson/docs/ARCHITECTURE.md](jetson/docs/ARCHITECTURE.md) | Jetson + Sphero RVR full pipeline + EKF model |
| [jetson/docs/BRINGUP.md](jetson/docs/BRINGUP.md) | Jetson Nano + Sphero bring-up guide |
