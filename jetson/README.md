# NVIDIA Jetson Nano — Sensor Fusion Platform (Sphero RVR)

Real-time sensor fusion on NVIDIA Jetson Nano 4GB with a Sphero RVR rover.
Fuses Sphero RVR IMU + wheel encoders with camera visual odometry using an
Extended Kalman Filter to produce a continuous 2D pose estimate.

---

## Hardware

| Component | Details |
|-----------|---------|
| Board | NVIDIA Jetson Nano 4GB Developer Kit |
| CPU | ARM Cortex-A57 quad-core @ 1.43 GHz |
| GPU | 128-core Maxwell, CUDA 10.2 |
| RAM | 4 GB LPDDR4 |
| OS | Ubuntu 18.04.6 LTS (JetPack 4.6) |
| IP | `100.112.94.123` |
| Camera | IMX219 / OV5647 CSI or USB UVC |
| Rover | Sphero RVR (UART via USB-C → `/dev/ttyUSB0`) |
| Rover IMU | LSM6DS3 6-DOF @ 20 Hz |
| Rover encoders | 108 CPR quadrature per wheel |

---

## Architecture

```
Sphero RVR ─(UART)─► SpheroProcessor ──┐
                                         ├──► Extended Kalman Filter ──► Pose {x,y,θ,v,ω}
Camera ──────────► CameraProcessor ─────┤     (predict 50 Hz)
                   (LK optical flow)    │
                                         └──► IMUProcessor (complementary filter)
```

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for full pipeline diagram.

---

## Repository Structure

```
jetson/
├── README.md
├── requirements.txt              # Python deps (sphero-sdk, opencv, filterpy, ...)
├── src/
│   ├── main.py                   # Asyncio fusion loop (50 Hz)
│   └── fusion/
│       ├── __init__.py
│       ├── ekf.py                # Extended Kalman Filter (state: x,y,θ,v,ω)
│       ├── sphero_processor.py   # Sphero RVR UART telemetry (IMU + encoders)
│       ├── imu_processor.py      # Complementary filter + optional MPU-6050
│       └── camera_processor.py   # CSI/USB camera + Lucas-Kanade visual odometry
├── docker/
│   ├── Dockerfile                # L4T r32.7.1 (JetPack 4.6) base
│   ├── build.sh
│   └── run.sh                    # --runtime nvidia + /dev passthrough
└── docs/
    ├── ARCHITECTURE.md           # Detailed pipeline + EKF model
    └── BRINGUP.md                # Step-by-step setup from scratch
```

---

## Quick Start

### 1. Connect hardware

```bash
# Sphero RVR: USB-C cable → Jetson USB-A
ls /dev/ttyUSB0    # should appear

# Camera: CSI ribbon or USB camera → /dev/video0
ls /dev/video0
```

### 2. Install dependencies

```bash
pip3 install -r requirements.txt
```

### 3. Run directly

```bash
cd src
python3 main.py --port /dev/ttyUSB0 --camera csi
# or USB camera:
python3 main.py --port /dev/ttyUSB0 --camera usb
```

### 4. Run in Docker

```bash
bash docker/build.sh
SPHERO_PORT=/dev/ttyUSB0 CAMERA_SOURCE=csi bash docker/run.sh
```

---

## Docs

- [Architecture & EKF model](docs/ARCHITECTURE.md)
- [Wiring — Sphero RVR ↔ Jetson Nano](docs/WIRING.md)
- [Bring-up guide](docs/BRINGUP.md)
