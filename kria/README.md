# Kria KV260 — Sensor Fusion Platform

Real-time sensor fusion on the AMD/Xilinx Kria KV260 Vision AI Starter Kit.
Fuses an ON Semi AR1335 3MP MIPI camera (via AP1302 ISP) with a 6-DOF IMU.

## Hardware

| Component | Details |
|-----------|---------|
| Board | AMD/Xilinx Kria KV260 (SCK-KV-G Rev2) |
| SOM | K26, ARM Cortex-A53 quad-core 1333 MHz, 4 GB RAM |
| OS | Ubuntu 22.04.5 LTS, kernel `5.15.0-1063-xilinx-zynqmp` |
| Camera | ON Semi AR1335 3MP CMOS + AP1302 ISP via IAS connector |
| Video node | `/dev/video0` (`isp_vcap_csi`) |
| IP | `100.101.58.66` |

## Directory layout (top-level references)

```
kria/ (this folder)
└── README.md            <- you are here

Top-level paths (canonical Kria source):
  src/                   <- fusion Python app (main.py, camera_processor, imu_processor)
  docker/                <- build.sh + run.sh for xilinx/kria-developer container
  Dockerfile             <- arm64 dev container (xilinx/kria-developer:latest)
  gazebo/                <- Ignition Fortress + ROS2 Humble simulation
  docs/ARCHITECTURE.md   <- detailed hardware pipeline diagram
  docs/BRINGUP.md        <- step-by-step camera bring-up from scratch
  requirements.txt
```

## Quick Start

```bash
# On KV260 board (100.101.58.66)
ssh ubuntu@100.101.58.66

# 1. Load smartcam FPGA bitstream
sudo xmutil loadapp kv260-smartcam

# 2. Load camera driver
sudo modprobe ap1302

# 3. Run sensor fusion (Docker)
bash /path/to/sensor_fusion/docker/run.sh
# or directly:
python3 /path/to/sensor_fusion/src/main.py
```

## Docs

- [Architecture diagram](../docs/ARCHITECTURE.md)
- [Camera bring-up guide](../docs/BRINGUP.md)

## Key Known Issues

| Issue | Status | Fix |
|-------|--------|-----|
| AP1302 I2C timeout on `modprobe ap1302` | Active | Re-seat IAS camera cable, power-cycle |
| `READ_CALORIES_BURNED` (unrelated, Android) | N/A | See health-dashboard project |

## Docker

```bash
# Build
bash ../docker/build.sh

# Run (mounts /dev, network=host)
bash ../docker/run.sh
```

Base image: `xilinx/kria-developer:latest` (arm64, Ubuntu 22.04, ~706 MB)
