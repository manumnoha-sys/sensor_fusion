# Sensor Fusion — Multi-Platform

Real-time sensor fusion across two hardware platforms:

| Platform | Hardware | Approach |
|----------|----------|----------|
| **Kria KV260** | AMD/Xilinx FPGA SoC + ON Semi AR1335 camera + IMU | Complementary filter, MIPI CSI-2 pipeline |
| **Jetson Nano** | NVIDIA GPU (Maxwell) + CSI/USB camera + Sphero RVR rover | Extended Kalman Filter, visual odometry |

---

## Platform Docs

- **Kria KV260**: [kria/README.md](kria/README.md) · [docs/BRINGUP.md](docs/BRINGUP.md)
- **Jetson Nano + Sphero RVR**: [jetson/README.md](jetson/README.md) · [jetson/docs/BRINGUP.md](jetson/docs/BRINGUP.md)

---

## Kria KV260 Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Kria KV260 Starter Kit                              │
│                                                                             │
│  IAS Connector (J2/J3)                                                     │
│  ┌───────────────────┐   MIPI CSI-2 (2-lane)                               │
│  │  ON Semi IAS Cam  │──────────────────────────────────────────────┐      │
│  │  ┌─────────────┐  │                                               │      │
│  │  │   AR1335    │  │   I2C (via AXI I2C in PL)                    │      │
│  │  │  3MP CMOS   │  │◄──────────────────────────────────────────┐  │      │
│  │  │  1/3.2"     │  │                                            │  │      │
│  │  └──────┬──────┘  │                                            │  │      │
│  │         │ RAW     │                                            │  │      │
│  │  ┌──────▼──────┐  │                                            │  │      │
│  │  │   AP1302    │──┤──────────────────────────────────────────► │  │      │
│  │  │  ISP 0x3C   │  │  MIPI CSI-2 out                            │  │      │
│  │  └─────────────┘  │                                            │  │      │
│  └───────────────────┘                                            │  │      │
│                                                                    │  │      │
│  ┌─────────────────── FPGA Fabric (PL) ───────────────────────┐   │  │      │
│  │                                                             │   │  │      │
│  │  AXI I2C (0x80030000, i2c-3) → I2C Mux 0x74 → AP1302◄─────┼───┘  │      │
│  │                                                             │      │      │
│  │  MIPI CSI-2 RX (xlnx,mipi-csi2-rx-subsystem-5.0) ◄────────┼──────┘      │
│  │  @ 0x80000000                                               │             │
│  │        │ AXI4-Stream                                        │             │
│  │  Frame Buffer Writer @ 0xB0010000                           │             │
│  │        │ DMA                                                │             │
│  │  isp_vcap_csi  →  /dev/video0                               │             │
│  │                                                             │             │
│  │  VCU (H.264/H.265 HW encode) @ 0x80100000                  │             │
│  └─────────────────────────────────────────────────────────────┘             │
│                                                                             │
│  ┌──────────────────── ARM Cortex-A53 (PS) ───────────────────────────────┐ │
│  │  Ubuntu 22.04 LTS · kernel 5.15.0-1063-xilinx-zynqmp · aarch64        │ │
│  │                                                                         │ │
│  │  Kernel: xilinx-csi2rxss (built-in) · ap1302 (module) · ar1335 (mod)  │ │
│  │                                                                         │ │
│  │  /dev/video0  →  OpenCV / V4L2  →  sensor_fusion Python app            │ │
│  │                   CameraProcessor + IMUProcessor (complementary filter) │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘

            ┌─────────────────────────────────────┐
            │         Gazebo Simulation            │
            │  Ignition Fortress + ROS2 Humble     │
            │  AR1335-spec camera + 6-DOF IMU      │
            │  ros_gz_bridge → sensor_fusion app   │
            └─────────────────────────────────────┘
```

---

## Hardware

| Component | Details |
|-----------|---------|
| Board | AMD/Xilinx Kria KV260 Vision AI Starter Kit (SCK-KV-G Rev2) |
| SOM | K26, ARM Cortex-A53 quad-core 1333 MHz, 4 GB RAM |
| OS | Ubuntu 22.04.5 LTS (Jammy), kernel `5.15.0-1063-xilinx-zynqmp` |
| Camera | ON Semi AR1335, 3MP AutoFocus, 1/3.2" CMOS via IAS connector |
| Camera ISP | ON Semi AP1302, I2C addr `0x3C`, MIPI CSI-2 2-lane output |
| FPGA app | `kv260-smartcam` (2022.1) loaded via `xmutil` |
| Video node | `/dev/video0` (`isp_vcap_csi`, `xilinx-vipp` driver) |

---

## Repository Structure

```
sensor_fusion/
│
├── kria/                        # AMD/Xilinx Kria KV260 platform
│   └── README.md                # Platform overview + quick start
│
├── jetson/                      # NVIDIA Jetson Nano + Sphero RVR platform
│   ├── README.md
│   ├── requirements.txt
│   ├── src/
│   │   ├── main.py              # Asyncio fusion loop (50 Hz)
│   │   └── fusion/
│   │       ├── ekf.py           # Extended Kalman Filter (x,y,θ,v,ω)
│   │       ├── sphero_processor.py  # Sphero RVR UART telemetry
│   │       ├── imu_processor.py     # Complementary filter + MPU-6050
│   │       └── camera_processor.py  # CSI/USB camera + LK optical flow
│   ├── docker/
│   │   ├── Dockerfile           # L4T r32.7.1 (JetPack 4.6) base
│   │   ├── build.sh
│   │   └── run.sh
│   └── docs/
│       ├── ARCHITECTURE.md      # Full pipeline + EKF model
│       └── BRINGUP.md           # Jetson + Sphero RVR bring-up guide
│
├── src/                         # Kria canonical source (complementary filter)
│   ├── main.py
│   └── fusion/
│       ├── camera_processor.py  # V4L2 + motion detection
│       └── imu_processor.py     # Complementary filter → roll/pitch
├── Dockerfile                   # Kria arm64 dev container
├── requirements.txt
├── gazebo/                      # Ignition Fortress + ROS2 Humble simulation
│   ├── worlds/kv260_world.sdf
│   ├── models/kv260_robot/
│   ├── launch/simulate.launch.py
│   ├── config/ar1335_params.yaml
│   ├── scripts/sim_bridge.py
│   └── docker/Dockerfile
├── docker/
│   ├── build.sh
│   └── run.sh
└── docs/
    ├── ARCHITECTURE.md          # Multi-platform overview + Kria pipeline
    └── BRINGUP.md               # Kria KV260 camera bring-up guide
```

---

## Quick Start

### On KV260 (real hardware)

```bash
# 1. Load smartcam FPGA bitstream
sudo xmutil loadapp kv260-smartcam

# 2. Load camera driver
sudo modprobe ap1302

# 3. Run sensor fusion
bash docker/run.sh
# or directly: python3 src/main.py
```

→ See [docs/BRINGUP.md](docs/BRINGUP.md) for full camera bring-up from scratch.

### Gazebo Simulation (x86 dev machine)

```bash
# Build simulation container
docker build -f gazebo/docker/Dockerfile -t sensor-fusion-sim .

# Run with GUI
docker run -it --rm \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  sensor-fusion-sim

# Run headless
docker run -it --rm sensor-fusion-sim \
  ros2 launch gazebo/launch/simulate.launch.py headless:=true

# Bridge simulation data to sensor_fusion app
docker exec -it <container> python3 gazebo/scripts/sim_bridge.py
```

**ROS2 topics exposed:**

| Topic | Type | Source |
|-------|------|--------|
| `/kv260/camera/image_raw` | `sensor_msgs/Image` | AR1335-spec camera |
| `/kv260/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/kv260/imu/data_raw` | `sensor_msgs/Imu` | 6-DOF IMU @ 200 Hz |
| `/kv260/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/kv260/cmd_vel` | `geometry_msgs/Twist` | Drive command |

---

## Docs

- [Architecture diagram](docs/ARCHITECTURE.md)
- [Camera bring-up guide](docs/BRINGUP.md)
