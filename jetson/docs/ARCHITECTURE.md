# Sensor Fusion Architecture — NVIDIA Jetson Nano + Sphero RVR

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       NVIDIA Jetson Nano 4GB                                │
│                  Ubuntu 18.04 LTS (JetPack 4.6)                             │
│                  CUDA 10.2 · Python 3.6 · aarch64                           │
│                                                                             │
│  ┌──────────────────────────────────────────────────────────────────────┐   │
│  │                        Camera Subsystem                              │   │
│  │                                                                      │   │
│  │  CSI connector (J13)                                                 │   │
│  │  ┌────────────────────────┐  MIPI CSI-2 (2-lane)                    │   │
│  │  │  IMX219 / OV5647       │──────────────────────►                  │   │
│  │  │  8MP / 5MP CSI camera  │                       │                 │   │
│  │  └────────────────────────┘                       │                 │   │
│  │                                                   ▼                 │   │
│  │  USB port                     ┌───────────────────────────────┐     │   │
│  │  ┌──────────────────────┐     │  nvarguscamerasrc (GStreamer)  │     │   │
│  │  │  USB camera (fallbk) │────►│  nvvidconv                    │     │   │
│  │  └──────────────────────┘     │  → /dev/video0 (V4L2)         │     │   │
│  │                               └─────────────┬─────────────────┘     │   │
│  │                                             │ BGR frames (30 fps)   │   │
│  └─────────────────────────────────────────────┼─────────────────────────┘   │
│                                                │                             │
│  ┌─────────────────────────────────────────────▼─────────────────────────┐   │
│  │                    sensor_fusion Python app                           │   │
│  │                                                                       │   │
│  │  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐    │   │
│  │  │ CameraProcessor  │  │  IMUProcessor    │  │ SpheroProcessor  │    │   │
│  │  │                  │  │                  │  │                  │    │   │
│  │  │ • GStreamer cap  │  │ • Complementary  │  │ • UART @ 115200  │    │   │
│  │  │ • Lucas-Kanade   │  │   filter (roll,  │  │ • IMU stream     │    │   │
│  │  │   optical flow   │  │   pitch, yaw)    │  │ • Encoder ticks  │    │   │
│  │  │ • Affine RANSAC  │  │ • MPU-6050 opt.  │  │ • Color sensor   │    │   │
│  │  │ • VO delta pose  │  │ • Wrap @ 20 Hz   │  │ • Ambient light  │    │   │
│  │  └────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘    │   │
│  │           │ (dx,dy,dθ)          │ (yaw_rad)            │ (v, ω)      │   │
│  │           └────────────────┬────┘──────────────────────┘             │   │
│  │                            ▼                                         │   │
│  │  ┌────────────────────────────────────────────────────────────────┐  │   │
│  │  │                  Extended Kalman Filter                        │  │   │
│  │  │                                                                │  │   │
│  │  │  State:  x = [x, y, θ, v, ω]   (2D pose + velocities)        │  │   │
│  │  │                                                                │  │   │
│  │  │  Predict (50 Hz): unicycle motion model                       │  │   │
│  │  │  Update A (20 Hz): IMU heading → yaw state                    │  │   │
│  │  │  Update B (20 Hz): encoders → v, ω states                     │  │   │
│  │  │  Update C (10 Hz): visual odometry → x, y, θ states           │  │   │
│  │  │                                                                │  │   │
│  │  │  Output: {x, y, theta, v, omega}                              │  │   │
│  │  └────────────────────────────────────────────────────────────────┘  │   │
│  └────────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
│  GPIO / UART  (/dev/ttyUSB0)                                                │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │ USB-C UART
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                          Sphero RVR Rover                                   │
│                                                                             │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────────────┐  │
│  │  LSM6DS3 IMU     │  │  Wheel Encoders  │  │  Sensor Suite            │  │
│  │  (6-DOF)         │  │  Left + Right    │  │                          │  │
│  │  Accel + Gyro    │  │  108 CPR quad.   │  │  • RGB color sensor      │  │
│  │  → roll/pitch/   │  │  → v (m/s)       │  │  • Ambient light (lux)   │  │
│  │    yaw @ 20 Hz   │  │  → ω (rad/s)     │  │  • IR proximity          │  │
│  └──────────────────┘  └──────────────────┘  └──────────────────────────┘  │
│                                                                             │
│  Drive: 2 × brushless DC motors, tank steering                              │
│  Top speed: ~2.0 m/s · Dimensions: 190 × 130 × 75 mm                       │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## EKF State & Noise Model

### State vector

```
x = [x_pos, y_pos, heading, linear_vel, angular_vel]
     metres  metres  radians    m/s          rad/s
```

### Motion model (unicycle, 50 Hz predict)

```
x'     = x + (v/ω) · (sin(θ + ω·dt) − sin(θ))    [arc motion]
y'     = y + (v/ω) · (cos(θ) − cos(θ + ω·dt))
θ'     = θ + ω · dt
v', ω' = v, ω                                      [constant velocity]
```

### Measurement sources

| Source | Rate | Measures | Noise σ |
|--------|------|----------|---------|
| Sphero IMU | 20 Hz | heading θ | 0.1 rad |
| Wheel encoders | 20 Hz | v (m/s), ω (rad/s) | 0.22, 0.14 |
| Visual odometry | 10 Hz | Δx, Δy, Δθ | 0.32, 0.32, 0.22 |

---

## Software Stack

```
┌──────────────────────────────────────────────────────┐
│              sensor_fusion Python app                │
│         src/main.py  (asyncio loop, 50 Hz)           │
├────────────────┬──────────────┬──────────────────────┤
│ CameraProcessor│ IMUProcessor │   SpheroProcessor    │
│ (OpenCV + LK)  │ (comp. filt) │   (Sphero SDK async) │
├────────────────┴──────────────┴──────────────────────┤
│              PoseEKF  (filterpy-style)               │
├──────────────────────────────────────────────────────┤
│   OpenCV 4.x   │  NumPy/SciPy  │  sphero-sdk 0.1.x  │
├──────────────────────────────────────────────────────┤
│   GStreamer 1.0  (nvarguscamerasrc + nvvidconv)       │
├──────────────────────────────────────────────────────┤
│   Python 3.6  ·  CUDA 10.2  ·  Ubuntu 18.04          │
├──────────────────────────────────────────────────────┤
│   L4T r32.7.1  (JetPack 4.6, Jetson Nano kernel)     │
└──────────────────────────────────────────────────────┘
```

---

## Component Details

| Component | Spec |
|-----------|------|
| Board | NVIDIA Jetson Nano 4GB Developer Kit |
| CPU | ARM Cortex-A57 quad-core @ 1.43 GHz |
| GPU | 128-core Maxwell, CUDA 10.2 |
| RAM | 4 GB LPDDR4 |
| OS | Ubuntu 18.04.6 LTS (JetPack 4.6) |
| Kernel | Linux 4.9.253-tegra (aarch64) |
| IP | 100.112.94.123 |
| Camera (CSI) | IMX219 / OV5647 — 8MP/5MP via MIPI CSI-2 |
| Camera (USB) | Any UVC-compatible camera |
| Rover | Sphero RVR (USB-C UART, /dev/ttyUSB0) |
| Rover IMU | LSM6DS3 6-DOF (accel + gyro) @ 20 Hz |
| Rover encoders | 108 CPR quadrature × 2 (left/right) |

---

## Data Flow

```
Sphero RVR (UART 115200)
  └─► SpheroProcessor._on_imu()       20 Hz → IMUProcessor.update_from_sphero()
  └─► SpheroProcessor._on_encoders()  20 Hz → ekf.update_encoders(v, ω)
  └─► SpheroProcessor._on_color()     20 Hz → latest.color_{r,g,b}
  └─► SpheroProcessor._on_ambient()   20 Hz → latest.ambient_lux

Camera (/dev/video0 via GStreamer)
  └─► CameraProcessor._capture_loop() 30 Hz → frame buffer
  └─► _update_visual_odometry()        30 Hz → LK optical flow → VO delta

main.py asyncio loop (50 Hz)
  ├─ ekf.predict(dt=0.02)              50 Hz
  ├─ ekf.update_imu(yaw_rad)           20 Hz
  ├─ ekf.update_encoders(v, ω)         20 Hz
  └─ ekf.update_visual_odometry(...)   10 Hz
       └─► pose = {x, y, theta, v, omega}
```
