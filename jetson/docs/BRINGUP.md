# NVIDIA Jetson Nano + Sphero RVR — Bring-Up Guide

## Board Details

- **Board:** NVIDIA Jetson Nano 4GB Developer Kit
- **IP:** `100.112.94.123`
- **User:** `jetson4gb` / **Password:** `A12b12345`
- **OS:** Ubuntu 18.04.6 LTS (Bionic Beaver)
- **Kernel:** Linux 4.9.253-tegra (aarch64)
- **CUDA:** 10.2
- **JetPack:** 4.6

---

## Step 1 — SSH to Jetson

```bash
ssh jetson4gb@100.112.94.123
# password: A12b12345

# Verify system
uname -a          # Linux jetson4gb-desktop 4.9.253-tegra ... aarch64
nvcc --version    # CUDA 10.2 (if JetPack fully set up)
```

---

## Step 2 — Install Docker (if not present)

```bash
# Jetson Nano comes with Docker via JetPack
docker --version
# Docker 20.x.x — already installed

# Verify NVIDIA container runtime
docker run --rm --runtime nvidia nvcr.io/nvidia/l4t-base:r32.7.1 nvcc --version
```

---

## Step 3 — Install pip3 and Python dependencies

```bash
sudo apt-get update
sudo apt-get install -y python3-pip python3-dev

# Verify
python3 --version    # Python 3.6.9
pip3 --version

# Install project dependencies
pip3 install --user -r /path/to/jetson/requirements.txt
```

---

## Step 4 — Connect Sphero RVR

1. Use the **USB-C cable** (included with RVR) to connect RVR's USB-C port to any USB-A port on the Jetson Nano.
2. Power on the Sphero RVR (hold power button ~2 seconds until LED lights).
3. Verify serial device:

```bash
ls /dev/ttyUSB* /dev/ttyACM*
# Should show: /dev/ttyUSB0  (or /dev/ttyACM0)

# Check permissions
ls -l /dev/ttyUSB0
# crw-rw---- 1 root dialout ...

# Add user to dialout group
sudo usermod -aG dialout jetson4gb
# Log out and back in, or: newgrp dialout
```

4. Test serial communication:

```bash
python3 -c "import serial; s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1); print('OK:', s.name); s.close()"
```

---

## Step 5 — Install Sphero SDK

```bash
pip3 install --user sphero-sdk-raspberrypi-python

# If pip install fails (Python 3.6 compatibility), install from source:
git clone https://github.com/sphero-inc/sphero-sdk-raspberrypi-python.git /tmp/sphero-sdk
pip3 install --user /tmp/sphero-sdk

# Verify
python3 -c "from sphero_sdk import SpheroRvrAsync; print('Sphero SDK OK')"
```

---

## Step 6 — Connect CSI Camera (optional)

1. Insert the IMX219 or OV5647 camera ribbon cable into the Jetson Nano's **CSI J13 connector** (lift the latch, insert blue side away from connector, lock latch).
2. Verify camera is detected:

```bash
ls /dev/video*
# /dev/video0

# Test GStreamer CSI capture (should open a window with live video)
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM),width=1280,height=720' ! \
    nvvidconv ! videoconvert ! autovideosink
```

3. If no CSI camera: use a USB webcam on any USB port. It appears as `/dev/video0`.

```bash
# Test USB camera
python3 -c "import cv2; cap=cv2.VideoCapture(0); ret,f=cap.read(); print('USB cam OK, frame:', f.shape); cap.release()"
```

---

## Step 7 — Build Docker image

```bash
# On Jetson Nano (or build locally and scp the image)
cd /path/to/sensor_fusion/jetson

# Clone or copy the project to Jetson
scp -r . jetson4gb@100.112.94.123:/home/jetson4gb/sensor_fusion/jetson/

# On Jetson:
bash docker/build.sh
# Builds nvcr.io/nvidia/l4t-base:r32.7.1 based image (~1.2 GB)
```

---

## Step 8 — Run sensor fusion

### Docker (recommended)

```bash
# CSI camera + Sphero RVR
SPHERO_PORT=/dev/ttyUSB0 CAMERA_SOURCE=csi bash docker/run.sh

# USB camera + Sphero RVR
SPHERO_PORT=/dev/ttyUSB0 CAMERA_SOURCE=usb bash docker/run.sh

# Sphero only (no camera)
docker run -it --rm --runtime nvidia --privileged -v /dev:/dev \
    jetson-sensor-fusion python3 main.py --no-camera --port /dev/ttyUSB0

# Camera only (no Sphero)
docker run -it --rm --runtime nvidia --privileged -v /dev:/dev \
    -v /tmp/argus_socket:/tmp/argus_socket \
    jetson-sensor-fusion python3 main.py --no-sphero --camera csi
```

### Directly on host

```bash
cd /path/to/jetson/src
python3 main.py --port /dev/ttyUSB0 --camera csi --verbose
```

---

## Step 9 — Verify fusion output

Expected console output:

```
2026-03-06 12:00:00 [INFO] Sphero RVR connected on /dev/ttyUSB0
2026-03-06 12:00:00 [INFO] Camera started (source=csi)
2026-03-06 12:00:00 [INFO] Fusion loop running at 50 Hz (Ctrl+C to stop)
2026-03-06 12:00:00 [INFO] pose  x=0.000 m  y=0.000 m  theta=0.0 deg  v=0.000 m/s  omega=0.000 rad/s | cam_fps=30.0
```

Drive the Sphero RVR via the Sphero app — the pose (x, y, theta) should update as the rover moves.

---

## Troubleshooting

### Sphero RVR not detected (`/dev/ttyUSB0` missing)

```bash
dmesg | tail -20   # Look for "cp210x" or "ch341" USB-serial adapter
lsusb              # Should show "Silicon Labs" or similar
```

- Try a different USB cable (some USB-C cables are charge-only)
- Try a different USB port on the Jetson

### Sphero SDK import error

```bash
pip3 install --user pyserial
pip3 install --user sphero-sdk-raspberrypi-python
```

### CSI camera — blank frames / no device

```bash
sudo nvpmodel -m 0   # Maximum performance mode
sudo jetson_clocks   # Max clocks

# Check nvargus daemon
sudo systemctl status nvargus-daemon
sudo systemctl restart nvargus-daemon
```

### Low camera FPS

```bash
# Check NVMM allocation
sudo tegrastats   # GPU usage, EMC bandwidth

# Reduce resolution in CameraProcessor (width=640, height=360 is default)
```

### CUDA not found

```bash
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
nvcc --version   # CUDA 10.2
```

---

## I2C — Optional hardware IMU (MPU-6050)

If attaching a hardware IMU to Jetson GPIO:

```
Jetson Nano J41 header:
  Pin 27 (SDA1) → MPU-6050 SDA
  Pin 28 (SCL1) → MPU-6050 SCL
  Pin 1  (3.3V) → VCC
  Pin 6  (GND)  → GND

# Enable I2C
sudo i2cdetect -y -r 1
# Should show 0x68 (or 0x69 if AD0 pulled high)
```

Enable in code:

```python
imu_proc = IMUProcessor(use_hardware_imu=True, i2c_bus=1, i2c_addr=0x68)
```
