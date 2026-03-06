# Kria KV260 Camera Bring-Up Guide

## Board

- **Board:** AMD/Xilinx Kria KV260 Vision AI Starter Kit (SCK-KV-G Rev2)
- **SOM:** K26 (ZynqMP, ARM Cortex-A53, 4-core @ 1333 MHz, 4 GB RAM)
- **OS:** Ubuntu 22.04.5 LTS — kernel `5.15.0-1063-xilinx-zynqmp`
- **Camera:** ON Semiconductor IAS AR1335 (3MP Auto Focus RGB, AP1302 ISP)
- **Interface:** IAS connectors J2/J3 (MIPI CSI-2, 2-lane)

---

## Prerequisites

```bash
# Verify board OS
uname -a
# Linux kria 5.15.0-1063-xilinx-zynqmp ... aarch64

# Tools needed (already installed on Ubuntu 22.04 KV260 image)
which dtc bootgen xmutil v4l2-ctl i2cdetect
```

---

## Step 1 — Download smartcam firmware from AMD GitHub

The `kv260-smartcam` FPGA bitstream is NOT in the default apt repos. Download and build it manually:

```bash
mkdir -p /tmp/smartcam && cd /tmp/smartcam

# Bitstream (7.5 MB)
curl -L -o kv260-smartcam.bit \
  "https://raw.githubusercontent.com/Xilinx/kria-apps-firmware/main/boards/kv260/smartcam/kv260-smartcam.bit"

# Device tree source
curl -o kv260-smartcam.dtsi \
  "https://raw.githubusercontent.com/Xilinx/kria-apps-firmware/main/boards/kv260/smartcam/kv260-smartcam.dtsi"

# Shell metadata
curl -o shell.json \
  "https://raw.githubusercontent.com/Xilinx/kria-apps-firmware/main/boards/kv260/smartcam/shell.json"
```

---

## Step 2 — Convert bitstream and compile device tree overlay

```bash
cd /tmp/smartcam

# Convert .bit → .bit.bin (required by FPGA manager)
echo "all: { kv260-smartcam.bit }" > kv260-smartcam.bif
bootgen -image kv260-smartcam.bif -arch zynqmp -process_bitstream bin -w
# Output: kv260-smartcam.bit.bin

# Compile .dtsi → .dtbo (device tree overlay blob)
dtc -@ -I dts -O dtb -o kv260-smartcam.dtbo kv260-smartcam.dtsi
# Warnings about reg_format are expected and non-fatal
```

---

## Step 3 — Install firmware files

```bash
sudo mkdir -p /lib/firmware/xilinx/kv260-smartcam
sudo cp kv260-smartcam.bit.bin /lib/firmware/xilinx/kv260-smartcam/
sudo cp kv260-smartcam.dtbo    /lib/firmware/xilinx/kv260-smartcam/
sudo cp shell.json             /lib/firmware/xilinx/kv260-smartcam/

# Verify
ls -lh /lib/firmware/xilinx/kv260-smartcam/
# kv260-smartcam.bit.bin  7.5M
# kv260-smartcam.dtbo     8.5K
# shell.json               57B
```

---

## Step 4 — Load the bitstream with xmutil

```bash
# Unload current base app (k26-starter-kits)
sudo xmutil unloadapp

# Verify kv260-smartcam is listed
sudo xmutil listapps
# Should show: kv260-smartcam  XRT_FLAT  ... Active_slot -1

# Load smartcam app (programs FPGA + applies DT overlay)
sudo xmutil loadapp kv260-smartcam
# Output: kv260-smartcam: loaded to slot 0

# Verify /dev/video0 appeared
ls /dev/video*
# /dev/video0
```

---

## Step 5 — Load camera sensor drivers

```bash
# AP1302 ISP companion driver
sudo modprobe ap1302

# AR1335 raw sensor driver (optional, AP1302 handles it internally)
sudo modprobe ar1335

# Verify I2C topology (should show new buses i2c-3 through i2c-7)
sudo i2cdetect -l
# i2c-3  xiic-i2c 80030000.i2c  (AXI I2C in PL)
# i2c-4  i2c-3-mux (chan_id 0)  (camera channel)
# ...

# AP1302 should be visible at 0x3C on bus 4
sudo i2cdetect -y 4
```

---

## Step 6 — Verify video device

```bash
v4l2-ctl --list-devices
# isp_vcap_csi output 0 (platform:isp_vcap_csi:0):
#     /dev/video0

v4l2-ctl -d /dev/video0 --info
# Driver name: xilinx-vipp
# Card type:   isp_vcap_csi output 0
```

---

## I2C Topology (after bitstream load)

| Bus | Device | Address | Description |
|-----|--------|---------|-------------|
| i2c-1 | Cadence I2C (PS) | – | Main PS I2C |
| i2c-1 | USB5744 | 0x2D | USB hub |
| i2c-1 | DA9130/DA9131 | 0x32/0x33 | PMICs |
| i2c-1 | INA260 | 0x40 | Power monitor |
| i2c-1 | 24C64 EEPROM | 0x50/0x51 | Board EEPROMs |
| i2c-3 | xiic-i2c 0x80030000 | – | AXI I2C in PL |
| i2c-4 | AP1302 ISP | 0x3C | Camera ISP (mux ch.0) |

---

## Kernel Drivers Summary

| Driver | Config | Status |
|--------|--------|--------|
| `xilinx-video` (pipeline) | `CONFIG_VIDEO_XILINX=y` | built-in |
| `xilinx-csi2rxss` | `CONFIG_VIDEO_XILINX_CSI2RXSS=y` | built-in |
| `ap1302` (ON Semi ISP) | `CONFIG_VIDEO_AP1302=m` | module |
| `ar1335` (raw sensor) | `CONFIG_VIDEO_AR1335=m` | module |
| `uvcvideo` (USB webcam) | `CONFIG_USB_VIDEO_CLASS=y` | built-in |

---

## Known Issues / Next Steps

### AP1302 I2C Timeout
After loading the bitstream, `ap1302` probe may fail with:
```
xiic-i2c 80030000.i2c: Timeout waiting at Tx empty
ap1302 4-003c: __ap1302_read: register 0x0000 read failed: -5
ap1302: probe of 4-003c failed with error -5
```
**Likely cause:** Camera cable not fully seated in IAS connector, or camera module not powered.

**Fix:**
1. Physically re-seat the camera FFC cable in the IAS J2 connector
2. Power-cycle the board
3. Re-run from Step 4

### Media Controller Not Created
If `/dev/media0` is absent, the `xilinx-video` entity initialization failed. Re-bind:
```bash
sudo sh -c "echo axi:isp_vcap_csi > /sys/bus/platform/drivers/xilinx-video/unbind"
sudo modprobe ap1302
sudo sh -c "echo axi:isp_vcap_csi > /sys/bus/platform/drivers/xilinx-video/bind"
```

---

## Make bitstream load persistent on boot

```bash
# /etc/rc.local approach
sudo tee /etc/rc.local << 'EOF'
#!/bin/bash
xmutil unloadapp
xmutil loadapp kv260-smartcam
modprobe ap1302
exit 0
EOF
sudo chmod +x /etc/rc.local
```
