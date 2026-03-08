#!/usr/bin/env bash
# deploy.sh — build OV5647 kernel module and patch DTB on Jetson Nano
# Run this ON THE JETSON (100.112.94.123) as jetson4gb

set -euo pipefail

KVER="4.9.337-tegra"
KDIR="/usr/src/linux-headers-${KVER}-ubuntu18.04_aarch64"
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BASE_DTB="/boot/tegra210-p3448-0000-p3449-0000-b00.dtb"
IMX477_OVERLAY="/boot/tegra210-p3448-all-p3449-0000-camera-imx477-dual.dtbo"
OV5647_OVERLAY="${SRC_DIR}/ov5647-csi0.dtbo"
OUT_DTB="/boot/kernel_tegra210-p3448-camera-ov5647-imx477.dtb"

echo "======================================================"
echo " OV5647 CSI0 + IMX477 CSI1 — Build & Deploy"
echo "======================================================"

# ---- Step 1: Build kernel module ----
echo ""
echo "[1/5] Building ov5647.ko..."
cd "$SRC_DIR"
make clean
make -j$(nproc)
echo "      ov5647.ko built OK"

# ---- Step 2: Install kernel module ----
echo ""
echo "[2/5] Installing ov5647.ko..."
sudo cp ov5647.ko "/lib/modules/${KVER}/kernel/drivers/media/i2c/"
sudo depmod -a "${KVER}"
echo "      module installed"

# ---- Step 3: Compile DT overlay ----
echo ""
echo "[3/5] Compiling ov5647-csi0.dts → .dtbo..."
dtc -@ -I dts -O dtb -o "$OV5647_OVERLAY" "${SRC_DIR}/ov5647-csi0.dts" 2>&1 | grep -v "Warning"
echo "      ${OV5647_OVERLAY}"

# ---- Step 4: Merge DTBs ----
# base + imx477-dual (enables IMX477 on CSI1) + ov5647-csi0 (OV5647 on CSI0)
echo ""
echo "[4/5] Merging DTBs: base + imx477-dual + ov5647-csi0..."
sudo fdtoverlay \
    -i "${BASE_DTB}" \
    -o "${OUT_DTB}" \
    "${IMX477_OVERLAY}" \
    "${OV5647_OVERLAY}"
echo "      ${OUT_DTB}"

# Verify OV5647 node is present and enabled
STATUS=$(sudo fdtget "${OUT_DTB}" "/cam_i2cmux/i2c@0/ov5647@36" status 2>/dev/null || echo "missing")
IMX477_STATUS=$(sudo fdtget "${OUT_DTB}" "/cam_i2cmux/i2c@1/rbpcv3_imx477_e@1a" status 2>/dev/null || echo "missing")
echo "      OV5647@CSI0  status: ${STATUS}"
echo "      IMX477@CSI1  status: ${IMX477_STATUS}"

# ---- Step 5: Update extlinux.conf ----
echo ""
echo "[5/5] Updating /boot/extlinux/extlinux.conf..."
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak2

sudo tee /boot/extlinux/extlinux.conf > /dev/null << 'EOF'
TIMEOUT 30
DEFAULT primary

MENU TITLE L4T boot options

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      INITRD /boot/initrd
      FDT /boot/kernel_tegra210-p3448-camera-ov5647-imx477.dtb
      APPEND ${cbootargs} quiet root=/dev/mmcblk0p1 rw rootwait rootfstype=ext4 console=ttyS0,115200n8 console=tty0 fbcon=map:0 net.ifnames=0
EOF

echo ""
echo "======================================================"
echo " Done! Reboot to activate both cameras."
echo ""
echo "  CSI0: OV5647  (5MP RPi Cam V1)  → /dev/video1"
echo "  CSI1: IMX477  (12MP RPi HQ Cam) → /dev/video0"
echo ""
echo "  sudo reboot"
echo "======================================================"
