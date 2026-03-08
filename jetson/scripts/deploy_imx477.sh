#!/usr/bin/env bash
# deploy_imx477.sh — Enable IMX477 (RPi HQ Camera) on CSI1 of Jetson Nano B01
#
# Tested on:
#   Board : NVIDIA Jetson Nano 4GB Developer Kit (p3448-0000-b00)
#   OS    : Ubuntu 18.04.6 LTS (JetPack 4.6 / L4T R32.7.x)
#   Camera: Raspberry Pi HQ Camera (IMX477) connected to CSI1 (J13 connector)
#
# After running this script, reboot once.  Camera will appear at:
#   /dev/video0     (V4L2)
#   sensor-id=0     (nvarguscamerasrc / libargus)
#
# Usage:
#   bash deploy_imx477.sh          # run on the Jetson itself
#
# Capture test after reboot:
#   gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 ! \
#     "video/x-raw(memory:NVMM),width=1920,height=1080" ! \
#     nvjpegenc ! filesink location=/tmp/test.jpg

set -euo pipefail

BASE_DTB="/boot/tegra210-p3448-0000-p3449-0000-b00.dtb"
IMX477_OVERLAY="/boot/tegra210-p3448-all-p3449-0000-camera-imx477-dual.dtbo"
OUT_DTB="/boot/kernel_tegra210-p3448-camera-imx477.dtb"

echo "======================================================"
echo " IMX477 CSI1 — Deploy on Jetson Nano B01"
echo "======================================================"

# ---- Preflight checks ----
echo ""
echo "[check] JetPack version"
cat /etc/nv_tegra_release | grep -o "R[0-9]*.*REVISION: [0-9.]*"

echo ""
echo "[check] Required files"
for f in "$BASE_DTB" "$IMX477_OVERLAY"; do
    if [ ! -f "$f" ]; then
        echo "ERROR: missing $f"
        exit 1
    fi
    echo "  OK  $f"
done

# ---- Merge DTBs ----
echo ""
echo "[1/2] Merging DTBs: base + imx477-dual..."
sudo fdtoverlay \
    -i "$BASE_DTB" \
    -o "$OUT_DTB" \
    "$IMX477_OVERLAY"
echo "      Written: $OUT_DTB ($(du -h "$OUT_DTB" | cut -f1))"

# Verify the IMX477 node is enabled in the merged DTB
IMX_STATUS=$(sudo fdtget "$OUT_DTB" "/cam_i2cmux/i2c@1/rbpcv3_imx477_e@1a" status 2>/dev/null || echo "missing")
echo "      IMX477@CSI1 status: ${IMX_STATUS}"
if [ "$IMX_STATUS" != "okay" ]; then
    echo "ERROR: IMX477 node not enabled in merged DTB. Check overlay."
    exit 1
fi

# ---- Update extlinux.conf ----
echo ""
echo "[2/2] Updating /boot/extlinux/extlinux.conf..."
sudo cp /boot/extlinux/extlinux.conf /boot/extlinux/extlinux.conf.bak

sudo tee /boot/extlinux/extlinux.conf > /dev/null << 'EOF'
TIMEOUT 30
DEFAULT primary

MENU TITLE L4T boot options

LABEL primary
      MENU LABEL primary kernel
      LINUX /boot/Image
      INITRD /boot/initrd
      FDT /boot/kernel_tegra210-p3448-camera-imx477.dtb
      APPEND ${cbootargs} quiet root=/dev/mmcblk0p1 rw rootwait rootfstype=ext4 console=ttyS0,115200n8 console=tty0 fbcon=map:0 net.ifnames=0
EOF

echo ""
echo "======================================================"
echo " Done! Reboot to activate the IMX477."
echo ""
echo "  sudo reboot"
echo ""
echo "  After reboot:"
echo "    /dev/video0        — IMX477 (V4L2)"
echo "    sensor-id=0        — nvarguscamerasrc / libargus"
echo ""
echo "  Capture test:"
echo "    gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 ! \\"
echo "      \"video/x-raw(memory:NVMM),width=1920,height=1080\" ! \\"
echo "      nvjpegenc ! filesink location=/tmp/test.jpg"
echo "======================================================"
