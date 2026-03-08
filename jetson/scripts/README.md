# Jetson Nano Deployment Scripts

## IMX477 (Raspberry Pi HQ Camera) on CSI1

### Hardware
- Board: NVIDIA Jetson Nano 4GB B01 (p3448-0000-b00)
- OS: Ubuntu 18.04.6 LTS, JetPack 4.6 (L4T R32.7.x)
- Camera: RPi HQ Camera (IMX477, 12MP) → CSI1 connector (J13)

### Quick deploy on a fresh board

```bash
bash deploy_imx477.sh
sudo reboot
```

### What the script does

1. Merges the stock base DTB with NVIDIA's `imx477-dual` overlay using `fdtoverlay`
2. Writes `kernel_tegra210-p3448-camera-imx477.dtb` to `/boot/`
3. Updates `/boot/extlinux/extlinux.conf` to load the merged DTB

Both files (`base DTB` and `imx477-dual.dtbo`) ship with every JetPack 4.6 image — no extra downloads needed.

### Pre-built DTB

`dtb/kernel_tegra210-p3448-camera-imx477.dtb` is the merged DTB saved from a verified working board. You can flash it directly instead of running the script:

```bash
sudo cp dtb/kernel_tegra210-p3448-camera-imx477.dtb /boot/
# update extlinux.conf FDT line to point to it, then reboot
```

### Verify after reboot

```bash
# Camera device
ls /dev/video0

# Capture a test frame
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 ! \
  "video/x-raw(memory:NVMM),width=1920,height=1080" ! \
  nvjpegenc ! filesink location=/tmp/test.jpg

ls -lh /tmp/test.jpg   # should be ~100–110 KB
```

### Sensor parameters

| Property | Value |
|----------|-------|
| Sensor | Sony IMX477 |
| Resolution | 4056 × 3040 (12.3 MP) |
| Interface | MIPI CSI-2, 2-lane |
| I2C address | 0x1a (bus 8, cam_i2cmux/i2c@1) |
| `/dev` node | `/dev/video0` |
| argus sensor-id | 0 |
| Max framerate | 60 fps (1080p) |
