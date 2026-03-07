# Sensor Fusion Architecture — Multi-Platform Overview

## Platforms

| Platform | Board | Rover | Docs |
|----------|-------|-------|------|
| Kria KV260 | AMD/Xilinx KV260 (ARM Cortex-A53) | — (camera + IMU only) | [kria/README.md](../kria/README.md) |
| Jetson Nano | NVIDIA Jetson Nano 4GB (Maxwell GPU) | Sphero RVR (UART) | [jetson/README.md](../jetson/README.md) |

---

# Kria KV260 — Camera + IMU Pipeline

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Kria KV260 Starter Kit                              │
│                                                                             │
│  IAS Connector (J2/J3)                                                     │
│  ┌───────────────────┐   MIPI CSI-2 (2-lane)                               │
│  │  ON Semi IAS Cam  │──────────────────────────────────────────────┐      │
│  │  ┌─────────────┐  │                                               │      │
│  │  │   AR1335    │  │   I2C                                         │      │
│  │  │  3MP CMOS   │──┤◄──────────────────────────────────────────┐  │      │
│  │  │  1/3.2"     │  │                                            │  │      │
│  │  └──────┬──────┘  │                                            │  │      │
│  │         │ RAW     │                                            │  │      │
│  │  ┌──────▼──────┐  │                                            │  │      │
│  │  │   AP1302    │  │                                            │  │      │
│  │  │  ISP chip   │──┤ MIPI CSI-2 OUT                            │  │      │
│  │  │  (0x3C)     │  │                                            │  │      │
│  │  └─────────────┘  │                                            │  │      │
│  └───────────────────┘                                            │  │      │
│                                                                    │  │      │
│  ┌─────────────────── FPGA Fabric (PL) ───────────────────────┐   │  │      │
│  │                                                             │   │  │      │
│  │  ┌─────────────┐     ┌──────────────┐     ┌────────────┐   │   │  │      │
│  │  │  AXI I2C    │◄────┤  I2C Mux     │◄────│  xiic-i2c  │   │   │  │      │
│  │  │ 0x80030000  │     │   @ 0x74     │     │  (bus i2c-3│   │   │  │      │
│  │  │ (i2c-3)     │     │  (i2c-4..7) │     │  → i2c-4)  │   │◄──┘  │      │
│  │  └─────────────┘     └──────────────┘     └────────────┘   │      │      │
│  │                                                             │      │      │
│  │  ┌─────────────────────────────────────────────────────┐   │      │      │
│  │  │        MIPI CSI-2 RX Subsystem (csiss)              │◄──┼──────┘      │
│  │  │   xlnx,mipi-csi2-rx-subsystem-5.0 @ 0x80000000     │   │             │
│  │  └─────────────────────┬───────────────────────────────┘   │             │
│  │                        │ AXI4-Stream                        │             │
│  │  ┌─────────────────────▼───────────────────────────────┐   │             │
│  │  │         Frame Buffer Writer (fb_wr)                  │   │             │
│  │  │              isp_fb_wr_csi @ 0xB0010000              │   │             │
│  │  └─────────────────────┬───────────────────────────────┘   │             │
│  │                        │ DMA                                │             │
│  │  ┌─────────────────────▼───────────────────────────────┐   │             │
│  │  │     Xilinx Video Pipeline (xilinx-video)             │   │             │
│  │  │           isp_vcap_csi  → /dev/video0                │   │             │
│  │  └─────────────────────────────────────────────────────┘   │             │
│  │                                                             │             │
│  │  ┌──────────────────────────────────────────────────────┐  │             │
│  │  │     VCU (Video Codec Unit) @ 0x80100000              │  │             │
│  │  │     H.264/H.265 HW encode/decode                     │  │             │
│  │  └──────────────────────────────────────────────────────┘  │             │
│  └─────────────────────────────────────────────────────────────┘             │
│                                                                             │
│  ┌──────────────────────── ARM Cortex-A53 (PS) ───────────────────────────┐ │
│  │                                                                         │ │
│  │   Ubuntu 22.04 LTS (kernel 5.15.0-1063-xilinx-zynqmp)                  │ │
│  │                                                                         │ │
│  │   Kernel drivers (built-in):                                            │ │
│  │     • xilinx-csi2rxss     • xilinx-video     • ap1302 (module)         │ │
│  │     • ar1335 (module)     • USB UVC (built-in)                          │ │
│  │                                                                         │ │
│  │   Userspace:                                                            │ │
│  │     • /dev/video0  (isp_vcap_csi)                                       │ │
│  │     • v4l2-ctl / media-ctl / OpenCV                                     │ │
│  │     • sensor_fusion Python app                                          │ │
│  └─────────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Component Details

| Component | Details |
|-----------|---------|
| Camera sensor | ON Semi AR1335 — 13MP, 1/3.2" CMOS, Auto Focus |
| ISP companion | ON Semi AP1302 — full ISP, I2C addr `0x3C` on mux ch.0 |
| Camera interface | IAS connector (J2/J3), MIPI CSI-2 2-lane |
| FPGA bitstream | `kv260-smartcam` (2022.1) — loaded via `xmutil` |
| MIPI RX | `xlnx,mipi-csi2-rx-subsystem-5.0` @ `0x80000000` |
| AXI I2C | `xiic-i2c` @ `0x80030000` (i2c-3 on PS) |
| I2C mux | PCA954x @ `0x74`, 4 channels (i2c-4 … i2c-7) |
| Video capture | `isp_vcap_csi` → `/dev/video0` |
| CPU | ARM Cortex-A53 quad-core, 1333 MHz, aarch64 |
| RAM | 3.8 GiB |
| OS | Ubuntu 22.04.5 LTS (Jammy Jellyfish) |

## Software Stack

```
┌─────────────────────────────────────────┐
│         sensor_fusion Python app        │
│  (fusion/imu_processor.py + camera)     │
├─────────────────────────────────────────┤
│       OpenCV  /  v4l2  /  Python        │
├─────────────────────────────────────────┤
│           /dev/video0  (V4L2)           │
├─────────────────────────────────────────┤
│    xilinx-video  ←→  ap1302  driver     │
├─────────────────────────────────────────┤
│    xilinx-csi2rxss  (built-in kernel)   │
├─────────────────────────────────────────┤
│  kv260-smartcam FPGA bitstream (PL)     │
└─────────────────────────────────────────┘
```
