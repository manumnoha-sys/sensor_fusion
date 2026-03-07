# Sphero RVR ↔ Jetson Nano — Wiring & Communication

## Physical Connection

```
                    USB-C to USB-A cable
                    (data-capable, not charge-only)

  ┌─────────────────────────────────┐         ┌────────────────────────────────────────┐
  │         SPHERO RVR              │         │        NVIDIA JETSON NANO 4GB          │
  │                                 │         │           Developer Kit (B01)          │
  │                                 │         │                                        │
  │  ┌──────────────────────────┐   │         │   USB-A 3.0 ports (×4)                │
  │  │   Main MCU               │   │  USB-C  │   ┌─────┐ ┌─────┐ ┌─────┐ ┌─────┐   │
  │  │   (Cortex-M4)            │   │ ───────►│   │USB 0│ │USB 1│ │USB 2│ │USB 3│   │
  │  │                          │   │         │   └──┬──┘ └─────┘ └─────┘ └─────┘   │
  │  │  ┌────────────────────┐  │   │         │      │  ← plug cable here            │
  │  │  │   LSM6DS3  6-DOF   │  │   │         │      │                               │
  │  │  │   IMU (accel+gyro) │  │   │         │   ┌──▼──────────────────────────┐    │
  │  │  └────────────────────┘  │   │         │   │  USB 3.0 Hub (internal)     │    │
  │  │                          │   │         │   │  Realtek 0bda:0411          │    │
  │  │  ┌────────────────────┐  │   │         │   └──┬──────────────────────────┘    │
  │  │  │   Wheel Encoders   │  │   │         │      │ USB-serial driver              │
  │  │  │   Left  (108 CPR)  │  │   │         │      │ cp210x.ko (kernel module)      │
  │  │  │   Right (108 CPR)  │  │   │         │   ┌──▼──────────────────────────┐    │
  │  │  └────────────────────┘  │   │         │   │  /dev/ttyUSB0               │    │
  │  │                          │   │         │   │  (115200 8N1)               │    │
  │  │  ┌────────────────────┐  │   │         │   └──┬──────────────────────────┘    │
  │  │  │   Color Sensor     │  │   │         │      │                               │
  │  │  │   RGB + Clear      │  │   │         │   ┌──▼──────────────────────────┐    │
  │  │  └────────────────────┘  │   │         │   │  pyserial-asyncio           │    │
  │  │                          │   │         │   │  SerialAsyncDal             │    │
  │  │  ┌────────────────────┐  │   │         │   └──┬──────────────────────────┘    │
  │  │  │   Ambient Light    │  │   │         │      │                               │
  │  │  │   (lux)            │  │   │         │   ┌──▼──────────────────────────┐    │
  │  │  └────────────────────┘  │   │         │   │  Sphero SDK (Python)        │    │
  │  │                          │   │         │   │  SpheroRvrAsync             │    │
  │  │  ┌────────────────────┐  │   │         │   └──┬──────────────────────────┘    │
  │  │  │   Motors (×2)      │  │   │         │      │                               │
  │  │  │   Brushless DC     │  │   │         │   ┌──▼──────────────────────────┐    │
  │  │  └────────────────────┘  │   │         │   │  SpheroProcessor            │    │
  │  │            │ UART        │   │         │   │  IMUProcessor               │    │
  │  └────────────┼─────────────┘   │         │   │  PoseEKF                    │    │
  │               │                 │         │   └─────────────────────────────┘    │
  │  ┌────────────▼─────────────┐   │         │                                      │
  │  │   CP2102N                │   │         │   ┌──────────────────────────────┐   │
  │  │   USB-UART Bridge        ├───┼─────────┤   │  ARM Cortex-A57 (4-core)     │   │
  │  │   Silicon Labs           │   │         │   │  128-core Maxwell GPU        │   │
  │  │   VID: 10c4  PID: ea60   │   │         │   │  4 GB LPDDR4                 │   │
  │  └──────────────────────────┘   │         │   │  CUDA 10.2 / Ubuntu 18.04    │   │
  │                                 │         │   └──────────────────────────────┘   │
  │   USB-C port (rear of RVR)      │         │                                      │
  └─────────────────────────────────┘         └────────────────────────────────────────┘
```

---

## Signal Path Detail

```
 SPHERO RVR (internal)                              JETSON NANO (internal)
 ─────────────────────                              ──────────────────────

  MCU firmware                                       Python asyncio loop
      │                                                      ▲
      │ UART (TX/RX)                                         │
      │ 115200 baud, 8N1                              sphero_sdk packet parser
      ▼                                                      ▲
  CP2102N chip                                              │
  (USB-UART bridge)                               pyserial_asyncio
      │                                            (async read/write)
      │ USB 2.0 Full Speed (12 Mbps)                         ▲
      │ Bulk transfers                                        │
      │ VID=10c4 PID=ea60                          /dev/ttyUSB0
      │                                            (character device)
      ├─────── USB-C ──── cable ──── USB-A ──►           ▲
      │                                                   │
   USB-C port                                     cp210x kernel module
   (rear of RVR)                                  (USB-serial driver)
                                                          ▲
                                                          │
                                                  USB 3.0 host controller
                                                  (Realtek on-board hub)
```

---

## Data Streams (bidirectional)

```
 JETSON → RVR (commands)              RVR → JETSON (telemetry)
 ──────────────────────               ───────────────────────────────

  wake()                               IMU:
  reset_yaw()                            roll    (deg)   ┐
  set_raw_motors(L, R)                   pitch   (deg)   ├─ 20 Hz
  drive_with_heading(speed, heading)     yaw     (deg)   ┘

                                       Encoders:
                                         left_ticks      ┐
                                         right_ticks     ┘─ 20 Hz → v (m/s), ω (rad/s)

                                       Color:
                                         R, G, B, Clear  ─ 20 Hz

                                       Ambient Light:
                                         lux             ─ 20 Hz
```

---

## Protocol

The Sphero SDK uses a custom binary framing protocol over UART:

```
Packet format:
 ┌──────┬──────┬──────┬──────┬──────┬──────────────┬──────┐
 │ SOP  │ FLAGS│ TID  │ CID  │ SEQ  │   PAYLOAD    │ CHK  │
 │ 0x8D │ 1B   │ 1B   │ 1B   │ 1B   │  0–N bytes   │ 1B   │
 └──────┴──────┴──────┴──────┴──────┴──────────────┴──────┘
  Start  Flags  Target Command  Seq    Data          Checksum
  byte         ID      ID       #     (varies)       (XOR)

SOP = 0x8D  (start of packet marker)
CHK = XOR of all bytes from FLAGS to end of PAYLOAD
```

---

## Kernel Module

When Sphero RVR is plugged in, the kernel loads `cp210x` automatically:

```bash
# Expected dmesg output after plugging in:
[ 1234.56] usb 1-2.1: new full-speed USB device number 6 using xhci-hcd
[ 1234.78] usb 1-2.1: New USB device found, idVendor=10c4, idProduct=ea60
[ 1234.78] usb 1-2.1: Product: CP2102N USB to UART Bridge Controller
[ 1234.80] cp210x 1-2.1:1.0: cp210x converter detected
[ 1234.82] cp210x 1-2.1:1.0: cp210x converter now attached to ttyUSB0

# Verify
ls -l /dev/ttyUSB0
# crw-rw---- 1 root dialout 188, 0 ... /dev/ttyUSB0

# Check permissions (jetson4gb must be in dialout group)
groups
# jetson4gb adm dialout ... (dialout present = OK)
```

---

## Quick Test (Sphero plugged in)

```python
# test_sphero.py — run on Jetson Nano
import asyncio
from sphero_sdk import SpheroRvrAsync, SerialAsyncDal, RvrStreamingServices

async def main():
    loop = asyncio.get_event_loop()
    rvr = SpheroRvrAsync(dal=SerialAsyncDal(loop, port="/dev/ttyUSB0"))

    await rvr.wake()
    await asyncio.sleep(2)

    await rvr.get_battery_percentage(handler=lambda d: print("Battery:", d))
    await asyncio.sleep(1)

    await rvr.close()

asyncio.run(main())
```

```bash
python3 test_sphero.py
# Battery: {'percentage': 87}
```

---

## Cable Requirements

| Requirement | Detail |
|-------------|--------|
| Connector A | USB-C (male) — plugs into Sphero RVR rear port |
| Connector B | USB-A (male) — plugs into any Jetson Nano USB-A port |
| Data lines | D+ and D- required (charge-only cables will NOT work) |
| USB spec | USB 2.0 Full Speed sufficient (480 Mbps cable works too) |
| Length | Up to ~3 m practical limit for USB 2.0 |
| Supplied cable | The white USB-C cable included with Sphero RVR works |
