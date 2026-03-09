#!/usr/bin/env python3
"""
Sphero RVR BLE control for Jetson Nano.

Hardware:
  - Bluetooth dongle: Broadcom BCM20702A0 (hci0)
  - RVR MAC: F0:0F:1B:C7:0F:03 (LE Random, name RV-0F03)

Usage:
  sudo python3 rvr_control.py

BLE protocol (API v2):
  Packet: 8d FLAGS [TARGET] DEV CMD SEQ [DATA] CHK d8
  CMD handle: 0x000e  (write + notify)
  CCCD:       0x000f  (write 0100 to enable notify)
"""

from bluepy.btle import Peripheral, ADDR_TYPE_RANDOM, DefaultDelegate
import time

RVR_ADDR   = "F0:0F:1B:C7:0F:03"
CMD_HANDLE = 0x000e
CMD_CCCD   = 0x000f

# Targets
PRIMARY   = 0x1   # Nordic — power, system
SECONDARY = 0x2   # STM    — drive, sensors, LEDs


class RVRDelegate(DefaultDelegate):
    def __init__(self):
        super().__init__()
        self.last_response = None

    def handleNotification(self, cHandle, data):
        self.last_response = data


class RVR:
    def __init__(self, address=RVR_ADDR):
        self._seq = 0
        self._delegate = RVRDelegate()
        self._p = Peripheral(address, ADDR_TYPE_RANDOM)
        self._p.setDelegate(self._delegate)
        self._p.writeCharacteristic(CMD_CCCD, b'\x01\x00')

    # ------------------------------------------------------------------ #
    def _send(self, target, device, cmd, data=b'', want_resp=False):
        self._seq = (self._seq + 1) & 0xff
        flags = 0x1a if want_resp else 0x18
        payload = bytes([flags, target, device, cmd, self._seq]) + data
        chk = (~sum(payload)) & 0xff
        pkt = bytes([0x8d]) + payload + bytes([chk, 0xd8])
        self._p.writeCharacteristic(CMD_HANDLE, pkt, withResponse=False)
        for _ in range(4):
            self._p.waitForNotifications(0.1)

    def _wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self._p.waitForNotifications(0.05)

    # ------------------------------------------------------------------ #
    # Power
    def wake(self):
        self._send(PRIMARY, 0x13, 0x0d, want_resp=True)
        self._wait(1)

    def sleep(self):
        self._send(PRIMARY, 0x13, 0x01)

    def get_battery(self):
        self._send(PRIMARY, 0x13, 0x10, want_resp=True)
        resp = self._delegate.last_response
        if resp and len(resp) >= 7:
            return resp[-3]  # percentage byte
        return None

    # ------------------------------------------------------------------ #
    # Drive (secondary processor)
    def reset_yaw(self):
        self._send(SECONDARY, 0x16, 0x06)

    def drive(self, speed, heading):
        """speed 0-255, heading 0-359 degrees"""
        data = bytes([speed, (heading >> 8) & 0xff, heading & 0xff, 0])
        self._send(SECONDARY, 0x16, 0x07, data)

    def raw_motors(self, left_speed, right_speed, reverse=False):
        """Direct motor control, speed 0-255"""
        mode = 2 if reverse else 1
        self._send(SECONDARY, 0x16, 0x01,
                   bytes([mode, left_speed, mode, right_speed]))

    def stop(self):
        self._send(SECONDARY, 0x16, 0x01, bytes([0, 0, 0, 0]))

    # ------------------------------------------------------------------ #
    # LEDs (secondary processor, dev=0x1a, cmd=0x1a)
    def set_leds(self, r, g, b):
        """Set all LEDs to RGB color"""
        mapping = bytes([0x3f, 0xff, 0xff, 0xff])
        self._send(SECONDARY, 0x1a, 0x1a, mapping + bytes([r, g, b] * 5))

    # ------------------------------------------------------------------ #
    def disconnect(self):
        self._p.disconnect()


# ------------------------------------------------------------------ #
if __name__ == '__main__':
    print("Connecting to RVR...")
    rvr = RVR()
    print("Connected!")

    rvr.wake()
    batt = rvr.get_battery()
    print(f"Battery: {batt}%")

    rvr.reset_yaw()
    time.sleep(0.3)

    SPEED = 80
    legs = [
        (0,   255, 0,   0,   "North"),
        (90,  0,   255, 0,   "East"),
        (180, 0,   0,   255, "South"),
        (270, 255, 255, 0,   "West"),
    ]

    print("Driving square...")
    for heading, r, g, b, name in legs:
        print(f"  {name}")
        rvr.set_leds(r, g, b)
        rvr.drive(SPEED, heading)
        time.sleep(1.5)

    rvr.stop()
    rvr.set_leds(0, 0, 0)
    rvr.disconnect()
    print("Done.")
