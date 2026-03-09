"""
Sphero RVR BLE client for Jetson Nano.
Handles connection, sensor streaming, and drive commands.
"""
import threading
import time
import logging
import struct

from bluepy.btle import Peripheral, ADDR_TYPE_RANDOM, DefaultDelegate, BTLEDisconnectError

from .packet import (
    build_packet, parse_packet, unpack_floats,
    FLAG_REQ_RESPONSE, FLAG_ACTIVITY, FLAG_HAS_TARGET,
    PRIMARY, SECONDARY,
    DEV_POWER, DEV_DRIVE, DEV_SENSOR, DEV_LED,
    CMD_WAKE, CMD_BATT, CMD_RESET_YAW, CMD_RAW_MOTORS,
    CMD_DRIVE_HEADING, CMD_SENSOR_CONFIG, CMD_SENSOR_START,
    CMD_SENSOR_STOP, CMD_SENSOR_STREAM, CMD_SET_ALL_LEDS,
    SENSOR_LOCATOR, SENSOR_IMU, SENSOR_ACCEL, SENSOR_GYRO,
    SENSOR_FIELDS,
)

logger = logging.getLogger(__name__)

CMD_HANDLE = 0x000e
CMD_CCCD   = 0x000f

STREAM_TOKEN = 0x0001
STREAM_SENSORS = [SENSOR_LOCATOR, SENSOR_IMU, SENSOR_ACCEL, SENSOR_GYRO]
STREAM_INTERVAL_MS = 80


class RVRStreamDelegate(DefaultDelegate):
    def __init__(self, state):
        super(RVRStreamDelegate, self).__init__()
        self._state = state

    def handleNotification(self, cHandle, data):
        parsed = parse_packet(data)
        if parsed is None:
            return
        flags, target, device, cmd, seq, payload = parsed

        if device == DEV_SENSOR and cmd == CMD_SENSOR_STREAM:
            self._parse_stream(payload)

    def _parse_stream(self, payload):
        if len(payload) < 2:
            return
        token = struct.unpack('>H', payload[:2])[0]
        if token != STREAM_TOKEN:
            return
        data = payload[2:]
        offset = 0
        values = {}
        for sensor_id in STREAM_SENSORS:
            n = SENSOR_FIELDS.get(sensor_id, 0)
            needed = n * 4
            if len(data) < offset + needed:
                break
            floats = unpack_floats(data[offset:], n)
            if floats:
                values[sensor_id] = floats
            offset += needed

        with self._state.lock:
            if SENSOR_LOCATOR in values:
                self._state.loc_x, self._state.loc_y = values[SENSOR_LOCATOR]
            if SENSOR_IMU in values:
                p, r, y = values[SENSOR_IMU]
                self._state.pitch   = p
                self._state.roll    = r
                self._state.heading = y
            if SENSOR_ACCEL in values:
                self._state.accel_x, self._state.accel_y, self._state.accel_z = values[SENSOR_ACCEL]
            if SENSOR_GYRO in values:
                self._state.gyro_roll, self._state.gyro_pitch, self._state.gyro_yaw = values[SENSOR_GYRO]
            self._state.timestamp = time.monotonic()


class RVRBLEClient(object):
    def __init__(self, state):
        self._state = state
        self._p = None
        self._seq = 0
        self._running = False

    # ------------------------------------------------------------------ #
    def _next_seq(self):
        self._seq = (self._seq + 1) & 0xff
        return self._seq

    def _send(self, target, device, cmd, data=b'', want_resp=False):
        flags = (FLAG_HAS_TARGET | FLAG_ACTIVITY)
        if want_resp:
            flags |= FLAG_REQ_RESPONSE
        pkt = build_packet(flags, target, device, cmd, self._next_seq(), data)
        self._p.writeCharacteristic(CMD_HANDLE, pkt, withResponse=False)
        for _ in range(4):
            self._p.waitForNotifications(0.1)

    # ------------------------------------------------------------------ #
    # Power
    def wake(self):
        self._send(PRIMARY, DEV_POWER, CMD_WAKE, want_resp=True)
        time.sleep(1.0)
        logger.info("RVR awake")

    # Drive
    def reset_yaw(self):
        self._send(SECONDARY, DEV_DRIVE, CMD_RESET_YAW)

    def drive(self, speed, heading):
        data = bytes([speed, (heading >> 8) & 0xff, heading & 0xff, 0])
        self._send(SECONDARY, DEV_DRIVE, CMD_DRIVE_HEADING, data)

    def raw_motors(self, left_speed, right_speed, reverse=False):
        mode = 2 if reverse else 1
        self._send(SECONDARY, DEV_DRIVE, CMD_RAW_MOTORS,
                   bytes([mode, left_speed, mode, right_speed]))

    def stop(self):
        self._send(SECONDARY, DEV_DRIVE, CMD_RAW_MOTORS, bytes([0, 0, 0, 0]))

    # LEDs
    def set_leds(self, r, g, b):
        mapping = bytes([0x3f, 0xff, 0xff, 0xff])
        self._send(SECONDARY, DEV_LED, CMD_SET_ALL_LEDS,
                   mapping + bytes([r, g, b] * 5))

    # Sensor streaming
    def configure_streaming(self):
        token_bytes = struct.pack('>H', STREAM_TOKEN)
        sensor_bytes = b''.join(struct.pack('>H', s) for s in STREAM_SENSORS)
        self._send(SECONDARY, DEV_SENSOR, CMD_SENSOR_CONFIG,
                   token_bytes + sensor_bytes)
        time.sleep(0.2)

    def start_streaming(self, interval_ms=STREAM_INTERVAL_MS):
        payload = struct.pack('>HH', interval_ms, 0)
        self._send(SECONDARY, DEV_SENSOR, CMD_SENSOR_START, payload)
        logger.info("Sensor streaming started at %dms interval", interval_ms)

    def stop_streaming(self):
        self._send(SECONDARY, DEV_SENSOR, CMD_SENSOR_STOP)

    # ------------------------------------------------------------------ #
    def connect_and_run(self, address):
        """Main loop: connect, configure, stream. Reconnects on drop."""
        self._running = True
        while self._running:
            try:
                logger.info("Connecting to RVR %s ...", address)
                self._p = Peripheral(address, ADDR_TYPE_RANDOM)
                self._p.setDelegate(RVRStreamDelegate(self._state))
                self._p.writeCharacteristic(CMD_CCCD, b'\x01\x00')
                logger.info("RVR connected")

                with self._state.lock:
                    self._state.ble_connected = True

                self.wake()
                self.reset_yaw()
                self.set_leds(0, 255, 0)
                self.configure_streaming()
                self.start_streaming()

                while self._running:
                    self._p.waitForNotifications(0.1)

            except BTLEDisconnectError:
                logger.warning("RVR BLE disconnected, retrying in 5s")
            except Exception as e:
                logger.error("RVR BLE error: %s", e)
            finally:
                with self._state.lock:
                    self._state.ble_connected = False
                if self._p:
                    try:
                        self._p.disconnect()
                    except Exception:
                        pass
                    self._p = None
                if self._running:
                    time.sleep(5)

    def stop(self):
        self._running = False
