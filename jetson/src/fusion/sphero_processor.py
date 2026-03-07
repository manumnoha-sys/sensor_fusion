"""
Sphero RVR telemetry processor for NVIDIA Jetson Nano.

Connects to Sphero RVR via UART (USB-C cable → /dev/ttyUSB0 or /dev/ttyACM0)
and streams sensor data:
  - IMU: roll, pitch, yaw (deg), pitch/roll rates
  - Encoders: left/right ticks → linear velocity, angular velocity
  - Color: RGBC from bottom sensor
  - Ambient light: lux

Usage (async):
    proc = SpheroProcessor(port="/dev/ttyUSB0")
    await proc.start()
    data = proc.latest          # dict with all sensor readings
    await proc.stop()

Requires:
    pip install sphero-sdk-raspberrypi-python
"""

import asyncio
import logging
import math
import time
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger(__name__)

# Wheel geometry (Sphero RVR)
WHEEL_CIRCUMFERENCE_M = 0.2199  # ~220 mm
WHEEL_TRACK_M = 0.1600          # ~160 mm track width
TICKS_PER_REV = 108.0 * 2       # 108 CPR encoder, quadrature → 216 edges


@dataclass
class SpheroData:
    # IMU (degrees)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    # Velocities (m/s, rad/s)
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    # Encoder cumulative ticks
    left_ticks: int = 0
    right_ticks: int = 0
    # Color sensor (0–255 each + clear channel)
    color_r: int = 0
    color_g: int = 0
    color_b: int = 0
    color_c: int = 0
    # Ambient light
    ambient_lux: float = 0.0
    # Timestamp
    timestamp: float = field(default_factory=time.monotonic)


class SpheroProcessor:
    """Asynchronous Sphero RVR telemetry processor."""

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200):
        self._port = port
        self._baud = baud
        self._rvr = None
        self._loop = None
        self._running = False
        self.latest = SpheroData()
        self._prev_left_ticks: Optional[int] = None
        self._prev_right_ticks: Optional[int] = None
        self._prev_tick_time: Optional[float] = None

    # ------------------------------------------------------------------
    async def start(self):
        """Initialize RVR and register streaming handlers."""
        try:
            from sphero_sdk import SpheroRvrAsync, SerialAsyncDal, RvrStreamingServices
        except ImportError:
            raise RuntimeError(
                "sphero-sdk-raspberrypi-python not installed. "
                "Run: pip install sphero-sdk-raspberrypi-python"
            )

        self._loop = asyncio.get_event_loop()
        self._rvr = SpheroRvrAsync(
            dal=SerialAsyncDal(self._loop, port=self._port, baud_rate=self._baud)
        )

        await self._rvr.wake()
        await asyncio.sleep(2.0)
        await self._rvr.reset_yaw()

        # Register streaming callbacks
        await self._rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.imu,
            handler=self._on_imu,
        )
        await self._rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.encoders,
            handler=self._on_encoders,
        )
        await self._rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.color_detection,
            handler=self._on_color,
        )
        await self._rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.ambient_light,
            handler=self._on_ambient,
        )

        await self._rvr.sensor_control.start(interval=50)  # 20 Hz
        self._running = True
        logger.info("SpheroProcessor: streaming started on %s", self._port)

    # ------------------------------------------------------------------
    async def stop(self):
        if not self._running or self._rvr is None:
            return
        await self._rvr.sensor_control.stop()
        await self._rvr.close()
        self._running = False
        logger.info("SpheroProcessor: stopped")

    # ------------------------------------------------------------------
    def _on_imu(self, imu_data):
        d = imu_data.get("IMU", {})
        self.latest.roll = d.get("Roll", 0.0)
        self.latest.pitch = d.get("Pitch", 0.0)
        self.latest.yaw = d.get("Yaw", 0.0)
        self.latest.timestamp = time.monotonic()

    def _on_encoders(self, enc_data):
        d = enc_data.get("Encoders", {})
        left = d.get("LeftTicks", 0)
        right = d.get("RightTicks", 0)
        now = time.monotonic()

        if self._prev_left_ticks is not None and self._prev_tick_time is not None:
            dt = now - self._prev_tick_time
            if dt > 0:
                dl = left - self._prev_left_ticks
                dr = right - self._prev_right_ticks

                dist_l = (dl / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_M
                dist_r = (dr / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_M

                self.latest.linear_velocity = (dist_r + dist_l) / (2.0 * dt)
                self.latest.angular_velocity = (dist_r - dist_l) / (WHEEL_TRACK_M * dt)

        self._prev_left_ticks = left
        self._prev_right_ticks = right
        self._prev_tick_time = now
        self.latest.left_ticks = left
        self.latest.right_ticks = right

    def _on_color(self, color_data):
        d = color_data.get("ColorDetection", {})
        self.latest.color_r = d.get("R", 0)
        self.latest.color_g = d.get("G", 0)
        self.latest.color_b = d.get("B", 0)
        self.latest.color_c = d.get("Index", 0)

    def _on_ambient(self, amb_data):
        d = amb_data.get("AmbientLight", {})
        self.latest.ambient_lux = d.get("Light", 0.0)

    # ------------------------------------------------------------------
    @property
    def yaw_rad(self) -> float:
        return math.radians(self.latest.yaw)

    @property
    def is_connected(self) -> bool:
        return self._running
