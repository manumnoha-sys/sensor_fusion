"""
IMU processor for NVIDIA Jetson Nano + Sphero RVR platform.

Primary IMU source: Sphero RVR built-in LSM6DS3 (6-DOF) via SpheroProcessor.
Optional secondary: hardware IMU on Jetson I2C (e.g. MPU-6050 on /dev/i2c-1).

Applies a complementary filter to produce stable roll/pitch estimates,
and exposes heading (yaw) from the Sphero gyro integration.
"""

import math
import time
import logging

logger = logging.getLogger(__name__)

# Complementary filter coefficient (0 = pure gyro, 1 = pure accel)
ALPHA = 0.02


class IMUProcessor:
    """
    Complementary filter wrapper around Sphero RVR IMU data.

    The Sphero RVR SDK already provides fused roll/pitch/yaw at 20 Hz,
    so this class acts as a rate-adapter + filter for use in the EKF.
    """

    def __init__(self, use_hardware_imu: bool = False, i2c_bus: int = 1, i2c_addr: int = 0x68):
        self._use_hw = use_hardware_imu
        self._i2c_bus = i2c_bus
        self._i2c_addr = i2c_addr
        self._hw_imu = None

        # Filter state
        self.roll = 0.0    # degrees
        self.pitch = 0.0   # degrees
        self.yaw = 0.0     # degrees (integrated from gyro)
        self._last_time = time.monotonic()

        if use_hardware_imu:
            self._init_hw_imu()

    # ------------------------------------------------------------------
    def _init_hw_imu(self):
        """Optionally init MPU-6050 on Jetson I2C."""
        try:
            import smbus2
            bus = smbus2.SMBus(self._i2c_bus)
            # Wake MPU-6050 (register 0x6B = 0x00 exits sleep)
            bus.write_byte_data(self._i2c_addr, 0x6B, 0x00)
            self._hw_imu = bus
            logger.info("Hardware IMU MPU-6050 initialised on i2c-%d addr 0x%02X",
                        self._i2c_bus, self._i2c_addr)
        except Exception as exc:
            logger.warning("Hardware IMU init failed (%s) — using Sphero IMU only", exc)
            self._hw_imu = None

    # ------------------------------------------------------------------
    def update_from_sphero(self, roll_deg: float, pitch_deg: float, yaw_deg: float):
        """
        Ingest Sphero SDK IMU output (already fused on-board).
        Applies a gentle low-pass filter for noise reduction.
        """
        alpha = 1.0 - ALPHA
        self.roll = alpha * self.roll + ALPHA * roll_deg
        self.pitch = alpha * self.pitch + ALPHA * pitch_deg
        self.yaw = yaw_deg  # Sphero integrates yaw internally; trust it directly
        self._last_time = time.monotonic()

    # ------------------------------------------------------------------
    def read_hw_imu(self):
        """
        Read raw accel + gyro from MPU-6050 and apply complementary filter.
        Call at ~100 Hz if hardware IMU is available.
        """
        if self._hw_imu is None:
            return

        try:
            import smbus2
            def _read_word(reg):
                high = self._hw_imu.read_byte_data(self._i2c_addr, reg)
                low = self._hw_imu.read_byte_data(self._i2c_addr, reg + 1)
                val = (high << 8) | low
                return val - 65536 if val >= 32768 else val

            # Accelerometer (registers 0x3B–0x40), scale ±2g → LSB/g = 16384
            ax = _read_word(0x3B) / 16384.0
            ay = _read_word(0x3D) / 16384.0
            az = _read_word(0x3F) / 16384.0

            # Gyroscope (registers 0x43–0x48), scale ±250°/s → LSB/(°/s) = 131
            gx = _read_word(0x43) / 131.0
            gy = _read_word(0x45) / 131.0
            gz = _read_word(0x47) / 131.0

            now = time.monotonic()
            dt = now - self._last_time
            self._last_time = now

            # Accel-derived roll/pitch (degrees)
            roll_accel = math.degrees(math.atan2(ay, az))
            pitch_accel = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

            # Complementary filter
            self.roll = (1.0 - ALPHA) * (self.roll + gx * dt) + ALPHA * roll_accel
            self.pitch = (1.0 - ALPHA) * (self.pitch + gy * dt) + ALPHA * pitch_accel
            self.yaw += gz * dt

        except Exception as exc:
            logger.debug("HW IMU read error: %s", exc)

    # ------------------------------------------------------------------
    @property
    def roll_rad(self) -> float:
        return math.radians(self.roll)

    @property
    def pitch_rad(self) -> float:
        return math.radians(self.pitch)

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw)

    @property
    def data(self) -> dict:
        return {
            "roll_deg": self.roll,
            "pitch_deg": self.pitch,
            "yaw_deg": self.yaw,
            "yaw_rad": self.yaw_rad,
            "timestamp": self._last_time,
        }
