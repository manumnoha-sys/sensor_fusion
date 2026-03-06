"""IMU data processor for Kria KV260.

Reads accelerometer + gyroscope via IIO/I2C and applies
a complementary filter to compute orientation.
"""

import math
import time


class IMUProcessor:
    def __init__(self, alpha: float = 0.98):
        """
        Args:
            alpha: Complementary filter weight for gyroscope (0–1).
                   Higher = trust gyro more, lower = trust accel more.
        """
        self.alpha = alpha
        self.roll = 0.0
        self.pitch = 0.0
        self._last_time = time.monotonic()

    def update(self, ax: float, ay: float, az: float,
               gx: float, gy: float, gz: float) -> dict:
        """Update orientation estimate from raw IMU readings.

        Args:
            ax, ay, az: Accelerometer (m/s²)
            gx, gy, gz: Gyroscope (rad/s)

        Returns:
            dict with roll and pitch in degrees.
        """
        now = time.monotonic()
        dt = now - self._last_time
        self._last_time = now

        # Accel-based angle estimate
        accel_roll = math.atan2(ay, az)
        accel_pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        # Complementary filter
        self.roll = self.alpha * (self.roll + gx * dt) + (1 - self.alpha) * accel_roll
        self.pitch = self.alpha * (self.pitch + gy * dt) + (1 - self.alpha) * accel_pitch

        return {
            "roll_deg": math.degrees(self.roll),
            "pitch_deg": math.degrees(self.pitch),
            "timestamp": now,
        }
