"""
Complementary filter fusing RVR onboard locator with camera visual odometry.

The RVR locator already integrates wheel odometry + IMU onboard → good
absolute position but slow update rate over BLE (~12 Hz).
VO provides high-frequency relative deltas but drifts.

Fusion:
  fused = alpha * locator + (1 - alpha) * (prev + vo_prediction)

alpha = 0.85 → trust locator heavily; VO fills in latency gaps.
"""
import math


class ComplementaryFilter(object):
    def __init__(self, alpha=0.85, pixels_per_metre=320.0):
        self._alpha = alpha
        self._ppm = pixels_per_metre
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0   # radians

    @staticmethod
    def _wrap_angle(a):
        """Wrap angle to [-pi, pi]."""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def update(self, loc_x_cm, loc_y_cm, heading_deg, vo):
        """
        Args:
            loc_x_cm, loc_y_cm: RVR onboard locator position in cm
            heading_deg: RVR onboard heading in degrees (0-359)
            vo: dict with keys dx, dy (pixels→metres), dtheta (radians)

        Returns:
            (x_m, y_m, heading_deg) fused estimate
        """
        loc_x = loc_x_cm / 100.0
        loc_y = loc_y_cm / 100.0
        loc_heading = math.radians(heading_deg)

        # Rotate VO displacement from camera frame to world frame
        h = self._heading
        vo_world_dx = vo["dx"] * math.cos(h) - vo["dy"] * math.sin(h)
        vo_world_dy = vo["dx"] * math.sin(h) + vo["dy"] * math.cos(h)

        # Predicted position from previous state + VO
        pred_x = self._x + vo_world_dx
        pred_y = self._y + vo_world_dy
        pred_h = self._heading + vo["dtheta"]

        # Fuse with locator
        a = self._alpha
        self._x = a * loc_x + (1.0 - a) * pred_x
        self._y = a * loc_y + (1.0 - a) * pred_y

        # Heading fusion with angle wrapping
        dh = self._wrap_angle(loc_heading - pred_h)
        self._heading = self._wrap_angle(pred_h + a * dh)

        return self._x, self._y, math.degrees(self._heading)

    def reset(self):
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
