"""
Extended Kalman Filter for 2D rover pose estimation.

State vector:  x = [x, y, theta, v, omega]
  x, y    : position in metres
  theta   : heading in radians
  v       : linear velocity (m/s)
  omega   : angular velocity (rad/s)

Measurements supported:
  - IMU heading (yaw from Sphero RVR gyro)
  - Wheel encoder odometry (v, omega)
  - Visual odometry delta pose (dx, dy, dtheta)
"""

import numpy as np
import math
import time


class PoseEKF:
    def __init__(self):
        # State: [x, y, theta, v, omega]
        self.state = np.zeros(5)

        # State covariance
        self.P = np.diag([1.0, 1.0, 0.1, 0.5, 0.1])

        # Process noise (motion model uncertainty)
        self.Q = np.diag([0.01, 0.01, 0.005, 0.1, 0.05])

        # IMU heading measurement noise (sigma^2 in rad^2)
        self.R_imu = np.array([[0.01]])

        # Encoder measurement noise [v, omega]
        self.R_enc = np.diag([0.05, 0.02])

        # Visual odometry measurement noise [dx, dy, dtheta]
        self.R_vo = np.diag([0.1, 0.1, 0.05])

        self._last_time = time.monotonic()

    # ------------------------------------------------------------------
    def predict(self, dt=None):
        """Propagate state forward using constant-velocity motion model."""
        now = time.monotonic()
        if dt is None:
            dt = now - self._last_time
        self._last_time = now

        x, y, th, v, w = self.state

        # Non-linear motion model
        if abs(w) < 1e-6:
            # Straight-line approximation
            xn = x + v * math.cos(th) * dt
            yn = y + v * math.sin(th) * dt
        else:
            xn = x + (v / w) * (math.sin(th + w * dt) - math.sin(th))
            yn = y + (v / w) * (math.cos(th) - math.cos(th + w * dt))

        thn = th + w * dt
        vn, wn = v, w  # constant velocity model

        self.state = np.array([xn, yn, thn, vn, wn])

        # Jacobian of motion model wrt state
        F = np.eye(5)
        if abs(w) < 1e-6:
            F[0, 2] = -v * math.sin(th) * dt
            F[0, 3] =  math.cos(th) * dt
            F[1, 2] =  v * math.cos(th) * dt
            F[1, 3] =  math.sin(th) * dt
        else:
            F[0, 2] = (v / w) * (math.cos(th + w * dt) - math.cos(th))
            F[0, 3] = (1.0 / w) * (math.sin(th + w * dt) - math.sin(th))
            F[0, 4] = (v / w**2) * (math.sin(th) - math.sin(th + w * dt)) + (v / w) * math.cos(th + w * dt) * dt
            F[1, 2] = (v / w) * (math.sin(th + w * dt) - math.sin(th))
            F[1, 3] = (1.0 / w) * (math.cos(th) - math.cos(th + w * dt))
            F[1, 4] = (v / w**2) * (math.cos(th + w * dt) - math.cos(th)) + (v / w) * math.sin(th + w * dt) * dt
        F[2, 4] = dt

        self.P = F @ self.P @ F.T + self.Q

    # ------------------------------------------------------------------
    def update_imu(self, yaw_rad):
        """Update with IMU heading measurement (yaw in radians)."""
        H = np.array([[0, 0, 1, 0, 0]])
        z = np.array([yaw_rad])
        self._update(z, H, self.R_imu, angle_state_idx=2)

    # ------------------------------------------------------------------
    def update_encoders(self, v_mps, omega_rps):
        """Update with wheel encoder velocity measurement."""
        H = np.array([
            [0, 0, 0, 1, 0],
            [0, 0, 0, 0, 1],
        ])
        z = np.array([v_mps, omega_rps])
        self._update(z, H, self.R_enc)

    # ------------------------------------------------------------------
    def update_visual_odometry(self, dx, dy, dtheta):
        """Update with visual odometry delta pose (in body frame)."""
        th = self.state[2]
        # Transform VO delta to world frame
        dx_w = dx * math.cos(th) - dy * math.sin(th)
        dy_w = dx * math.sin(th) + dy * math.cos(th)

        H = np.zeros((3, 5))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0

        z = np.array([self.state[0] + dx_w, self.state[1] + dy_w, self.state[2] + dtheta])
        self._update(z, H, self.R_vo, angle_state_idx=2)

    # ------------------------------------------------------------------
    def _update(self, z, H, R, angle_state_idx=None):
        """Standard EKF update step."""
        y = z - H @ self.state

        # Wrap angle innovation
        if angle_state_idx is not None:
            ai = np.where(H[:, angle_state_idx] != 0)[0]
            for i in ai:
                y[i] = self._wrap_angle(y[i])

        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y

        if angle_state_idx is not None:
            self.state[angle_state_idx] = self._wrap_angle(self.state[angle_state_idx])

        self.P = (np.eye(5) - K @ H) @ self.P

    @staticmethod
    def _wrap_angle(a):
        return math.atan2(math.sin(a), math.cos(a))

    # ------------------------------------------------------------------
    @property
    def pose(self):
        """Return current best estimate as dict."""
        return {
            "x": float(self.state[0]),
            "y": float(self.state[1]),
            "theta": float(self.state[2]),
            "v": float(self.state[3]),
            "omega": float(self.state[4]),
        }
