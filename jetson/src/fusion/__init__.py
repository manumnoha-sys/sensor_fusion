"""
Sensor fusion modules for NVIDIA Jetson Nano + Sphero RVR platform.

Modules:
  sphero_processor  - Sphero RVR telemetry (IMU, encoders, color, IR)
  imu_processor     - IMU data processing and complementary filter
  camera_processor  - CSI/USB camera capture and visual odometry
  ekf               - Extended Kalman Filter for pose estimation
"""

from .imu_processor import IMUProcessor
from .camera_processor import CameraProcessor
from .sphero_processor import SpheroProcessor
from .ekf import PoseEKF

__all__ = ["IMUProcessor", "CameraProcessor", "SpheroProcessor", "PoseEKF"]
