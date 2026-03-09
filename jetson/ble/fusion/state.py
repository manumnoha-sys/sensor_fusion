"""Thread-safe shared state containers."""
import threading
import collections
import time


class RVRState(object):
    """Written by BLE thread, read by fusion thread."""
    def __init__(self):
        self.lock = threading.Lock()
        self.loc_x = 0.0       # cm
        self.loc_y = 0.0       # cm
        self.heading = 0.0     # degrees (yaw from IMU)
        self.pitch = 0.0
        self.roll = 0.0
        self.accel_x = 0.0    # G
        self.accel_y = 0.0
        self.accel_z = 0.0
        self.gyro_roll = 0.0  # deg/s
        self.gyro_pitch = 0.0
        self.gyro_yaw = 0.0
        self.ble_connected = False
        self.timestamp = 0.0


class PoseState(object):
    """Written by fusion thread, read by Flask thread."""
    def __init__(self):
        self.lock = threading.Lock()
        self.x = 0.0            # metres (fused)
        self.y = 0.0
        self.heading = 0.0      # degrees
        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.pitch = 0.0
        self.roll = 0.0
        self.ble_connected = False
        self.vo_fps = 0.0
        self.trajectory = collections.deque(maxlen=500)

    def to_dict(self):
        with self.lock:
            return {
                "x": round(self.x, 4),
                "y": round(self.y, 4),
                "heading": round(self.heading, 2),
                "pitch": round(self.pitch, 2),
                "roll": round(self.roll, 2),
                "accel": [round(v, 4) for v in self.accel],
                "gyro":  [round(v, 3) for v in self.gyro],
                "ble_connected": self.ble_connected,
                "vo_fps": round(self.vo_fps, 1),
            }

    def trajectory_list(self):
        with self.lock:
            return [[round(x, 4), round(y, 4)] for x, y in self.trajectory]

    def reset_trajectory(self):
        with self.lock:
            self.trajectory.clear()


class FrameBuffer(object):
    """Written by camera thread, read by Flask and fusion threads."""
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_frame = None   # BGR numpy array
        self._vo_dx = 0.0
        self._vo_dy = 0.0
        self._vo_dtheta = 0.0

    def update(self, frame, vo_dx, vo_dy, vo_dtheta):
        with self.lock:
            self.latest_frame = frame
            self._vo_dx += vo_dx
            self._vo_dy += vo_dy
            self._vo_dtheta += vo_dtheta

    def consume_vo(self):
        """Return accumulated VO delta and reset."""
        with self.lock:
            d = {"dx": self._vo_dx, "dy": self._vo_dy, "dtheta": self._vo_dtheta}
            self._vo_dx = self._vo_dy = self._vo_dtheta = 0.0
        return d

    def get_frame(self):
        with self.lock:
            return self.latest_frame
