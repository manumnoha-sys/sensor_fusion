"""
Simulation bridge: subscribes to Gazebo/ROS2 topics and feeds
data into the sensor_fusion pipeline (same interface as real hardware).

Usage:
  # With ROS2 sourced:
  python3 gazebo/scripts/sim_bridge.py

  # Standalone (reads from Ignition transport directly):
  python3 gazebo/scripts/sim_bridge.py --transport ignition
"""

import argparse
import sys
import time
import logging
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))
from fusion import IMUProcessor, CameraProcessor

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger(__name__)


def run_ros2(args):
    """Bridge via ROS2 topics (requires ros_gz_bridge running)."""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, Imu
    import numpy as np
    import cv2
    from cv_bridge import CvBridge

    class SimBridgeNode(Node):
        def __init__(self):
            super().__init__("sim_bridge")
            self.bridge = CvBridge()
            self.imu_proc = IMUProcessor(alpha=0.98)
            self.prev_frame = None

            self.create_subscription(Image, "/kv260/camera/image_raw",
                                     self._on_image, 10)
            self.create_subscription(Imu, "/kv260/imu/data_raw",
                                     self._on_imu, 10)
            log.info("SimBridgeNode ready — subscribed to Gazebo topics")

        def _on_image(self, msg):
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self.prev_frame is not None:
                motion = _detect_motion(self.prev_frame, frame)
                log.info("camera motion_score=%.4f bbox=%s", motion["motion_score"], motion["bbox"])
            self.prev_frame = frame

        def _on_imu(self, msg):
            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            gx = msg.angular_velocity.x
            gy = msg.angular_velocity.y
            gz = msg.angular_velocity.z
            result = self.imu_proc.update(ax, ay, az, gx, gy, gz)
            log.debug("imu roll=%.2f° pitch=%.2f°",
                      result["roll_deg"], result["pitch_deg"])

    rclpy.init()
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_ignition():
    """Bridge directly via Ignition Transport (no ROS2 needed)."""
    try:
        from ignition.msgs import image_pb2, imu_pb2
        import ignition.transport as ign_transport
    except ImportError:
        log.error("ignition-transport Python bindings not found. "
                  "Install: pip3 install ignition-transport")
        sys.exit(1)

    imu_proc = IMUProcessor(alpha=0.98)
    node = ign_transport.Node()

    def on_image(msg):
        log.info("camera frame received: %dx%d", msg.width, msg.height)

    def on_imu(msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        result = imu_proc.update(ax, ay, az, gx, gy, gz)
        log.info("imu roll=%.2f° pitch=%.2f°", result["roll_deg"], result["pitch_deg"])

    node.subscribe(image_pb2.Image, "/kv260/camera/image_raw", on_image)
    node.subscribe(imu_pb2.IMU,   "/kv260/imu/data_raw",       on_imu)

    log.info("Listening on Ignition Transport topics...")
    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


def _detect_motion(frame1, frame2, threshold=25):
    import cv2
    import numpy as np
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)
    diff = cv2.absdiff(gray1, gray2)
    _, mask = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    score = float(np.sum(mask)) / mask.size
    bbox = None
    if contours:
        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        bbox = (x, y, w, h)
    return {"motion_score": score, "bbox": bbox}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Gazebo → sensor_fusion bridge")
    parser.add_argument("--transport", choices=["ros2", "ignition"], default="ros2",
                        help="Transport layer (default: ros2)")
    args = parser.parse_args()

    if args.transport == "ros2":
        run_ros2(args)
    else:
        run_ignition()
