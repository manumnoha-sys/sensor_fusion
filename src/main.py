"""Sensor fusion entry point for Kria KV260.

Fuses IMU orientation data with camera motion detection.
"""

import time
import logging

from fusion import IMUProcessor, CameraProcessor

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger(__name__)


def main():
    log.info("Starting sensor fusion on Kria KV260")

    imu = IMUProcessor(alpha=0.98)
    camera = CameraProcessor(device="/dev/video0")

    prev_frame = camera.read_frame()

    try:
        while True:
            frame = camera.read_frame()
            motion = camera.detect_motion(prev_frame, frame)
            prev_frame = frame

            # TODO: replace with real IMU reads from IIO / I2C
            imu_data = imu.update(ax=0.0, ay=0.0, az=9.81, gx=0.0, gy=0.0, gz=0.0)

            log.info(
                "roll=%.2f° pitch=%.2f° | motion_score=%.4f bbox=%s",
                imu_data["roll_deg"],
                imu_data["pitch_deg"],
                motion["motion_score"],
                motion["bbox"],
            )

            time.sleep(0.033)  # ~30 Hz

    except KeyboardInterrupt:
        log.info("Shutting down")
    finally:
        camera.release()


if __name__ == "__main__":
    main()
