"""Sensor fusion entry point for Raspberry Pi with Arducam IMX219 (cam0).

Fuses IMU orientation data with camera motion detection.
Camera accessed via picamera2/libcamera — see rpi/src/fusion/camera_processor.py.
"""

import time
import logging

from fusion.camera_processor import CameraProcessor
from fusion.imu_processor import IMUProcessor

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger(__name__)

# IMX219 on cam0 — 1920x1080 @ ~30fps
CAMERA_CONFIGS = [
    {"camera_num": 0, "width": 1920, "height": 1080},
]


def main():
    log.info("Starting sensor fusion on Raspberry Pi (IMX219 cam0)")

    cameras = []
    for cfg in CAMERA_CONFIGS:
        cam = CameraProcessor.try_open(**cfg)
        if cam:
            cameras.append(cam)

    if not cameras:
        log.error("No cameras available — exiting")
        return

    log.info("%d camera(s) active: %s", len(cameras), [c.device for c in cameras])

    imu = IMUProcessor(alpha=0.98)

    prev_frames = []
    for cam in cameras:
        try:
            prev_frames.append(cam.read_frame())
        except Exception as e:
            log.warning("Initial read failed for %s: %s", cam.device, e)
            prev_frames.append(None)

    try:
        while True:
            for i, cam in enumerate(cameras):
                try:
                    frame = cam.read_frame()
                    if prev_frames[i] is not None:
                        motion = cam.detect_motion(prev_frames[i], frame)
                        log.info(
                            "[%s] motion_score=%.4f bbox=%s",
                            cam.device,
                            motion["motion_score"],
                            motion["bbox"],
                        )
                    prev_frames[i] = frame
                except Exception as e:
                    log.warning("Frame read error on %s: %s", cam.device, e)

            # TODO: replace with real IMU reads (I2C / SPI on Pi GPIO)
            imu_data = imu.update(ax=0.0, ay=0.0, az=9.81, gx=0.0, gy=0.0, gz=0.0)
            log.info("roll=%.2f° pitch=%.2f°", imu_data["roll_deg"], imu_data["pitch_deg"])

            time.sleep(0.033)  # ~30 Hz

    except KeyboardInterrupt:
        log.info("Shutting down")
    finally:
        for cam in cameras:
            cam.release()


if __name__ == "__main__":
    main()
