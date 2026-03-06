"""Sensor fusion entry point for Kria KV260.

Fuses IMU orientation data with camera motion detection.
Supports dual-camera: Logitech C925e (/dev/video1) + Xilinx ISP (/dev/video0).
"""

import time
import logging

from fusion import IMUProcessor, CameraProcessor

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger(__name__)

# Camera configs in priority order.
# /dev/video0 (Xilinx ISP VCAP) is excluded: AP1302 probe fails → cv2.VideoCapture blocks forever.
# Re-add once the IAS FFC cable is reseated and AP1302 comes up.
CAMERA_CONFIGS = [
    {"device": "/dev/video1", "width": 1280, "height": 720, "fourcc": "MJPG"},  # Logitech C925e
]


def main():
    log.info("Starting sensor fusion on Kria KV260")

    # Open all available cameras
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

    # Seed previous frames
    prev_frames = []
    for cam in cameras:
        try:
            prev_frames.append(cam.read_frame())
        except RuntimeError as e:
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
                except RuntimeError as e:
                    log.warning("Frame read error on %s: %s", cam.device, e)

            # TODO: replace with real IMU reads from IIO / I2C
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
