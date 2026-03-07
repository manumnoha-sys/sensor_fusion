"""
Sensor fusion main loop — NVIDIA Jetson Nano + Sphero RVR

Architecture:
  - Sphero RVR: IMU (20 Hz) + wheel encoders (20 Hz) via UART
  - Jetson CSI/USB camera: visual odometry (30 Hz)
  - Extended Kalman Filter: fuses all sources → 2D pose estimate

Run:
    python3 src/main.py [--port /dev/ttyUSB0] [--camera csi|usb] [--verbose]
"""

import asyncio
import argparse
import logging
import time
import signal
import sys

from fusion import IMUProcessor, CameraProcessor, SpheroProcessor, PoseEKF

logger = logging.getLogger("sensor_fusion")

# ------------------------------------------------------------------
# Fusion loop configuration
LOOP_HZ = 50          # EKF predict rate
VO_UPDATE_HZ = 10     # Visual odometry update rate (subset of camera fps)
PRINT_HZ = 2          # Console print rate


def parse_args():
    p = argparse.ArgumentParser(description="Jetson Nano + Sphero RVR sensor fusion")
    p.add_argument("--port", default="/dev/ttyUSB0", help="Sphero RVR serial port")
    p.add_argument("--camera", choices=["csi", "usb"], default="csi", help="Camera source")
    p.add_argument("--cam-device", type=int, default=0, help="USB camera device index")
    p.add_argument("--no-sphero", action="store_true", help="Disable Sphero (camera-only mode)")
    p.add_argument("--no-camera", action="store_true", help="Disable camera (Sphero-only mode)")
    p.add_argument("--verbose", action="store_true")
    return p.parse_args()


# ------------------------------------------------------------------
async def run(args):
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    ekf = PoseEKF()
    imu_proc = IMUProcessor()
    sphero: SpheroProcessor | None = None
    camera: CameraProcessor | None = None

    # --- Start Sphero ---
    if not args.no_sphero:
        sphero = SpheroProcessor(port=args.port)
        try:
            await sphero.start()
            logger.info("Sphero RVR connected on %s", args.port)
        except Exception as exc:
            logger.warning("Sphero unavailable: %s — continuing without it", exc)
            sphero = None

    # --- Start Camera ---
    if not args.no_camera:
        camera = CameraProcessor(source=args.camera, device_id=args.cam_device)
        try:
            camera.start()
            logger.info("Camera started (source=%s)", args.camera)
        except Exception as exc:
            logger.warning("Camera unavailable: %s — continuing without it", exc)
            camera = None

    # --- Shutdown handler ---
    stop_event = asyncio.Event()

    def _shutdown(sig, frame):
        logger.info("Caught signal %s, shutting down...", sig)
        stop_event.set()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # --- Fusion loop ---
    dt = 1.0 / LOOP_HZ
    vo_interval = LOOP_HZ // VO_UPDATE_HZ
    print_interval = LOOP_HZ // PRINT_HZ
    step = 0

    logger.info("Fusion loop running at %d Hz (Ctrl+C to stop)", LOOP_HZ)

    while not stop_event.is_set():
        t0 = time.monotonic()

        # 1. EKF predict
        ekf.predict(dt=dt)

        # 2. Sphero sensor updates
        if sphero and sphero.is_connected:
            sd = sphero.latest
            imu_proc.update_from_sphero(sd.roll, sd.pitch, sd.yaw)
            ekf.update_imu(imu_proc.yaw_rad)
            ekf.update_encoders(sd.linear_velocity, sd.angular_velocity)

        # 3. Visual odometry (lower rate)
        if camera and camera.is_open and (step % vo_interval == 0):
            vo = camera.visual_odometry_delta()
            if abs(vo["dx"]) + abs(vo["dy"]) > 1e-6:
                ekf.update_visual_odometry(vo["dx"], vo["dy"], vo["dtheta"])

        # 4. Print status
        if step % print_interval == 0:
            pose = ekf.pose
            logger.info(
                "pose  x=%.3f m  y=%.3f m  theta=%.1f deg  v=%.3f m/s  omega=%.3f rad/s | "
                "cam_fps=%.1f",
                pose["x"], pose["y"],
                pose["theta"] * 57.2958,
                pose["v"], pose["omega"],
                camera.fps if camera else 0.0,
            )

        step += 1
        elapsed = time.monotonic() - t0
        sleep_time = dt - elapsed
        if sleep_time > 0:
            await asyncio.sleep(sleep_time)

    # --- Cleanup ---
    logger.info("Shutting down...")
    if sphero:
        await sphero.stop()
    if camera:
        camera.stop()
    logger.info("Done.")


# ------------------------------------------------------------------
if __name__ == "__main__":
    args = parse_args()
    asyncio.run(run(args))
