#!/usr/bin/env python3
"""
RVR Sensor Fusion — main entry point.

Starts:
  1. BLE thread   — connects to RVR, streams IMU + locator
  2. Camera thread — captures CSI frames, computes optical flow
  3. Fusion thread — complementary filter, updates pose state
  4. Flask server  — serves live dashboard at http://0.0.0.0:8080

Usage:
  sudo python3 main.py [--port 8080] [--no-camera] [--alpha 0.85]
"""
import argparse
import threading
import time
import logging
import sys
import os

# Allow running from repo root or jetson/ble/
sys.path.insert(0, os.path.dirname(__file__))

from rvr.ble_client import RVRBLEClient
from fusion.state import RVRState, PoseState, FrameBuffer
from fusion.complementary import ComplementaryFilter
from fusion.camera_processor import CameraProcessor
from dashboard.server import create_app

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S',
)
logger = logging.getLogger(__name__)

RVR_ADDR   = "F0:0F:1B:C7:0F:03"
FUSION_HZ  = 10


def fusion_loop(rvr_state, frame_buf, pose_state, filt):
    dt = 1.0 / FUSION_HZ
    while True:
        t0 = time.monotonic()

        with rvr_state.lock:
            loc_x    = rvr_state.loc_x
            loc_y    = rvr_state.loc_y
            heading  = rvr_state.heading
            pitch    = rvr_state.pitch
            roll     = rvr_state.roll
            accel    = [rvr_state.accel_x, rvr_state.accel_y, rvr_state.accel_z]
            gyro     = [rvr_state.gyro_roll, rvr_state.gyro_pitch, rvr_state.gyro_yaw]
            ble_conn = rvr_state.ble_connected

        vo = frame_buf.consume_vo()
        fx, fy, fh = filt.update(loc_x, loc_y, heading, vo)

        with pose_state.lock:
            pose_state.x             = fx
            pose_state.y             = fy
            pose_state.heading       = fh
            pose_state.pitch         = pitch
            pose_state.roll          = roll
            pose_state.accel         = accel
            pose_state.gyro          = gyro
            pose_state.ble_connected = ble_conn
            pose_state.trajectory.append((fx, fy))

        elapsed = time.monotonic() - t0
        rem = dt - elapsed
        if rem > 0:
            time.sleep(rem)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port',      type=int,   default=8080)
    parser.add_argument('--alpha',     type=float, default=0.85,
                        help='Complementary filter alpha (0-1)')
    parser.add_argument('--no-camera', action='store_true',
                        help='Disable camera (BLE only)')
    args = parser.parse_args()

    rvr_state  = RVRState()
    pose_state = PoseState()
    frame_buf  = FrameBuffer()
    filt       = ComplementaryFilter(alpha=args.alpha)

    # BLE thread
    rvr = RVRBLEClient(rvr_state)
    ble_thread = threading.Thread(
        target=rvr.connect_and_run,
        args=(RVR_ADDR,),
        daemon=True,
        name='ble',
    )
    ble_thread.start()
    logger.info("BLE thread started")

    # Camera thread
    if not args.no_camera:
        cam = CameraProcessor(frame_buf, source="csi")
        try:
            cam.start()
            logger.info("Camera started")
        except RuntimeError as e:
            logger.warning("Camera failed (%s), running without it", e)
    else:
        logger.info("Camera disabled")

    # Fusion thread
    fus_thread = threading.Thread(
        target=fusion_loop,
        args=(rvr_state, frame_buf, pose_state, filt),
        daemon=True,
        name='fusion',
    )
    fus_thread.start()
    logger.info("Fusion thread started")

    # Flask (blocking)
    app = create_app(pose_state, frame_buf, fusion_filter=filt)
    logger.info("Dashboard at http://0.0.0.0:%d", args.port)
    app.run(host='0.0.0.0', port=args.port, threaded=True)


if __name__ == '__main__':
    main()
