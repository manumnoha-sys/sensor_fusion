#!/usr/bin/env python3
"""
Drives the Sphero RVR in random directions continuously until Ctrl+C.

Usage:
  sudo python3 rvr_random_drive.py [--speed 80] [--min-dur 0.8] [--max-dur 2.5]
"""

import argparse
import random
import time

from rvr_control import RVR

# Random heading colours (HSV wheel approximation — just for fun)
COLORS = [
    (255, 0,   0),    # red
    (255, 128, 0),    # orange
    (255, 255, 0),    # yellow
    (0,   255, 0),    # green
    (0,   255, 255),  # cyan
    (0,   0,   255),  # blue
    (128, 0,   255),  # violet
    (255, 0,   255),  # magenta
]


def random_move(rvr: RVR, speed: int, min_dur: float, max_dur: float):
    heading = random.randint(0, 359)
    duration = random.uniform(min_dur, max_dur)
    r, g, b = random.choice(COLORS)

    print(f"  heading={heading:3d}°  speed={speed}  dur={duration:.1f}s", flush=True)
    rvr.set_leds(r, g, b)
    rvr.drive(speed, heading)
    time.sleep(duration)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--speed",   type=int,   default=80,  help="Drive speed 0-255")
    parser.add_argument("--min-dur", type=float, default=0.8, help="Min leg duration (s)")
    parser.add_argument("--max-dur", type=float, default=2.5, help="Max leg duration (s)")
    parser.add_argument("--pause",   type=float, default=0.3, help="Pause between legs (s)")
    args = parser.parse_args()

    print("Connecting to RVR…")
    rvr = RVR()
    print("Connected!")

    rvr.wake()
    batt = rvr.get_battery()
    print(f"Battery: {batt}%")
    rvr.reset_yaw()
    time.sleep(0.3)

    print("Random driving — Ctrl+C to stop\n")
    try:
        while True:
            random_move(rvr, args.speed, args.min_dur, args.max_dur)
            rvr.stop()
            time.sleep(args.pause)
    except KeyboardInterrupt:
        print("\nStopping…")
    finally:
        rvr.stop()
        rvr.set_leds(0, 0, 0)
        rvr.disconnect()
        print("Done.")


if __name__ == "__main__":
    main()
