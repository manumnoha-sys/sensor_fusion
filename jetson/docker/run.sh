#!/usr/bin/env bash
# Run the Jetson Nano sensor fusion container
# - Passes through GPU (--runtime nvidia)
# - Passes through /dev for camera (V4L2), serial port (Sphero RVR), and I2C
# - Host network for dashboard access

set -euo pipefail

IMAGE_NAME="jetson-sensor-fusion"
TAG="${1:-latest}"
SPHERO_PORT="${SPHERO_PORT:-/dev/ttyUSB0}"
CAMERA_SOURCE="${CAMERA_SOURCE:-csi}"

echo "==> Starting $IMAGE_NAME:$TAG"
echo "    Sphero port : $SPHERO_PORT"
echo "    Camera      : $CAMERA_SOURCE"

# Detect --runtime nvidia vs --gpus all (JetPack 4 uses --runtime nvidia)
RUNTIME_FLAG="--runtime nvidia"

docker run -it --rm \
    $RUNTIME_FLAG \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -e SPHERO_PORT="$SPHERO_PORT" \
    -e CAMERA_SOURCE="$CAMERA_SOURCE" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    "$IMAGE_NAME:$TAG" \
    python3 main.py \
        --port "$SPHERO_PORT" \
        --camera "$CAMERA_SOURCE"
