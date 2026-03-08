#!/bin/bash
# Run sensor_fusion container on Raspberry Pi with IMX219 camera (cam0)

IMAGE="sensor_fusion:rpi"
CONTAINER="sensor_fusion_rpi"

docker run -it --rm \
  --name "$CONTAINER" \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v /sys:/sys \
  -v /run/libcamera:/run/libcamera \
  -v "$(pwd)/..":/workspace \
  -e LIBCAMERA_LOG_LEVELS='*:ERROR' \
  "$IMAGE"
