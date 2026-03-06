#!/bin/bash
# Run sensor_fusion container on Kria KV260

IMAGE="sensor_fusion:latest"
CONTAINER="sensor_fusion"

docker run -it --rm \
  --name "$CONTAINER" \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v /sys:/sys \
  -v "$(pwd)":/workspace \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  "$IMAGE"
