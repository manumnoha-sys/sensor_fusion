#!/bin/bash
# Build sensor_fusion Docker image for Raspberry Pi (arm64)
set -e

cd "$(dirname "$0")"
docker build -t sensor_fusion:rpi .
echo "Built sensor_fusion:rpi"
