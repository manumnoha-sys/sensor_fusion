#!/bin/bash
# Build sensor_fusion Docker image
set -e

cd "$(dirname "$0")/.."
docker build -t sensor_fusion:latest .
echo "Build complete: sensor_fusion:latest"
