#!/usr/bin/env bash
# Build the Jetson Nano sensor fusion Docker image
# Run on the Jetson Nano itself (100.112.94.123) or cross-build with buildx

set -euo pipefail

IMAGE_NAME="jetson-sensor-fusion"
TAG="${1:-latest}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "==> Building $IMAGE_NAME:$TAG from $PROJECT_DIR"

docker build \
    -f "$SCRIPT_DIR/Dockerfile" \
    -t "$IMAGE_NAME:$TAG" \
    "$PROJECT_DIR"

echo "==> Build complete: $IMAGE_NAME:$TAG"
