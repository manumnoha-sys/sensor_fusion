FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Base tools + libcamera stack + OpenCV + picamera2
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    python3-picamera2 \
    python3-libcamera \
    libcamera-tools \
    libcamera-v4l2 \
    v4l-utils \
    i2c-tools \
    iio-sensor-proxy \
    && rm -rf /var/lib/apt/lists/*

# Extra Python deps (scipy, etc.)
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /workspace
# Platform-specific camera processor overrides src/fusion/camera_processor.py
COPY src/ /workspace/src/
# Shared IMU processor and entry point
COPY ../src/fusion/imu_processor.py /workspace/src/fusion/imu_processor.py
COPY ../src/fusion/__init__.py /workspace/src/fusion/__init__.py

ENV LIBCAMERA_LOG_LEVELS=*:ERROR

CMD ["python3", "src/main.py"]
