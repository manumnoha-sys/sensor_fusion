# NVIDIA Jetson Nano — Sensor Fusion dev container
# Base: L4T Ubuntu 18.04 (JetPack 4.6, CUDA 10.2)
# Works on Jetson Nano 4GB (100.112.94.123)

FROM nvcr.io/nvidia/l4t-base:r32.7.1

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-dev \
        python3-numpy \
        python3-opencv \
        libopencv-dev \
        v4l-utils \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        python3-gi \
        python3-gst-1.0 \
        i2c-tools \
        python3-smbus \
        usbutils \
        git \
        wget \
        curl \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip
RUN python3 -m pip install --upgrade pip

# Python dependencies (Sphero SDK, filterpy, etc.)
COPY requirements.txt /app/requirements.txt
RUN pip3 install --no-cache-dir -r /app/requirements.txt

# Install Sphero SDK from source (JetPack/Python 3.6 compatible)
RUN pip3 install --no-cache-dir \
    "sphero-sdk-raspberrypi-python @ git+https://github.com/sphero-inc/sphero-sdk-raspberrypi-python.git" \
    || pip3 install --no-cache-dir sphero-sdk-raspberrypi-python

# Copy application source
COPY src/ /app/src/

WORKDIR /app/src

CMD ["python3", "main.py", "--camera", "usb"]
