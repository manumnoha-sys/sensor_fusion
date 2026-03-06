FROM xilinx/kria-developer:latest

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-opencv \
    cmake \
    build-essential \
    libv4l-dev \
    v4l-utils \
    i2c-tools \
    libi2c-dev \
    libgpiod-dev \
    gpiod \
    iio-sensor-proxy \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

WORKDIR /workspace
COPY src/ /workspace/src/

CMD ["python3", "src/main.py"]
