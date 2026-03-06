# Gazebo simulation container for KV260 sensor fusion
# Base: Ubuntu 22.04 + ROS2 Humble + Ignition Gazebo Fortress
# Architecture: x86_64 (run on development machine, NOT on KV260)

FROM osrf/ros:humble-desktop

# Install Ignition Gazebo Fortress + ros_gz bridge
RUN apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-cv-bridge \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    python3-pip \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy scipy filterpy

WORKDIR /workspace
COPY . /workspace/

# Set Gazebo model/resource paths
ENV GZ_SIM_RESOURCE_PATH=/workspace/gazebo/models
ENV IGN_GAZEBO_RESOURCE_PATH=/workspace/gazebo/models

# Source ROS2 on shell start
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \"$@\"", "--"]
CMD ["ros2", "launch", "gazebo/launch/simulate.launch.py"]
