"""
ROS2 Humble launch file for KV260 Gazebo simulation.

Starts:
  - Ignition Gazebo with kv260_world
  - ros_gz_bridge: bridges Gz topics → ROS2 topics
  - robot_state_publisher
  - (optional) sensor_fusion node

Usage:
  ros2 launch gazebo/launch/simulate.launch.py
  ros2 launch gazebo/launch/simulate.launch.py headless:=true
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


GAZEBO_MODEL_PATH = str(Path(__file__).parent.parent / "models")
WORLD_FILE       = str(Path(__file__).parent.parent / "worlds" / "kv260_world.sdf")


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run Gazebo without GUI"
    )
    headless = LaunchConfiguration("headless")

    # Set model path so Gazebo finds kv260_robot
    gz_model_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if gz_model_path:
        gz_model_path = gz_model_path + ":" + GAZEBO_MODEL_PATH
    else:
        gz_model_path = GAZEBO_MODEL_PATH

    # Launch Ignition Gazebo (with GUI)
    gazebo_gui = ExecuteProcess(
        cmd=["ign", "gazebo", WORLD_FILE, "-v", "4"],
        additional_env={"GZ_SIM_RESOURCE_PATH": gz_model_path,
                        "IGN_GAZEBO_RESOURCE_PATH": gz_model_path},
        output="screen",
        condition=UnlessCondition(headless),
    )

    # Launch headless (no GUI)
    gazebo_headless = ExecuteProcess(
        cmd=["ign", "gazebo", WORLD_FILE, "-v", "4", "-s"],
        additional_env={"GZ_SIM_RESOURCE_PATH": gz_model_path,
                        "IGN_GAZEBO_RESOURCE_PATH": gz_model_path},
        output="screen",
        condition=IfCondition(headless),
    )

    # ros_gz_bridge: Ignition → ROS2 topic bridges
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_ros_bridge",
        arguments=[
            # Camera image
            "/kv260/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            # Camera info
            "/kv260/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            # IMU
            "/kv260/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            # Odometry
            "/kv260/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            # Velocity command
            "/kv260/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            # Clock
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    # Robot state publisher (uses URDF/SDF for TF tree)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        headless_arg,
        gazebo_gui,
        gazebo_headless,
        bridge,
        robot_state_publisher,
    ])
