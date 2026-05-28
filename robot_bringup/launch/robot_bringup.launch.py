#!/usr/bin/env python3
"""
robot_bringup.launch.py — Approach 1 complete launch file
----------------------------------------------------------
Starts all nodes needed to run one 2WD robot:

  1. robot_state_publisher  — publishes TF from URDF
  2. serial_bridge_node     — /cmd_vel ↔ ESP32 serial ↔ /odom/unfiltered
  3. mpu6050_node           — /imu/data from RPi4 GPIO I2C
  4. ekf_filter_node        — fuses odom + IMU → /odom

Usage:
  ros2 launch robot_bringup robot_bringup.launch.py

Optional arguments:
  serial_port:=/dev/ttyUSB0     (default: /dev/ttyUSB0)
  baud_rate:=115200             (default: 115200)
  publish_tf:=true              (default: true)
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # ── Paths ────────────────────────────────────────────────────────────────
    # All config files are in the same directory as this launch file
    launch_dir = Path(__file__).parent.resolve()
    urdf_path  = str(launch_dir.parent / 'config' / 'robot.urdf')
    ekf_config = str(launch_dir.parent / 'config' / 'ekf.yaml')

    # ── Launch arguments (can be overridden on command line) ─────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 connection')

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Serial baud rate')

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='Publish odom→base_footprint TF from serial bridge')

    # ── Read URDF ────────────────────────────────────────────────────────────
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # ── Node 1: robot_state_publisher ─────────────────────────────────────
    # Reads URDF and publishes all fixed TF transforms (base_link, wheels, IMU)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # ── Node 2: joint_state_publisher ─────────────────────────────────────
    # Publishes wheel joint states so the TF tree is complete
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # ── Node 3: serial_bridge_node ────────────────────────────────────────
    # Custom node: /cmd_vel → ESP32 serial → /odom/unfiltered
    serial_bridge_node = Node(
        package='robot_bringup',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate':   LaunchConfiguration('baud_rate'),
            'publish_tf':  LaunchConfiguration('publish_tf'),
        }],
        output='screen'
    )

    # ── Node 4: mpu6050_node ──────────────────────────────────────────────
    # Custom node: reads MPU6050 via RPi4 I2C → /imu/data
    mpu6050_node = Node(
        package='robot_bringup',
        executable='mpu6050_node',
        name='mpu6050_node',
        parameters=[{
            'i2c_bus':      1,
            'i2c_address':  0x68,
            'publish_rate': 50.0,
            'frame_id':     'imu_link',
        }],
        output='screen'
    )

    # ── Node 5: EKF filter (robot_localization) ───────────────────────────
    # Fuses /odom/unfiltered + /imu/data → /odom
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config],
        remappings=[('odometry/filtered', '/odom')],
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        publish_tf_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        serial_bridge_node,
        mpu6050_node,
        ekf_node,
    ])
