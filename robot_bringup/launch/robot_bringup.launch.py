#!/usr/bin/env python3
"""
robot_bringup.launch.py — complete launch file for one 2WD robot

Starts:
  1. robot_state_publisher
  2. joint_state_publisher
  3. serial_bridge_node     (/cmd_vel ↔ ESP32 serial ↔ /odom/unfiltered)
  4. mpu6050_node           (/imu/data)
  5. ekf_filter_node        (/odom)

Usage:
  ros2 launch robot_bringup robot_bringup.launch.py \
    serial_port:=/dev/ttyUSB0 \
    baud_rate:=115200 \
    left_ticks_per_meter:=1001.0 \
    right_ticks_per_meter:=1001.0 \
    wheel_diameter:=0.065 \
    wheel_separation:=0.170
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Paths ────────────────────────────────────────────────────────────────
    launch_dir = Path(__file__).parent.resolve()
    urdf_path = str(launch_dir.parent / 'config' / 'robot.urdf')
    ekf_config = str(launch_dir.parent / 'config' / 'ekf.yaml')

    # ── Launch arguments ─────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for ESP32 connection')

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='115200',
        description='Serial baud rate')

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='true',
        description='Publish odom→base_footprint TF from serial bridge')

    wheel_diameter_arg = DeclareLaunchArgument(
        'wheel_diameter', default_value='0.065',
        description='Wheel diameter in metres')

    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.170',
        description='Wheel centre-to-centre separation in metres')

    left_ticks_per_meter_arg = DeclareLaunchArgument(
        'left_ticks_per_meter', default_value='1001.0',
        description='Left encoder ticks per metre from 1 m push-test')

    right_ticks_per_meter_arg = DeclareLaunchArgument(
        'right_ticks_per_meter', default_value='1001.0',
        description='Right encoder ticks per metre from 1 m push-test')

    # ── Read URDF ────────────────────────────────────────────────────────────
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    serial_bridge_node = Node(
        package='robot_bringup',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'wheel_diameter': LaunchConfiguration('wheel_diameter'),
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'left_ticks_per_meter': LaunchConfiguration('left_ticks_per_meter'),
            'right_ticks_per_meter': LaunchConfiguration('right_ticks_per_meter'),
        }],
        output='screen'
    )

    mpu6050_node = Node(
        package='robot_bringup',
        executable='mpu6050_node',
        name='mpu6050_node',
        parameters=[{
            'i2c_bus': 1,
            'i2c_address': 0x68,
            'publish_rate': 50.0,
            'frame_id': 'imu_link',
        }],
        output='screen'
    )

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
        wheel_diameter_arg,
        wheel_separation_arg,
        left_ticks_per_meter_arg,
        right_ticks_per_meter_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        serial_bridge_node,
        mpu6050_node,
        ekf_node,
    ])
