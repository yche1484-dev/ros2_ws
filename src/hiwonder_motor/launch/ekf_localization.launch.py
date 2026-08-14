#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    package_share = get_package_share_directory(
        'hiwonder_motor'
    )

    ekf_config = os.path.join(
        package_share,
        'config',
        'ekf.yaml'
    )

    # Convert wheel velocities into standard ROS Odometry
    wheel_odometry = Node(
        package='hiwonder_motor',
        executable='mecanum_odometry_node',
        name='mecanum_odometry_node',
        output='screen'
    )

    # Fixed mounting relationship:
    #
    # IMU +X = robot left
    # IMU +Y = robot forward
    # IMU +Z = robot down
    #
    # Robot:
    # +X = forward
    # +Y = left
    # +Z = up
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '3.14159265',
            '--pitch', '0',
            '--yaw', '1.57079633',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ],
        output='screen'
    )

    # Standard robot_localization EKF
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    return LaunchDescription([
        imu_static_tf,
        wheel_odometry,
        ekf,
    ])