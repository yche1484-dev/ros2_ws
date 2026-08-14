#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    package_share = get_package_share_directory(
        'hiwonder_motor'
    )

    ekf_config = os.path.join(
        package_share,
        'config',
        'ekf.yaml'
    )

    wheel_odometry = Node(
        package='hiwonder_motor',
        executable='mecanum_odometry_node',
        name='mecanum_odometry_node',
        output='screen'
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config]
    )

    return LaunchDescription([
        wheel_odometry,
        ekf,
    ])
