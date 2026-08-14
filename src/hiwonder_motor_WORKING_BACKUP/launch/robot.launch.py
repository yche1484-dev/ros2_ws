#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from ament_index_python.packages import (
    get_package_share_directory
)


def generate_launch_description():

    # Find installed hiwonder_motor package
    package_share = get_package_share_directory(
        'hiwonder_motor'
    )

    # =====================================================
    # Sensor + fusion launch
    # =====================================================

    sensors_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                'launch',
                'sensors_fusion.launch.py'
            )
        )
    )

    # =====================================================
    # Motor control launch
    # =====================================================

    motor_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_share,
                'launch',
                'motor_control.launch.py'
            )
        )
    )

    # =====================================================
    # Launch everything
    # =====================================================

    return LaunchDescription([
        sensors_fusion_launch,
        motor_control_launch,
    ])
