#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # =====================================================
    # Package locations
    # =====================================================

    hiwonder_share = get_package_share_directory(
        'hiwonder_motor'
    )

    # =====================================================
    # 1. BNO085 IMU
    # =====================================================
    #
    # Publishes:
    #   /imu/data
    #

    bno085_node = Node(
        package='bno085_driver',
        executable='bno085_node',
        name='bno085_node',
        output='screen'
    )

    # =====================================================
    # 2. Hiwonder encoder reader
    # =====================================================
    #
    # Publishes:
    #   /motor_encoders
    #   /motor_rpm
    #   /wheel_velocity
    #

    encoder_node = Node(
        package='hiwonder_motor',
        executable='hiwonder_encoder_node',
        name='hiwonder_encoder_node',
        output='screen'
    )

    # =====================================================
    # 3. Hiwonder motor hardware driver
    # =====================================================
    #
    # Subscribes:
    #   /motor_speeds
    #
    # Sends commands through:
    #   I2C Bus 1 -> 0x34
    #

    hiwonder_motor_node = Node(
        package='hiwonder_motor',
        executable='hiwonder_motor_node',
        name='hiwonder_motor_node',
        output='screen'
    )

    # =====================================================
    # 4. HC-SR04 ultrasonic driver
    # =====================================================

    hcsr04_driver = Node(
        package='hcsr04_pkg',
        executable='hcsr04_driver',
        name='hcsr04_driver_node',
        output='screen'
    )

    # =====================================================
    # 5. Ultrasonic autonomous controller
    # =====================================================
    #
    # IMPORTANT:
    #
    # This replaces:
    #   omni_motor_node
    #   omni_keyboard_node
    #
    # in this operating mode.
    #

    ultrasonic_avoidance = Node(
        package='hcsr04_pkg',
        executable='ultrasonic_avoidance',
        name='ultrasonic_avoidance_node',
        output='screen',

        parameters=[
            {
                'block_distance': 0.20,
                'required_detections': 3,
                'escape_speed': 10,
                'escape_duration': 0.5,
            }
        ]
    )

    # =====================================================
    # 6. Standard localization
    # =====================================================
    #
    # ekf_localization.launch.py starts:
    #
    #   mecanum_odometry_node
    #   static IMU TF
    #   robot_localization EKF
    #
    # Produces:
    #   /wheel_odom
    #   /odometry/filtered
    #

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                hiwonder_share,
                'launch',
                'ekf_localization.launch.py'
            )
        )
    )

    # =====================================================
    # Launch everything
    # =====================================================

    return LaunchDescription([

        # Sensors
        bno085_node,
        encoder_node,

        # Motor hardware
        hiwonder_motor_node,

        # Ultrasonic autonomous control
        hcsr04_driver,
        ultrasonic_avoidance,

        # Localization
        localization_launch,
    ])
