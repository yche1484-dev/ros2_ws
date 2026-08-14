#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # =====================================================
    # Hiwonder motor hardware driver
    # =====================================================
    #
    # Receives /motor_speeds and sends the commands
    # over I2C Bus 1 to Hiwonder address 0x34.
    #
    # We DO NOT start omni_motor_node here.
    # We DO NOT start omni_keyboard_node here.

    hiwonder_motor_node = Node(
        package='hiwonder_motor',
        executable='hiwonder_motor_node',
        name='hiwonder_motor_node',
        output='screen'
    )

    # =====================================================
    # HC-SR04 ultrasonic sensor driver
    # =====================================================

    hcsr04_driver = Node(
        package='hcsr04_pkg',
        executable='hcsr04_driver',
        name='hcsr04_driver_node',
        output='screen'
    )

    # =====================================================
    # Autonomous ultrasonic avoidance controller
    # =====================================================

    ultrasonic_avoidance = Node(
        package='hcsr04_pkg',
        executable='ultrasonic_avoidance',
        name='ultrasonic_avoidance_node',
        output='screen',

        parameters=[
            {
                # Obstacle threshold: 20 cm
                'block_distance': 0.20,

                # Require 3 consecutive detections
                'required_detections': 3,

                # Initial test speed
                'escape_speed': 10,

                # Reverse for 0.5 seconds
                'escape_duration': 0.5,
            }
        ]
    )

    # =====================================================
    # Launch
    # =====================================================

    return LaunchDescription([
        hiwonder_motor_node,
        hcsr04_driver,
        ultrasonic_avoidance,
    ])
