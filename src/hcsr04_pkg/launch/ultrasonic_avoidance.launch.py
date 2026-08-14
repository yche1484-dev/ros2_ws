#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    hcsr04_driver = Node(
        package='hcsr04_pkg',
        executable='hcsr04_driver',
        name='hcsr04_driver_node',
        output='screen'
    )

    ultrasonic_avoidance = Node(
        package='hcsr04_pkg',
        executable='ultrasonic_avoidance',
        name='ultrasonic_avoidance_node',
        output='screen',

        parameters=[
            {
                # Obstacle threshold: 20 cm
                'block_distance': 0.20,

                # Require 3 consecutive complete scans
                'required_detections': 3,

                # Low speed for initial testing
                'escape_speed': 10,

                # Move backward for 0.5 seconds
                'escape_duration': 0.5,
            }
        ]
    )

    return LaunchDescription([
        hcsr04_driver,
        ultrasonic_avoidance,
    ])
