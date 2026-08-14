#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # =====================================================
    # BNO085 hardware node
    #
    # Reads BNO085 from I2C Bus 3
    # Publishes:
    #     /imu/data
    # =====================================================

    bno085_node = Node(
        package='bno085_driver',
        executable='bno085_node',
        name='bno085_node',
        output='screen'
    )

    # =====================================================
    # IMU integrator
    #
    # Package: bno085_driver
    # Subscribes:
    #     /imu/data
    # =====================================================

    imu_integrator_node = Node(
        package='bno085_driver',
        executable='imu_integrator_node',
        name='imu_integrator_node',
        output='screen'
    )

    # =====================================================
    # Hiwonder encoder node
    #
    # Reads encoder counts from:
    #     I2C Bus 1
    #     address 0x34
    #
    # Publishes:
    #     /motor_encoders
    #     /motor_rpm
    #     /wheel_velocity
    # =====================================================

    encoder_node = Node(
        package='hiwonder_motor',
        executable='hiwonder_encoder_node',
        name='hiwonder_encoder_node',
        output='screen'
    )

    # =====================================================
    # IMU + encoder fusion
    #
    # Subscribes:
    #     /imu/data
    #     /wheel_velocity
    #
    # Publishes:
    #     /odom
    # =====================================================

    fusion_node = Node(
        package='hiwonder_motor',
        executable='imu_encoder_fusion_node',
        name='imu_encoder_fusion_node',
        output='screen'
    )

    # =====================================================
    # Launch all sensor/fusion nodes
    # =====================================================

    return LaunchDescription([
        bno085_node,
        imu_integrator_node,
        encoder_node,
        fusion_node,
    ])
