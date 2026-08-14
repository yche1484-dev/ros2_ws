#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # =====================================================
    # Omni/mecanum kinematics node
    #
    # Subscribes:
    #     /cmd_vel
    #
    # Publishes:
    #     /motor_speeds
    #
    # Converts:
    #
    #     Vx
    #     Vy
    #     Wz
    #
    # into:
    #
    #     M1
    #     M2
    #     M3
    #     M4
    # =====================================================

    omni_motor_node = Node(
        package='hiwonder_motor',
        executable='omni_motor_node',
        name='omni_motor_node',
        output='screen'
    )

    # =====================================================
    # Hiwonder hardware motor node
    #
    # Subscribes:
    #     /motor_speeds
    #
    # Sends motor commands to:
    #
    #     I2C Bus 1
    #     address 0x34
    #
    # Controls:
    #     M1
    #     M2
    #     M3
    #     M4
    # =====================================================

    hiwonder_motor_node = Node(
        package='hiwonder_motor',
        executable='hiwonder_motor_node',
        name='hiwonder_motor_node',
        output='screen'
    )

    return LaunchDescription([
        omni_motor_node,
        hiwonder_motor_node,
    ])
