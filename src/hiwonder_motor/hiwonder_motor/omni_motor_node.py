#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray


class OmniMotorNode(Node):

    def __init__(self):
        super().__init__('omni_motor_node')

        # Subscribe to keyboard velocity commands
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish motor commands
        self.motor_pub = self.create_publisher(
            Int32MultiArray,
            '/motor_speeds',
            10
        )

        self.speed_multiplier = 30.0

        self.get_logger().info(
            'Omni Motor Node initialized and linked to /motor_speeds.'
        )

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Standard mecanum kinematics
        # +X = forward
        # +Y = left
        # +Z = counterclockwise
        front_left = (
            linear_x - linear_y - angular_z
        )

        front_right = (
            linear_x + linear_y + angular_z
        )

        rear_left = (
            linear_x + linear_y - angular_z
        )

        rear_right = (
            linear_x - linear_y + angular_z
        )

        # Scale and constrain commands
        fl = int(max(min(
            front_left * self.speed_multiplier, 100), -100))

        fr = int(max(min(
            front_right * self.speed_multiplier, 100), -100))

        rl = int(max(min(
            rear_left * self.speed_multiplier, 100), -100))

        rr = int(max(min(
            rear_right * self.speed_multiplier, 100), -100))

        # Hiwonder hardware order:
        #
        # M1 = Rear Left
        #      positive = physical forward
        #
        # M2 = Front Right
        #      positive = physical forward
        #
        # M3 = Rear Right
        #      positive = physical backward
        #      therefore invert
        #
        # M4 = Front Left
        #      positive = physical backward
        #      therefore invert

        array_msg = Int32MultiArray()

        array_msg.data = [
            rl,       # M1
            fr,       # M2
            -rr,      # M3
            -fl       # M4
        ]

        self.motor_pub.publish(array_msg)


def main(args=None):

    rclpy.init(args=args)

    node = OmniMotorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Stop motors when node exits
        stop_msg = Int32MultiArray()
        stop_msg.data = [0, 0, 0, 0]
        node.motor_pub.publish(stop_msg)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()