#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


class MecanumOdometryNode(Node):

    def __init__(self):
        super().__init__('mecanum_odometry_node')

        # Robot geometry
        self.track_width = 0.210
        self.wheelbase = 0.190

        self.lx = self.wheelbase / 2.0
        self.ly = self.track_width / 2.0
        self.rotation_radius = self.lx + self.ly

        self.create_subscription(
            Float32MultiArray,
            '/wheel_velocity',
            self.wheel_callback,
            20
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/wheel_odom',
            20
        )

        self.get_logger().info(
            'Mecanum wheel odometry node started.'
        )

    def wheel_callback(self, msg):

        if len(msg.data) != 4:
            self.get_logger().error(
                'Expected four wheel velocities.'
            )
            return

        # Incoming hardware order:
        #
        # M1 = rear left
        # M2 = front right
        # M3 = rear right
        # M4 = front left
        #
        # Encoder signs have already been normalized.

        rl = float(msg.data[0])
        fr = float(msg.data[1])
        rr = float(msg.data[2])
        fl = float(msg.data[3])

        # Mecanum forward kinematics

        vx = (
            fl + fr + rl + rr
        ) / 4.0

        vy = (
            -fl + fr + rl - rr
        ) / 4.0

        wz = (
            -fl + fr - rl + rr
        ) / (
            4.0 * self.rotation_radius
        )

        msg_out = Odometry()

        msg_out.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg_out.header.frame_id = 'odom'
        msg_out.child_frame_id = 'base_link'

        # IMPORTANT:
        # We're providing velocity to the EKF,
        # not integrating position here.

        msg_out.twist.twist.linear.x = vx
        msg_out.twist.twist.linear.y = vy
        msg_out.twist.twist.angular.z = wz

        # Encoder velocity uncertainty.
        #
        # Mecanum lateral velocity is less trustworthy
        # because the rollers can slip.

        msg_out.twist.covariance[0] = 0.02
        msg_out.twist.covariance[7] = 0.05
        msg_out.twist.covariance[35] = 0.08

        self.odom_pub.publish(msg_out)


def main(args=None):

    rclpy.init(args=args)

    node = MecanumOdometryNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
