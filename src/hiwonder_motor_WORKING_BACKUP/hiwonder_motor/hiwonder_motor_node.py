#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from smbus2 import SMBus


class HiwonderMotorNode(Node):

    def __init__(self):
        super().__init__('hiwonder_motor_node')

        # Hiwonder V1.4
        self.address = 0x34
        self.speed_register = 51

        # Raspberry Pi I2C Bus 1
        self.bus = SMBus(1)

        # Listen to commands from omni_motor_node
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/motor_speeds',
            self.motor_callback,
            10
        )

        self.get_logger().info(
            'Hiwonder motor driver connected on I2C bus 1, address 0x34'
        )

    def motor_callback(self, msg):

        if len(msg.data) != 4:
            self.get_logger().error(
                'Expected 4 motor speeds'
            )
            return

        # Incoming ROS command
        m1 = int(msg.data[0])
        m2 = int(msg.data[1])
        m3 = int(msg.data[2])
        m4 = int(msg.data[3])

        # Limit closed-loop speed commands
        speeds = [
            max(-50, min(50, m1)),
            max(-50, min(50, m2)),
            max(-50, min(50, m3)),
            max(-50, min(50, m4))
        ]

        # Convert signed values to bytes
        data = [speed & 0xFF for speed in speeds]

        try:
            # Register 51 = M1
            # Register 52 = M2
            # Register 53 = M3
            # Register 54 = M4
            self.bus.write_i2c_block_data(
                self.address,
                self.speed_register,
                data
            )

        except OSError as e:
            self.get_logger().error(
                f'I2C communication error: {e}'
            )

    def stop_motors(self):
        try:
            self.bus.write_i2c_block_data(
                self.address,
                self.speed_register,
                [0, 0, 0, 0]
            )
        except OSError:
            pass

    def destroy_node(self):
        self.stop_motors()
        self.bus.close()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = HiwonderMotorNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop_motors()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()