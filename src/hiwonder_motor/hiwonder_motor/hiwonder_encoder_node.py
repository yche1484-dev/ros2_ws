#!/usr/bin/env python3

import math
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from smbus2 import SMBus


class HiwonderEncoderNode(Node):

    def __init__(self):
        super().__init__('hiwonder_encoder_node')

        # -------------------------------------------------
        # Hiwonder V1.4 I2C configuration
        # -------------------------------------------------
        self.i2c_bus = 1
        self.address = 0x34
        self.encoder_register = 60

        self.bus = SMBus(self.i2c_bus)

        # -------------------------------------------------
        # Wheel / encoder configuration
        # -------------------------------------------------

        # Measured encoder counts for one complete
        # wheel revolution.
        self.counts_per_rev = 3784.0

        # Omni wheel diameter: 97 mm = 0.097 m
        self.wheel_diameter = 0.097

        self.wheel_circumference = (
            math.pi * self.wheel_diameter
        )

        # -------------------------------------------------
        # Previous measurement
        # -------------------------------------------------

        self.previous_counts = None
        self.previous_time = None

        # -------------------------------------------------
        # ROS2 publishers
        # -------------------------------------------------

        # Accumulated encoder counts
        self.encoder_pub = self.create_publisher(
            Int32MultiArray,
            '/motor_encoders',
            10
        )

        # Wheel RPM
        self.rpm_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_rpm',
            10
        )

        # Wheel linear velocity in meters/second
        self.velocity_pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_velocity',
            10
        )

        # -------------------------------------------------
        # Timer
        # -------------------------------------------------

        # 0.05 seconds = 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.read_encoders
        )

        self.get_logger().info(
            'Hiwonder encoder reader started on '
            'I2C bus 1, address 0x34'
        )

        self.get_logger().info(
            f'Encoder calibration: '
            f'{self.counts_per_rev:.1f} counts/rev'
        )

        self.get_logger().info(
            f'Wheel diameter: '
            f'{self.wheel_diameter * 1000:.1f} mm'
        )

        self.get_logger().info(
            f'Wheel circumference: '
            f'{self.wheel_circumference:.4f} m'
        )

    # =====================================================
    # Read encoders
    # =====================================================

    def read_encoders(self):

        try:

            # ---------------------------------------------
            # Read raw encoder counts
            # ---------------------------------------------

            # Four motors x 4 bytes = 16 bytes
            data = self.bus.read_i2c_block_data(
                self.address,
                self.encoder_register,
                16
            )

            # Convert 16 bytes into four signed
            # 32-bit integers.
            raw = struct.unpack(
                '<iiii',
                bytes(data)
            )

            # ---------------------------------------------
            # Normalize encoder directions
            # ---------------------------------------------
            #
            # From our physical testing:
            #
            # M1 forward = positive
            # M2 forward = positive
            # M3 forward = negative
            # M4 forward = negative
            #
            # After normalization:
            #
            # ALL wheels:
            # positive = physical forward
            # negative = physical backward

            m1 = raw[0]
            m2 = raw[1]
            m3 = -raw[2]
            m4 = -raw[3]

            counts = [
                m1,
                m2,
                m3,
                m4
            ]

            # ---------------------------------------------
            # Publish accumulated encoder counts
            # ---------------------------------------------

            encoder_msg = Int32MultiArray()

            # Hardware order:
            # [M1, M2, M3, M4]
            encoder_msg.data = counts

            self.encoder_pub.publish(
                encoder_msg
            )

            # ---------------------------------------------
            # Get current ROS time
            # ---------------------------------------------

            current_time = self.get_clock().now()

            # On the first reading we don't have an old
            # encoder value, so speed cannot be calculated.
            if (
                self.previous_counts is None
                or self.previous_time is None
            ):
                self.previous_counts = counts.copy()
                self.previous_time = current_time
                return

            # ---------------------------------------------
            # Calculate actual elapsed time
            # ---------------------------------------------

            dt = (
                current_time - self.previous_time
            ).nanoseconds / 1e9

            if dt <= 0.0:
                return

            # ---------------------------------------------
            # Encoder difference
            # ---------------------------------------------

            delta_counts = [
                counts[i] - self.previous_counts[i]
                for i in range(4)
            ]

            # ---------------------------------------------
            # Calculate RPM
            # ---------------------------------------------
            #
            # revolutions =
            # delta_counts / counts_per_rev
            #
            # RPM =
            # revolutions / dt * 60

            rpm = [
                (
                    delta_counts[i]
                    / self.counts_per_rev
                )
                * (60.0 / dt)
                for i in range(4)
            ]

            # ---------------------------------------------
            # Calculate wheel linear velocity
            # ---------------------------------------------
            #
            # distance =
            # revolutions * circumference
            #
            # velocity =
            # distance / time

            wheel_velocity = [
                (
                    delta_counts[i]
                    / self.counts_per_rev
                )
                * self.wheel_circumference
                / dt
                for i in range(4)
            ]

            # ---------------------------------------------
            # Publish RPM
            # ---------------------------------------------

            rpm_msg = Float32MultiArray()

            rpm_msg.data = [
                float(value)
                for value in rpm
            ]

            self.rpm_pub.publish(
                rpm_msg
            )

            # ---------------------------------------------
            # Publish wheel velocity
            # ---------------------------------------------

            velocity_msg = Float32MultiArray()

            velocity_msg.data = [
                float(value)
                for value in wheel_velocity
            ]

            self.velocity_pub.publish(
                velocity_msg
            )

            # ---------------------------------------------
            # Save current measurement for next iteration
            # ---------------------------------------------

            self.previous_counts = counts.copy()
            self.previous_time = current_time

        except OSError as e:

            self.get_logger().error(
                f'I2C encoder read error: {e}'
            )

        except struct.error as e:

            self.get_logger().error(
                f'Encoder data unpack error: {e}'
            )

    # =====================================================
    # Shutdown
    # =====================================================

    def destroy_node(self):

        try:
            self.bus.close()
        except Exception:
            pass

        super().destroy_node()


# =========================================================
# Main
# =========================================================

def main(args=None):

    rclpy.init(args=args)

    node = HiwonderEncoderNode()

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