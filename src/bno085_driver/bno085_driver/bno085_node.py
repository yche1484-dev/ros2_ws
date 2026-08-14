#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from adafruit_extended_bus import ExtendedI2C as I2C
from adafruit_bno08x.i2c import BNO08X_I2C

from adafruit_bno08x import (
    BNO_REPORT_LINEAR_ACCELERATION,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)


class BNO085Node(Node):

    def __init__(self):
        super().__init__('bno085_node')

        # ---------------------------------------------
        # ROS publisher
        # ---------------------------------------------

        self.publisher_ = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )

        # 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.timer_callback
        )

        self.get_logger().info(
            'Connecting directly to BNO085 on I2C Bus 3...'
        )

        try:

            # -----------------------------------------
            # BNO085 on custom Raspberry Pi I2C Bus 3
            # -----------------------------------------

            i2c = I2C(3)

            self.bno = BNO08X_I2C(
                i2c,
                address=0x4A
            )

            # -----------------------------------------
            # Enable BNO085 reports
            # -----------------------------------------

            # Gravity-removed linear acceleration
            self.bno.enable_feature(
                BNO_REPORT_LINEAR_ACCELERATION
            )

            # Angular velocity
            self.bno.enable_feature(
                BNO_REPORT_GYROSCOPE
            )

            # BNO085 internally fused orientation
            self.bno.enable_feature(
                BNO_REPORT_ROTATION_VECTOR
            )

            self.get_logger().info(
                'BNO085 initialized on I2C Bus 3.'
            )

            self.get_logger().info(
                'Enabled: linear acceleration, '
                'gyroscope, rotation vector.'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to initialize BNO085 '
                f'on Bus 3: {e}'
            )

            self.bno = None

    # =================================================
    # IMU reading
    # =================================================

    def timer_callback(self):

        if self.bno is None:
            return

        try:

            msg = Imu()

            msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            msg.header.frame_id = 'imu_link'

            # =========================================
            # Orientation
            # =========================================

            quat = self.bno.quaternion

            if quat and quat[0] is not None:

                # Adafruit BNO08x quaternion order:
                #
                # (x, y, z, w)

                msg.orientation.x = float(quat[0])
                msg.orientation.y = float(quat[1])
                msg.orientation.z = float(quat[2])
                msg.orientation.w = float(quat[3])

                # We now HAVE orientation information,
                # so DO NOT use -1 here.
                #
                # These are initial approximate
                # covariance values.

                msg.orientation_covariance[0] = 0.01
                msg.orientation_covariance[4] = 0.01
                msg.orientation_covariance[8] = 0.01

            else:

                # Tell ROS orientation is unavailable
                # for this particular sample.

                msg.orientation_covariance[0] = -1.0

            # =========================================
            # Linear acceleration
            # =========================================

            accel = self.bno.linear_acceleration

            if accel and accel[0] is not None:

                msg.linear_acceleration.x = float(
                    accel[0]
                )

                msg.linear_acceleration.y = float(
                    accel[1]
                )

                msg.linear_acceleration.z = float(
                    accel[2]
                )

            # Approximate covariance
            msg.linear_acceleration_covariance[0] = 0.05
            msg.linear_acceleration_covariance[4] = 0.05
            msg.linear_acceleration_covariance[8] = 0.05

            # =========================================
            # Angular velocity
            # =========================================

            gyro = self.bno.gyro

            if gyro and gyro[0] is not None:

                # Adafruit BNO08x gyro values are
                # reported in radians/second.

                msg.angular_velocity.x = float(
                    gyro[0]
                )

                msg.angular_velocity.y = float(
                    gyro[1]
                )

                msg.angular_velocity.z =  float(
                    gyro[2]
                )

            # Approximate covariance
            msg.angular_velocity_covariance[0] = 0.01
            msg.angular_velocity_covariance[4] = 0.01
            msg.angular_velocity_covariance[8] = 0.01

            # =========================================
            # Publish
            # =========================================

            self.publisher_.publish(msg)

        except Exception as e:

            self.get_logger().warning(
                f'Error reading BNO085 data: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = BNO085Node()

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