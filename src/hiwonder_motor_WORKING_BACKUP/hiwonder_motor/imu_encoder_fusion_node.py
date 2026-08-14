
#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class ImuEncoderFusionNode(Node):

    def __init__(self):
        super().__init__('imu_encoder_fusion_node')

        # =====================================================
        # Robot geometry
        # =====================================================

        # Left-right wheel center distance
        self.track_width = 0.21

        # Front-rear wheel center distance
        self.wheelbase = 0.19

        self.lx = self.wheelbase / 2.0
        self.ly = self.track_width / 2.0

        # Lx + Ly for mecanum rotational kinematics
        self.rotation_radius = self.lx + self.ly

        # =====================================================
        # Robot state
        # =====================================================

        # Position in odom frame
        self.x = 0.0
        self.y = 0.0

        # Heading in radians
        self.yaw = 0.0
        # Total accumulated rotation, NOT wrapped
        # Useful for gyro calibration
        self.total_yaw = 0.0

        # Robot-frame linear velocity
        self.vx = 0.0
        self.vy = 0.0

        # BNO085 angular velocity in ROS convention
        self.wz = 0.0

        # Encoder-derived angular velocity
        # Used for debugging/comparison
        self.encoder_wz = 0.0

        # =====================================================
        # Timing
        # =====================================================

        self.last_encoder_time = None
        self.last_imu_time = None

        self.imu_received = False

        # =====================================================
        # Subscribers
        # =====================================================

        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            20
        )

        self.create_subscription(
            Float32MultiArray,
            '/wheel_velocity',
            self.wheel_velocity_callback,
            20
        )

        # =====================================================
        # Publishers
        # =====================================================

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            20
        )
        self.total_yaw_pub = self.create_publisher(
            Float32,
            '/total_yaw_deg',
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        # =====================================================
        # Startup messages
        # =====================================================

        self.get_logger().info(
            'IMU + mecanum encoder fusion node started.'
        )

        self.get_logger().info(
            f'Wheelbase: {self.wheelbase:.3f} m | '
            f'Track width: {self.track_width:.3f} m'
        )

        self.get_logger().info(
            'Waiting for /imu/data and /wheel_velocity...'
        )

    # =========================================================
    # IMU callback
    # =========================================================

    def imu_callback(self, msg):

        current_time = (
            msg.header.stamp.sec
            + msg.header.stamp.nanosec * 1e-9
        )

        # -----------------------------------------------------
        # BNO085 mounting correction
        # -----------------------------------------------------
        #
        # Your physical mounting:
        #
        # IMU +X = robot left
        # IMU +Y = robot forward
        # IMU +Z = robot down
        #
        # Experimentally:
        #
        # Clockwise rotation -> IMU gyro Z positive
        #
        # ROS convention:
        #
        # CCW = positive angular Z
        # CW  = negative angular Z
        #
        # Therefore invert BNO085 gyro Z.

        raw_wz = float(msg.angular_velocity.z)

        self.wz = -raw_wz

        # -----------------------------------------------------
        # First IMU measurement
        # -----------------------------------------------------

        if self.last_imu_time is None:

            self.last_imu_time = current_time
            self.imu_received = True

            self.get_logger().info(
                'IMU gyro initialized. Robot yaw = 0 deg.'
            )

            return

        # -----------------------------------------------------
        # Time difference
        # -----------------------------------------------------

        dt = current_time - self.last_imu_time

        self.last_imu_time = current_time

        if dt <= 0.0 or dt > 0.5:
            return

        # -----------------------------------------------------
        # Gyro deadband
        # -----------------------------------------------------
        #
        # Ignore very small stationary gyro noise.

        if abs(self.wz) < 0.02:
            self.wz = 0.0

        # -----------------------------------------------------
        # Integrate gyro -> robot heading
        # -----------------------------------------------------

# Accumulate rotation without wrapping
        delta_yaw = self.wz * dt

        self.total_yaw += delta_yaw

# ROS odometry heading is wrapped to -pi ... +pi
        self.yaw = self.normalize_angle(
            self.total_yaw
        )

        self.imu_received = True
        yaw_msg = Float32()

        yaw_msg.data = float(
            math.degrees(self.total_yaw)
        )

        self.total_yaw_pub.publish(yaw_msg)

    # =========================================================
    # Wheel velocity callback
    # =========================================================

    def wheel_velocity_callback(self, msg):

        if len(msg.data) != 4:

            self.get_logger().error(
                'Expected four wheel velocities '
                'in /wheel_velocity'
            )

            return

        # -----------------------------------------------------
        # Incoming Hiwonder hardware order
        # -----------------------------------------------------
        #
        # M1 = rear left
        # M2 = front right
        # M3 = rear right
        # M4 = front left
        #
        # Encoder node has already normalized all signs so:
        #
        # positive = physical wheel forward

        rear_left = float(msg.data[0])
        front_right = float(msg.data[1])
        rear_right = float(msg.data[2])
        front_left = float(msg.data[3])

        fl = front_left
        fr = front_right
        rl = rear_left
        rr = rear_right

        # -----------------------------------------------------
        # Mecanum forward kinematics
        # -----------------------------------------------------

        # Forward/backward velocity
        self.vx = (
            fl
            + fr
            + rl
            + rr
        ) / 4.0

        # Left/right velocity
        self.vy = (
            -fl
            + fr
            + rl
            - rr
        ) / 4.0

        # Encoder-estimated angular velocity
        self.encoder_wz = (
            -fl
            + fr
            - rl
            + rr
        ) / (
            4.0 * self.rotation_radius
        )

        # -----------------------------------------------------
        # Time
        # -----------------------------------------------------

        current_time = self.get_clock().now()

        if self.last_encoder_time is None:

            self.last_encoder_time = current_time
            return

        dt = (
            current_time
            - self.last_encoder_time
        ).nanoseconds / 1e9

        self.last_encoder_time = current_time

        if dt <= 0.0 or dt > 0.5:
            return

        # Wait for BNO085 gyro
        if not self.imu_received:
            return

        # -----------------------------------------------------
        # Robot-frame velocity -> odom-frame velocity
        # -----------------------------------------------------

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        world_vx = (
            self.vx * cos_yaw
            - self.vy * sin_yaw
        )

        world_vy = (
            self.vx * sin_yaw
            + self.vy * cos_yaw
        )

        # -----------------------------------------------------
        # Integrate translation
        # -----------------------------------------------------

        self.x += world_vx * dt
        self.y += world_vy * dt

        # -----------------------------------------------------
        # Publish odometry
        # -----------------------------------------------------

        self.publish_odometry(
            current_time
        )

    # =========================================================
    # Publish odometry
    # =========================================================

    def publish_odometry(self, current_time):

        # -----------------------------------------------------
        # Yaw -> quaternion
        # -----------------------------------------------------

        half_yaw = self.yaw / 2.0

        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        # -----------------------------------------------------
        # Odometry message
        # -----------------------------------------------------

        odom = Odometry()

        odom.header.stamp = current_time.to_msg()

        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # -----------------------------------------------------
        # Velocity
        # -----------------------------------------------------

        # These are robot/base_link frame velocities.
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.wz

        # -----------------------------------------------------
        # Basic covariance
        # -----------------------------------------------------

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.02

        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.05
        odom.twist.covariance[35] = 0.02

        self.odom_pub.publish(odom)

        # -----------------------------------------------------
        # TF: odom -> base_link
        # -----------------------------------------------------

        transform = TransformStamped()

        transform.header.stamp = current_time.to_msg()

        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(
            transform
        )

    # =========================================================
    # Angle helper
    # =========================================================

    @staticmethod
    def normalize_angle(angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


# =============================================================
# Main
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ImuEncoderFusionNode()

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

