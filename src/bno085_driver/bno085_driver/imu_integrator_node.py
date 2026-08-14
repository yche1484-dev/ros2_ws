#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuIntegratorNode(Node):
    def __init__(self):
        super().__init__('imu_integrator_node')

        # Subscriptions
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)

        self.vel = [0.0, 0.0, 0.0]
        self.pos = [0.0, 0.0, 0.0]
        self.last_time = None
        self.DAMPING_FACTOR = 0.95

        self.get_logger().info('Pure IMU Integrator Node initialized.')

    def imu_callback(self, msg):
        current_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0.0 or dt > 0.5:
            return

        ax = -msg.linear_acceleration.x
        ay = -msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        # Integrate acceleration to velocity
        self.vel[0] = (self.vel[0] + ax * dt) * self.DAMPING_FACTOR
        self.vel[1] = (self.vel[1] + ay * dt) * self.DAMPING_FACTOR
        self.vel[2] = (self.vel[2] + az * dt) * self.DAMPING_FACTOR

        # Integrate velocity to position
        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt
        self.pos[2] += self.vel[2] * dt

      #3  self.get_logger().info(
        #    f"Pos (m): X={self.pos[0]:.2f}, Y={self.pos[1]:.2f} | "
         #   f"Vel (m/s): X={self.vel[0]:.2f}, Y={self.vel[1]:.2f}"
       # )

def main(args=None):
    rclpy.init(args=args)
    node = ImuIntegratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()