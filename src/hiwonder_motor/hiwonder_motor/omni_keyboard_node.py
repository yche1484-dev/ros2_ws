#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

# Defines the key bindings (X, Y, Z velocities)
KEY_BINDINGS = {
    'w': ( 1.0,  0.0,  0.0), # Forward
    's': (-1.0,  0.0,  0.0), # Backward
    'a': ( 0.0,  1.0,  0.0), # Strafe Left
    'd': ( 0.0, -1.0,  0.0), # Strafe Right
    'q': ( 0.0,  0.0,  1.0), # Rotate CCW
    'e': ( 0.0,  0.0, -1.0), # Rotate CW
    ' ': ( 0.0,  0.0,  0.0)  # Stop
}

class OmniKeyboardNode(Node):
    def __init__(self):
        super().__init__('omni_keyboard_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.get_logger().info('Omni Keyboard Control Ready. W/A/S/D to move, Q/E to rotate, Space to stop.')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                print(f"I heard the key: {repr(key)}")
                if key in KEY_BINDINGS:
                    x, y, z = KEY_BINDINGS[key]
                    twist.linear.x = x
                    twist.linear.y = y
                    twist.angular.z = z
                    self.publisher_.publish(twist)
                elif key == '\x03': # CTRL+C
                    break
        finally:
            # Send zero velocity on exit
            twist = Twist()
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = OmniKeyboardNode()
    try:
        node.run()
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()