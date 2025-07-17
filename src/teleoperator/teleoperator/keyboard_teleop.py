#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel_keyboard', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)

        # Velocity settings
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.linear_step = 0.5
        self.angular_step = 0.5
        self.max_linear = 3.0
        self.max_angular = 3.0

        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.get_logger().info('Keyboard Teleop Node started')
        self.print_usage()

    def print_usage(self):
        msg = """
        Keyboard Teleop Controls:
        ---------------------------
        w/s: increase/decrease linear velocity
        a/d: increase/decrease angular velocity
        x: stop (zero velocities)
        q: quit

        Current: linear=%.2f, angular=%.2f
        """ % (self.linear_vel, self.angular_vel)
        print(msg)

    def publish_twist(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)

            if key == 'w':
                self.linear_vel = min(self.linear_vel + self.linear_step, self.max_linear)
            elif key == 's':
                self.linear_vel = max(self.linear_vel - self.linear_step, -self.max_linear)
            elif key == 'a':
                self.angular_vel = min(self.angular_vel + self.angular_step, self.max_angular)
            elif key == 'd':
                self.angular_vel = max(self.angular_vel - self.angular_step, -self.max_angular)
            elif key == 'x':
                self.linear_vel = 0.0
                self.angular_vel = 0.0
            elif key == 'q':
                self.cleanup()
                rclpy.shutdown()
                return

            self.get_logger().info(f'Linear: {self.linear_vel:.2f}, Angular: {self.angular_vel:.2f}')

        # Publish twist message
        twist = Twist()
        twist.linear.x = self.linear_vel
        twist.angular.z = self.angular_vel
        self.publisher.publish(twist)

    def cleanup(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def __del__(self):
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
