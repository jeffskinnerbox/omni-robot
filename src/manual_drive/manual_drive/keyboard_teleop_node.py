#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_node')

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_keyboard', 10)

        # Parameters
        self.linear_speed = 1.0
        self.angular_speed = 1.0

        # Keyboard setup
        self.old_settings = None
        self.setup_keyboard()

        # Timer for keyboard checking
        self.timer = self.create_timer(0.1, self.check_keyboard)

        self.get_logger().info('Keyboard Teleop Node started')
        self.get_logger().info('Use WASD for movement, Q/E for rotation')

    def setup_keyboard(self):
        """Setup non-blocking keyboard input"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except:
            self.get_logger().warn('Could not setup keyboard input')

    def check_keyboard(self):
        """Check for keyboard input and publish commands"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            try:
                key = sys.stdin.read(1).lower()
                twist = Twist()

                # Movement commands
                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                elif key == 'a':
                    twist.linear.y = self.linear_speed
                elif key == 'd':
                    twist.linear.y = -self.linear_speed
                elif key == 'q':
                    twist.angular.z = self.angular_speed
                elif key == 'e':
                    twist.angular.z = -self.angular_speed

                # Publish command
                self.cmd_vel_pub.publish(twist)

            except:
                pass

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
