#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')

        # Publishers
        self.emergency_publisher = self.create_publisher(Bool, '/emergency_stop', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_emergency', 10)

        # Emergency state
        self.emergency_active = False

        # Timer for checking keyboard input
        self.timer = self.create_timer(0.1, self.check_emergency_input)

        # Store terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())

        self.get_logger().info('Emergency Stop Node started')
        self.get_logger().info('Press SPACE for emergency stop, ESC to reset, q to quit')

    def check_emergency_input(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)

            if key == ' ':  # Space bar - emergency stop
                self.emergency_active = True
                self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
                self.publish_emergency_state()
                self.publish_stop_command()

            elif key == '\x1b':  # ESC - reset emergency
                self.emergency_active = False
                self.get_logger().info('Emergency stop reset')
                self.publish_emergency_state()

            elif key == 'q':  # Quit
                self.get_logger().info('Shutting down emergency stop node')
                self.cleanup()
                rclpy.shutdown()

    def publish_emergency_state(self):
        msg = Bool()
        msg.data = self.emergency_active
        self.emergency_publisher.publish(msg)

    def publish_stop_command(self):
        if self.emergency_active:
            twist = Twist()
            # All velocities are 0.0 by default
            self.cmd_vel_publisher.publish(twist)

    def cleanup(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def __del__(self):
        self.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()

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
