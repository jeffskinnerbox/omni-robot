#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
import sys
import select
import tty
import termios


class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')

        # Publishers
        self.emergency_pub = self.create_publisher(Bool, 'emergency_stop', 10)

        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        # State variables
        self.emergency_active = False
        self.old_settings = None

        # Setup keyboard input
        self.setup_keyboard()

        # Timer for keyboard checking
        self.timer = self.create_timer(0.1, self.check_keyboard)

        self.get_logger().info('Emergency Stop Node started')
        self.get_logger().info('Press SPACE for emergency stop, R to reset')
        self.get_logger().info('Gamepad: Press BACK button for emergency stop')

    def setup_keyboard(self):
        """Setup non-blocking keyboard input"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.setraw(sys.stdin.fileno())
        except:
            self.get_logger().warn('Could not setup keyboard input')

    def check_keyboard(self):
        """Check for keyboard input"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            try:
                key = sys.stdin.read(1)
                if key == ' ':  # Space bar
                    self.activate_emergency_stop()
                elif key.lower() == 'r':  # R key
                    self.reset_emergency_stop()
                elif key == '\x1b':  # ESC key
                    self.destroy_node()
                    rclpy.shutdown()
            except:
                pass

    def joy_callback(self, msg):
        """Handle joystick input for emergency stop"""
        if len(msg.buttons) > 6:
            # BACK button (button 6) pressed
            if msg.buttons[6] == 1:
                self.activate_emergency_stop()

    def activate_emergency_stop(self):
        """Activate emergency stop"""
        if not self.emergency_active:
            self.emergency_active = True
            msg = Bool()
            msg.data = True
            self.emergency_pub.publish(msg)
            self.get_logger().warn('EMERGENCY STOP ACTIVATED!')

    def reset_emergency_stop(self):
        """Reset emergency stop"""
        if self.emergency_active:
            self.emergency_active = False
            msg = Bool()
            msg.data = False
            self.emergency_pub.publish(msg)
            self.get_logger().info('Emergency stop reset')

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
