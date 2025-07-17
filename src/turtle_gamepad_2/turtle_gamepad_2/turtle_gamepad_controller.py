#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import math


class TurtleGamepadController(Node):
    def __init__(self):
        super().__init__('turtle_gamepad_controller')

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.emergency_status_publisher = self.create_publisher(Bool, '/emergency_stop_status', 10)

        # Subscribers
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.emergency_subscriber = self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        # Parameters
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_angular_yaw', 0)
        self.declare_parameter('scale_linear', 2.0)
        self.declare_parameter('scale_angular', 2.0)
        self.declare_parameter('scale_linear_turbo', 5.0)
        self.declare_parameter('scale_angular_turbo', 5.0)
        self.declare_parameter('enable_button', 4)
        self.declare_parameter('turbo_button', 5)
        self.declare_parameter('emergency_button', 1)
        self.declare_parameter('reset_button', 0)
        self.declare_parameter('deadzone', 0.05)

        # State variables
        self.emergency_stop_active = False
        self.last_emergency_button_state = False
        self.last_reset_button_state = False

        # Timer for status updates
        self.status_timer = self.create_timer(0.1, self.publish_status)

        self.get_logger().info('Turtle Gamepad Controller initialized')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left stick: Move turtle')
        self.get_logger().info('  LB (Left Bumper): Enable movement')
        self.get_logger().info('  RB (Right Bumper): Turbo mode')
        self.get_logger().info('  B button: Emergency stop')
        self.get_logger().info('  A button: Reset emergency stop')

    def joy_callback(self, msg):
        """Handle joystick input"""
        if len(msg.buttons) == 0 or len(msg.axes) == 0:
            return

        # Get parameters
        axis_linear = self.get_parameter('axis_linear_x').value
        axis_angular = self.get_parameter('axis_angular_yaw').value
        scale_linear = self.get_parameter('scale_linear').value
        scale_angular = self.get_parameter('scale_angular').value
        scale_linear_turbo = self.get_parameter('scale_linear_turbo').value
        scale_angular_turbo = self.get_parameter('scale_angular_turbo').value
        enable_button = self.get_parameter('enable_button').value
        turbo_button = self.get_parameter('turbo_button').value
        emergency_button = self.get_parameter('emergency_button').value
        reset_button = self.get_parameter('reset_button').value
        deadzone = self.get_parameter('deadzone').value

        # Handle emergency stop button (B button)
        if emergency_button < len(msg.buttons):
            emergency_pressed = msg.buttons[emergency_button] == 1
            if emergency_pressed and not self.last_emergency_button_state:
                self.emergency_stop_active = True
                self.get_logger().warn('EMERGENCY STOP ACTIVATED!')
            self.last_emergency_button_state = emergency_pressed

        # Handle reset button (A button)
        if reset_button < len(msg.buttons):
            reset_pressed = msg.buttons[reset_button] == 1
            if reset_pressed and not self.last_reset_button_state:
                if self.emergency_stop_active:
                    self.emergency_stop_active = False
                    self.get_logger().info('Emergency stop RESET - Normal operation resumed')
            self.last_reset_button_state = reset_pressed

        # Don't move if emergency stop is active
        if self.emergency_stop_active:
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)
            return

        # Check if enable button is pressed
        enable_pressed = False
        if enable_button < len(msg.buttons):
            enable_pressed = msg.buttons[enable_button] == 1

        # Check if turbo button is pressed
        turbo_pressed = False
        if turbo_button < len(msg.buttons):
            turbo_pressed = msg.buttons[turbo_button] == 1

        # Create twist message
        twist = Twist()

        # Only move if enable button is pressed
        if enable_pressed:
            # Get axis values with deadzone
            linear_val = 0.0
            angular_val = 0.0

            if axis_linear < len(msg.axes):
                raw_linear = -msg.axes[axis_linear]  # Invert for forward/backward
                if abs(raw_linear) > deadzone:
                    linear_val = raw_linear

            if axis_angular < len(msg.axes):
                raw_angular = msg.axes[axis_angular]
                if abs(raw_angular) > deadzone:
                    angular_val = raw_angular

            # Apply scaling
            if turbo_pressed:
                twist.linear.x = linear_val * scale_linear_turbo
                twist.angular.z = angular_val * scale_angular_turbo
            else:
                twist.linear.x = linear_val * scale_linear
                twist.angular.z = angular_val * scale_angular

        # Publish the twist message
        self.cmd_vel_publisher.publish(twist)

    def emergency_callback(self, msg):
        """Handle external emergency stop commands"""
        self.emergency_stop_active = msg.data
        if msg.data:
            self.get_logger().warn('External EMERGENCY STOP received!')
            # Stop the turtle immediately
            twist = Twist()
            self.cmd_vel_publisher.publish(twist)

    def publish_status(self):
        """Publish emergency stop status"""
        status_msg = Bool()
        status_msg.data = self.emergency_stop_active
        self.emergency_status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = TurtleGamepadController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
