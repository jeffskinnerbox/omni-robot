#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import time
import threading


class TestIntegration:
    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.shutdown()

    def test_joy_to_twist_conversion(self):
        """
        Test that joystick input generates correct twist messages
        """
        from turtle_gamepad_3.turtle_controller import TurtleGamepadController

        controller = TurtleGamepadController()

        # Create a subscriber to capture published twist messages
        received_messages = []

        def twist_callback(msg):
            received_messages.append(msg)

        twist_sub = controller.create_subscription(
            Twist, f"/{controller.turtle_name}/cmd_vel", twist_callback, 10
        )

        # Create mock joy message
        joy_msg = Joy()
        joy_msg.buttons = [0] * 8
        joy_msg.axes = [0.0] * 8

        # Set joystick values
        joy_msg.axes[controller.linear_axis] = 0.5  # Forward
        joy_msg.axes[controller.angular_axis] = 0.3  # Turn right

        # Process the message
        controller.joy_callback(joy_msg)

        # Spin briefly to process messages
        rclpy.spin_once(controller, timeout_sec=0.1)

        # Check that a twist message was published
        assert len(received_messages) > 0
        last_twist = received_messages[-1]

        # Check that the values are reasonable
        assert last_twist.linear.x > 0  # Moving forward
        assert last_twist.angular.z != 0  # Turning

        controller.destroy_node()


if __name__ == "__main__":
    pytest.main([__file__])
