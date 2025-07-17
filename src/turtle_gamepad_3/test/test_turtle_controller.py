#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from turtle_gamepad_3.turtle_controller import TurtleGamepadController
import time

class TestTurtleController:
    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.shutdown()

    def test_controller_initialization(self):
        """Test that the controller initializes properly"""
        controller = TurtleGamepadController()
        assert controller.turtle_name == 'turtle1'
        assert controller.emergency_stopped == False
        controller.destroy_node()

    def test_deadzone_functionality(self):
        """Test deadzone application"""
        controller = TurtleGamepadController()

        # Test values within deadzone
        assert controller.apply_deadzone(0.05) == 0.0
        assert controller.apply_deadzone(-0.05) == 0.0

        # Test values outside deadzone
        assert controller.apply_deadzone(0.5) == 0.5
        assert controller.apply_deadzone(-0.5) == -0.5

        controller.destroy_node()

    def test_emergency_stop_functionality(self):
        """Test emergency stop state changes"""
        controller = TurtleGamepadController()

        # Create mock joy message
        joy_msg = Joy()
        joy_msg.buttons = [0] * 8  # 8 buttons
        joy_msg.axes = [0.0] * 8   # 8 axes

        # Test emergency stop activation
        initial_state = controller.emergency_stopped
        joy_msg.buttons[controller.emergency_stop_button] = 1

        # Simulate button press
        controller.joy_callback(joy_msg)
        time.sleep(0.1)  # Small delay

        # Note: Emergency stop toggles, so we test the toggle behavior
        assert controller.emergency_stopped != initial_state

        controller.destroy_node()

def test_gamepad_config_loading():
    """Test that gamepad configuration can be loaded"""
    # This would test loading the YAML config
    # In a real scenario, you'd test parameter loading
    pass

if __name__ == '__main__':
    pytest.main([__file__])
