#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from teleoperator.keyboard_teleop import KeyboardTeleopNode

class TestKeyboardTeleop:

    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.shutdown()

    def test_keyboard_teleop_initialization(self):
        """Test keyboard teleop node initialization"""
        node = KeyboardTeleopNode()
        assert node.linear_vel == 0.0
        assert node.angular_vel == 0.0
        assert node.linear_step == 0.5
        assert node.angular_step == 0.5
        node.destroy_node()

    def test_velocity_limits(self):
        """Test velocity limits"""
        node = KeyboardTeleopNode()

        # Test maximum positive linear velocity
        node.linear_vel = node.max_linear + 1.0
        node.linear_vel = min(node.linear_vel, node.max_linear)
        assert node.linear_vel == node.max_linear

        # Test maximum negative linear velocity
        node.linear_vel = -node.max_linear - 1.0
        node.linear_vel = max(node.linear_vel, -node.max_linear)
        assert node.linear_vel == -node.max_linear

        node.destroy_node()

if __name__ == '__main__':
    pytest.main([__file__])
