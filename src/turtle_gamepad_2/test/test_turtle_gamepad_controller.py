#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import time
import threading


class TestTurtleGamepadController:
    @classmethod
    def setup_class(cls):
        """Setup test environment"""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Cleanup test environment"""
        rclpy.shutdown()

    def test_joy_message_format(self):
        """Test joy message format"""
        joy_msg = Joy()
        joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        assert len(joy_msg.axes) == 8
        assert len(joy_msg.buttons) == 11

    def test_twist_message_creation(self):
        """Test twist message creation"""
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.5

        assert twist.linear.x == 1.0
        assert twist.angular.z == 0.5
        assert twist.linear.y == 0.0
        assert twist.linear.z == 0.0
        assert twist.angular.x == 0.0
        assert twist.angular.y == 0.0

    def test_emergency_stop_message(self):
        """Test emergency stop message"""
        emergency_msg = Bool()
        emergency_msg.data = True

        assert emergency_msg.data == True

        emergency_msg.data = False
        assert emergency_msg.data == False


if __name__ == '__main__':
    pytest.main([__file__])
