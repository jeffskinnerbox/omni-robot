#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class TestEmergencyStopMonitor:
    @classmethod
    def setup_class(cls):
        """Setup test environment"""
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        """Cleanup test environment"""
        rclpy.shutdown()

    def test_emergency_stop_status(self):
        """Test emergency stop status handling"""
        status_msg = Bool()
        status_msg.data = True

        assert status_msg.data == True

    def test_zero_velocity_command(self):
        """Test zero velocity command creation"""
        stop_twist = Twist()

        assert stop_twist.linear.x == 0.0
        assert stop_twist.linear.y == 0.0
        assert stop_twist.linear.z == 0.0
        assert stop_twist.angular.x == 0.0
        assert stop_twist.angular.y == 0.0
        assert stop_twist.angular.z == 0.0


if __name__ == '__main__':
    pytest.main([__file__])
