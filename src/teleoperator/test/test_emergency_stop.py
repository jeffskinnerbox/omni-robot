#!/usr/bin/env python3

import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from teleoperator.emergency_stop import EmergencyStopNode
import threading
import time

class TestEmergencyStop:

    @classmethod
    def setup_class(cls):
        rclpy.init()

    @classmethod
    def teardown_class(cls):
        rclpy.shutdown()

    def test_emergency_stop_initialization(self):
        """Test emergency stop node initialization"""
        node = EmergencyStopNode()
        assert node.emergency_active == False
        node.destroy_node()

    def test_emergency_state_publishing(self):
        """Test emergency state publishing"""
        node = EmergencyStopNode()

        # Create a test subscriber
        test_node = Node('test_node')
        received_messages = []

        def callback(msg):
            received_messages.append(msg.data)

        subscription = test_node.create_subscription(
            Bool,
            '/emergency_stop',
            callback,
            10
        )

        # Publish emergency state
        node.emergency_active = True
        node.publish_emergency_state()

        # Spin to process messages
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(test_node)
        executor.spin_once(timeout_sec=1.0)

        # Check results
        assert len(received_messages) > 0
        assert received_messages[-1] == True

        node.destroy_node()
        test_node.destroy_node()

if __name__ == '__main__':
    pytest.main([__file__])
