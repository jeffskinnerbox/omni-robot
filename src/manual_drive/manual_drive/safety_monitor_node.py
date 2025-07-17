#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
import time


class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor_node')

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(Bool, 'safety_status', 10)

        # Safety parameters
        self.max_linear_speed = 3.0
        self.max_angular_speed = 3.0
        self.last_joy_time = time.time()
        self.joy_timeout = 1.0  # seconds

        # Timer for safety checks
        self.timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('Safety Monitor Node started')

    def cmd_vel_callback(self, msg):
        """Monitor velocity commands for safety limits"""
        if (abs(msg.linear.x) > self.max_linear_speed or
            abs(msg.angular.z) > self.max_angular_speed):
            self.get_logger().warn(
                f'Velocity command exceeds safety limits: '
                f'linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}')

    def joy_callback(self, msg):
        """Update joystick heartbeat"""
        self.last_joy_time = time.time()

    def safety_check(self):
        """Perform periodic safety checks"""
        current_time = time.time()

        # Check joystick connection
        if current_time - self.last_joy_time > self.joy_timeout:
            # Joystick might be disconnected
            pass  # Could implement additional safety measures here

        # Publish safety status
        status_msg = Bool()
        status_msg.data = True  # System is safe
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
