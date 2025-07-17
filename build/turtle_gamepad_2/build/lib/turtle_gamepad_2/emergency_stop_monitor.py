#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class EmergencyStopMonitor(Node):
    def __init__(self):
        super().__init__('emergency_stop_monitor')

        # Publishers
        self.emergency_stop_publisher = self.create_publisher(Bool, '/emergency_stop', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscribers
        self.status_subscriber = self.create_subscription(
            Bool, '/emergency_stop_status', self.status_callback, 10)

        # State
        self.emergency_active = False

        # Timer for emergency stop enforcement
        self.timer = self.create_timer(0.1, self.enforce_emergency_stop)

        self.get_logger().info('Emergency Stop Monitor initialized')

    def status_callback(self, msg):
        """Monitor emergency stop status"""
        if msg.data != self.emergency_active:
            self.emergency_active = msg.data
            if self.emergency_active:
                self.get_logger().error('EMERGENCY STOP ACTIVE - All movement stopped!')
            else:
                self.get_logger().info('Emergency stop released - Movement enabled')

    def enforce_emergency_stop(self):
        """Continuously enforce emergency stop by sending zero velocity"""
        if self.emergency_active:
            stop_twist = Twist()
            self.cmd_vel_publisher.publish(stop_twist)

    def trigger_emergency_stop(self):
        """Trigger emergency stop externally"""
        msg = Bool()
        msg.data = True
        self.emergency_stop_publisher.publish(msg)
        self.get_logger().warn('External emergency stop triggered!')


def main(args=None):
    rclpy.init(args=args)
    monitor = EmergencyStopMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
