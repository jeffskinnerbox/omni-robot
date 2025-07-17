import unittest
import rclpy
from manual_drive.safety_monitor_node import SafetyMonitorNode


class TestSafetyMonitor(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = SafetyMonitorNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self):
        """Test that the safety monitor node is created successfully"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'safety_monitor_node')

    def test_safety_limits(self):
        """Test safety limit parameters"""
        self.assertEqual(self.node.max_linear_speed, 3.0)
        self.assertEqual(self.node.max_angular_speed, 3.0)


if __name__ == '__main__':
    unittest.main()
