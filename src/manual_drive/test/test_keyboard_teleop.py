import unittest
import rclpy
from manual_drive.keyboard_teleop_node import KeyboardTeleopNode


class TestKeyboardTeleop(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = KeyboardTeleopNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_node_creation(self):
        """Test that the node is created successfully"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'keyboard_teleop_node')

    def test_parameters(self):
        """Test default parameters"""
        self.assertEqual(self.node.linear_speed, 1.0)
        self.assertEqual(self.node.angular_speed, 1.0)


if __name__ == '__main__':
    unittest.main()
