import unittest
import rclpy
from manual_drive.emergency_stop_node import EmergencyStopNode


class TestEmergencyStop(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = EmergencyStopNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_emergency_stop_activation(self):
        """Test emergency stop activation"""
        initial_state = self.node.emergency_active
        self.node.activate_emergency_stop()
        self.assertTrue(self.node.emergency_active)
        self.assertNotEqual(initial_state, self.node.emergency_active)

    def test_emergency_stop_reset(self):
        """Test emergency stop reset"""
        self.node.activate_emergency_stop()
        self.node.reset_emergency_stop()
        self.assertFalse(self.node.emergency_active)


if __name__ == '__main__':
    unittest.main()
