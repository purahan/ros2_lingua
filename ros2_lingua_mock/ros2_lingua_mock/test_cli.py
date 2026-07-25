# complete code
import unittest
import sys
import rclpy
from rclpy.utilities import remove_ros_args
from unittest.mock import patch
from ros2_lingua_mock.cli import main

class TestCli(unittest.TestCase):
    @patch('rclpy.create_node')
    def test_service_not_found(self, mock_create_node):
        # Remove ROS-specific arguments
        sys.argv = remove_ros_args(sys.argv)

        # Parse the remaining arguments
        parser = argparse.ArgumentParser(description='ros2_lingua_mock cli')
        parser.add_argument('--namespace', help='namespace for the service')
        args, _ = parser.parse_known_args()

        # Test that the service not found error is raised
        with self.assertRaises(rclpy.exceptions.ServiceException):
            main()

if __name__ == '__main__':
    unittest.main()