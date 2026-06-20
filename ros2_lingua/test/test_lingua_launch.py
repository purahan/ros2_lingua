"""
test/test_lingua_launch.py
---------------------------
Integration tests to ensure that the canonical lingua.launch.py works
and that the system comes up and communicates correctly over namespaced topics.
"""

import os
import unittest
import time

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ros2_lingua_interfaces.srv import RegisterCapability


@pytest.mark.launch_test
def generate_test_description():
    """Launch the canonical lingua.launch.py file."""
    
    launch_file_dir = os.path.join(
        get_package_share_directory("ros2_lingua"), "launch"
    )

    lingua_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "lingua.launch.py")
        ),
        launch_arguments={
            "llm_backend": "mock",
            "robot_namespace": "test_ns",
        }.items()
    )

    return launch.LaunchDescription([
        lingua_launch,
        launch_testing.actions.ReadyToTest()
    ]), {
        "lingua_launch": lingua_launch,
    }


class TestLinguaLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize ROS 2 context
        rclpy.init()

        # Create a test node to interact with the namespaced system
        cls.node = rclpy.create_node("test_lingua_launch_client")

        # The namespace is 'test_ns', so the relative topics from the launch
        # file become '/test_ns/lingua/...'
        cls.register_client = cls.node.create_client(
            RegisterCapability, "/test_ns/lingua/register_capability"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_services_available(self):
        """Wait for the namespaced services to appear."""
        assert self.register_client.wait_for_service(timeout_sec=10.0), \
            "RegisterCapability service not available under namespace"


@launch_testing.post_shutdown_test()
class TestLinguaLaunchShutdown(unittest.TestCase):
    def test_node_exit_code(self, proc_info):
        """Check that all launched processes exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, 130]
        )
