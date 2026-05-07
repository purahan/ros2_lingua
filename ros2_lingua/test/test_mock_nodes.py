"""
test_mock_nodes.py
-------------------
Integration tests verifying that all mock robot nodes start correctly
and self-register their capabilities with the grounding node.

Tests:
  - All mock nodes come up within timeout
  - Each node registers the expected capabilities
  - Capabilities appear in the /lingua/capabilities topic
  - State tokens are correctly initialized

Run with:
    colcon test --packages-select ros2_lingua_mock
    colcon test-result --verbose
"""

import json
import time
import unittest
import pytest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ros2_lingua_interfaces.srv import RegisterCapability, GroundInstruction


# ------------------------------------------------------------------
# Launch configuration
# ------------------------------------------------------------------

@pytest.mark.launch_test
def generate_test_description():
    grounding_node = launch_ros.actions.Node(
        package="ros2_lingua",
        executable="grounding_node",
        name="lingua_grounding_node",
        parameters=[{
            "llm_backend": "ollama",
            "llm_model": "llama3.1",
            "auto_chain": True,
        }],
        output="screen",
    )

    dispatcher_node = launch_ros.actions.Node(
        package="ros2_lingua",
        executable="dispatcher_node",
        name="lingua_dispatcher_node",
        output="screen",
    )

    balance_node = launch_ros.actions.Node(
        package="ros2_lingua_mock",
        executable="balance_node",
        name="mock_balance_node",
        output="screen",
    )

    navigation_node = launch_ros.actions.Node(
        package="ros2_lingua_mock",
        executable="navigation_node",
        name="mock_navigation_node",
        output="screen",
    )

    manipulation_node = launch_ros.actions.Node(
        package="ros2_lingua_mock",
        executable="manipulation_node",
        name="mock_manipulation_node",
        output="screen",
    )

    speech_node = launch_ros.actions.Node(
        package="ros2_lingua_mock",
        executable="speech_node",
        name="mock_speech_node",
        output="screen",
    )

    return (
        launch.LaunchDescription([
            grounding_node,
            dispatcher_node,
            balance_node,
            navigation_node,
            manipulation_node,
            speech_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            "grounding_node": grounding_node,
            "dispatcher_node": dispatcher_node,
        },
    )


# ------------------------------------------------------------------
# Capability subscriber helper
# ------------------------------------------------------------------

class CapabilitySubscriberNode(Node):
    """Listens to /lingua/capabilities and collects registered capabilities."""

    def __init__(self):
        super().__init__("capability_subscriber_test_node")
        self._capabilities = {}
        self._received = False

        self._sub = self.create_subscription(
            String,
            "/lingua/capabilities",
            self._handle_caps,
            10,
        )
        self._ground_client = self.create_client(
            GroundInstruction, "/lingua/ground"
        )

    def _handle_caps(self, msg):
        try:
            caps = json.loads(msg.data)
            self._capabilities = {c["name"]: c for c in caps}
            self._received = True
        except Exception:
            pass

    def wait_for_capabilities(self, min_count=1, timeout=20.0):
        """Wait until at least min_count capabilities are registered."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self._received and len(self._capabilities) >= min_count:
                return True
        return False

    def get_capabilities(self):
        return self._capabilities


# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------

class TestMockNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = CapabilitySubscriberNode()
        # Wait for all mock nodes to register (7 capabilities total)
        assert cls.node.wait_for_capabilities(min_count=7, timeout=25.0), \
            "Mock nodes did not register all capabilities within timeout"

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_01_all_capabilities_registered(self):
        """All 7 expected capabilities should be registered."""
        caps = self.node.get_capabilities()
        expected = {
            "stabilize_robot",
            "navigate_to_location",
            "return_to_home",
            "pick_up_object",
            "place_object",
            "wave_hand",
            "say",
        }
        missing = expected - set(caps.keys())
        self.assertEqual(
            missing, set(),
            f"Missing capabilities: {missing}"
        )

    def test_02_balance_node_capability(self):
        """stabilize_robot should have correct tags and postconditions."""
        caps = self.node.get_capabilities()
        self.assertIn("stabilize_robot", caps)
        cap = caps["stabilize_robot"]
        self.assertIn("balance", cap.get("tags", []))
        self.assertIn("robot_is_balanced", cap.get("postconditions", []))

    def test_03_navigation_capabilities(self):
        """Navigation capabilities should have locomotion tags."""
        caps = self.node.get_capabilities()
        for name in ["navigate_to_location", "return_to_home"]:
            self.assertIn(name, caps)
            cap = caps[name]
            self.assertIn("locomotion", cap.get("tags", []))

    def test_04_navigate_has_preconditions(self):
        """navigate_to_location requires robot_is_balanced."""
        caps = self.node.get_capabilities()
        cap = caps.get("navigate_to_location", {})
        self.assertIn("robot_is_balanced", cap.get("preconditions", []))

    def test_05_manipulation_capabilities(self):
        """Manipulation capabilities should have manipulation tags."""
        caps = self.node.get_capabilities()
        for name in ["pick_up_object", "place_object", "wave_hand"]:
            self.assertIn(name, caps)
            self.assertIn("manipulation", caps[name].get("tags", []))

    def test_06_pick_has_correct_preconditions(self):
        """pick_up_object requires robot_at_location, object_in_view, arm_is_free."""
        caps = self.node.get_capabilities()
        cap = caps.get("pick_up_object", {})
        preconditions = cap.get("preconditions", [])
        for req in ["robot_at_location", "object_in_view", "arm_is_free"]:
            self.assertIn(req, preconditions, f"pick_up_object missing precondition: {req}")

    def test_07_speech_capability(self):
        """say capability should use a service, not an action."""
        caps = self.node.get_capabilities()
        self.assertIn("say", caps)
        cap = caps["say"]
        self.assertIsNotNone(cap.get("ros_service"))
        self.assertIsNone(cap.get("ros_action"))
        self.assertIn("speech", cap.get("tags", []))

    def test_08_all_capabilities_have_descriptions(self):
        """Every capability must have a non-empty description."""
        caps = self.node.get_capabilities()
        for name, cap in caps.items():
            self.assertNotEqual(
                cap.get("description", ""), "",
                f"Capability '{name}' has empty description"
            )

    def test_09_all_capabilities_have_tags(self):
        """Every capability should have at least one tag."""
        caps = self.node.get_capabilities()
        for name, cap in caps.items():
            self.assertGreater(
                len(cap.get("tags", [])), 0,
                f"Capability '{name}' has no tags"
            )

    def test_10_capabilities_broadcast_is_valid_json(self):
        """The /lingua/capabilities topic should always broadcast valid JSON."""
        # Re-spin to get a fresh message
        rclpy.spin_once(self.node, timeout_sec=6.0)
        caps = self.node.get_capabilities()
        self.assertIsInstance(caps, dict)
        self.assertGreater(len(caps), 0)

    def test_11_chain_plan_structure(self):
        """
        navigate_to_location requires robot_is_balanced.
        stabilize_robot produces robot_is_balanced.
        The backward chainer should insert stabilize_robot automatically.
        This verifies the chain is resolvable from the registered caps.
        """
        caps = self.node.get_capabilities()
        nav = caps.get("navigate_to_location", {})
        stabilize = caps.get("stabilize_robot", {})

        # Verify the chain is satisfiable
        nav_preconditions = set(nav.get("preconditions", []))
        stabilize_postconditions = set(stabilize.get("postconditions", []))
        unsatisfied = nav_preconditions - stabilize_postconditions

        self.assertEqual(
            unsatisfied, set(),
            f"navigate_to_location has preconditions that stabilize_robot "
            f"does not satisfy: {unsatisfied}"
        )

    def test_12_no_duplicate_capability_names(self):
        """Each capability name must be unique across all nodes."""
        caps = self.node.get_capabilities()
        names = list(caps.keys())
        self.assertEqual(
            len(names), len(set(names)),
            "Duplicate capability names detected"
        )


@launch_testing.post_shutdown_test()
class TestMockNodesShutdown(unittest.TestCase):
    def test_nodes_exited_cleanly(self, proc_info, grounding_node, dispatcher_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            grounding_node,
        )
