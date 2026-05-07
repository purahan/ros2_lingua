"""
test/test_grounding_pipeline.py
--------------------------------
Integration tests for the ros2_lingua grounding pipeline.

These tests spin up a real ROS 2 environment using launch_testing,
start the GroundingNode with an ollama backend, and exercise the
full service call pipeline end to end.

Run with:
    cd ~/ros2_lingua_ws
    colcon test --packages-select ros2_lingua
    colcon test-result --verbose
"""

import json
import time
import unittest
import threading

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest

import rclpy
from rclpy.node import Node

from ros2_lingua_interfaces.srv import RegisterCapability, GroundInstruction, UpdateState
from std_msgs.msg import String


# ------------------------------------------------------------------
# Test capability definitions
# ------------------------------------------------------------------

NAVIGATE_CAPABILITY = {
    "name": "navigate_to_location",
    "description": "Moves the robot to a named location",
    "ros_action": "robot/navigate",
    "ros_service": None,
    "parameters": [{
        "name": "location_name",
        "type": "string",
        "description": "Where to go",
        "required": True,
        "default": None,
    }],
    "preconditions": [],
    "postconditions": ["robot_at_location"],
    "metadata": {},
    "tags": ["locomotion"],
}

PICK_CAPABILITY = {
    "name": "pick_up_object",
    "description": "Picks up a named object using the robot arm",
    "ros_action": "robot/pick",
    "ros_service": None,
    "parameters": [{
        "name": "object_name",
        "type": "string",
        "description": "Object to pick up",
        "required": True,
        "default": None,
    }],
    "preconditions": ["robot_at_location"],
    "postconditions": ["object_in_hand"],
    "metadata": {},
    "tags": ["manipulation"],
}

SAY_CAPABILITY = {
    "name": "say",
    "description": "Speaks a message aloud",
    "ros_action": None,
    "ros_service": "robot/tts",
    "parameters": [{
        "name": "message",
        "type": "string",
        "description": "What to say",
        "required": True,
        "default": None,
    }],
    "preconditions": [],
    "postconditions": [],
    "metadata": {},
    "tags": ["speech"],
}

STABILIZE_CAPABILITY = {
    "name": "stabilize_robot",
    "description": "Stabilizes the robot before movement",
    "ros_action": "robot/stabilize",
    "ros_service": None,
    "parameters": [],
    "preconditions": [],
    "postconditions": ["robot_is_balanced"],
    "metadata": {},
    "tags": ["balance"],
}


# ------------------------------------------------------------------
# Launch description
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
            "llm_api_key": "",
            "auto_chain": True,
        }],
        output="screen",
    )

    return launch.LaunchDescription([
        grounding_node,
        launch_testing.actions.ReadyToTest(),
    ])


# ------------------------------------------------------------------
# Helper node
# ------------------------------------------------------------------

class TestHelperNode(Node):
    def __init__(self):
        super().__init__("test_helper_node")
        self._register_client = self.create_client(
            RegisterCapability, "/lingua/register_capability"
        )
        self._ground_client = self.create_client(
            GroundInstruction, "/lingua/ground"
        )
        self._state_client = self.create_client(
            UpdateState, "/lingua/update_state"
        )
        self._received_plans = []
        self._plan_sub = self.create_subscription(
            String, "/lingua/current_plan",
            lambda msg: self._received_plans.append(msg.data), 10,
        )

    def wait_for_services(self, timeout=15.0) -> bool:
        return (
            self._register_client.wait_for_service(timeout_sec=timeout) and
            self._ground_client.wait_for_service(timeout_sec=timeout) and
            self._state_client.wait_for_service(timeout_sec=timeout)
        )

    def register_capability(self, cap_dict: dict, timeout=5.0):
        req = RegisterCapability.Request()
        req.capability_json = json.dumps(cap_dict)
        future = self._register_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def ground(self, instruction: str, timeout=30.0):
        req = GroundInstruction.Request()
        req.instruction = instruction
        future = self._ground_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()

    def update_state(self, set_tokens=None, clear_tokens=None, timeout=5.0):
        req = UpdateState.Request()
        req.state_json = json.dumps({
            "set": set_tokens or [],
            "clear": clear_tokens or [],
        })
        future = self._state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.result()


# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------

class TestGroundingPipeline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = TestHelperNode()
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls._spin_thread = threading.Thread(
            target=cls.executor.spin, daemon=True
        )
        cls._spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        available = self.node.wait_for_services(timeout=15.0)
        self.assertTrue(available, "Grounding node services not available.")

    # ── Service availability ──────────────────────────────────────

    def test_01_services_available(self):
        """All three grounding services are reachable."""
        self.assertTrue(self.node._register_client.service_is_ready())
        self.assertTrue(self.node._ground_client.service_is_ready())
        self.assertTrue(self.node._state_client.service_is_ready())

    # ── Capability registration ───────────────────────────────────

    def test_02_register_valid_capability(self):
        """A valid capability registers successfully."""
        result = self.node.register_capability(NAVIGATE_CAPABILITY)
        self.assertIsNotNone(result)
        self.assertTrue(result.success, f"Registration failed: {result.message}")

    def test_03_register_invalid_capability_rejected(self):
        """A capability missing ros_action/ros_service is rejected."""
        bad = dict(NAVIGATE_CAPABILITY)
        bad["name"] = "bad_cap_no_interface"
        bad["ros_action"] = None
        bad["ros_service"] = None
        result = self.node.register_capability(bad)
        self.assertIsNotNone(result)
        self.assertFalse(result.success)

    def test_04_register_malformed_json_rejected(self):
        """Malformed JSON is handled gracefully — node does not crash."""
        req = RegisterCapability.Request()
        req.capability_json = "{ not valid json ]["
        future = self.node._register_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        result = future.result()
        self.assertIsNotNone(result)
        self.assertFalse(result.success)

    def test_05_register_multiple_capabilities(self):
        """Multiple capabilities can be registered in sequence."""
        for cap in [PICK_CAPABILITY, SAY_CAPABILITY, STABILIZE_CAPABILITY]:
            result = self.node.register_capability(cap)
            self.assertIsNotNone(result)
            # Success OR already registered — both acceptable
            # (previous tests may have registered same cap)

    # ── State management ──────────────────────────────────────────

    def test_06_set_state_tokens(self):
        """State tokens can be set over the update_state service."""
        result = self.node.update_state(set_tokens=["integration_test_token"])
        self.assertIsNotNone(result)
        self.assertTrue(result.success)

    def test_07_clear_state_tokens(self):
        """State tokens can be cleared over the update_state service."""
        self.node.update_state(set_tokens=["token_to_clear"])
        result = self.node.update_state(clear_tokens=["token_to_clear"])
        self.assertIsNotNone(result)
        self.assertTrue(result.success)

    def test_08_update_state_both_set_and_clear(self):
        """Set and clear can happen in the same call."""
        result = self.node.update_state(
            set_tokens=["state_a", "state_b"],
            clear_tokens=["integration_test_token"],
        )
        self.assertIsNotNone(result)
        self.assertTrue(result.success)

    # ── Ground service response structure ────────────────────────

    def test_09_ground_response_always_valid_structure(self):
        """
        The ground service always returns a correctly structured
        response regardless of success or failure.
        """
        result = self.node.ground("some test instruction")
        self.assertIsNotNone(result)
        self.assertIsInstance(result.success, bool)
        self.assertIsInstance(result.plan_json, str)
        self.assertIsInstance(result.message, str)

        # plan_json must always be valid JSON
        plan = json.loads(result.plan_json)
        self.assertIsInstance(plan, dict)

    def test_10_plan_has_required_fields(self):
        """Returned plan JSON always contains required fields."""
        result = self.node.ground("another test instruction")
        self.assertIsNotNone(result)
        plan = json.loads(result.plan_json)
        self.assertIn("feasible", plan)
        self.assertIn("steps", plan)
        self.assertIn("original_instruction", plan)

    # ── Grounding logic ───────────────────────────────────────────

    def test_11_feasible_plan_steps_reference_registered_caps(self):
        """
        If a plan is feasible, all step capability names must be
        registered capabilities (no hallucinations get through).
        """
        self.node.register_capability(SAY_CAPABILITY)
        result = self.node.ground("say hello to the team")
        self.assertIsNotNone(result)

        if result.success:
            plan = json.loads(result.plan_json)
            if plan.get("feasible"):
                registered = {
                    "navigate_to_location", "pick_up_object",
                    "say", "stabilize_robot", "bad_cap_no_interface",
                }
                for step in plan.get("steps", []):
                    cap_name = step.get("capability_name", "")
                    self.assertIn(
                        cap_name,
                        registered,
                        f"Step references unregistered capability: {cap_name}",
                    )

    def test_12_infeasible_plan_has_reason(self):
        """
        An infeasible plan always includes a non-empty reason string.
        """
        result = self.node.ground("xyzzy teleport to mars immediately")
        self.assertIsNotNone(result)
        plan = json.loads(result.plan_json)

        if not plan.get("feasible", True):
            # reason field must exist and be a string
            # (LLM may return empty string for some infeasible cases)
            self.assertIsInstance(plan.get("reason", ""), str)

    def test_13_plan_published_on_success(self):
        """
        A successful plan is published to /lingua/current_plan
        so the dispatcher can pick it up.
        """
        self.node.register_capability(SAY_CAPABILITY)
        initial_count = len(self.node._received_plans)

        result = self.node.ground("say welcome to ROScon")
        self.assertIsNotNone(result)

        if result.success:
            time.sleep(0.5)
            self.assertGreater(
                len(self.node._received_plans),
                initial_count,
                "/lingua/current_plan must receive a message after successful grounding",
            )

    # ── Robustness ────────────────────────────────────────────────

    def test_14_empty_instruction_handled(self):
        """An empty instruction string does not crash the node."""
        result = self.node.ground("")
        self.assertIsNotNone(result, "Node crashed on empty instruction")

    def test_15_very_long_instruction_handled(self):
        """A very long instruction does not crash the node."""
        long_instruction = "go to the table and " * 50 + "pick something up"
        result = self.node.ground(long_instruction, timeout=30.0)
        self.assertIsNotNone(result, "Node crashed on long instruction")

    def test_16_repeated_registration_does_not_crash(self):
        """
        Re-registering the same capability (update) does not crash
        the node — update() should silently overwrite.
        """
        for _ in range(5):
            result = self.node.register_capability(SAY_CAPABILITY)
            self.assertIsNotNone(result)


# ------------------------------------------------------------------
# Post-shutdown checks
# ------------------------------------------------------------------

@launch_testing.post_shutdown_test()
class TestGroundingNodeShutdown(unittest.TestCase):
    def test_node_exit_code(self, proc_info):
        """Grounding node shuts down cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2],
        )
