"""
test/test_dispatch_end_to_end.py
---------------------------------
End-to-end test: a plan from /lingua/ground must actually run on a node.

Starts the GroundingNode (mock backend with a fixed plan) and the
DispatcherNode, then runs a LinguaActionServer test robot inside this
process so the test can see exactly what the dispatcher sent it.

The fixed plan is [e2e_fetch]. e2e_fetch needs e2e_ready, which only
e2e_prepare produces, so the auto-chainer grounds it as
[e2e_prepare, e2e_fetch] until e2e_ready is in the symbolic state.
Grounding the same instruction again therefore shows whether the
dispatcher updated the state:

  1. No server running -> the plan ends FAILED and the state is unchanged.
  2. Server running    -> both steps run with the planned parameters,
                          the plan ends COMPLETED and e2e_ready is set.

Run with:
    colcon test --packages-select ros2_lingua
    colcon test-result --verbose
"""

import json
import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros2_lingua.lingua_action_server import LinguaActionServer
from std_msgs.msg import String

from ros2_lingua_interfaces.msg import ExecutionStatus
from ros2_lingua_interfaces.srv import GroundInstruction, RegisterCapability

ROBOT_NODE_NAME = "e2e_test_robot"
INSTRUCTION = "fetch the cup"
TERMINAL_STATUSES = ("COMPLETED", "FAILED", "CANCELLED")

PREPARE_CAPABILITY = {
    "name": "e2e_prepare",
    "description": "Gets the test robot ready",
    "ros_action": f"{ROBOT_NODE_NAME}/lingua",
    "ros_service": None,
    "parameters": [],
    "preconditions": [],
    "postconditions": ["e2e_ready"],
    "metadata": {},
    "tags": ["test"],
}

FETCH_CAPABILITY = {
    "name": "e2e_fetch",
    "description": "Fetches a named object",
    "ros_action": f"{ROBOT_NODE_NAME}/lingua",
    "ros_service": None,
    "parameters": [{
        "name": "object_name",
        "type": "string",
        "description": "Object to fetch",
        "required": True,
        "default": None,
    }],
    "preconditions": ["e2e_ready"],
    "postconditions": ["e2e_holding"],
    "metadata": {},
    "tags": ["test"],
}

FIXED_PLAN = json.dumps({
    "feasible": True,
    "steps": [{"capability_name": "e2e_fetch", "parameters": {"object_name": "cup"}}],
    "reason": "test plan",
})


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
            "llm_backend": "mock",
            "mock_response": FIXED_PLAN,
            "auto_chain": True,
        }],
        output="screen",
    )

    dispatcher_node = launch_ros.actions.Node(
        package="ros2_lingua",
        executable="dispatcher_node",
        name="lingua_dispatcher_node",
        parameters=[{"allow_demo_mode": False}],
        output="screen",
    )

    return (
        launch.LaunchDescription([
            grounding_node,
            dispatcher_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            "grounding_node": grounding_node,
            "dispatcher_node": dispatcher_node,
        },
    )


# ------------------------------------------------------------------
# Test nodes
# ------------------------------------------------------------------

class E2ETestRobot(LinguaActionServer, Node):
    """Records every capability call it executes."""

    def __init__(self):
        Node.__init__(self, ROBOT_NODE_NAME)
        LinguaActionServer.__init__(self, ["e2e_prepare", "e2e_fetch"])
        self.calls = []

    async def execute_capability(self, capability_name, parameters, goal_handle):
        self.calls.append((capability_name, parameters))
        if capability_name == "e2e_prepare":
            return True, "ready", ["e2e_ready"], []
        return True, f"fetched {parameters.get('object_name')}", ["e2e_holding"], []


class E2EClient(Node):
    """Registers capabilities, grounds instructions and records execution status."""

    def __init__(self):
        super().__init__("e2e_test_client")
        self._lock = threading.Lock()
        self._statuses = []
        self._capability_names = set()

        self.ground_client = self.create_client(GroundInstruction, "/lingua/ground")
        self.register_client = self.create_client(
            RegisterCapability, "/lingua/register_capability"
        )
        self.create_subscription(
            ExecutionStatus, "/lingua/execution_status", self._handle_status, 10
        )
        self.create_subscription(String, "/lingua/capabilities", self._handle_caps, 10)

    def _handle_status(self, msg):
        with self._lock:
            self._statuses.append(msg.status)

    def _handle_caps(self, msg):
        with self._lock:
            self._capability_names = {c["name"] for c in json.loads(msg.data)}

    def status_count(self):
        with self._lock:
            return len(self._statuses)

    def capability_names(self):
        with self._lock:
            return set(self._capability_names)

    def wait_for_terminal_status(self, since, timeout):
        """Return the first terminal status published after index `since`, or None."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                for status in self._statuses[since:]:
                    if status in TERMINAL_STATUSES:
                        return status
            time.sleep(0.05)
        return None


def _wait_for_future(future, timeout):
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    return future.result() if future.done() else None


# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------

class TestDispatchEndToEnd(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        cls.client = E2EClient()
        cls.robot = None
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.client)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

        assert cls.client.ground_client.wait_for_service(timeout_sec=15.0), \
            "lingua/ground not available"
        assert cls.client.register_client.wait_for_service(timeout_sec=15.0), \
            "lingua/register_capability not available"

        for capability in (PREPARE_CAPABILITY, FETCH_CAPABILITY):
            request = RegisterCapability.Request()
            request.capability_json = json.dumps(capability)
            result = _wait_for_future(cls.client.register_client.call_async(request), 5.0)
            assert result is not None and result.success, f"could not register {capability['name']}"

        # The dispatcher learns ros_action names from the same broadcast
        deadline = time.monotonic() + 10.0
        while {"e2e_prepare", "e2e_fetch"} - cls.client.capability_names():
            assert time.monotonic() < deadline, "capabilities were never broadcast"
            time.sleep(0.1)
        time.sleep(1.0)

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=3.0)
        cls.spin_thread.join(timeout=3.0)
        cls.client.destroy_node()
        if cls.robot is not None:
            cls.robot.destroy_node()
        rclpy.shutdown()

    def _ground_and_wait(self, timeout=30.0):
        """Ground INSTRUCTION, wait for the resulting run to finish, return (step names, status)."""
        since = self.client.status_count()
        request = GroundInstruction.Request()
        request.instruction = INSTRUCTION
        result = _wait_for_future(self.client.ground_client.call_async(request), 10.0)
        self.assertIsNotNone(result, "lingua/ground did not respond")
        self.assertTrue(result.success, f"grounding failed: {result.message}")
        steps = [s["capability_name"] for s in json.loads(result.plan_json)["steps"]]
        return steps, self.client.wait_for_terminal_status(since, timeout)

    def test_01_plan_fails_when_no_server_is_running(self):
        """With no action server, the plan ends FAILED and no state is set."""
        steps, status = self._ground_and_wait()
        self.assertEqual(steps, ["e2e_prepare", "e2e_fetch"])
        self.assertEqual(status, "FAILED")

        # e2e_ready was not set, so the chainer still has to insert e2e_prepare
        steps, status = self._ground_and_wait()
        self.assertEqual(steps, ["e2e_prepare", "e2e_fetch"])
        self.assertEqual(status, "FAILED")

    def test_02_plan_runs_on_the_node_and_updates_state(self):
        """With the action server up, each step runs with its planned parameters."""
        type(self).robot = E2ETestRobot()
        self.executor.add_node(self.robot)

        steps, status = self._ground_and_wait()
        self.assertEqual(steps, ["e2e_prepare", "e2e_fetch"])
        self.assertEqual(status, "COMPLETED")
        self.assertEqual(
            self.robot.calls,
            [("e2e_prepare", {}), ("e2e_fetch", {"object_name": "cup"})],
        )

        # e2e_ready is now in the state, so the chainer no longer inserts e2e_prepare
        steps, status = self._ground_and_wait()
        self.assertEqual(steps, ["e2e_fetch"])
        self.assertEqual(status, "COMPLETED")
        self.assertEqual(self.robot.calls[-1], ("e2e_fetch", {"object_name": "cup"}))


@launch_testing.post_shutdown_test()
class TestDispatchEndToEndShutdown(unittest.TestCase):
    def test_nodes_exited_cleanly(self, proc_info, grounding_node, dispatcher_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            grounding_node,
        )
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [launch_testing.asserts.EXIT_OK],
            dispatcher_node,
        )
