"""
test/test_dispatcher_state.py
------------------------------
The dispatcher's view of the symbolic state during a plan (issue #35).

Runs a DispatcherNode in this process with a recovery planner that records
what it is given, a LinguaActionServer test robot, and a stand-in for the
grounding node's lingua/update_state service.

The plan carries a state snapshot. Step 1 succeeds and its result clears a
token; step 2 fails and its result clears another. When recovery runs, the
dispatcher must hand it the snapshot plus every change the steps reported,
not just the declared postconditions starting from an empty set.

Run with:
    colcon test --packages-select ros2_lingua
    colcon test-result --verbose
"""

import json
import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ros2_lingua.dispatcher_node import DispatcherNode
from ros2_lingua.lingua_action_server import LinguaActionServer
from ros2_lingua_core import RecoveryConfig, RecoveryPlanner
from ros2_lingua_interfaces.msg import ExecutionStatus
from ros2_lingua_interfaces.srv import UpdateState

ROBOT_NODE_NAME = "state_test_robot"
TERMINAL_STATUSES = ("COMPLETED", "FAILED", "CANCELLED", "RECOVERY_FAILED")


def _capability(name, postconditions):
    return {
        "name": name,
        "description": "test capability",
        "ros_action": f"{ROBOT_NODE_NAME}/lingua",
        "ros_service": None,
        "parameters": [],
        "preconditions": [],
        "postconditions": postconditions,
        "metadata": {},
        "tags": ["test"],
    }


CAPABILITIES = [_capability("open_door", ["door_open"]), _capability("lift", ["lifted"])]

PLAN = {
    "original_instruction": "open the door and lift",
    "feasible": True,
    "reason": "",
    "steps": [
        {"capability_name": "open_door", "parameters": {}, "rationale": ""},
        {"capability_name": "lift", "parameters": {}, "rationale": ""},
    ],
    "state": ["battery_ok", "door_closed", "map_loaded"],
}


class StateTestRobot(LinguaActionServer, Node):
    def __init__(self):
        Node.__init__(self, ROBOT_NODE_NAME)
        LinguaActionServer.__init__(self, ["open_door", "lift"])

    async def execute_capability(self, capability_name, parameters, goal_handle):
        if capability_name == "open_door":
            return True, "opened", ["door_open"], ["door_closed"]
        # The lift fails and reports that it drained the battery
        return False, "motor stalled", [], ["battery_ok"]


class RecordingPlanner(RecoveryPlanner):
    """Records what the dispatcher passes in, then aborts."""

    def __init__(self):
        super().__init__(RecoveryConfig(max_retries=0, enable_replan=False))
        self.calls = []

    def on_step_failed(self, *args, **kwargs):
        self.calls.append(kwargs)
        return super().on_step_failed(*args, **kwargs)


class Harness(Node):
    """Publishes capabilities and plans, stands in for lingua/update_state."""

    def __init__(self):
        super().__init__("state_test_harness")
        self._lock = threading.Lock()
        self.statuses = []
        self.state_updates = []
        self.caps_pub = self.create_publisher(String, "lingua/capabilities", 10)
        self.plan_pub = self.create_publisher(String, "lingua/current_plan", 10)
        self.create_subscription(ExecutionStatus, "lingua/execution_status", self._on_status, 10)
        self.create_service(UpdateState, "lingua/update_state", self._on_update_state)

    def _on_status(self, msg):
        with self._lock:
            self.statuses.append(msg.status)

    def _on_update_state(self, request, response):
        with self._lock:
            self.state_updates.append(json.loads(request.state_json))
        response.success = True
        return response

    def terminal_status(self):
        with self._lock:
            return next((s for s in self.statuses if s in TERMINAL_STATUSES), None)


class TestDispatcherState(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()
        cls.planner = RecordingPlanner()
        cls.dispatcher = DispatcherNode(recovery_planner=cls.planner)
        cls.robot = StateTestRobot()
        cls.harness = Harness()
        cls.executor = MultiThreadedExecutor()
        for node in (cls.dispatcher, cls.robot, cls.harness):
            cls.executor.add_node(node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown(timeout_sec=3.0)
        cls.spin_thread.join(timeout=3.0)
        for node in (cls.dispatcher, cls.robot, cls.harness):
            node.destroy_node()
        rclpy.shutdown()

    def _wait(self, condition, timeout, message):
        deadline = time.monotonic() + timeout
        while not condition():
            self.assertLess(time.monotonic(), deadline, message)
            time.sleep(0.05)

    def test_recovery_sees_snapshot_and_reported_changes(self):
        # The dispatcher learns ros_action names from the capability broadcast
        caps = String(data=json.dumps(CAPABILITIES))

        def capabilities_cached():
            self.harness.caps_pub.publish(caps)
            return "lift" in self.dispatcher._capability_map

        self._wait(capabilities_cached, 10.0, "dispatcher never cached the capabilities")
        self._wait(
            lambda: self.harness.plan_pub.get_subscription_count() > 0,
            10.0,
            "dispatcher is not subscribed to lingua/current_plan",
        )

        self.harness.plan_pub.publish(String(data=json.dumps(PLAN)))
        self._wait(
            lambda: self.harness.terminal_status() is not None,
            30.0,
            "plan never finished",
        )
        self.assertEqual(self.harness.terminal_status(), "RECOVERY_FAILED")

        self.assertEqual(len(self.planner.calls), 1)
        call = self.planner.calls[0]
        # Snapshot tokens are kept, minus what the steps cleared, plus what they set
        self.assertEqual(call["current_state"], {"map_loaded", "door_open"})
        self.assertEqual(call["state_clear"], {"door_closed", "battery_ok"})

        # Every reported change was also pushed to the grounding node
        pushed_clear = {t for u in self.harness.state_updates for t in u["clear"]}
        self.assertEqual(pushed_clear, {"door_closed", "battery_ok"})
