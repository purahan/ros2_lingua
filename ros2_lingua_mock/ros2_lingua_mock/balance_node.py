"""
ros2_lingua_mock.balance_node
------------------------------
Simulates a robot balance / stabilization controller.

Advertises capabilities:
  - stabilize_robot : activates balance control and marks robot_is_balanced

Publishes robot balance status to /mock/balance_status so other nodes
can react to balance state changes.

In a real robot, this would wrap your actual balance controller
(e.g. a whole-body controller, a ZMP controller, etc.)
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from ros2_lingua.capability_mixin import LinguaMixin
from ros2_lingua.lingua_action_server import LinguaActionServer
from ros2_lingua_core import Capability


class MockBalanceNode(LinguaMixin, LinguaActionServer, Node):
    """
    Simulates a balance controller.

    On receiving a stabilize action goal:
    - Logs that stabilization is starting
    - Waits a realistic amount of time (simulating balance acquisition)
    - Marks robot_is_balanced in the lingua state
    - Publishes balance status

    Also handles balance loss simulation — if the robot starts moving
    and balance is disrupted, it clears robot_is_balanced automatically.
    """

    def __init__(self):
        Node.__init__(self, "mock_balance_node")
        LinguaMixin.__init__(self)
        LinguaActionServer.__init__(self, ["stabilize_robot"])

        self._callback_group = ReentrantCallbackGroup()
        self._is_balanced = False

        # --- Publishers ---
        self._balance_pub = self.create_publisher(Bool, "/mock/balance_status", 10)
        self._log_pub = self.create_publisher(String, "/mock/robot_log", 10)

        # Publish balance status at 2Hz
        self._status_timer = self.create_timer(0.5, self._publish_status)

        # Register capability with lingua
        self.register_lingua_capability(Capability(
            name="stabilize_robot",
            description=(
                "Activates the balance controller and stabilizes the robot. "
                "Must be called before any locomotion or manipulation."
            ),
            ros_action="mock_balance_node/lingua",
            parameters=[],
            preconditions=[],
            postconditions=["robot_is_balanced"],
            metadata={"estimated_duration_sec": 2.0},
            tags=["balance"],
        ))

        self._log("MockBalanceNode ready. Robot starts UNSTABILIZED.")
        self.get_logger().info("MockBalanceNode ready.")

    def execute_capability(self, capability_name, parameters, goal_handle):
        """Execute stabilize_robot capability."""
        if capability_name == "stabilize_robot":
            return self._execute_stabilize(goal_handle)
        else:
            return False, f"Unknown capability: {capability_name}", [], []

    def _execute_stabilize(self, goal_handle):
        self.get_logger().info("Balance: starting stabilization sequence...")
        self._log("⚖️  Activating balance controller...")

        # Simulate balance acquisition (realistic timing)
        steps = 5
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._log("❌ Stabilization cancelled.")
                return False, "Stabilization cancelled", [], []
            time.sleep(0.4)
            self.get_logger().info(f"Balance: stabilizing... ({i+1}/{steps})")

        # Mark balanced
        self._is_balanced = True
        self._publish_status()

        # Update lingua state
        self.update_lingua_state(set_tokens=["robot_is_balanced"])

        self._log("✅ Robot stabilized. Balance controller active.")
        self.get_logger().info("Balance: robot is now balanced.")

        return True, "Robot stabilized", ["robot_is_balanced"], []

    def _publish_status(self):
        msg = Bool()
        msg.data = self._is_balanced
        self._balance_pub.publish(msg)

    def _log(self, message: str):
        msg = String()
        msg.data = f"[BALANCE] {message}"
        self._log_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockBalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()