"""
ros2_lingua_mock.balance_node
------------------------------
Simulates a robot balance / stabilization controller.

Advertises the capability:
  - stabilize_robot: activates balance control and marks robot_is_balanced

Publishes robot balance status to /mock/balance_status so other nodes
can react to balance state changes.

Serves the generic LinguaAction interface at mock_balance_node/lingua
(see ros2_lingua.lingua_action_server).

In a real robot, this would wrap your actual balance controller
(e.g. a whole-body controller, a ZMP controller, etc.)
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from ros2_lingua.capability_mixin import LinguaMixin
from ros2_lingua.lingua_action_server import LinguaActionServer
from std_msgs.msg import Bool, String

from ros2_lingua_core import Capability


class MockBalanceNode(LinguaMixin, LinguaActionServer, Node):
    """
    Simulates a balance controller.

    On receiving a stabilize_robot goal:
    - Logs that stabilization is starting
    - Waits a realistic amount of time (simulating balance acquisition)
    - Returns robot_is_balanced in the result's state_set
    - Publishes balance status
    """

    CAPABILITIES = ["stabilize_robot"]

    def __init__(self):
        Node.__init__(self, "mock_balance_node")
        LinguaMixin.__init__(self)
        LinguaActionServer.__init__(self, self.CAPABILITIES)

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
            ros_action=f"{self.get_name()}/lingua",
            parameters=[],
            preconditions=[],
            postconditions=["robot_is_balanced"],
            metadata={"estimated_duration_sec": 2.0},
            tags=["balance"],
        ))

        self._log("MockBalanceNode ready. Robot starts UNSTABILIZED.")
        self.get_logger().info("MockBalanceNode ready.")

    def lingua_current_state(self):
        if self._is_balanced:
            return ["robot_is_balanced"], []
        return [], []

    async def execute_capability(self, capability_name, parameters, goal_handle):
        if capability_name == "stabilize_robot":
            return self._stabilize(goal_handle)
        return False, f"Unknown capability: {capability_name}", [], []

    def _stabilize(self, goal_handle):
        self.get_logger().info("Balance: starting stabilization sequence...")
        self._log("⚖️  Activating balance controller...")

        # Simulate balance acquisition (realistic timing)
        steps = 5
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                self._log("❌ Stabilization cancelled.")
                return False, "Stabilization cancelled", [], []
            time.sleep(0.4)
            goal_handle.publish_feedback(
                self._make_feedback("Stabilizing...", (i + 1) / steps)
            )
            self.get_logger().info(f"Balance: stabilizing... ({i + 1}/{steps})")

        # Mark balanced
        self._is_balanced = True
        self._publish_status()

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
    # Multi-threaded so cancel requests are handled while a goal is executing
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT may already have shut the context down


if __name__ == "__main__":
    main()
