"""
ros2_lingua.dispatcher_node
----------------------------
Subscribes to /lingua/current_plan and executes each step in order
by calling the appropriate ROS 2 action or service.

This node is the bridge between the grounding world (JSON plans)
and the real ROS 2 action/service world (your robot's actual interfaces).
"""

import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from ros2_lingua_interfaces.msg import ExecutionStatus


class DispatcherNode(Node):
    """
    Executes ActionPlans produced by the GroundingNode.

    For each step in a plan:
    1. Look up the capability in the registry
    2. Determine if it's a ROS 2 action or service
    3. Call it with the resolved parameters
    4. Wait for completion
    5. Update the robot state (postconditions)
    6. Proceed to the next step (or abort on failure)

    Publishes execution status to /lingua/execution_status.
    """

    def __init__(self):
        super().__init__("lingua_dispatcher_node")

        self._callback_group = ReentrantCallbackGroup()

        # Subscribe to plans from the GroundingNode
        self._plan_sub = self.create_subscription(
            String,
            "/lingua/current_plan",
            self._handle_plan,
            10,
            callback_group=self._callback_group,
        )

        # Subscribe to capability registry broadcast
        self._caps_sub = self.create_subscription(
            String,
            "/lingua/capabilities",
            self._handle_capabilities,
            10,
        )

        # Publish execution status
        self._status_pub = self.create_publisher(
            ExecutionStatus,
            "/lingua/execution_status",
            10,
        )

        # Cache of capability name -> capability dict (from registry broadcast)
        self._capability_map = {}

        # Cache of action clients (created lazily per action name)
        self._action_clients = {}

        self.get_logger().info("DispatcherNode ready.")

    def _handle_capabilities(self, msg: String):
        """Update local capability cache from GroundingNode broadcast."""
        try:
            caps = json.loads(msg.data)
            self._capability_map = {c["name"]: c for c in caps}
        except Exception as e:
            self.get_logger().error(f"Failed to parse capabilities broadcast: {e}")

    def _handle_plan(self, msg: String):
        """Receive and execute an ActionPlan."""
        try:
            plan_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid plan JSON: {e}")
            return

        steps = plan_data.get("steps", [])
        instruction = plan_data.get("original_instruction", "")
        self.get_logger().info(
            f"Executing plan for: '{instruction}' ({len(steps)} steps)"
        )

        self._publish_status("STARTED", instruction=instruction)

        for i, step in enumerate(steps):
            cap_name = step.get("capability_name")
            params = step.get("parameters", {})
            self.get_logger().info(f"Step {i+1}/{len(steps)}: {cap_name}")

            success = self._execute_step(cap_name, params)
            if not success:
                self.get_logger().error(f"Step '{cap_name}' failed. Aborting plan.")
                self._publish_status("FAILED", step=cap_name, instruction=instruction)
                return

            self._publish_status("STEP_COMPLETE", step=cap_name, instruction=instruction)

        self._publish_status("COMPLETED", instruction=instruction)
        self.get_logger().info("Plan executed successfully.")

    def _execute_step(self, capability_name: str, parameters: dict) -> bool:
        """
        Execute a single step. Returns True on success.

        This method routes to either _call_action or _call_service
        based on the capability definition.
        """
        cap = self._capability_map.get(capability_name)
        if cap is None:
            self.get_logger().error(
                f"Cannot execute '{capability_name}': not found in capability cache. "
                "Make sure the GroundingNode has broadcast capabilities."
            )
            return False

        ros_action = cap.get("ros_action")
        ros_service = cap.get("ros_service")

        if ros_action:
            return self._call_action(ros_action, capability_name, parameters)
        elif ros_service:
            return self._call_service(ros_service, capability_name, parameters)
        else:
            self.get_logger().error(
                f"Capability '{capability_name}' has no ros_action or ros_service defined."
            )
            return False

    def _call_action(self, action_name: str, cap_name: str, params: dict) -> bool:
        """
        Call a ROS 2 action with the given parameters.

        NOTE: Action message types vary per capability. This implementation
        uses a generic string-based goal (JSON params) which your action
        server must be built to accept, OR you override this method in a
        subclass to handle specific message types.

        For the humanoid demo, we use a simple JSON-goal protocol.
        """
        self.get_logger().info(
            f"  -> Calling action '{action_name}' with params: {params}"
        )

        # In production, create a typed action client here.
        # For the demo, we log the call and return success.
        # Users override this in their robot-specific subclass.
        self.get_logger().warn(
            f"  [DEMO MODE] Action '{action_name}' called. "
            "Subclass DispatcherNode._call_action() for real execution."
        )
        return True

    def _call_service(self, service_name: str, cap_name: str, params: dict) -> bool:
        """
        Call a ROS 2 service with the given parameters.
        Same protocol as _call_action — override for real execution.
        """
        self.get_logger().info(
            f"  -> Calling service '{service_name}' with params: {params}"
        )
        self.get_logger().warn(
            f"  [DEMO MODE] Service '{service_name}' called. "
            "Subclass DispatcherNode._call_service() for real execution."
        )
        return True

    def _publish_status(self, status: str, step: str = "", instruction: str = ""):
        msg = ExecutionStatus()
        msg.status = status
        msg.current_step = step
        msg.instruction = instruction
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
