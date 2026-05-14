"""
ros2_lingua.lingua_action_server
----------------------------------
LinguaActionServer is the Level 1 integration path — the easiest way
to make a robot node fully compatible with ros2_lingua.

Instead of implementing a custom typed action, your node implements
the generic LinguaAction interface. The dispatcher calls it directly
with the capability name and JSON parameters. Your node unpacks them
and runs its logic.

This is the recommended approach for NEW robot nodes built alongside
ros2_lingua. For existing nodes with their own typed interfaces, use
DispatchConfig (Level 2) instead.

Usage:
    from ros2_lingua.lingua_action_server import LinguaActionServer
    from ros2_lingua import LinguaMixin
    from ros2_lingua_core import Capability, CapabilityParameter, Tags

    class MyNavigationNode(LinguaMixin, LinguaActionServer, Node):

        CAPABILITIES = ["navigate_to_location", "return_to_home"]

        def __init__(self):
            Node.__init__(self, "my_navigation_node")
            LinguaMixin.__init__(self)
            LinguaActionServer.__init__(self, self.CAPABILITIES)

            self.register_lingua_capability(Capability(
                name="navigate_to_location",
                description="Moves the robot to a named location",
                ros_action="my_navigation_node/lingua",   # <- the LinguaAction server
                parameters=[
                    CapabilityParameter("location_name", "string", "Where to go"),
                ],
                preconditions=["robot_is_ready"],
                postconditions=["robot_at_location"],
                tags=[Tags.LOCOMOTION],
            ))

        async def execute_capability(
            self,
            capability_name: str,
            parameters: dict,
            goal_handle,
        ) -> tuple[bool, str, list, list]:
            '''
            Execute a capability and return (success, message, state_set, state_clear).

            This is the ONE method you implement. No subclassing DispatcherNode needed.

            Args:
                capability_name: Which capability is being called
                parameters:      Dict of parameter values from the plan
                goal_handle:     ROS 2 action goal handle for feedback/cancellation

            Returns:
                (success, message, state_set_tokens, state_clear_tokens)
            '''
            if capability_name == "navigate_to_location":
                location = parameters.get("location_name", "unknown")
                speed    = float(parameters.get("speed", 0.5))

                # --- your actual navigation logic here ---
                goal_handle.publish_feedback(
                    self._make_feedback(f"Navigating to {location}...", 0.0)
                )
                # e.g. await self._nav2_client.send_goal_async(...)

                return True, f"Arrived at {location}", ["robot_at_location"], []

            return False, f"Unknown capability: {capability_name}", [], []
"""

import json
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup


class LinguaActionServer:
    """
    Base class for robot nodes that expose the generic LinguaAction interface.

    Inherit alongside Node and LinguaMixin. Override execute_capability()
    to handle each capability your node declares.

    The action server is automatically created at the topic:
        <node_name>/lingua

    You reference this in your Capability.ros_action field:
        ros_action="my_navigation_node/lingua"
    """

    def __init__(self, capability_names: List[str]):
        """
        Args:
            capability_names: List of capability names this server handles.
                              Used for logging and validation only.
        """
        self._lingua_capabilities = set(capability_names)
        self._lingua_callback_group = ReentrantCallbackGroup()

        # Import here to avoid hard dep at module level
        try:
            from ros2_lingua_interfaces.action import LinguaAction
            self._LinguaAction = LinguaAction
        except ImportError:
            raise ImportError(
                "ros2_lingua_interfaces not found. "
                "Did you build the workspace? Try: colcon build"
            )

        node_name = self.get_name()  # type: ignore
        self._lingua_action_server = ActionServer(
            self,                              # type: ignore
            LinguaAction,
            f"{node_name}/lingua",
            execute_callback=self._lingua_execute,
            goal_callback=self._lingua_goal_callback,
            cancel_callback=self._lingua_cancel_callback,
            callback_group=self._lingua_callback_group,
        )

        self.get_logger().info(  # type: ignore
            f"LinguaActionServer ready for: {sorted(capability_names)}"
        )

    def _lingua_goal_callback(self, goal_request):
        cap_name = goal_request.capability_name
        if cap_name not in self._lingua_capabilities:
            self.get_logger().warn(  # type: ignore
                f"LinguaActionServer: rejecting unknown capability '{cap_name}'"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _lingua_cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    async def _lingua_execute(self, goal_handle):
        cap_name   = goal_handle.request.capability_name
        params_raw = goal_handle.request.parameters_json

        try:
            parameters = json.loads(params_raw) if params_raw else {}
        except json.JSONDecodeError:
            self.get_logger().error(  # type: ignore
                f"LinguaActionServer: invalid JSON params for '{cap_name}': {params_raw}"
            )
            goal_handle.abort()
            result = self._LinguaAction.Result()
            result.success = False
            result.message = f"Invalid JSON parameters: {params_raw}"
            return result

        self.get_logger().info(  # type: ignore
            f"LinguaActionServer executing: {cap_name} {parameters}"
        )

        try:
            success, message, state_set, state_clear = await self.execute_capability(
                cap_name, parameters, goal_handle
            )
        except Exception as e:
            self.get_logger().error(  # type: ignore
                f"LinguaActionServer: exception in execute_capability: {e}"
            )
            goal_handle.abort()
            result = self._LinguaAction.Result()
            result.success = False
            result.message = str(e)
            return result

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        result = self._LinguaAction.Result()
        result.success = success
        result.message = message
        result.state_set   = list(state_set)
        result.state_clear = list(state_clear)
        return result

    async def execute_capability(
        self,
        capability_name: str,
        parameters: dict,
        goal_handle,
    ) -> Tuple[bool, str, List[str], List[str]]:
        """
        Override this method in your node.

        Execute the named capability with the given parameters and return:
            (success: bool, message: str, state_set: list, state_clear: list)

        The dispatcher automatically calls update_lingua_state() with
        the returned state_set and state_clear lists.

        Publish progress feedback via:
            goal_handle.publish_feedback(self._make_feedback("status", 0.5))
        """
        raise NotImplementedError(
            f"{self.__class__.__name__} must implement execute_capability()"
        )

    def _make_feedback(self, status: str, progress: float = 0.0):
        """Convenience method to create feedback messages."""
        feedback = self._LinguaAction.Feedback()
        feedback.status   = status
        feedback.progress = progress
        return feedback
