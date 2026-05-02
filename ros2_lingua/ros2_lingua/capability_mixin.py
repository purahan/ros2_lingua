"""
ros2_lingua.capability_mixin
-----------------------------
A mixin class that any ROS 2 Node can inherit from to register
capabilities with the GroundingNode cleanly.

Usage:
    from ros2_lingua import LinguaMixin
    from ros2_lingua_core import Capability, CapabilityParameter

    class NavigationNode(LinguaMixin, Node):
        def __init__(self):
            Node.__init__(self, "navigation_node")
            LinguaMixin.__init__(self)

            self.register_lingua_capability(Capability(
                name="navigate_to_location",
                description="Walks the robot to a named location",
                ros_action="humanoid/navigate_to_pose",
                parameters=[
                    CapabilityParameter("location_name", "string", "Where to go")
                ],
                preconditions=["robot_is_balanced"],
                postconditions=["robot_at_location"],
            ))
"""

import json
import time
import rclpy
from rclpy.node import Node

from ros2_lingua_interfaces.srv import RegisterCapability, UpdateState
from ros2_lingua_core import Capability


class LinguaMixin:
    """
    Mixin for ROS 2 nodes to self-register capabilities with the GroundingNode.

    Inherit alongside rclpy.node.Node. Call register_lingua_capability()
    in your __init__ for each capability your node exposes.
    """

    def __init__(self):
        # These will be available because the sibling class is Node
        self._lingua_register_client = self.create_client(  # type: ignore
            RegisterCapability, "/lingua/register_capability"
        )
        self._lingua_state_client = self.create_client(  # type: ignore
            UpdateState, "/lingua/update_state"
        )

    def register_lingua_capability(
        self, capability: Capability, wait_timeout: float = 5.0
    ) -> bool:
        """
        Register a capability with the GroundingNode.

        Blocks until the service is available or timeout is reached.
        Returns True on success, False on failure.
        """
        logger = self.get_logger()  # type: ignore

        if not self._lingua_register_client.wait_for_service(timeout_sec=wait_timeout):
            logger.warn(
                "/lingua/register_capability service not available. "
                "Is the GroundingNode running?"
            )
            return False

        request = RegisterCapability.Request()
        request.capability_json = capability.to_json()

        future = self._lingua_register_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait_timeout)  # type: ignore

        if future.result() is None:
            logger.error(f"Failed to register capability '{capability.name}': timeout")
            return False

        result = future.result()
        if result.success:
            logger.info(f"Registered capability: '{capability.name}'")
        else:
            logger.error(f"Failed to register '{capability.name}': {result.message}")

        return result.success

    def update_lingua_state(
        self,
        set_tokens: list = None,
        clear_tokens: list = None,
    ) -> bool:
        """
        Notify the GroundingNode of a symbolic state change.

        Call this when your node's state changes in a way that affects
        preconditions — e.g. after the robot becomes balanced, call:
            self.update_lingua_state(set_tokens=["robot_is_balanced"])

        Args:
            set_tokens: State tokens to mark as True
            clear_tokens: State tokens to mark as False
        """
        if set_tokens is None:
            set_tokens = []
        if clear_tokens is None:
            clear_tokens = []

        if not self._lingua_state_client.wait_for_service(timeout_sec=2.0):
            return False

        request = UpdateState.Request()
        request.state_json = json.dumps({
            "set": set_tokens,
            "clear": clear_tokens,
        })

        future = self._lingua_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)  # type: ignore

        result = future.result()
        return result is not None and result.success
