"""
ros2_lingua.capability_mixin
-----------------------------
Mixin for ROS 2 nodes to self-register capabilities with the GroundingNode.

Robustness additions:
- Registration retries with backoff if the GroundingNode isn't up yet
- State update failures are logged but don't crash the node
"""

import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from ros2_lingua_interfaces.srv import RegisterCapability, UpdateState
from ros2_lingua_core import Capability


class LinguaMixin:
    """
    Inherit alongside rclpy.node.Node. Call register_lingua_capability()
    in your __init__ for each capability your node exposes.

    Usage:
        class NavigationNode(LinguaMixin, Node):
            def __init__(self):
                Node.__init__(self, "navigation_node")
                LinguaMixin.__init__(self)
                self.register_lingua_capability(my_capability)
    """

    def __init__(self):
        self._lingua_register_client = self.create_client(
            RegisterCapability, "/lingua/register_capability"
        )
        self._lingua_state_client = self.create_client(
            UpdateState, "/lingua/update_state"
        )
        self._registered_capabilities = []
        
        self._reregister_sub = self.create_subscription(
            Empty,
            "/lingua/request_reregister",
            self._handle_reregister_request,
            10
        )

    def _handle_reregister_request(self, msg: Empty):
        """Callback when GroundingNode restarts and requests re-registration."""
        if not self._registered_capabilities:
            return
            
        self.get_logger().info("[Lingua] Grounding node requested re-registration. Re-registering capabilities...")
        for cap in self._registered_capabilities:
            # Run in a separate thread or timer to avoid blocking the subscription callback
            # but since we already have retry logic with timeouts, it's safer to avoid blocking.
            # Using a one-shot timer allows returning immediately.
            self.create_timer(
                0.1, 
                lambda c=cap: self._execute_registration(c, 10.0, 3), 
                callback_group=rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
            )

    def register_lingua_capability(
        self,
        capability: Capability,
        wait_timeout: float = 10.0,
        max_retries: int = 3,
    ) -> bool:
        """
        Register a capability with the GroundingNode.

        Retries up to max_retries times with exponential backoff if the
        GroundingNode service isn't available yet — useful during startup
        when node launch order isn't guaranteed.

        Args:
            capability:    The Capability to register
            wait_timeout:  Seconds to wait for the service per attempt
            max_retries:   Number of retry attempts before giving up

        Returns:
            True on success, False on failure.
        """
        if capability not in self._registered_capabilities:
            self._registered_capabilities.append(capability)
            
        return self._execute_registration(capability, wait_timeout, max_retries)

    def _execute_registration(self, capability: Capability, wait_timeout: float, max_retries: int) -> bool:
        """Internal helper to execute the registration with retries."""
        logger = self.get_logger()
        delay = 1.0

        for attempt in range(1, max_retries + 1):
            if not self._lingua_register_client.wait_for_service(
                timeout_sec=wait_timeout
            ):
                if attempt < max_retries:
                    logger.warn(
                        f"[Lingua] /lingua/register_capability not available "
                        f"(attempt {attempt}/{max_retries}). "
                        f"Retrying in {delay:.0f}s..."
                    )
                    time.sleep(delay)
                    delay *= 2.0   # exponential backoff
                    continue
                else:
                    logger.error(
                        f"[Lingua] Could not reach /lingua/register_capability "
                        f"after {max_retries} attempts. "
                        f"Is the GroundingNode running?"
                    )
                    return False

            request = RegisterCapability.Request()
            request.capability_json = capability.to_json()

            future = self._lingua_register_client.call_async(request)
            rclpy.spin_until_future_complete(
                self, future, timeout_sec=wait_timeout
            )

            if future.result() is None:
                if attempt < max_retries:
                    logger.warn(
                        f"[Lingua] Registration of '{capability.name}' timed out "
                        f"(attempt {attempt}/{max_retries}). Retrying..."
                    )
                    time.sleep(delay)
                    delay *= 2.0
                    continue
                else:
                    logger.error(
                        f"[Lingua] Registration of '{capability.name}' "
                        f"timed out after {max_retries} attempts."
                    )
                    return False

            result = future.result()
            if result.success:
                logger.info(f"Registered capability: '{capability.name}'")
                return True
            else:
                logger.error(
                    f"[Lingua] Registration rejected for '{capability.name}': "
                    f"{result.message}"
                )
                return False   # Rejection is not worth retrying

        return False

    def update_lingua_state(
        self,
        set_tokens: list = None,
        clear_tokens: list = None,
        timeout_sec: float = 3.0,
    ) -> bool:
        """
        Notify the GroundingNode of a symbolic state change.

        Args:
            set_tokens:   State tokens to mark as True
            clear_tokens: State tokens to mark as False
            timeout_sec:  How long to wait for the service

        Returns:
            True on success, False on failure (failure is non-fatal — logged only).
        """
        logger = self.get_logger()

        if set_tokens is None:
            set_tokens = []
        if clear_tokens is None:
            clear_tokens = []

        if not set_tokens and not clear_tokens:
            return True   # no-op

        if not self._lingua_state_client.wait_for_service(timeout_sec=2.0):
            logger.warn("[Lingua] /lingua/update_state not available — state update skipped.")
            return False

        request = UpdateState.Request()
        request.state_json = json.dumps({
            "set": set_tokens,
            "clear": clear_tokens,
        })

        future = self._lingua_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.result() is None:
            logger.warn(
                f"[Lingua] State update timed out. "
                f"set={set_tokens}, clear={clear_tokens}"
            )
            return False

        result = future.result()
        if not result.success:
            logger.warn(f"[Lingua] State update failed: {result.message}")
        return result.success
