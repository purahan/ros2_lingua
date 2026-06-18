"""
ros2_lingua.dispatcher_node
----------------------------
Subscribes to /lingua/current_plan and executes each step by routing
to the appropriate dispatch protocol.

Three dispatch levels, checked in order:

  Level 1 — LinguaAction server (preferred)
    The capability's ros_action points to a node implementing the
    generic LinguaAction interface. Works out of the box.

  Level 2 — DispatchConfig mapping
    A DispatchConfig instance is provided mapping capability names to
    existing typed actions/services. Zero subclassing required.

  Level 3 — Subclass override (escape hatch)
    Override _call_action() / _call_service() for full control.

After each step completes, the dispatcher automatically updates
/lingua/update_state with the state_set/state_clear from the result.
"""

import contextlib
import json
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

from ros2_lingua_interfaces.msg import ExecutionStatus
from ros2_lingua_interfaces.srv import UpdateState


class DispatcherNode(Node):
    """
    Executes ActionPlans produced by the GroundingNode.

    Parameters:
        step_timeout_sec (float): Max seconds to wait for a single step.
                                  Default: 60.0
        dispatch_config:          Optional DispatchConfig instance (Level 2).
        recovery_planner:         Optional RecoveryPlanner for step failure handling.
                                  If not provided, steps that fail abort the plan.
    """

    def __init__(self, dispatch_config=None, recovery_planner=None):
        super().__init__("lingua_dispatcher_node")

        self.declare_parameter("step_timeout_sec", 60.0)
        self._step_timeout = self.get_parameter("step_timeout_sec").value
        self._dispatch_config = dispatch_config
        self._recovery_planner = recovery_planner

        self._callback_group = ReentrantCallbackGroup()

        # Subscribe to plans from the GroundingNode
        self._plan_sub = self.create_subscription(
            String,
            "/lingua/current_plan",
            self._handle_plan,
            10,
            callback_group=self._callback_group,
        )

        # Subscribe to capability registry for ros_action/ros_service lookup
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

        # State update client — used after each step completes
        self._state_client = self.create_client(
            UpdateState,
            "/lingua/update_state",
            callback_group=self._callback_group,
        )

        # Plan cancellation handling
        self._cancel_requested = False
        self._plan_running_event = threading.Event()
        self._plan_running_event.set()  # Initial state: not running (so set to True, wait() won't block)
        self._cancel_srv = self.create_service(
            Empty,
            "/lingua/cancel",
            self._handle_cancel,
        )

        # Cached action clients keyed by action name
        self._action_clients = {}

        # Capability cache from registry broadcast
        self._capability_map = {}

        recovery_status = (
            f"recovery={self._recovery_planner.config.max_retries} retries"
            if self._recovery_planner else "recovery=off"
        )
        self.get_logger().info(f"DispatcherNode ready ({recovery_status}).")

    def set_dispatch_config(self, dispatch_config) -> None:
        """Set or replace the DispatchConfig after construction."""
        self._dispatch_config = dispatch_config
        self.get_logger().info(
            f"DispatchConfig registered for: "
            f"{dispatch_config.registered_capabilities()}"
        )

    def set_recovery_planner(self, recovery_planner) -> None:
        """Set or replace the RecoveryPlanner after construction."""
        self._recovery_planner = recovery_planner
        self.get_logger().info(
            f"RecoveryPlanner set: max_retries={recovery_planner.config.max_retries}, "
            f"replan={recovery_planner.config.enable_replan}, "
            f"fallback={recovery_planner.config.safe_fallback}"
        )

    # ------------------------------------------------------------------
    # Capability cache
    # ------------------------------------------------------------------

    def _handle_capabilities(self, msg: String):
        try:
            caps = json.loads(msg.data)
            self._capability_map = {c["name"]: c for c in caps}
        except Exception as e:
            self.get_logger().error(f"Failed to parse capabilities: {e}")

    # ------------------------------------------------------------------
    # Plan execution
    # ------------------------------------------------------------------

    def _handle_plan(self, msg: String):
        try:
            plan_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid plan JSON: {e}")
            return

        steps       = plan_data.get("steps", [])
        instruction = plan_data.get("original_instruction", "")
        tag_filter  = plan_data.get("tag_filter", None)
        total       = len(steps)

        # If a plan is currently running, cancel it and wait for it to finish
        if not self._plan_running_event.is_set():
            self.get_logger().info("New plan received. Cancelling current running plan...")
            self._cancel_requested = True
            self._plan_running_event.wait()

        self._cancel_requested = False
        self._plan_running_event.clear()  # Mark as running

        self.get_logger().info(
            f"Executing plan: '{instruction}' ({total} steps)"
        )
        self._publish_status("STARTED", instruction=instruction)

        # Reset recovery planner state for this new plan
        if self._recovery_planner:
            self._recovery_planner.reset()

        # Track current symbolic state for recovery replanning
        current_state = set()

        i = 0
        try:
            while i < len(steps):
                if self._cancel_requested:
                    self.get_logger().warn(f"Plan cancelled during step {i+1}/{len(steps)}")
                    self._publish_status("CANCELLED", instruction=instruction)
                    return

                step     = steps[i]
                cap_name = step.get("capability_name", "")
                params   = step.get("parameters", {})

                self.get_logger().info(f"Step {i+1}/{len(steps)}: {cap_name}")
                self._publish_status("STEP_STARTED", step=cap_name, instruction=instruction)

                success, state_set, state_clear = self._execute_step(cap_name, params)

                if state_set or state_clear:
                    self._update_state(state_set, state_clear)
                    current_state.update(state_set)
                    current_state -= set(state_clear)

                if success:
                    self._publish_status("STEP_COMPLETE", step=cap_name, instruction=instruction)
                    i += 1
                    continue

                # ── Step failed — attempt recovery if planner available ──
                if self._recovery_planner is None:
                    self.get_logger().error(
                        f"Step '{cap_name}' failed. Aborting plan (no recovery planner)."
                    )
                    self._publish_status("FAILED", step=cap_name, instruction=instruction)
                    return

                decision = self._recovery_planner.on_step_failed(
                    failed_step=step,
                    step_index=i,
                    original_instruction=instruction,
                    current_state=current_state,
                    error=f"Step '{cap_name}' returned failure",
                    tag_filter=tag_filter,
                )

                if decision.strategy == "retry":
                    self.get_logger().info(f"Recovery: retrying '{cap_name}'")
                    self._publish_status("STEP_RETRY", step=cap_name, instruction=instruction)
                    # Loop will re-execute the same step (i not incremented)
                    continue

                elif decision.strategy == "replan":
                    self.get_logger().info(
                        f"Recovery: replanned — {len(decision.new_plan.steps)} new step(s)"
                    )
                    self._publish_status("REPLANNING", step=cap_name, instruction=instruction)
                    # Replace remaining steps with the new plan
                    steps = [s.to_dict() if hasattr(s, 'to_dict') else s
                             for s in decision.new_plan.steps]
                    i = 0  # restart from the beginning of the new plan
                    continue

                elif decision.strategy == "fallback":
                    fallback_name = self._recovery_planner.config.safe_fallback
                    fallback_params = self._recovery_planner.config.safe_fallback_params
                    self.get_logger().warn(
                        f"Recovery: executing fallback '{fallback_name}'"
                    )
                    self._publish_status("FALLBACK", step=fallback_name, instruction=instruction)
                    fb_success, fb_set, fb_clear = self._execute_step(
                        fallback_name, fallback_params
                    )
                    if fb_set or fb_clear:
                        self._update_state(fb_set, fb_clear)
                    if not fb_success:
                        self.get_logger().error(
                            f"Fallback '{fallback_name}' also failed."
                        )
                    self._publish_status(
                        "RECOVERY_FAILED", step=cap_name, instruction=instruction
                    )
                    return

                else:  # abort
                    self.get_logger().error(
                        f"Recovery: aborting. Reason: {decision.reason}"
                    )
                    self._publish_status(
                        "RECOVERY_FAILED", step=cap_name, instruction=instruction
                    )
                    return

            self._publish_status("COMPLETED", instruction=instruction)
            self.get_logger().info("Plan executed successfully.")
        finally:
            self._plan_running_event.set()  # Mark as no longer running

    def _handle_cancel(self, request, response):
        """Service callback to cancel the currently running plan."""
        self.get_logger().warn("Cancellation requested via /lingua/cancel service.")
        self._cancel_requested = True
        return response

    def _execute_step(
        self, capability_name: str, parameters: dict
    ) -> tuple:
        """
        Route a single step to the appropriate dispatch level.
        Returns (success: bool, state_set: list, state_clear: list).
        """
        cap = self._capability_map.get(capability_name)

        # ── Level 2: DispatchConfig ──────────────────────────────────
        if self._dispatch_config and self._dispatch_config.has(capability_name):
            return self._dispatch_via_config(capability_name, parameters)

        # ── Level 1 / Level 3: Route via capability interface type ───
        if cap is None:
            self.get_logger().error(
                f"Capability '{capability_name}' not in cache. "
                "Is the GroundingNode broadcasting capabilities?"
            )
            return False, [], []

        ros_action  = cap.get("ros_action")
        ros_service = cap.get("ros_service")
        postconditions = cap.get("postconditions", [])

        if ros_action:
            success = self._call_action(ros_action, capability_name, parameters)
            if success:
                return True, postconditions, []
            return False, [], []

        elif ros_service:
            success = self._call_service(ros_service, capability_name, parameters)
            if success:
                return True, postconditions, []
            return False, [], []

        self.get_logger().error(
            f"Capability '{capability_name}' has no ros_action or ros_service."
        )
        return False, [], []

    # ------------------------------------------------------------------
    # Level 1 — LinguaAction protocol
    # ------------------------------------------------------------------

    def _call_action(
        self, action_name: str, cap_name: str, params: dict
    ) -> bool:
        """
        Call a ROS 2 action.

        If the action server implements the generic LinguaAction protocol,
        the goal is sent automatically. Otherwise falls back to demo mode
        unless overridden by a subclass (Level 3).
        """
        try:
            from ros2_lingua_interfaces.action import LinguaAction
            lingua_action_available = True
        except ImportError:
            lingua_action_available = False

        if lingua_action_available:
            return self._call_lingua_action(action_name, cap_name, params)

        # Level 3 fallback (subclass override)
        return self._call_action_typed(action_name, cap_name, params)

    def _call_lingua_action(
        self, action_name: str, cap_name: str, params: dict
    ) -> bool:
        """Send a LinguaAction goal to the action server."""
        from ros2_lingua_interfaces.action import LinguaAction

        if action_name not in self._action_clients:
            self._action_clients[action_name] = ActionClient(
                self, LinguaAction, action_name,
                callback_group=self._callback_group,
            )

        client = self._action_clients[action_name]

        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"Action server '{action_name}' not available within 5s. "
                f"Trying demo mode."
            )
            return self._call_action_demo(action_name, cap_name, params)

        goal = LinguaAction.Goal()
        goal.capability_name  = cap_name
        goal.parameters_json  = json.dumps(params)

        self.get_logger().info(
            f"  → LinguaAction '{action_name}' | {cap_name} | {params}"
        )

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self._step_timeout
        )

        if future.result() is None:
            self.get_logger().error(f"Goal rejected by '{action_name}'")
            return False

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected by '{action_name}'")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._step_timeout
        )

        if result_future.result() is None:
            self.get_logger().error(
                f"Action '{action_name}' timed out after {self._step_timeout}s"
            )
            return False

        result = result_future.result().result
        if not result.success:
            self.get_logger().error(
                f"Action '{action_name}' failed: {result.message}"
            )

        # Apply state updates from result
        if result.state_set or result.state_clear:
            self._update_state(
                list(result.state_set), list(result.state_clear)
            )

        return result.success

    def _call_service(
        self, service_name: str, cap_name: str, params: dict
    ) -> bool:
        """
        Call a ROS 2 service.

        Tries the generic LinguaService protocol first, then falls back
        to demo mode (or subclass override).
        """
        try:
            from ros2_lingua_interfaces.srv import LinguaService
            lingua_svc_available = True
        except ImportError:
            lingua_svc_available = False

        if lingua_svc_available:
            return self._call_lingua_service(service_name, cap_name, params)

        return self._call_service_typed(service_name, cap_name, params)

    def _call_lingua_service(
        self, service_name: str, cap_name: str, params: dict
    ) -> bool:
        """Send a LinguaService request."""
        from ros2_lingua_interfaces.srv import LinguaService

        client = self.create_client(
            LinguaService, service_name,
            callback_group=self._callback_group,
        )

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                f"Service '{service_name}' not available. Using demo mode."
            )
            return self._call_service_demo(service_name, cap_name, params)

        request = LinguaService.Request()
        request.capability_name = cap_name
        request.parameters_json = json.dumps(params)

        self.get_logger().info(
            f"  → LinguaService '{service_name}' | {cap_name} | {params}"
        )

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=self._step_timeout
        )

        if future.result() is None:
            self.get_logger().error(f"Service '{service_name}' timed out.")
            return False

        result = future.result()
        if result.state_set or result.state_clear:
            self._update_state(list(result.state_set), list(result.state_clear))

        return result.success

    # ------------------------------------------------------------------
    # Level 2 — DispatchConfig
    # ------------------------------------------------------------------

    def _dispatch_via_config(
        self, capability_name: str, parameters: dict
    ) -> tuple:
        """Dispatch using a registered DispatchConfig mapping."""
        config = self._dispatch_config

        action_mapping = config.get_action(capability_name)
        if action_mapping:
            return self._call_typed_action(action_mapping, parameters)

        service_mapping = config.get_service(capability_name)
        if service_mapping:
            return self._call_typed_service(service_mapping, parameters)

        return False, [], []

    def _call_typed_action(self, mapping, parameters: dict) -> tuple:
        """Call a typed action via a DispatchConfig ActionMapping."""
        key = f"{mapping.action_name}::{mapping.action_type.__name__}"
        if key not in self._action_clients:
            self._action_clients[key] = ActionClient(
                self, mapping.action_type, mapping.action_name,
                callback_group=self._callback_group,
            )

        client = self._action_clients[key]
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                f"Action server '{mapping.action_name}' not available."
            )
            return False, [], []

        try:
            goal = mapping.goal_adapter(parameters)
        except Exception as e:
            self.get_logger().error(
                f"goal_adapter for '{mapping.capability_name}' raised: {e}"
            )
            return False, [], []

        self.get_logger().info(
            f"  → Config action '{mapping.action_name}' ({mapping.capability_name})"
        )

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=mapping.timeout_sec
        )

        if future.result() is None or not future.result().accepted:
            return False, [], []

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=mapping.timeout_sec
        )

        if result_future.result() is None:
            self.get_logger().error(
                f"Action '{mapping.action_name}' timed out."
            )
            return False, [], []

        result = result_future.result().result
        success = True
        if mapping.result_checker:
            success = mapping.result_checker(result)

        cap = self._capability_map.get(mapping.capability_name, {})
        postconditions = cap.get("postconditions", []) if success else []
        return success, postconditions, []

    def _call_typed_service(self, mapping, parameters: dict) -> tuple:
        """Call a typed service via a DispatchConfig ServiceMapping."""
        client = self.create_client(
            mapping.service_type, mapping.service_name,
            callback_group=self._callback_group,
        )
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f"Service '{mapping.service_name}' not available."
            )
            return False, [], []

        try:
            request = mapping.request_adapter(parameters)
        except Exception as e:
            self.get_logger().error(
                f"request_adapter for '{mapping.capability_name}' raised: {e}"
            )
            return False, [], []

        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=mapping.timeout_sec
        )

        if future.result() is None:
            return False, [], []

        result = future.result()
        success = True
        if mapping.result_checker:
            success = mapping.result_checker(result)

        cap = self._capability_map.get(mapping.capability_name, {})
        postconditions = cap.get("postconditions", []) if success else []
        return success, postconditions, []

    # ------------------------------------------------------------------
    # Level 3 — Override these in subclasses
    # ------------------------------------------------------------------

    def _call_action_typed(
        self, action_name: str, cap_name: str, params: dict
    ) -> bool:
        """
        Override this in a subclass to call typed actions on existing robots.
        Called when no LinguaAction server is available and no DispatchConfig.
        """
        return self._call_action_demo(action_name, cap_name, params)

    def _call_service_typed(
        self, service_name: str, cap_name: str, params: dict
    ) -> bool:
        """Override this in a subclass to call typed services."""
        return self._call_service_demo(service_name, cap_name, params)

    # ------------------------------------------------------------------
    # Demo mode (when nothing else is available)
    # ------------------------------------------------------------------

    def _call_action_demo(
        self, action_name: str, cap_name: str, params: dict
    ) -> bool:
        self.get_logger().info(
            f"  → [DEMO] action '{action_name}' | {cap_name} | {params}"
        )
        self.get_logger().warn(
            "  Running in demo mode. For real execution either:\n"
            "  1. Implement LinguaActionServer on your node\n"
            "  2. Use DispatchConfig to map to your existing interfaces\n"
            "  3. Subclass DispatcherNode and override _call_action_typed()"
        )
        return True

    def _call_service_demo(
        self, service_name: str, cap_name: str, params: dict
    ) -> bool:
        self.get_logger().info(
            f"  → [DEMO] service '{service_name}' | {cap_name} | {params}"
        )
        return True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _update_state(self, set_tokens: list, clear_tokens: list) -> None:
        """Push state token updates back to the GroundingNode."""
        if not self._state_client.wait_for_service(timeout_sec=2.0):
            return
        req = UpdateState.Request()
        req.state_json = json.dumps({
            "set": set_tokens, "clear": clear_tokens
        })
        future = self._state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

    def _publish_status(
        self, status: str, step: str = "", instruction: str = ""
    ) -> None:
        msg = ExecutionStatus()
        msg.status       = status
        msg.current_step = step
        msg.instruction  = instruction
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
        with contextlib.suppress(Exception):
            rclpy.shutdown()


if __name__ == "__main__":
    main()
