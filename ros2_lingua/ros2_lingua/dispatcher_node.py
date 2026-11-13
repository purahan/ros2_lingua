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
    Override _call_action_typed() / _call_service_typed(), which are
    called when no LinguaAction / LinguaService server answers.

After each step completes, the dispatcher automatically updates
/lingua/update_state with the state_set/state_clear from the result.

Threading model:
  Plans run on a dedicated worker thread. ROS callbacks only enqueue
  work or set flags, so they never block the executor. The worker waits
  on futures that the executor completes, which keeps /lingua/cancel
  responsive and lets a new plan preempt the running one.

Failure handling:
  A step whose server is unavailable FAILS. Set the allow_demo_mode
  parameter to simulate missing servers instead (never on a real robot).
  On timeout or cancellation the in-flight goal is cancelled on the
  server, and no postconditions are applied.
"""

import contextlib
import json
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty

from ros2_lingua_interfaces.action import LinguaAction
from ros2_lingua_interfaces.msg import ExecutionStatus
from ros2_lingua_interfaces.srv import LinguaService, UpdateState


# How long to wait for a cancel request to be acknowledged by an action server
CANCEL_TIMEOUT_SEC = 5.0

# How long to wait for a server to appear before treating it as unavailable
SERVER_WAIT_SEC = 5.0

# Outcomes of _wait_for_future
DONE = "done"
TIMED_OUT = "timed_out"
CANCELLED = "cancelled"


class DispatcherNode(Node):
    """
    Executes ActionPlans produced by the GroundingNode.

    Parameters:
        step_timeout_sec (float): Max seconds to wait for a single step.
                                  Default: 60.0
        allow_demo_mode (bool):   Simulate steps whose server is unavailable
                                  and report them as successful. Default: False.
                                  Never enable this on a real robot.
        dispatch_config:          Optional DispatchConfig instance (Level 2).
        recovery_planner:         Optional RecoveryPlanner for step failure handling.
                                  If not provided, steps that fail abort the plan.
    """

    def __init__(self, dispatch_config=None, recovery_planner=None):
        super().__init__("lingua_dispatcher_node")

        self.declare_parameter("step_timeout_sec", 60.0)
        self.declare_parameter("allow_demo_mode", False)
        self._step_timeout = self.get_parameter("step_timeout_sec").value
        self._allow_demo_mode = self.get_parameter("allow_demo_mode").value
        self._dispatch_config = dispatch_config
        self._recovery_planner = recovery_planner

        self._callback_group = ReentrantCallbackGroup()

        # Subscribe to plans from the GroundingNode
        self._plan_sub = self.create_subscription(
            String,
            "lingua/current_plan",
            self._handle_plan,
            10,
            callback_group=self._callback_group,
        )

        # Subscribe to capability registry for ros_action/ros_service lookup
        self._caps_sub = self.create_subscription(
            String,
            "lingua/capabilities",
            self._handle_capabilities,
            10,
        )

        # Publish execution status
        self._status_pub = self.create_publisher(
            ExecutionStatus,
            "lingua/execution_status",
            10,
        )

        # State update client — used after each step completes
        self._state_client = self.create_client(
            UpdateState,
            "lingua/update_state",
            callback_group=self._callback_group,
        )

        # Plan cancellation handling
        self._cancel_srv = self.create_service(
            Empty,
            "lingua/cancel",
            self._handle_cancel,
            callback_group=self._callback_group,
        )

        # Cached action clients keyed by action name
        self._action_clients = {}

        # Cached service clients keyed by service name
        self._service_clients = {}

        # Capability cache from registry broadcast
        self._capability_map = {}

        # --- Plan worker ---
        # _pending_plan holds the newest plan not yet started (latest wins).
        # _cancel_event is set to stop the running plan.
        self._plan_lock = threading.Lock()
        self._pending_plan = None
        self._plan_active = False
        self._plan_ready = threading.Event()
        self._cancel_event = threading.Event()
        self._shutdown_event = threading.Event()
        self._worker = threading.Thread(
            target=self._worker_loop, name="lingua_plan_worker", daemon=True
        )
        self._worker.start()

        recovery_status = (
            f"recovery={self._recovery_planner.config.max_retries} retries"
            if self._recovery_planner
            else "recovery=off"
        )
        self.get_logger().info(f"DispatcherNode ready ({recovery_status}).")
        if self._allow_demo_mode:
            self.get_logger().warn(
                "allow_demo_mode is ON: steps whose server is unavailable are "
                "SIMULATED and reported as successful. Never use this on a real robot."
            )

    def set_dispatch_config(self, dispatch_config) -> None:
        """Set or replace the DispatchConfig after construction."""
        self._dispatch_config = dispatch_config
        self.get_logger().info(
            f"DispatchConfig registered for: {dispatch_config.registered_capabilities()}"
        )

    def set_recovery_planner(self, recovery_planner) -> None:
        """Set or replace the RecoveryPlanner after construction."""
        self._recovery_planner = recovery_planner
        self.get_logger().info(
            f"RecoveryPlanner set: max_retries={recovery_planner.config.max_retries}, "
            f"replan={recovery_planner.config.enable_replan}, "
            f"fallback={recovery_planner.config.safe_fallback}"
        )

    def destroy_node(self):
        """Stop the plan worker before tearing down the node."""
        self._shutdown_event.set()
        self._cancel_event.set()
        self._plan_ready.set()
        self._worker.join(timeout=CANCEL_TIMEOUT_SEC + 1.0)
        super().destroy_node()

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
    # Plan intake (ROS callbacks — must never block)
    # ------------------------------------------------------------------

    def _handle_plan(self, msg: String):
        try:
            plan_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid plan JSON: {e}")
            return

        with self._plan_lock:
            if self._plan_active:
                self.get_logger().info("New plan received. Cancelling current running plan...")
                self._cancel_event.set()
            self._pending_plan = plan_data
        self._plan_ready.set()

    def _handle_cancel(self, request, response):
        """Service callback to cancel the running plan and any queued plan."""
        self.get_logger().warn("Cancellation requested via lingua/cancel service.")
        with self._plan_lock:
            self._pending_plan = None
            if self._plan_active:
                self._cancel_event.set()
        return response

    # ------------------------------------------------------------------
    # Plan execution (worker thread)
    # ------------------------------------------------------------------

    def _worker_loop(self):
        while not self._shutdown_event.is_set():
            if not self._plan_ready.wait(timeout=0.2):
                continue
            with self._plan_lock:
                plan_data = self._pending_plan
                self._pending_plan = None
                self._plan_ready.clear()
                if plan_data is None or self._shutdown_event.is_set():
                    continue
                self._cancel_event.clear()
                self._plan_active = True
            try:
                self._execute_plan(plan_data)
            except Exception as e:
                self.get_logger().error(f"Plan execution crashed: {e}")
                self._publish_status(
                    "FAILED", instruction=plan_data.get("original_instruction", "")
                )
            finally:
                with self._plan_lock:
                    self._plan_active = False

    def _execute_plan(self, plan_data: dict):
        steps = plan_data.get("steps", [])
        instruction = plan_data.get("original_instruction", "")
        tag_filter = plan_data.get("tag_filter")
        total = len(steps)

        self.get_logger().info(f"Executing plan: '{instruction}' ({total} steps)")
        self._publish_status("STARTED", instruction=instruction)

        # Reset recovery planner state for this new plan
        if self._recovery_planner:
            self._recovery_planner.reset()

        # Track current symbolic state for recovery replanning
        current_state = set()

        i = 0
        while i < len(steps):
            if self._cancel_event.is_set():
                self.get_logger().warn(f"Plan cancelled before step {i + 1}/{len(steps)}")
                self._publish_status("CANCELLED", instruction=instruction)
                return

            step = steps[i]
            cap_name = step.get("capability_name", "")
            params = step.get("parameters", {})

            self.get_logger().info(f"Step {i + 1}/{len(steps)}: {cap_name}")
            self._publish_status("STEP_STARTED", step=cap_name, instruction=instruction)

            success, state_set, state_clear = self._execute_step(cap_name, params)

            # A cancelled step never counts, whatever it returned
            if self._cancel_event.is_set():
                self.get_logger().warn(f"Plan cancelled during step {i + 1}/{len(steps)}")
                self._publish_status("CANCELLED", step=cap_name, instruction=instruction)
                return

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
                steps = [
                    s.to_dict() if hasattr(s, "to_dict") else s for s in decision.new_plan.steps
                ]
                i = 0  # restart from the beginning of the new plan
                continue

            elif decision.strategy == "fallback":
                fallback_name = self._recovery_planner.config.safe_fallback
                fallback_params = self._recovery_planner.config.safe_fallback_params
                self.get_logger().warn(f"Recovery: executing fallback '{fallback_name}'")
                self._publish_status("FALLBACK", step=fallback_name, instruction=instruction)
                fb_success, fb_set, fb_clear = self._execute_step(fallback_name, fallback_params)
                if (fb_set or fb_clear) and not self._cancel_event.is_set():
                    self._update_state(fb_set, fb_clear)
                if not fb_success:
                    self.get_logger().error(f"Fallback '{fallback_name}' also failed.")
                self._publish_status("RECOVERY_FAILED", step=cap_name, instruction=instruction)
                return

            else:  # abort
                self.get_logger().error(f"Recovery: aborting. Reason: {decision.reason}")
                self._publish_status("RECOVERY_FAILED", step=cap_name, instruction=instruction)
                return

        self._publish_status("COMPLETED", instruction=instruction)
        self.get_logger().info("Plan executed successfully.")

    def _execute_step(self, capability_name: str, parameters: dict) -> tuple:
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

        ros_action = cap.get("ros_action")
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

        self.get_logger().error(f"Capability '{capability_name}' has no ros_action or ros_service.")
        return False, [], []

    # ------------------------------------------------------------------
    # Waiting helpers (worker thread only)
    # ------------------------------------------------------------------

    def _wait_for_future(self, future, timeout_sec, cancellable: bool = False) -> str:
        """
        Block the worker until the future completes, the timeout expires,
        or (if cancellable) the plan is cancelled. The executor completes
        the future on another thread, so this never spins the node.

        Returns DONE, TIMED_OUT or CANCELLED.
        """
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        deadline = time.monotonic() + timeout_sec
        while not done.is_set():
            if cancellable and self._cancel_event.is_set():
                return CANCELLED
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return TIMED_OUT
            done.wait(timeout=min(0.1, remaining))
        return DONE

    def _run_goal(self, client, goal, timeout_sec: float, action_name: str):
        """
        Send a goal and wait for its result.

        On timeout or plan cancellation the goal is cancelled on the server
        before returning, so the robot never keeps executing a step the
        dispatcher has given up on.

        Returns the GetResult response (with .status and .result), or None
        if the goal was rejected, timed out or was cancelled.
        """
        send_future = client.send_goal_async(goal)
        outcome = self._wait_for_future(send_future, timeout_sec, cancellable=True)
        if outcome != DONE:
            if outcome == TIMED_OUT:
                self.get_logger().error(
                    f"Action '{action_name}' did not respond to the goal request "
                    f"within {timeout_sec}s."
                )
            # If the server accepts it later, cancel it straight away
            send_future.add_done_callback(self._cancel_late_goal)
            return None

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected by '{action_name}'")
            return None

        result_future = goal_handle.get_result_async()
        outcome = self._wait_for_future(result_future, timeout_sec, cancellable=True)
        if outcome == DONE:
            return result_future.result()

        if outcome == TIMED_OUT:
            self.get_logger().error(
                f"Action '{action_name}' timed out after {timeout_sec}s. Cancelling goal."
            )
        else:
            self.get_logger().warn(f"Plan cancelled. Cancelling goal on '{action_name}'.")
        self._cancel_goal(goal_handle, result_future, action_name)
        return None

    def _cancel_goal(self, goal_handle, result_future, action_name: str) -> None:
        """
        Cancel a goal and wait until it has actually stopped, so the next
        step or plan never overlaps with it on the robot.
        """
        deadline = time.monotonic() + CANCEL_TIMEOUT_SEC
        cancel_future = goal_handle.cancel_goal_async()
        acknowledged = self._wait_for_future(cancel_future, CANCEL_TIMEOUT_SEC) == DONE
        remaining = max(0.0, deadline - time.monotonic())
        if not acknowledged or self._wait_for_future(result_future, remaining) != DONE:
            self.get_logger().error(
                f"Goal on '{action_name}' did not stop within {CANCEL_TIMEOUT_SEC}s "
                f"of the cancel request. It may still be running."
            )

    def _cancel_late_goal(self, send_future) -> None:
        goal_handle = send_future.result()
        if goal_handle is not None and goal_handle.accepted:
            goal_handle.cancel_goal_async()

    def _call_service_once(self, client, request, timeout_sec: float, service_name: str):
        """
        Call a service and wait for the response. Services cannot be
        cancelled, so on timeout or plan cancellation the request is
        abandoned. Returns the response, or None.
        """
        future = client.call_async(request)
        outcome = self._wait_for_future(future, timeout_sec, cancellable=True)
        if outcome == DONE:
            return future.result()
        client.remove_pending_request(future)
        if outcome == TIMED_OUT:
            self.get_logger().error(f"Service '{service_name}' timed out after {timeout_sec}s.")
        return None

    # ------------------------------------------------------------------
    # Level 1 — LinguaAction protocol
    # ------------------------------------------------------------------

    def _call_action(self, action_name: str, cap_name: str, params: dict) -> bool:
        """
        Call a ROS 2 action.

        Sends a LinguaAction goal. If no LinguaAction server answers,
        falls back to _call_action_typed() (Level 3), which fails the
        step unless a subclass overrides it or demo mode is enabled.
        """
        return self._call_lingua_action(action_name, cap_name, params)

    def _call_lingua_action(self, action_name: str, cap_name: str, params: dict) -> bool:
        """Send a LinguaAction goal to the action server."""
        if action_name not in self._action_clients:
            self._action_clients[action_name] = ActionClient(
                self,
                LinguaAction,
                action_name,
                callback_group=self._callback_group,
            )

        client = self._action_clients[action_name]

        if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            return self._call_action_typed(action_name, cap_name, params)

        goal = LinguaAction.Goal()
        goal.capability_name = cap_name
        goal.parameters_json = json.dumps(params)

        self.get_logger().info(f"  → LinguaAction '{action_name}' | {cap_name} | {params}")

        response = self._run_goal(client, goal, self._step_timeout, action_name)
        if response is None:
            return False

        result = response.result
        if not result.success:
            self.get_logger().error(f"Action '{action_name}' failed: {result.message}")

        # Apply state updates from result
        if result.state_set or result.state_clear:
            self._update_state(list(result.state_set), list(result.state_clear))

        return result.success

    def _call_service(self, service_name: str, cap_name: str, params: dict) -> bool:
        """
        Call a ROS 2 service.

        Sends a LinguaService request. If no LinguaService server answers,
        falls back to _call_service_typed() (Level 3).
        """
        return self._call_lingua_service(service_name, cap_name, params)

    def _call_lingua_service(self, service_name: str, cap_name: str, params: dict) -> bool:
        """Send a LinguaService request."""
        if service_name not in self._service_clients:
            self._service_clients[service_name] = self.create_client(
                LinguaService,
                service_name,
                callback_group=self._callback_group,
            )

        client = self._service_clients[service_name]

        if not client.wait_for_service(timeout_sec=SERVER_WAIT_SEC):
            return self._call_service_typed(service_name, cap_name, params)

        request = LinguaService.Request()
        request.capability_name = cap_name
        request.parameters_json = json.dumps(params)

        self.get_logger().info(f"  → LinguaService '{service_name}' | {cap_name} | {params}")

        result = self._call_service_once(client, request, self._step_timeout, service_name)
        if result is None:
            return False

        if result.state_set or result.state_clear:
            self._update_state(list(result.state_set), list(result.state_clear))

        return result.success

    # ------------------------------------------------------------------
    # Level 2 — DispatchConfig
    # ------------------------------------------------------------------

    def _dispatch_via_config(self, capability_name: str, parameters: dict) -> tuple:
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
                self,
                mapping.action_type,
                mapping.action_name,
                callback_group=self._callback_group,
            )

        client = self._action_clients[key]
        if not client.wait_for_server(timeout_sec=SERVER_WAIT_SEC):
            self.get_logger().error(f"Action server '{mapping.action_name}' not available.")
            return False, [], []

        try:
            goal = mapping.goal_adapter(parameters)
        except Exception as e:
            self.get_logger().error(f"goal_adapter for '{mapping.capability_name}' raised: {e}")
            return False, [], []

        self.get_logger().info(
            f"  → Config action '{mapping.action_name}' ({mapping.capability_name})"
        )

        response = self._run_goal(client, goal, mapping.timeout_sec, mapping.action_name)
        if response is None:
            return False, [], []

        # Without a result_checker, only a SUCCEEDED goal counts as success
        if mapping.result_checker:
            success = mapping.result_checker(response.result)
        else:
            success = response.status == GoalStatus.STATUS_SUCCEEDED

        cap = self._capability_map.get(mapping.capability_name, {})
        postconditions = cap.get("postconditions", []) if success else []
        return success, postconditions, []

    def _call_typed_service(self, mapping, parameters: dict) -> tuple:
        """Call a typed service via a DispatchConfig ServiceMapping."""
        key = f"{mapping.service_name}::{mapping.service_type.__name__}"
        if key not in self._service_clients:
            self._service_clients[key] = self.create_client(
                mapping.service_type,
                mapping.service_name,
                callback_group=self._callback_group,
            )

        client = self._service_clients[key]
        if not client.wait_for_service(timeout_sec=SERVER_WAIT_SEC):
            self.get_logger().error(f"Service '{mapping.service_name}' not available.")
            return False, [], []

        try:
            request = mapping.request_adapter(parameters)
        except Exception as e:
            self.get_logger().error(f"request_adapter for '{mapping.capability_name}' raised: {e}")
            return False, [], []

        result = self._call_service_once(client, request, mapping.timeout_sec, mapping.service_name)
        if result is None:
            return False, [], []

        success = True
        if mapping.result_checker:
            success = mapping.result_checker(result)

        cap = self._capability_map.get(mapping.capability_name, {})
        postconditions = cap.get("postconditions", []) if success else []
        return success, postconditions, []

    # ------------------------------------------------------------------
    # Level 3 — Override these in subclasses
    # ------------------------------------------------------------------

    def _call_action_typed(self, action_name: str, cap_name: str, params: dict) -> bool:
        """
        Override this in a subclass to call typed actions on existing robots.
        Called when no LinguaAction server answers on action_name.

        The default fails the step, or simulates it if allow_demo_mode is set.
        """
        if self._allow_demo_mode:
            return self._call_action_demo(action_name, cap_name, params)
        self.get_logger().error(
            f"Action server '{action_name}' for '{cap_name}' is not available "
            f"(waited {SERVER_WAIT_SEC}s). Step failed."
        )
        return False

    def _call_service_typed(self, service_name: str, cap_name: str, params: dict) -> bool:
        """
        Override this in a subclass to call typed services.
        Called when no LinguaService server answers on service_name.

        The default fails the step, or simulates it if allow_demo_mode is set.
        """
        if self._allow_demo_mode:
            return self._call_service_demo(service_name, cap_name, params)
        self.get_logger().error(
            f"Service '{service_name}' for '{cap_name}' is not available "
            f"(waited {SERVER_WAIT_SEC}s). Step failed."
        )
        return False

    # ------------------------------------------------------------------
    # Demo mode (only when allow_demo_mode is set)
    # ------------------------------------------------------------------

    def _call_action_demo(self, action_name: str, cap_name: str, params: dict) -> bool:
        self.get_logger().warn(
            f"  → [DEMO] action '{action_name}' unavailable; simulating "
            f"{cap_name} {params} as successful."
        )
        return True

    def _call_service_demo(self, service_name: str, cap_name: str, params: dict) -> bool:
        self.get_logger().warn(
            f"  → [DEMO] service '{service_name}' unavailable; simulating "
            f"{cap_name} {params} as successful."
        )
        return True

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _update_state(self, set_tokens: list, clear_tokens: list) -> None:
        """Push state token updates back to the GroundingNode."""
        if not self._state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("lingua/update_state not available — state update skipped.")
            return
        req = UpdateState.Request()
        req.state_json = json.dumps({"set": set_tokens, "clear": clear_tokens})
        future = self._state_client.call_async(req)
        if self._wait_for_future(future, 3.0) != DONE:
            self._state_client.remove_pending_request(future)
            self.get_logger().warn("State update timed out.")

    def _publish_status(self, status: str, step: str = "", instruction: str = "") -> None:
        msg = ExecutionStatus()
        msg.status = status
        msg.current_step = step
        msg.instruction = instruction
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DispatcherNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        with contextlib.suppress(Exception):
            rclpy.shutdown()


if __name__ == "__main__":
    main()
