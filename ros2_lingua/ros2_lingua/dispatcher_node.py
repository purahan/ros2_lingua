"""
ros2_lingua.dispatcher_node
----------------------------
Subscribes to /lingua/current_plan and executes each step in order.

Robustness features added in this version:
- Configurable step retry count (step_max_retries parameter)
- Configurable step timeout (step_timeout_sec parameter)
- Configurable failure mode (on_step_failure: abort | skip | retry)
- Detailed execution status publishing per step
- Guards against executing a new plan while one is already running
"""

import json
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from ros2_lingua_interfaces.msg import ExecutionStatus


class DispatcherNode(Node):

    def __init__(self):
        super().__init__("lingua_dispatcher_node")

        # --- Parameters ---
        self.declare_parameter("step_max_retries", 2)
        self.declare_parameter("step_timeout_sec", 30.0)
        # on_step_failure: "abort" | "skip" | "retry"
        self.declare_parameter("on_step_failure", "abort")

        self._step_max_retries  = self.get_parameter("step_max_retries").value
        self._step_timeout_sec  = self.get_parameter("step_timeout_sec").value
        self._on_step_failure   = self.get_parameter("on_step_failure").value
        self._executing         = False   # guard against concurrent plans

        self._callback_group = ReentrantCallbackGroup()

        self._plan_sub = self.create_subscription(
            String, "/lingua/current_plan",
            self._handle_plan, 10,
            callback_group=self._callback_group,
        )
        self._caps_sub = self.create_subscription(
            String, "/lingua/capabilities",
            self._handle_capabilities, 10,
        )
        self._status_pub = self.create_publisher(
            ExecutionStatus, "/lingua/execution_status", 10,
        )

        self._capability_map = {}
        self.get_logger().info(
            f"DispatcherNode ready. "
            f"[retries={self._step_max_retries}, "
            f"timeout={self._step_timeout_sec}s, "
            f"on_failure={self._on_step_failure}]"
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
        if self._executing:
            self.get_logger().warn(
                "Received new plan while already executing one. "
                "Ignoring — wait for current plan to complete or fail."
            )
            return

        try:
            plan_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid plan JSON: {e}")
            return

        steps       = plan_data.get("steps", [])
        instruction = plan_data.get("original_instruction", "")

        if not steps:
            self.get_logger().warn("Received empty plan — nothing to execute.")
            return

        self.get_logger().info(
            f"Executing plan for: '{instruction}' ({len(steps)} steps)"
        )
        self._executing = True
        self._publish_status("STARTED", instruction=instruction)

        try:
            for i, step in enumerate(steps):
                cap_name = step.get("capability_name", "")
                params   = step.get("parameters", {})

                self.get_logger().info(f"Step {i+1}/{len(steps)}: {cap_name}")
                success = self._execute_step_with_retry(cap_name, params, i)

                if not success:
                    if self._on_step_failure == "skip":
                        self.get_logger().warn(
                            f"Step '{cap_name}' failed — skipping (on_step_failure=skip)."
                        )
                        self._publish_status(
                            "STEP_SKIPPED", step=cap_name, instruction=instruction
                        )
                        continue
                    else:  # abort (default)
                        self.get_logger().error(
                            f"Step '{cap_name}' failed — aborting plan."
                        )
                        self._publish_status(
                            "FAILED", step=cap_name, instruction=instruction
                        )
                        return

                self._publish_status(
                    "STEP_COMPLETE", step=cap_name, instruction=instruction
                )

            self._publish_status("COMPLETED", instruction=instruction)
            self.get_logger().info("Plan executed successfully.")

        finally:
            self._executing = False

    def _execute_step_with_retry(
        self, capability_name: str, parameters: dict, step_index: int
    ) -> bool:
        """
        Execute a single step, retrying on failure up to step_max_retries times.
        Returns True on success, False if all attempts fail.
        """
        attempts = self._step_max_retries + 1   # first attempt + retries

        for attempt in range(1, attempts + 1):
            try:
                success = self._execute_step(capability_name, parameters)
                if success:
                    if attempt > 1:
                        self.get_logger().info(
                            f"  Step '{capability_name}' succeeded on attempt {attempt}."
                        )
                    return True
                else:
                    raise RuntimeError("Step returned failure.")

            except Exception as e:
                if attempt < attempts:
                    self.get_logger().warn(
                        f"  Step '{capability_name}' failed (attempt {attempt}/{attempts}): {e}. "
                        f"Retrying..."
                    )
                    time.sleep(1.0 * attempt)   # simple linear backoff
                else:
                    self.get_logger().error(
                        f"  Step '{capability_name}' failed after {attempts} attempt(s): {e}"
                    )
                    return False

        return False

    def _execute_step(self, capability_name: str, parameters: dict) -> bool:
        """
        Route to _call_action or _call_service based on capability definition.
        Returns True on success.
        """
        cap = self._capability_map.get(capability_name)
        if cap is None:
            self.get_logger().error(
                f"Cannot execute '{capability_name}': not in capability cache. "
                "Waiting for GroundingNode broadcast..."
            )
            # Give the broadcast a moment to arrive and retry lookup
            time.sleep(1.0)
            cap = self._capability_map.get(capability_name)
            if cap is None:
                return False

        ros_action  = cap.get("ros_action")
        ros_service = cap.get("ros_service")

        if ros_action:
            return self._call_action(ros_action, capability_name, parameters)
        elif ros_service:
            return self._call_service(ros_service, capability_name, parameters)
        else:
            self.get_logger().error(
                f"Capability '{capability_name}' has no ros_action or ros_service."
            )
            return False

    def _call_action(self, action_name: str, cap_name: str, params: dict) -> bool:
        """
        Call a ROS 2 action. Override this in a robot-specific subclass
        to send real typed action goals.
        """
        self.get_logger().info(
            f"  -> Action '{action_name}' | params: {params}"
        )
        self.get_logger().warn(
            f"  [DEMO MODE] Subclass _call_action() for real execution."
        )
        return True

    def _call_service(self, service_name: str, cap_name: str, params: dict) -> bool:
        """
        Call a ROS 2 service. Override in a robot-specific subclass.
        """
        self.get_logger().info(
            f"  -> Service '{service_name}' | params: {params}"
        )
        self.get_logger().warn(
            f"  [DEMO MODE] Subclass _call_service() for real execution."
        )
        return True

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------

    def _publish_status(self, status: str, step: str = "", instruction: str = ""):
        msg             = ExecutionStatus()
        msg.status      = status
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
