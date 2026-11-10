"""
ros2_lingua_mock.cli
---------------------
A clean CLI tool for sending natural language instructions to the robot,
and for listing currently registered robot capabilities.

Usage:
    ros2 run ros2_lingua_mock cli ground "go to the table and pick up the bottle"
    ros2 run ros2_lingua_mock cli list-capabilities [--namespace NS] [--tag TAG]

Prints the resulting plan in a human-readable format and shows
live execution status as the dispatcher works through the steps.
"""

import contextlib
import json
import math
import sys
import time

import rclpy
import rclpy.utilities
from rclpy.node import Node
from ros2_lingua_interfaces.msg import ExecutionStatus

from ros2_lingua_interfaces.srv import GroundInstruction

BANNER = """
╔══════════════════════════════════════════╗
║         ros2_lingua  —  CLI Tool         ║
╚══════════════════════════════════════════╝
"""

DEFAULT_GROUNDING_TIMEOUT = 30.0


def _positive_timeout(value: str) -> float:
    try:
        timeout = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("timeout must be a positive number of seconds") from error
    if not math.isfinite(timeout) or timeout <= 0:
        raise argparse.ArgumentTypeError("timeout must be a positive number of seconds")
    return timeout


class LinguaCLI(Node):
    def __init__(
        self,
        instruction: str,
        namespace: str = "",
        grounding_timeout: float = DEFAULT_GROUNDING_TIMEOUT,
    ) -> None:
        super().__init__("lingua_cli")
        if not math.isfinite(grounding_timeout) or grounding_timeout <= 0:
            raise ValueError("grounding_timeout must be a positive number of seconds")
        self._instruction = instruction
        self._grounding_timeout = grounding_timeout
        self._done = False

        namespace = namespace.strip("/")
        prefix = f"/{namespace}" if namespace else ""
        self._service_name = f"{prefix}/lingua/ground"

        self._client = self.create_client(GroundInstruction, self._service_name)
        self._status_sub = self.create_subscription(
            ExecutionStatus,
            f"{prefix}/lingua/execution_status",
            self._handle_status,
            10,
        )

    def run(self) -> None:
        print(BANNER)
        print(f'📢  Instruction: "{self._instruction}"\n')

        if not self._client.wait_for_service(timeout_sec=5.0):
            print(f"❌  {self._service_name} service not found.")
            print("    Is the grounding node running?")
            print("    Try: ros2 launch ros2_lingua_mock demo.launch.py")
            return

        request = GroundInstruction.Request()
        request.instruction = self._instruction

        print("⏳  Sending to grounding engine...")
        future = self._client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=self._grounding_timeout)

        if future.result() is None:
            print("❌  Grounding timed out. Is the LLM backend running?")
            return

        result = future.result()

        if not result.success:
            print(f"\n❌  Not feasible: {result.message}\n")
            return

        try:
            plan = json.loads(result.plan_json)
        except json.JSONDecodeError:
            print(f"❌  Invalid plan returned: {result.plan_json}")
            return

        steps = plan.get("steps", [])
        print(f"✅  Plan generated — {len(steps)} step(s):\n")
        for i, step in enumerate(steps, 1):
            auto = "  ← auto-chained" if "Auto-inserted" in step.get("rationale", "") else ""
            params_str = ""
            if step.get("parameters"):
                params_str = "  " + str(step["parameters"])
            print(f"   {i}.  {step['capability_name']}{auto}")
            if params_str:
                print(f"       params: {step['parameters']}")
            if step.get("rationale") and "Auto-inserted" not in step["rationale"]:
                print(f"       reason: {step['rationale']}")

        print("\n🚀  Dispatching to robot...\n")

        timeout = time.time() + 60.0
        while time.time() < timeout and not self._done:
            rclpy.spin_once(self, timeout_sec=0.5)

    def _handle_status(self, msg: ExecutionStatus):
        status = msg.status
        step = msg.current_step
        instruction = msg.instruction

        if status == "COMPLETED":
            print(f"\n✅  Execution complete.{f'  Step: {step}' if step else ''}\n")
            self._done = True
        elif status == "STEP_RETRY":
            print(f"   🔄  Retrying step: {step or 'unknown'}")
        elif status == "REPLANNING":
            print(f"   🔄  Replanning...{f'  Step: {step}' if step else ''}")
        elif status == "FALLBACK":
            print(f"   ⚠️  Falling back to: {step or 'unknown'}")
        elif status == "RECOVERY_FAILED":
            print(f"\n❌  Recovery failed for step: {step or 'unknown'}\n")
            self._done = True
        elif status == "CANCELLED":
            print(f"\n⚠️  Execution cancelled.{f'  Step: {step}' if step else ''}\n")
            self._done = True
        elif status == "STEP_STARTED":
            print(f"   ➤  Starting step: {step or 'unknown'}")
        elif status == "STEP_COMPLETE":
            print(f"   ✓  Step complete: {step or 'unknown'}")
        elif status == "FAILED":
            print(f"\n❌  Execution failed at step: {step or 'unknown'}\n")
            self._done = True
        else:
            print(f"   ℹ️  Status: {status}{f'  Step: {step}' if step else ''}")


class CapabilitiesListener(Node):
    """
    Subscribes briefly to lingua/capabilities, caches the first message
    received, then lets the caller print/filter it.
    """

    def __init__(self, namespace: str = ""):
        super().__init__("lingua_cli_list_capabilities")
        prefix = f"/{namespace}" if namespace else ""
        self._capabilities = None
        self._sub = self.create_subscription(
            String, f"{prefix}/lingua/capabilities", self._handle, 10
        )

    def _handle(self, msg: String):
        try:
            self._capabilities = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse capabilities: {e}")

    def wait_for_capabilities(self, timeout_sec: float = 5.0) -> bool:
        """Spin until a capabilities message arrives or the timeout expires."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline and self._capabilities is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self._capabilities is not None


def _print_capabilities(caps: list, tag: str = None) -> None:
    """Pretty-print a list of capability dicts, optionally filtered by tag."""
    if tag:
        caps = [c for c in caps if tag in c.get("tags", [])]

    if not caps:
        print("No capabilities found." + (f" (tag='{tag}')" if tag else ""))
        return

    print(f"📋  {len(caps)} capabilit{'y' if len(caps) == 1 else 'ies'} registered:\n")
    for c in caps:
        print(f"  • {c.get('name', '<unnamed>')}")
        if c.get("description"):
            print(f"      description: {c['description']}")
        if c.get("ros_action"):
            print(f"      ros_action:  {c['ros_action']}")
        if c.get("ros_service"):
            print(f"      ros_service: {c['ros_service']}")
        print()


def main() -> None:
    rclpy.init()

    remaining_args = rclpy.utilities.remove_ros_args(sys.argv)

    parser = argparse.ArgumentParser(
        description="Send a natural language instruction to the ros2_lingua "
        "grounding engine, or list registered capabilities."
    )
    subparsers = parser.add_subparsers(dest="command")

    ground_parser = subparsers.add_parser(
        "ground", help="Send a natural language instruction to the robot"
    )
    ground_parser.add_argument(
        "--namespace",
        default="",
        metavar="NS",
        help="Robot namespace to prefix service and topic names (e.g. robot_1)",
    )
    ground_parser.add_argument(
        "--grounding-timeout",
        type=_positive_timeout,
        default=DEFAULT_GROUNDING_TIMEOUT,
        metavar="SECONDS",
        help="Maximum time to wait for the grounding service (default: 30 seconds)",
    )
    ground_parser.add_argument(
        "instruction",
        nargs="+",
        help="Natural language instruction to send to the robot",
    )

    list_parser = subparsers.add_parser(
        "list-capabilities", help="List currently registered robot capabilities"
    )
    list_parser.add_argument(
        "--namespace",
        default="",
        metavar="NS",
        help="Robot namespace to prefix topic names (e.g. robot_1)",
    )
    list_parser.add_argument(
        "--tag",
        default=None,
        metavar="TAG",
        help="Only show capabilities that include this tag",
    )

    args = parser.parse_args(remaining_args[1:])

    if args.command == "list-capabilities":
        node = CapabilitiesListener(namespace=args.namespace)
        try:
            found = node.wait_for_capabilities(timeout_sec=5.0)
            if not found:
                print("❌  No capabilities received within 5s.")
                print("    Is the DispatcherNode running and broadcasting?")
            else:
                _print_capabilities(node._capabilities, tag=args.tag)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    if args.command != "ground" or not args.instruction:
        print("Usage: cli ground <instruction>  |  cli list-capabilities [--tag TAG]")
        rclpy.shutdown()
        return

    instruction = " ".join(args.instruction)

    cli = LinguaCLI(
        instruction,
        namespace=args.namespace,
        grounding_timeout=args.grounding_timeout,
    )
    try:
        cli.run()
    except KeyboardInterrupt:
        pass
    finally:
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
