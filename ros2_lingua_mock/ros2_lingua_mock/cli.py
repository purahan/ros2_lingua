"""
ros2_lingua_mock.cli
---------------------
A clean CLI tool for sending natural language instructions to the robot.

Usage:
    ros2 run ros2_lingua_mock cli "go to the table and pick up the bottle"
    ros2 run ros2_lingua_mock cli --namespace robot_1 "go to the table and pick up the bottle"

Or as a ros2 verb (if installed):
    ros2 lingua ground "go to the table and pick up the bottle"

Prints the resulting plan in a human-readable format and shows
live execution status as the dispatcher works through the steps.
"""

import argparse
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from ros2_lingua_interfaces.srv import GroundInstruction


BANNER = """
╔══════════════════════════════════════════╗
║         ros2_lingua  —  CLI Tool         ║
╚══════════════════════════════════════════╝
"""


class LinguaCLI(Node):

    def __init__(self, instruction: str, namespace: str = ""):
        super().__init__("lingua_cli")
        self._instruction = instruction

        prefix = f"/{namespace}" if namespace else ""
        self._done = False

        self._client = self.create_client(GroundInstruction, f"{prefix}/lingua/ground")
        self._status_sub = self.create_subscription(
            String, f"{prefix}/lingua/execution_status", self._handle_status, 10
        )

    def run(self):
        print(BANNER)
        print(f'📢  Instruction: "{self._instruction}"\n')

        if not self._client.wait_for_service(timeout_sec=5.0):
            print("❌  /lingua/ground service not found.")
            print("    Is the grounding node running?")
            print("    Try: ros2 launch ros2_lingua_mock demo.launch.py")
            return

        request = GroundInstruction.Request()
        request.instruction = self._instruction

        print("⏳  Sending to grounding engine...")
        future = self._client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is None:
            print("❌  Grounding timed out. Is the LLM backend running?")
            return

        result = future.result()

        if not result.success:
            print(f"\n❌  Not feasible: {result.message}\n")
            return

        # Parse and display the plan
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

        # Wait a moment for execution status updates
        timeout = time.time() + 60.0
        while time.time() < timeout and not self._done:
            rclpy.spin_once(self, timeout_sec=0.5)

    def _handle_status(self, msg: String):
        # ExecutionStatus is published as a string in the mock
        # (the full message type comes in a later iteration)
        status = msg.data if hasattr(msg, 'data') else str(msg)
        if "COMPLETED" in status:
            print("\n✅  Execution complete.\n")
            self._done = True
        elif "FAILED" in status:
            print(f"\n❌  Execution failed: {status}\n")
            self._done = True
        elif "STEP_COMPLETE" in status:
            print(f"   ✓  Step done")


def main():
    parser = argparse.ArgumentParser(
        description="Send a natural language instruction to the ros2_lingua grounding engine."
    )
    parser.add_argument(
        "--namespace",
        default="",
        metavar="NS",
        help="Robot namespace to prefix service and topic names (e.g. robot_1)",
    )
    parser.add_argument(
        "instruction",
        nargs="+",
        help="Natural language instruction to send to the robot",
    )
    args = parser.parse_args()

    instruction = " ".join(args.instruction)

    rclpy.init()
    cli = LinguaCLI(instruction, namespace=args.namespace)
    try:
        cli.run()
    except KeyboardInterrupt:
        pass
    finally:
        cli.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
