"""
ros2_lingua_mock.robot_monitor
--------------------------------
A lightweight monitor node that subscribes to all mock robot log topics
and prints a clean, unified real-time stream.

This is what you'd show on screen during a ROScon demo — a single
terminal showing everything the robot is doing in plain English.

Run it alongside the other nodes:
    ros2 run ros2_lingua_mock robot_monitor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class RobotMonitor(Node):
    """
    Aggregates and pretty-prints all mock robot activity.
    """

    def __init__(self):
        super().__init__("robot_monitor")

        self._log_sub = self.create_subscription(
            String, "/mock/robot_log", self._handle_log, 50
        )
        self._location_sub = self.create_subscription(
            String, "/mock/current_location", self._handle_location, 10
        )
        self._balance_sub = self.create_subscription(
            Bool, "/mock/balance_status", self._handle_balance, 10
        )
        self._speech_sub = self.create_subscription(
            String, "/mock/speech_output", self._handle_speech, 10
        )
        self._plan_sub = self.create_subscription(
            String, "/lingua/current_plan", self._handle_plan, 10
        )
        self._status_sub = self.create_subscription(
            String, "/lingua/execution_status", self._handle_status, 10
        )

        self._last_location = "home"
        self._is_balanced = False

        print("\n" + "="*60)
        print("  🤖  ros2_lingua Robot Monitor")
        print("="*60 + "\n")

    def _handle_log(self, msg: String):
        print(f"  {msg.data}")

    def _handle_location(self, msg: String):
        if msg.data != self._last_location:
            self._last_location = msg.data
            print(f"  📍 Location updated: {msg.data}")

    def _handle_balance(self, msg: Bool):
        if msg.data != self._is_balanced:
            self._is_balanced = msg.data
            status = "BALANCED ✅" if msg.data else "UNBALANCED ⚠️"
            print(f"  ⚖️  Balance status: {status}")

    def _handle_speech(self, msg: String):
        print(f'\n  🔊 Robot says: "{msg.data}"\n')

    def _handle_plan(self, msg: String):
        import json
        try:
            plan = json.loads(msg.data)
            steps = plan.get("steps", [])
            instruction = plan.get("original_instruction", "")
            print(f"\n{'─'*60}")
            print(f"  📋 New plan for: \"{instruction}\"")
            for i, step in enumerate(steps, 1):
                auto = " [auto]" if "Auto-inserted" in step.get("rationale", "") else ""
                print(f"     {i}. {step['capability_name']}{auto}")
            print(f"{'─'*60}\n")
        except Exception:
            pass

    def _handle_status(self, msg):
        pass  # ExecutionStatus handled via robot_log


def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[Monitor] Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
