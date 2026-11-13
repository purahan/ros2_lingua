"""
ros2_lingua_mock.manipulation_node
------------------------------------
Simulates a robot arm / manipulation controller.

Advertises capabilities:
  - pick_up_object : picks up a named object
  - place_object   : places the held object on a named surface
  - wave_hand      : waves at a person (social gesture)
  - open_door      : opens a door

Serves the generic LinguaAction interface at mock_manipulation_node/lingua
(see ros2_lingua.lingua_action_server).

In a real robot, this would wrap MoveIt 2, a custom arm controller,
or your robot's manipulation stack.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ros2_lingua.capability_mixin import LinguaMixin
from ros2_lingua.lingua_action_server import LinguaActionServer
from ros2_lingua_core import Capability, CapabilityParameter

# Simulated pick durations per object type (seconds)
OBJECT_PICK_TIMES = {
    "bottle":   3.5,
    "cup":      3.0,
    "book":     2.5,
    "box":      4.0,
    "tool":     3.5,
    "phone":    2.5,
}

DEFAULT_PICK_TIME = 3.0


class MockManipulationNode(LinguaMixin, LinguaActionServer, Node):
    """
    Simulates arm/gripper control with realistic timing.

    Tracks what the robot is currently holding and returns the matching
    state changes (object_in_hand, arm_is_free) in each result.
    """

    CAPABILITIES = ["pick_up_object", "place_object", "wave_hand", "open_door"]

    def __init__(self):
        Node.__init__(self, "mock_manipulation_node")
        LinguaMixin.__init__(self)
        LinguaActionServer.__init__(self, self.CAPABILITIES)

        self._held_object = None    # None means arm is free

        # --- Publishers ---
        self._log_pub = self.create_publisher(String, "/mock/robot_log", 10)
        self._arm_state_pub = self.create_publisher(String, "/mock/arm_state", 10)

        # Publish arm state at 1Hz
        self._state_timer = self.create_timer(1.0, self._publish_arm_state)

        action_name = f"{self.get_name()}/lingua"

        # Register capabilities
        self.register_lingua_capability(Capability(
            name="pick_up_object",
            description=(
                "Picks up a named object using the robot's arm and gripper. "
                "The robot must be at the object's location before picking."
            ),
            ros_action=action_name,
            parameters=[
                CapabilityParameter(
                    name="object_name",
                    type="string",
                    description=(
                        "Name of the object to pick up. "
                        "Known objects: " + ", ".join(OBJECT_PICK_TIMES.keys())
                    ),
                    required=True,
                ),
                CapabilityParameter(
                    name="arm",
                    type="string",
                    description="Which arm to use: 'left', 'right', or 'auto'",
                    required=False,
                    default="auto",
                ),
            ],
            preconditions=["robot_at_location", "object_in_view", "arm_is_free"],
            postconditions=["object_in_hand"],
            metadata={"max_payload_kg": 1.5},
            tags=["manipulation"],
        ))

        self.register_lingua_capability(Capability(
            name="place_object",
            description=(
                "Places the currently held object on a named surface. "
                "The robot must be holding something and be at the target location."
            ),
            ros_action=action_name,
            parameters=[
                CapabilityParameter(
                    name="surface_name",
                    type="string",
                    description="Surface to place the object on, e.g. 'table', 'shelf', 'floor'",
                    required=True,
                ),
            ],
            preconditions=["object_in_hand", "robot_at_location"],
            postconditions=["object_placed", "arm_is_free"],
            metadata={},
            tags=["manipulation"],
        ))

        self.register_lingua_capability(Capability(
            name="wave_hand",
            description="Waves at a person as a greeting gesture.",
            ros_action=action_name,
            parameters=[
                CapabilityParameter(
                    name="hand",
                    type="string",
                    description="Which hand to wave with: 'left', 'right'",
                    required=False,
                    default="right",
                ),
            ],
            preconditions=["robot_is_balanced", "arm_is_free"],
            postconditions=[],
            metadata={"social_gesture": True},
            tags=["manipulation", "social"],
        ))

        self.register_lingua_capability(Capability(
            name="open_door",
            description="Opens a door.",
            ros_action=action_name,
            parameters=[
                CapabilityParameter(
                    name="door",
                    type="string",
                    description="Door to open.",
                    required=False,
                    default="nearest",
                ),
            ],
            preconditions=["arm_is_free"],
            postconditions=[],
            metadata={},
            tags=["manipulation"],
        ))

        # Report the initial state (arm free, object in view) once registered
        self.publish_lingua_current_state()

        self._log("MockManipulationNode ready. Arm is free.")
        self.get_logger().info("MockManipulationNode ready.")

    def lingua_current_state(self):
        # The mock always has an object in view; the arm is free unless holding something
        if self._held_object is None:
            return ["arm_is_free", "object_in_view"], ["object_in_hand"]
        return ["object_in_hand", "object_in_view"], ["arm_is_free"]

    async def execute_capability(self, capability_name, parameters, goal_handle):
        if capability_name == "pick_up_object":
            return self._pick(str(parameters.get("object_name", "")).strip(), goal_handle)
        if capability_name == "place_object":
            return self._place(str(parameters.get("surface_name", "")).strip(), goal_handle)
        if capability_name == "wave_hand":
            return self._wave(str(parameters.get("hand", "right")), goal_handle)
        if capability_name == "open_door":
            return self._open_door(str(parameters.get("door", "nearest")), goal_handle)
        return False, f"Unknown capability: {capability_name}", [], []

    def _pick(self, object_name: str, goal_handle):
        if not object_name:
            return False, "pick_up_object needs an object_name", [], []
        if self._held_object is not None:
            self._log(f"❌ Pick failed: already holding '{self._held_object}'.")
            return False, f"Already holding '{self._held_object}'", [], []

        pick_time = OBJECT_PICK_TIMES.get(object_name, DEFAULT_PICK_TIME)

        self.get_logger().info(f"Manipulation: picking up '{object_name}'...")
        self._log(f"🦾 Reaching for '{object_name}'...")

        # Simulate arm movement phases
        phases = [
            (pick_time * 0.3, "Planning grasp trajectory..."),
            (pick_time * 0.4, "Executing reach..."),
            (pick_time * 0.3, "Closing gripper..."),
        ]
        for i, (duration, phase_msg) in enumerate(phases):
            if self._sleep_unless_cancelled(goal_handle, duration, phase_msg, i / len(phases)):
                self._log(f"❌ Pick of '{object_name}' cancelled.")
                return False, f"Pick of '{object_name}' cancelled", [], []

        self._held_object = object_name

        self._log(f"✅ Picked up '{object_name}'.")
        self.get_logger().info(f"Manipulation: picked up '{object_name}'.")

        return True, f"Picked up '{object_name}'", ["object_in_hand"], ["arm_is_free"]

    def _place(self, surface: str, goal_handle):
        if not surface:
            return False, "place_object needs a surface_name", [], []
        if self._held_object is None:
            self._log("❌ Place failed: not holding anything.")
            return False, "Not holding anything", [], []

        self.get_logger().info(f"Manipulation: placing '{self._held_object}' on '{surface}'...")
        self._log(f"📦 Placing '{self._held_object}' on '{surface}'...")

        if self._sleep_unless_cancelled(goal_handle, 2.5, f"Placing on {surface}..."):
            self._log("❌ Place cancelled.")
            return False, "Place cancelled", [], []

        placed = self._held_object
        self._held_object = None

        self._log(f"✅ Placed '{placed}' on '{surface}'.")
        self.get_logger().info(f"Manipulation: placed '{placed}' on '{surface}'.")

        return (
            True,
            f"Placed '{placed}' on '{surface}'",
            ["object_placed", "arm_is_free"],
            ["object_in_hand"],
        )

    def _wave(self, hand: str, goal_handle):
        self.get_logger().info(f"Manipulation: waving {hand} hand...")
        self._log(f"👋 Waving {hand} hand...")
        if self._sleep_unless_cancelled(goal_handle, 2.0, "Waving..."):
            return False, "Wave cancelled", [], []
        self._log("✅ Wave complete.")
        return True, f"Waved {hand} hand", [], []

    def _open_door(self, door: str, goal_handle):
        self.get_logger().info(f"Manipulation: opening {door}...")
        self._log(f"🚪 Opening {door}...")
        if self._sleep_unless_cancelled(goal_handle, 2.0, f"Opening {door}..."):
            return False, "Open door cancelled", [], []
        self._log("✅ Door opened.")
        return True, f"Opened {door}", [], []

    def _sleep_unless_cancelled(self, goal_handle, duration, status, progress=0.0) -> bool:
        """Sleep for duration in small slices. Returns True if the goal was cancelled."""
        self._log(f"  {status}")
        goal_handle.publish_feedback(self._make_feedback(status, progress))
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return True
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        return goal_handle.is_cancel_requested

    def _publish_arm_state(self):
        msg = String()
        msg.data = f"holding:{self._held_object or 'nothing'}"
        self._arm_state_pub.publish(msg)

    def _log(self, message: str):
        msg = String()
        msg.data = f"[MANIPULATION] {message}"
        self._log_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockManipulationNode()
    # Multi-threaded so cancel requests are handled while a goal is executing
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()  # SIGINT may already have shut the context down


if __name__ == "__main__":
    main()
