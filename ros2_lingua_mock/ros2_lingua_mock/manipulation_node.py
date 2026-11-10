"""
ros2_lingua_mock.manipulation_node
----------------------------------
Simulates a robot arm / manipulation controller.

Advertises capabilities:
  - pick_up_object : picks up a named object
  - place_object   : places the held object on a named surface
  - wave_hand      : waves at a person (social gesture)
  - open_door       : opens a door 

In a real robot, this would wrap MoveIt 2, a custom arm controller,
or your robot's manipulation stack.
"""

import time
import rclpy
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

    Tracks what the robot is currently holding and updates
    lingua state (object_in_hand, arm_is_free) accordingly.
    """

    def __init__(self):
        Node.__init__(self, "mock_manipulation_node")
        LinguaMixin.__init__(self)
        LinguaActionServer.__init__(self, [
            "pick_up_object",
            "place_object",
            "wave_hand",
            "open_door",
        ])

        self._callback_group = ReentrantCallbackGroup()
        self._held_object = None    # None means arm is free

        # --- Publishers ---
        self._log_pub = self.create_publisher(String, "/mock/robot_log", 10)
        self._arm_state_pub = self.create_publisher(String, "/mock/arm_state", 10)

        # Publish arm state at 1Hz
        self._state_timer = self.create_timer(1.0, self._publish_arm_state)

        # Set initial lingua state — arm starts free
        self.update_lingua_state(set_tokens=["arm_is_free", "object_in_view"])

        # Register capabilities
        self.register_lingua_capability(Capability(
            name="pick_up_object",
            description=(
                "Picks up a named object using the robot's arm and gripper. "
                "The robot must be at the object's location before picking."
            ),
            ros_action="mock_manipulation_node/lingua",
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
            ros_action="mock_manipulation_node/lingua",
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
            ros_action="mock_manipulation_node/lingua",
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
            ros_action="mock_manipulation_node/lingua",
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

        self._log("MockManipulationNode ready. Arm is free.")
        self.get_logger().info("MockManipulationNode ready.")

    def execute_capability(self, capability_name, parameters, goal_handle):
        """Execute a manipulation capability."""
        if capability_name == "pick_up_object":
            return self._execute_pick(parameters, goal_handle)
        elif capability_name == "place_object":
            return self._execute_place(parameters, goal_handle)
        elif capability_name == "wave_hand":
            return self._execute_wave(parameters, goal_handle)
        elif capability_name == "open_door":
            return self._execute_open_door(parameters, goal_handle)
        else:
            return False, f"Unknown capability: {capability_name}", [], []

    def _execute_pick(self, parameters, goal_handle):
        object_name = parameters.get("object_name", "bottle")
        pick_time = OBJECT_PICK_TIMES.get(object_name, DEFAULT_PICK_TIME)

        self.get_logger().info(f"Manipulation: picking up '{object_name}'...")
        self._log(f"🦾 Reaching for '{object_name}'...")

        self.update_lingua_state(clear_tokens=["arm_is_free"])

        # Simulate arm movement phases
        phases = [
            (pick_time * 0.3, "Planning grasp trajectory..."),
            (pick_time * 0.4, "Executing reach..."),
            (pick_time * 0.3, "Closing gripper..."),
        ]
        for duration, phase_msg in phases:
            if goal_handle.is_cancel_requested:
                self.update_lingua_state(set_tokens=["arm_is_free"])
                goal_handle.canceled()
                return False, f"Cancelled picking up '{object_name}'", [], []
            self._log(f"  {phase_msg}")
            time.sleep(duration)

        self._held_object = object_name
        self.update_lingua_state(set_tokens=["object_in_hand"])

        self._log(f"✅ Picked up '{object_name}'.")
        self.get_logger().info(f"Manipulation: picked up '{object_name}'.")

        return True, f"Picked up {object_name}", ["object_in_hand"], []

    def _execute_place(self, parameters, goal_handle):
        surface = parameters.get("surface_name", "table")

        if self._held_object is None:
            self._log("❌ Place failed: not holding anything.")
            goal_handle.canceled()
            return False, "Failed: not holding anything", [], []

        self.get_logger().info(f"Manipulation: placing '{self._held_object}' on '{surface}'...")
        self._log(f"📦 Placing '{self._held_object}' on '{surface}'...")

        time.sleep(2.5)

        placed = self._held_object
        self._held_object = None
        self.update_lingua_state(
            set_tokens=["object_placed", "arm_is_free"],
            clear_tokens=["object_in_hand"],
        )

        self._log(f"✅ Placed '{placed}' on '{surface}'.")
        self.get_logger().info(f"Manipulation: placed '{placed}' on '{surface}'.")

        return True, f"Placed {placed} on {surface}", ["object_placed", "arm_is_free"], ["object_in_hand"]

    def _execute_wave(self, parameters, goal_handle):
        hand = parameters.get("hand", "right")
        self.get_logger().info(f"Manipulation: waving {hand} hand...")
        self._log(f"👋 Waving {hand} hand...")
        time.sleep(2.0)
        self._log("✅ Wave complete.")
        goal_handle.succeed()
        return True, "Waved", [], []

    def _execute_open_door(self, parameters, goal_handle):
        door = parameters.get("door", "nearest")
        self.get_logger().info(f"Manipulation: opening {door}...")
        self._log(f"🚪 Opening {door}...")
        time.sleep(2.0)
        self._log("✅ Door opened.")
        return True, "Opened door", [], []

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()