"""
ros2_lingua_mock.navigation_node
--------------------------------
Simulates a robot navigation / locomotion controller.

Advertises capabilities:
  - navigate_to_location : moves the robot to a named location
  - return_to_home       : returns the robot to its home position

In a real robot, this would wrap nav2, a custom locomotion controller,
or whatever drives your robot from A to B.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from ros2_lingua.capability_mixin import LinguaMixin
from ros2_lingua.lingua_action_server import LinguaActionServer
from ros2_lingua_core import Capability, CapabilityParameter


# Simulated map of named locations and how long it takes to walk there (seconds)
LOCATION_WALK_TIMES = {
    "table":        3.0,
    "door":         4.0,
    "shelf":        3.5,
    "charging_dock": 5.0,
    "home":         3.0,
    "kitchen":      6.0,
    "window":       4.5,
}

DEFAULT_WALK_TIME = 4.0   # for locations not in the map


class MockNavigationNode(LinguaMixin, LinguaActionServer, Node):
    """
    Simulates navigation by waiting a realistic amount of time
    and then reporting arrival.

    Tracks current location and updates lingua state accordingly.
    """

    def __init__(self):
        Node.__init__(self, "mock_navigation_node")
        LinguaMixin.__init__(self)
        LinguaActionServer.__init__(self, ["navigate_to_location", "return_to_home"])

        self._callback_group = ReentrantCallbackGroup()
        self._current_location = "home"

        # --- Publishers ---
        self._log_pub = self.create_publisher(String, "/mock/robot_log", 10)
        self._location_pub = self.create_publisher(String, "/mock/current_location", 10)

        # Publish location at 1Hz
        self._location_timer = self.create_timer(1.0, self._publish_location)

        # Register capabilities
        self.register_lingua_capability(Capability(
            name="navigate_to_location",
            description=(
                "Moves the robot to a named location in the environment. "
                "The robot must be balanced before navigation can begin."
            ),
            ros_action="mock_navigation_node/lingua",
            parameters=[
                CapabilityParameter(
                    name="location_name",
                    type="string",
                    description=(
                        "Name of the destination. Known locations: "
                        + ", ".join(LOCATION_WALK_TIMES.keys())
                    ),
                    required=True,
                ),
                CapabilityParameter(
                    name="speed",
                    type="float",
                    description="Speed as fraction of maximum (0.1 - 1.0)",
                    required=False,
                    default=0.5,
                ),
            ],
            preconditions=["robot_is_balanced"],
            postconditions=["robot_at_location"],
            metadata={},
            tags=["locomotion", "navigation"],
        ))

        self.register_lingua_capability(Capability(
            name="return_to_home",
            description="Returns the robot to its home/starting position.",
            ros_action="mock_navigation_node/lingua",
            parameters=[],
            preconditions=["robot_is_balanced"],
            postconditions=["robot_at_location"],
            metadata={},
            tags=["locomotion", "navigation"],
        ))

        self._log(f"MockNavigationNode ready. Starting at: {self._current_location}")
        self.get_logger().info("MockNavigationNode ready.")

    def execute_capability(self, capability_name, parameters, goal_handle):
        """Execute a navigation capability."""
        if capability_name == "navigate_to_location":
            return self._navigate_to_location(parameters, goal_handle)
        elif capability_name == "return_to_home":
            return self._return_to_home(parameters, goal_handle)
        else:
            return False, f"Unknown capability: {capability_name}", [], []

    def _navigate_to_location(self, parameters, goal_handle):
        location = parameters.get("location_name", "table")
        walk_time = LOCATION_WALK_TIMES.get(location, DEFAULT_WALK_TIME)

        self.get_logger().info(f"Navigation: walking to '{location}'...")
        self._log(f"🚶 Navigating to '{location}' (ETA: {walk_time}s)...")

        # Clear previous location state
        self.update_lingua_state(clear_tokens=["robot_at_location"])

        steps = int(walk_time / 0.5)
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._log(f"❌ Navigation to '{location}' cancelled.")
                return False, f"Cancelled navigation to '{location}'", [], []
            time.sleep(0.5)

        # Arrived
        self._current_location = location
        self._publish_location()
        self.update_lingua_state(set_tokens=["robot_at_location"])

        self._log(f"✅ Arrived at '{location}'.")
        self.get_logger().info(f"Navigation: arrived at '{location}'.")

        return True, f"Arrived at {location}", ["robot_at_location"], []

    def _return_to_home(self, parameters, goal_handle):
        self.get_logger().info("Navigation: returning home...")
        self._log("🏠 Returning to home position...")

        self.update_lingua_state(clear_tokens=["robot_at_location"])

        time.sleep(3.0)

        self._current_location = "home"
        self._publish_location()
        self.update_lingua_state(set_tokens=["robot_at_location"])

        self._log("✅ Returned home.")
        goal_handle.succeed()
        return True, "Returned home", ["robot_at_location"], []

    def _publish_location(self):
        msg = String()
        msg.data = self._current_location
        self._location_pub.publish(msg)

    def _log(self, message: str):
        msg = String()
        msg.data = f"[NAVIGATION] {message}"
        self._log_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()