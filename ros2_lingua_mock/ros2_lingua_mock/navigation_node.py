"""
ros2_lingua_mock.navigation_node
---------------------------------
Simulates a robot navigation / locomotion controller.

Advertises capabilities:
  - navigate_to_location : moves the robot to a named location
  - return_to_home       : returns the robot to its home position

Serves the generic LinguaAction interface at mock_navigation_node/lingua
(see ros2_lingua.lingua_action_server).

In a real robot, this would wrap nav2, a custom locomotion controller,
or whatever drives your robot from A to B.
"""

import time

import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ros2_lingua.capability_mixin import LinguaMixin
from ros2_lingua.lingua_action_server import LinguaActionServer
from ros2_lingua_core import Capability, CapabilityParameter

# Simulated map of named locations and how long it takes to walk there (seconds)
LOCATION_WALK_TIMES = {
    "table": 3.0,
    "door": 4.0,
    "shelf": 3.5,
    "charging_dock": 5.0,
    "home": 3.0,
    "kitchen": 6.0,
    "window": 4.5,
}

DEFAULT_WALK_TIME = 4.0  # for locations not in the map


class MockNavigationNode(LinguaMixin, LinguaActionServer, Node):
    """
    Simulates navigation by waiting a realistic amount of time
    and then reporting arrival.

    Tracks the current location and returns robot_at_location in the result.
    """

    CAPABILITIES = ["navigate_to_location", "return_to_home"]

    def __init__(self):
        Node.__init__(self, "mock_navigation_node")
        LinguaMixin.__init__(self)
        LinguaActionServer.__init__(self, self.CAPABILITIES)

        self._current_location = "home"
        self._arrived = False  # True once a navigation has finished at a location

        # --- Publishers ---
        self._log_pub = self.create_publisher(String, "/mock/robot_log", 10)
        self._location_pub = self.create_publisher(String, "/mock/current_location", 10)

        # Publish location at 1Hz
        self._location_timer = self.create_timer(1.0, self._publish_location)

        # Register capabilities
        self.register_lingua_capability(
            Capability(
                name="navigate_to_location",
                description=(
                    "Moves the robot to a named location in the environment. "
                    "The robot must be balanced before navigation can begin."
                ),
                ros_action=f"{self.get_name()}/lingua",
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
            )
        )

        self.register_lingua_capability(
            Capability(
                name="return_to_home",
                description="Returns the robot to its home/starting position.",
                ros_action=f"{self.get_name()}/lingua",
                parameters=[],
                preconditions=["robot_is_balanced"],
                postconditions=["robot_at_location"],
                metadata={},
                tags=["locomotion", "navigation"],
            )
        )

        self._log(f"MockNavigationNode ready. Starting at: {self._current_location}")
        self.get_logger().info("MockNavigationNode ready.")

    def lingua_current_state(self):
        if self._arrived:
            return ["robot_at_location"], []
        return [], []

    async def execute_capability(self, capability_name, parameters, goal_handle):
        if capability_name == "navigate_to_location":
            location = str(parameters.get("location_name", "")).strip()
            if not location:
                return False, "navigate_to_location needs a location_name", [], []
            return self._walk_to(location, goal_handle)
        if capability_name == "return_to_home":
            return self._walk_to("home", goal_handle)
        return False, f"Unknown capability: {capability_name}", [], []

    def _walk_to(self, location: str, goal_handle):
        walk_time = LOCATION_WALK_TIMES.get(location, DEFAULT_WALK_TIME)
        self._arrived = False  # left the previous location

        self.get_logger().info(f"Navigation: walking to '{location}'...")
        self._log(f"🚶 Navigating to '{location}' (ETA: {walk_time}s)...")

        steps = int(walk_time / 0.5)
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                self._log(f"❌ Navigation to '{location}' cancelled.")
                # The robot stopped somewhere along the way
                return False, f"Navigation to '{location}' cancelled", [], ["robot_at_location"]
            time.sleep(0.5)
            goal_handle.publish_feedback(
                self._make_feedback(f"Walking to {location}...", (i + 1) / steps)
            )

        # Arrived
        self._current_location = location
        self._arrived = True
        self._publish_location()

        self._log(f"✅ Arrived at '{location}'.")
        self.get_logger().info(f"Navigation: arrived at '{location}'.")

        return True, f"Arrived at '{location}'", ["robot_at_location"], []

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
