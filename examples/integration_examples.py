"""
examples/integration_examples.py
----------------------------------
Complete working examples for all three integration levels.

These are not runnable as-is (they reference robot-specific types)
but show exactly what integration looks like for common robot setups.

Copy the example that matches your situation and adapt it.
"""

# ==================================================================
# LEVEL 1 — LinguaActionServer (recommended for new robots)
# ==================================================================
#
# Your node implements the generic LinguaAction interface.
# The dispatcher calls it automatically. No configuration needed.
# This is the path to take if you're building a new robot node.

LEVEL_1_EXAMPLE = '''
from rclpy.node import Node
from ros2_lingua import LinguaMixin
from ros2_lingua.lingua_action_server import LinguaActionServer
from ros2_lingua_core import Capability, CapabilityParameter, Tags
import asyncio

LOCATIONS = {
    "table":        (1.5, 0.8),
    "door":         (3.2, 0.0),
    "charging_dock": (-1.0, 0.5),
}

class NavigationNode(LinguaMixin, LinguaActionServer, Node):

    CAPABILITIES = ["navigate_to_location", "return_to_home"]

    def __init__(self):
        Node.__init__(self, "navigation_node")
        LinguaMixin.__init__(self)
        LinguaActionServer.__init__(self, self.CAPABILITIES)

        # Register capabilities with lingua
        self.register_lingua_capability(Capability(
            name="navigate_to_location",
            description="Drives the robot to a named location. "
                        "Known: " + ", ".join(LOCATIONS.keys()),
            ros_action="navigation_node/lingua",
            parameters=[
                CapabilityParameter(
                    name="location_name",
                    type="string",
                    description="Destination name",
                    required=True,
                ),
            ],
            preconditions=["robot_is_ready"],
            postconditions=["robot_at_location"],
            tags=[Tags.LOCOMOTION],
        ))

        # Set initial state
        self.update_lingua_state(set_tokens=["robot_is_ready"])

    async def execute_capability(self, capability_name, parameters, goal_handle):
        if capability_name == "navigate_to_location":
            location = parameters.get("location_name", "unknown")
            if location not in LOCATIONS:
                return False, f"Unknown location: {location}", [], []

            goal_handle.publish_feedback(
                self._make_feedback(f"Navigating to {location}...", 0.0)
            )

            # --- your actual navigation call here ---
            # e.g. result = await self._nav2_client.send_goal_async(goal)

            await asyncio.sleep(3.0)  # simulate navigation

            goal_handle.publish_feedback(
                self._make_feedback(f"Arrived at {location}", 1.0)
            )
            return (
                True,
                f"Arrived at {location}",
                ["robot_at_location"],   # state_set
                [],                      # state_clear
            )

        if capability_name == "return_to_home":
            await asyncio.sleep(2.0)
            return True, "Returned home", ["robot_at_location"], []

        return False, f"Unknown capability: {capability_name}", [], []
'''


# ==================================================================
# LEVEL 2 — DispatchConfig (for existing robots with typed interfaces)
# ==================================================================
#
# You already have a nav2 stack, MoveIt 2, or your own action servers.
# You don't want to change them. Just configure a mapping.

LEVEL_2_NAV2_EXAMPLE = '''
from ros2_lingua.dispatch_config import DispatchConfig
from ros2_lingua.dispatcher_node import DispatcherNode
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import rclpy

# Known location coordinates in the map frame
LOCATION_MAP = {
    "table":        (1.5, 0.8, 0.0),
    "door":         (3.2, 0.0, 0.0),
    "charging_dock": (-1.0, 0.5, 0.0),
    "kitchen":      (5.0, 2.1, 0.0),
}

def nav2_goal_adapter(params: dict) -> NavigateToPose.Goal:
    """Convert lingua parameters to a NavigateToPose goal."""
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    location = params.get("location_name", "table")
    x, y, yaw = LOCATION_MAP.get(location, (0.0, 0.0, 0.0))
    goal.pose.pose.position.x = x
    goal.pose.pose.position.y = y
    # Could also set orientation from yaw here
    return goal

def nav2_success_checker(result) -> bool:
    """Check if NavigateToPose succeeded."""
    # nav2 returns a result with error_code=0 on success
    return result.error_code == 0

config = DispatchConfig()
config.register_action(
    "navigate_to_location",
    NavigateToPose,
    "navigate_to_pose",           # your nav2 action server name
    goal_adapter=nav2_goal_adapter,
    result_checker=nav2_success_checker,
    timeout_sec=60.0,
)

# Launch the dispatcher with the config
def main(args=None):
    rclpy.init(args=args)
    node = DispatcherNode(dispatch_config=config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
'''


LEVEL_2_MOVEIT_EXAMPLE = '''
from ros2_lingua.dispatch_config import DispatchConfig
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints

def moveit_pick_adapter(params: dict) -> MoveGroup.Goal:
    """Convert lingua pick_up_object params to MoveGroup goal."""
    goal = MoveGroup.Goal()
    # ... set up your MoveIt goal from params["object_name"] ...
    return goal

config = DispatchConfig()
config.register_action(
    "pick_up_object",
    MoveGroup,
    "move_action",
    goal_adapter=moveit_pick_adapter,
    timeout_sec=30.0,
)
'''


# ==================================================================
# LEVEL 3 — Subclass override (full control)
# ==================================================================
#
# Override _call_action_typed() for maximum flexibility.
# Use this when Levels 1 and 2 don't fit your architecture.

LEVEL_3_EXAMPLE = '''
from ros2_lingua.dispatcher_node import DispatcherNode
from rclpy.action import ActionClient
import rclpy

class MyRobotDispatcher(DispatcherNode):

    def __init__(self):
        super().__init__()
        # Pre-create your action clients
        from my_robot_interfaces.action import NavigateTo, PickObject
        self._nav_client  = ActionClient(self, NavigateTo,  "navigate")
        self._pick_client = ActionClient(self, PickObject, "pick_object")

    def _call_action_typed(self, action_name, cap_name, params):
        if cap_name == "navigate_to_location":
            from my_robot_interfaces.action import NavigateTo
            goal = NavigateTo.Goal()
            goal.location_name = params.get("location_name", "")
            goal.speed = float(params.get("speed", 0.5))
            # send goal, wait, return True/False
            return self._send_and_wait(self._nav_client, goal)

        if cap_name == "pick_up_object":
            from my_robot_interfaces.action import PickObject
            goal = PickObject.Goal()
            goal.object_name = params.get("object_name", "")
            return self._send_and_wait(self._pick_client, goal)

        return super()._call_action_typed(action_name, cap_name, params)

    def _send_and_wait(self, client, goal, timeout=30.0):
        if not client.wait_for_server(timeout_sec=5.0):
            return False
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.result() or not future.result().accepted:
            return False
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        return result_future.result() is not None
'''

if __name__ == "__main__":
    print("Integration level examples — see source comments for usage.")
    print()
    print("Level 1 (LinguaActionServer):")
    print(LEVEL_1_EXAMPLE)
    print("Level 2 (DispatchConfig with nav2):")
    print(LEVEL_2_NAV2_EXAMPLE)
    print("Level 3 (subclass override):")
    print(LEVEL_3_EXAMPLE)
