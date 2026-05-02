"""
examples/humanoid_demo/humanoid_demo.py
----------------------------------------
A complete demonstration of ros2_lingua running on a humanoid robot.

This example shows:
1. A NavigationNode that registers a "navigate_to_location" capability
2. A ManipulationNode that registers "pick_up_object" and "place_object"
3. A BalanceNode that registers "stabilize_robot" and manages state
4. The GroundingEngine receiving "go pick up the bottle from the table"
   and automatically chaining: stabilize -> navigate -> pick_up_object

Run this demo WITHOUT a full ROS 2 setup to see the core logic in action:
    python3 humanoid_demo.py

For a full ROS 2 launch, use the launch file:
    ros2 launch ros2_lingua humanoid_demo.launch.py
"""

import json
from ros2_lingua_core import (
    Capability,
    CapabilityParameter,
    CapabilityRegistry,
    GroundingEngine,
    MockBackend,
)


# ------------------------------------------------------------------
# Define humanoid robot capabilities
# ------------------------------------------------------------------

STABILIZE = Capability(
    name="stabilize_robot",
    description="Activates the balance controller and ensures the robot is stable before movement",
    ros_action="humanoid/stabilize",
    parameters=[],
    preconditions=[],
    postconditions=["robot_is_balanced"],
)

NAVIGATE = Capability(
    name="navigate_to_location",
    description="Walks the robot to a named location in the environment",
    ros_action="humanoid/navigate_to_pose",
    parameters=[
        CapabilityParameter(
            name="location_name",
            type="string",
            description="Named location to walk to, e.g. 'table', 'door', 'charging_dock'",
            required=True,
        ),
        CapabilityParameter(
            name="speed",
            type="float",
            description="Walking speed as fraction of max speed (0.1 - 1.0)",
            required=False,
            default=0.5,
        ),
    ],
    preconditions=["robot_is_balanced"],
    postconditions=["robot_at_location"],
    metadata={"locomotion_type": "bipedal"},
)

PICK_UP = Capability(
    name="pick_up_object",
    description="Picks up a named object using the robot's arm",
    ros_action="humanoid/pick_object",
    parameters=[
        CapabilityParameter(
            name="object_name",
            type="string",
            description="Name of the object to pick up, e.g. 'bottle', 'cup', 'book'",
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
    metadata={"body_part": "arm", "max_payload_kg": 1.5},
)

PLACE_OBJECT = Capability(
    name="place_object",
    description="Places the currently held object at a named location",
    ros_action="humanoid/place_object",
    parameters=[
        CapabilityParameter(
            name="surface_name",
            type="string",
            description="Where to place the object, e.g. 'table', 'shelf', 'floor'",
            required=True,
        ),
    ],
    preconditions=["object_in_hand", "robot_at_location"],
    postconditions=["object_placed", "arm_is_free"],
)

SAY = Capability(
    name="say",
    description="Speaks a message aloud using the robot's text-to-speech system",
    ros_service="humanoid/tts",
    parameters=[
        CapabilityParameter(
            name="message",
            type="string",
            description="The message to say",
            required=True,
        ),
    ],
    preconditions=[],
    postconditions=[],
)


# ------------------------------------------------------------------
# Mock LLM responses for demo (maps instruction -> JSON plan)
# ------------------------------------------------------------------

DEMO_PLANS = {
    "pick up the bottle from the table": json.dumps({
        "feasible": True,
        "reason": "",
        "steps": [
            {
                "capability_name": "navigate_to_location",
                "parameters": {"location_name": "table"},
                "rationale": "Must navigate to the table before picking up the bottle",
            },
            {
                "capability_name": "pick_up_object",
                "parameters": {"object_name": "bottle", "arm": "right"},
                "rationale": "Pick up the bottle with the right arm",
            },
        ],
    }),
    "go to the door and say hello": json.dumps({
        "feasible": True,
        "reason": "",
        "steps": [
            {
                "capability_name": "navigate_to_location",
                "parameters": {"location_name": "door"},
                "rationale": "Navigate to the door as instructed",
            },
            {
                "capability_name": "say",
                "parameters": {"message": "Hello!"},
                "rationale": "Say hello as instructed",
            },
        ],
    }),
    "fly to the moon": json.dumps({
        "feasible": False,
        "reason": "No capability exists for flying or space travel.",
        "steps": [],
    }),
}


class DemoMockBackend:
    """Mock backend that uses the DEMO_PLANS dictionary."""
    def complete(self, messages):
        # Extract the instruction from the last user message
        instruction = messages[-1]["content"].lower().strip()
        # Find a matching plan (fuzzy match for demo purposes)
        for key, plan in DEMO_PLANS.items():
            if key in instruction or instruction in key:
                return plan
        # Default: not feasible
        return json.dumps({
            "feasible": False,
            "reason": f"No matching capability for: '{instruction}'",
            "steps": [],
        })


# ------------------------------------------------------------------
# Run the demo
# ------------------------------------------------------------------

def run_demo():
    print("=" * 60)
    print("  ros2_lingua — Humanoid Robot Demo")
    print("=" * 60)

    # Build registry and register all capabilities
    registry = CapabilityRegistry()
    for cap in [STABILIZE, NAVIGATE, PICK_UP, PLACE_OBJECT, SAY]:
        registry.register(cap)
    print(f"\n✅ Registered {len(registry.get_all())} capabilities.\n")

    # Set initial robot state (what's true at startup)
    # Note: robot_is_balanced and object_in_view are NOT set initially
    # This means the auto-chainer will need to insert 'stabilize_robot' first
    registry.update_state({"arm_is_free", "object_in_view"})
    print(f"🤖 Initial state: {sorted(registry.get_state())}\n")

    # Build the engine with our demo backend
    engine = GroundingEngine(registry, DemoMockBackend(), auto_chain=True)

    # Run demo instructions
    instructions = [
        "pick up the bottle from the table",
        "go to the door and say hello",
        "fly to the moon",
    ]

    for instruction in instructions:
        print("-" * 60)
        print(f"📢 Instruction: \"{instruction}\"")

        plan = engine.ground(instruction)

        if not plan.feasible:
            print(f"❌ Not feasible: {plan.reason}")
        else:
            print(f"✅ Plan ({len(plan.steps)} steps):")
            for i, step in enumerate(plan.steps, 1):
                auto = " [auto-chained]" if "Auto-inserted" in step.rationale else ""
                print(f"   {i}. {step.capability_name}{auto}")
                print(f"      params: {step.parameters}")
                print(f"      reason: {step.rationale}")
        print()

    print("=" * 60)
    print("  Demo complete. All the logic above runs with zero ROS 2 setup.")
    print("  On a real robot, each step dispatches to a live ROS 2 action/service.")
    print("=" * 60)


if __name__ == "__main__":
    run_demo()
