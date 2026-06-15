"""
ros2_lingua_core.schema
-----------------------
Defines the core data structures for capability advertisement.

A 'Capability' is the fundamental unit of this library — it describes
ONE thing a ROS 2 node can do, in a structured way that both the
grounding engine and the LLM can reason about.
"""

import json
from dataclasses import dataclass, field
from typing import Any

# ------------------------------------------------------------------
# Standard capability tags
# ------------------------------------------------------------------
# These are the recommended tag values for common robot domains.
# Using these ensures consistent filtering across different robots.
# You can also define your own tags freely — these are just conventions.

class Tags:
    """Standard tag constants for capability categories."""

    # Motion / locomotion
    LOCOMOTION   = "locomotion"   # moving the robot base (navigate, drive, walk)
    MANIPULATION = "manipulation" # arm, gripper, pick/place
    BALANCE      = "balance"      # stabilization, posture control

    # Perception
    PERCEPTION   = "perception"   # cameras, lidar, object detection
    MAPPING      = "mapping"      # SLAM, map building, localization

    # Interaction
    SPEECH       = "speech"       # TTS, STT, voice I/O
    SOCIAL       = "social"       # gestures, expressions, HRI

    # System
    SYSTEM       = "system"       # power, mode switching, diagnostics
    SAFETY       = "safety"       # e-stop, collision avoidance

    # Domain-specific (add your own as needed)
    NAVIGATION   = "navigation"   # alias for locomotion in mobile robot contexts
    INSPECTION   = "inspection"   # for ROVs, drones, industrial robots


@dataclass
class CapabilityParameter:
    """
    Describes a single input parameter for a capability.

    Example:
        CapabilityParameter(
            name="location_name",
            type="string",
            description="Named location to walk to, e.g. 'table', 'door'",
            required=True
        )
    """
    name: str
    type: str           # "string" | "float" | "int" | "bool" | "geometry_msgs/Pose" | etc.
    description: str
    required: bool = True
    default: Any | None = None

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "type": self.type,
            "description": self.description,
            "required": self.required,
            "default": self.default,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "CapabilityParameter":
        return cls(
            name=data["name"],
            type=data["type"],
            description=data["description"],
            required=data.get("required", True),
            default=data.get("default"),
        )


@dataclass
class Capability:
    """
    Describes one thing a ROS 2 node can do.

    This is the core schema of ros2_lingua. Every node that wants to
    participate in natural language grounding registers one or more
    Capability objects with the GroundingEngine.

    The preconditions and postconditions fields are what allow the
    grounding engine to chain capabilities together automatically.

    Example:
        Capability(
            name="navigate_to_location",
            description="Walks the robot to a named location",
            ros_action="humanoid/navigate_to_pose",
            parameters=[...],
            preconditions=["robot_is_balanced"],
            postconditions=["robot_at_location"]
        )
    """

    # --- Identity ---
    name: str
    description: str

    # --- ROS 2 Interface ---
    # Exactly one of these should be set (action OR service)
    ros_action: str | None = None
    ros_service: str | None = None

    # --- What it needs ---
    parameters: list[CapabilityParameter] = field(default_factory=list)

    # --- State conditions (used for chaining) ---
    # These are symbolic state tokens, e.g. "robot_is_balanced"
    preconditions: list[str] = field(default_factory=list)
    postconditions: list[str] = field(default_factory=list)

    # --- Optional metadata ---
    # Arbitrary key-value pairs for domain-specific info
    # e.g. {"body_part": "left_arm", "max_payload_kg": 1.5}
    metadata: dict[str, Any] = field(default_factory=dict)

    # --- Tags ---
    # Free-form labels for filtering and categorization.
    # Use the Tags constants for standard categories, or define your own.
    # e.g. tags=["locomotion", "outdoor"] or tags=[Tags.MANIPULATION]
    tags: list[str] = field(default_factory=list)

    def validate(self) -> None:
        """Raises ValueError if the capability definition is malformed."""
        if not self.name:
            raise ValueError("Capability must have a name.")
        if not self.description:
            raise ValueError(f"Capability '{self.name}' must have a description.")
        if self.ros_action and self.ros_service:
            raise ValueError(
                f"Capability '{self.name}' cannot define both ros_action and ros_service."
            )
        if not self.ros_action and not self.ros_service:
            raise ValueError(
                f"Capability '{self.name}' must define either ros_action or ros_service."
            )

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "description": self.description,
            "ros_action": self.ros_action,
            "ros_service": self.ros_service,
            "parameters": [p.to_dict() for p in self.parameters],
            "preconditions": self.preconditions,
            "postconditions": self.postconditions,
            "metadata": self.metadata,
            "tags": self.tags,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)

    @classmethod
    def from_dict(cls, data: dict) -> "Capability":
        return cls(
            name=data["name"],
            description=data["description"],
            ros_action=data.get("ros_action"),
            ros_service=data.get("ros_service"),
            parameters=[CapabilityParameter.from_dict(p) for p in data.get("parameters", [])],
            preconditions=data.get("preconditions", []),
            postconditions=data.get("postconditions", []),
            metadata=data.get("metadata", {}),
            tags=data.get("tags", []),
        )

    @classmethod
    def from_json(cls, json_str: str) -> "Capability":
        return cls.from_dict(json.loads(json_str))

    def to_llm_description(self) -> str:
        """
        Returns a compact, human-readable description of this capability
        formatted for inclusion in an LLM prompt.
        """
        lines = [
            f"- name: {self.name}",
            f"  description: {self.description}",
        ]
        if self.parameters:
            lines.append("  parameters:")
            for p in self.parameters:
                req = "required" if p.required else f"optional (default: {p.default})"
                lines.append(f"    - {p.name} ({p.type}, {req}): {p.description}")
        if self.preconditions:
            lines.append(f"  requires: {', '.join(self.preconditions)}")
        if self.postconditions:
            lines.append(f"  produces: {', '.join(self.postconditions)}")
        if self.tags:
            lines.append(f"  tags: {', '.join(self.tags)}")
        return "\n".join(lines)
