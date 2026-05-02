"""
ros2_lingua_core.registry
--------------------------
The CapabilityRegistry is the in-memory store of all capabilities
that have been registered by nodes in the system.

It also handles:
- Capability lookup by name
- State tracking (which symbolic states are currently true)
- Dependency resolution (chaining capabilities via pre/postconditions)
"""

from typing import Dict, List, Optional, Set
from .schema import Capability


class CapabilityRegistry:
    """
    Central store for all registered capabilities and current robot state.

    This is intentionally kept free of any ROS 2 dependencies so it
    can be unit-tested independently.

    Usage:
        registry = CapabilityRegistry()
        registry.register(my_capability)
        caps = registry.get_all()
        chain = registry.resolve_chain("pick_up_object", current_state={"robot_is_balanced"})
    """

    def __init__(self):
        # name -> Capability
        self._capabilities: Dict[str, Capability] = {}

        # Current symbolic state of the robot
        # e.g. {"robot_is_balanced", "arm_is_free"}
        self._state: Set[str] = set()

    # ------------------------------------------------------------------
    # Registration
    # ------------------------------------------------------------------

    def register(self, capability: Capability) -> None:
        """
        Register a capability. Raises ValueError if malformed or duplicate.
        """
        capability.validate()
        if capability.name in self._capabilities:
            raise ValueError(
                f"Capability '{capability.name}' is already registered. "
                "Use update() to replace it."
            )
        self._capabilities[capability.name] = capability

    def update(self, capability: Capability) -> None:
        """Register or overwrite a capability."""
        capability.validate()
        self._capabilities[capability.name] = capability

    def unregister(self, name: str) -> None:
        """Remove a capability by name."""
        self._capabilities.pop(name, None)

    # ------------------------------------------------------------------
    # Lookup
    # ------------------------------------------------------------------

    def get(self, name: str) -> Optional[Capability]:
        return self._capabilities.get(name)

    def get_all(self) -> List[Capability]:
        return list(self._capabilities.values())

    def names(self) -> List[str]:
        return list(self._capabilities.keys())

    # ------------------------------------------------------------------
    # State management
    # ------------------------------------------------------------------

    def set_state(self, token: str) -> None:
        """Mark a symbolic state token as true."""
        self._state.add(token)

    def clear_state(self, token: str) -> None:
        """Mark a symbolic state token as false."""
        self._state.discard(token)

    def update_state(self, tokens: Set[str]) -> None:
        """Replace the entire state set."""
        self._state = set(tokens)

    def get_state(self) -> Set[str]:
        return set(self._state)

    def is_satisfied(self, preconditions: List[str]) -> bool:
        """Returns True if all preconditions are in the current state."""
        return all(p in self._state for p in preconditions)

    # ------------------------------------------------------------------
    # Dependency resolution
    # ------------------------------------------------------------------

    def resolve_chain(
        self,
        goal_capability_name: str,
        current_state: Optional[Set[str]] = None,
        max_depth: int = 10,
    ) -> List[Capability]:
        """
        Given a goal capability, return the ordered sequence of capabilities
        that need to execute to satisfy all preconditions, starting from
        the current state.

        Uses a simple backward-chaining planner:
        - Start from the goal capability
        - For each unsatisfied precondition, find a capability that produces it
        - Recursively satisfy that capability's preconditions
        - Return the flattened, ordered execution plan

        Args:
            goal_capability_name: The capability we ultimately want to run
            current_state: The symbolic states currently true. Defaults to
                           the registry's tracked state if not provided.
            max_depth: Prevents infinite recursion in circular dependencies

        Returns:
            Ordered list of capabilities to execute (goal is last)

        Raises:
            ValueError: If a precondition cannot be satisfied by any capability
            RecursionError: If dependency depth exceeds max_depth
        """
        if current_state is None:
            current_state = self.get_state()

        goal = self.get(goal_capability_name)
        if goal is None:
            raise ValueError(f"Capability '{goal_capability_name}' is not registered.")

        plan: List[Capability] = []
        self._backward_chain(goal, current_state, plan, depth=0, max_depth=max_depth)
        return plan

    def _backward_chain(
        self,
        capability: Capability,
        current_state: Set[str],
        plan: List[Capability],
        depth: int,
        max_depth: int,
    ) -> Set[str]:
        """
        Recursive backward chaining. Returns the state after this
        capability and all its prerequisites have been resolved.
        """
        if depth > max_depth:
            raise RecursionError(
                f"Dependency chain exceeded max depth of {max_depth}. "
                "Possible circular dependency."
            )

        # Find unsatisfied preconditions
        unsatisfied = [p for p in capability.preconditions if p not in current_state]

        for precondition in unsatisfied:
            # Find a registered capability that produces this precondition
            producer = self._find_producer(precondition)
            if producer is None:
                raise ValueError(
                    f"Cannot satisfy precondition '{precondition}' required by "
                    f"'{capability.name}'. No registered capability produces it."
                )
            # Recursively satisfy the producer's preconditions
            current_state = self._backward_chain(
                producer, current_state, plan, depth + 1, max_depth
            )

        # All preconditions satisfied — add this capability to the plan
        if capability not in plan:
            plan.append(capability)

        # Apply postconditions to the state
        for token in capability.postconditions:
            current_state.add(token)

        return current_state

    def _find_producer(self, token: str) -> Optional[Capability]:
        """Find the first registered capability whose postconditions include token."""
        for cap in self._capabilities.values():
            if token in cap.postconditions:
                return cap
        return None

    # ------------------------------------------------------------------
    # LLM context generation
    # ------------------------------------------------------------------

    def to_llm_context(self) -> str:
        """
        Returns a formatted string describing all registered capabilities,
        suitable for injection into an LLM system prompt.
        """
        if not self._capabilities:
            return "No capabilities are currently registered."

        lines = ["Available robot capabilities:\n"]
        for cap in self._capabilities.values():
            lines.append(cap.to_llm_description())
            lines.append("")  # blank line between capabilities

        lines.append(f"\nCurrent robot state: {sorted(self._state) or 'none'}")
        return "\n".join(lines)
