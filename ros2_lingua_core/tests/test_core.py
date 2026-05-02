"""
tests/test_core.py
-------------------
Unit tests for the ros2_lingua_core package.
No ROS 2 required — runs with plain pytest.

Run with:
    cd ros2_lingua_core
    pytest tests/ -v
"""

import json
import pytest

from ros2_lingua_core import (
    Capability,
    CapabilityParameter,
    CapabilityRegistry,
    GroundingEngine,
    MockBackend,
)


# ------------------------------------------------------------------
# Fixtures
# ------------------------------------------------------------------

@pytest.fixture
def basic_capability():
    return Capability(
        name="navigate_to_location",
        description="Walks the robot to a named location",
        ros_action="humanoid/navigate",
        parameters=[
            CapabilityParameter("location_name", "string", "Where to go")
        ],
        preconditions=["robot_is_balanced"],
        postconditions=["robot_at_location"],
    )


@pytest.fixture
def stabilize_capability():
    return Capability(
        name="stabilize_robot",
        description="Stabilizes the robot",
        ros_action="humanoid/stabilize",
        parameters=[],
        preconditions=[],
        postconditions=["robot_is_balanced"],
    )


@pytest.fixture
def registry(basic_capability, stabilize_capability):
    r = CapabilityRegistry()
    r.register(stabilize_capability)
    r.register(basic_capability)
    return r


# ------------------------------------------------------------------
# Schema tests
# ------------------------------------------------------------------

class TestCapability:
    def test_valid_capability(self, basic_capability):
        basic_capability.validate()  # should not raise

    def test_missing_name_raises(self):
        cap = Capability(
            name="", description="desc", ros_action="test/action"
        )
        with pytest.raises(ValueError, match="must have a name"):
            cap.validate()

    def test_both_action_and_service_raises(self):
        cap = Capability(
            name="test",
            description="desc",
            ros_action="test/action",
            ros_service="test/service",
        )
        with pytest.raises(ValueError, match="cannot define both"):
            cap.validate()

    def test_neither_action_nor_service_raises(self):
        cap = Capability(name="test", description="desc")
        with pytest.raises(ValueError, match="must define either"):
            cap.validate()

    def test_serialization_roundtrip(self, basic_capability):
        json_str = basic_capability.to_json()
        restored = Capability.from_json(json_str)
        assert restored.name == basic_capability.name
        assert restored.description == basic_capability.description
        assert restored.ros_action == basic_capability.ros_action
        assert len(restored.parameters) == len(basic_capability.parameters)
        assert restored.preconditions == basic_capability.preconditions
        assert restored.postconditions == basic_capability.postconditions

    def test_llm_description_contains_name(self, basic_capability):
        desc = basic_capability.to_llm_description()
        assert "navigate_to_location" in desc
        assert "location_name" in desc


# ------------------------------------------------------------------
# Registry tests
# ------------------------------------------------------------------

class TestCapabilityRegistry:
    def test_register_and_retrieve(self, basic_capability):
        r = CapabilityRegistry()
        r.register(basic_capability)
        assert r.get("navigate_to_location") is not None

    def test_duplicate_registration_raises(self, basic_capability):
        r = CapabilityRegistry()
        r.register(basic_capability)
        with pytest.raises(ValueError, match="already registered"):
            r.register(basic_capability)

    def test_update_overwrites(self, basic_capability):
        r = CapabilityRegistry()
        r.register(basic_capability)
        r.update(basic_capability)  # should not raise
        assert r.get("navigate_to_location") is not None

    def test_state_management(self):
        r = CapabilityRegistry()
        r.set_state("robot_is_balanced")
        assert "robot_is_balanced" in r.get_state()
        r.clear_state("robot_is_balanced")
        assert "robot_is_balanced" not in r.get_state()

    def test_preconditions_satisfied(self, registry):
        registry.set_state("robot_is_balanced")
        assert registry.is_satisfied(["robot_is_balanced"]) is True
        assert registry.is_satisfied(["robot_is_balanced", "arm_is_free"]) is False

    def test_llm_context_contains_all_caps(self, registry):
        context = registry.to_llm_context()
        assert "navigate_to_location" in context
        assert "stabilize_robot" in context


# ------------------------------------------------------------------
# Backward chaining tests
# ------------------------------------------------------------------

class TestBackwardChaining:
    def test_chain_with_unsatisfied_precondition(self, registry):
        # robot_is_balanced is NOT in state
        # navigate_to_location requires robot_is_balanced
        # stabilize_robot produces robot_is_balanced
        # So chain should be: [stabilize_robot, navigate_to_location]
        chain = registry.resolve_chain("navigate_to_location", current_state=set())
        names = [c.name for c in chain]
        assert names == ["stabilize_robot", "navigate_to_location"]

    def test_chain_with_satisfied_precondition(self, registry):
        # If robot_is_balanced is already true, no need to stabilize
        chain = registry.resolve_chain(
            "navigate_to_location", current_state={"robot_is_balanced"}
        )
        names = [c.name for c in chain]
        assert names == ["navigate_to_location"]

    def test_unsatisfiable_precondition_raises(self):
        r = CapabilityRegistry()
        r.register(Capability(
            name="do_thing",
            description="Does a thing",
            ros_action="robot/do_thing",
            preconditions=["impossible_condition"],
            postconditions=[],
        ))
        with pytest.raises(ValueError, match="Cannot satisfy precondition"):
            r.resolve_chain("do_thing", current_state=set())


# ------------------------------------------------------------------
# Grounding Engine tests
# ------------------------------------------------------------------

class TestGroundingEngine:
    def test_successful_grounding(self, registry):
        mock_response = json.dumps({
            "feasible": True,
            "reason": "",
            "steps": [{
                "capability_name": "navigate_to_location",
                "parameters": {"location_name": "table"},
                "rationale": "Go to table",
            }],
        })
        engine = GroundingEngine(registry, MockBackend(mock_response), auto_chain=False)
        plan = engine.ground("go to the table")
        assert plan.feasible is True
        assert len(plan.steps) == 1
        assert plan.steps[0].capability_name == "navigate_to_location"
        assert plan.steps[0].parameters["location_name"] == "table"

    def test_auto_chain_inserts_prerequisites(self, registry):
        # LLM suggests navigate_to_location but robot_is_balanced is not set
        mock_response = json.dumps({
            "feasible": True,
            "reason": "",
            "steps": [{
                "capability_name": "navigate_to_location",
                "parameters": {"location_name": "table"},
                "rationale": "Go to table",
            }],
        })
        engine = GroundingEngine(registry, MockBackend(mock_response), auto_chain=True)
        # State is empty — robot_is_balanced not satisfied
        plan = engine.ground("go to the table")
        assert plan.feasible is True
        names = [s.capability_name for s in plan.steps]
        # stabilize_robot should have been auto-inserted
        assert "stabilize_robot" in names
        assert names.index("stabilize_robot") < names.index("navigate_to_location")

    def test_infeasible_plan(self, registry):
        mock_response = json.dumps({
            "feasible": False,
            "reason": "No capability for flying.",
            "steps": [],
        })
        engine = GroundingEngine(registry, MockBackend(mock_response))
        plan = engine.ground("fly to the moon")
        assert plan.feasible is False
        assert "flying" in plan.reason

    def test_hallucinated_capability_caught(self, registry):
        mock_response = json.dumps({
            "feasible": True,
            "reason": "",
            "steps": [{
                "capability_name": "nonexistent_capability",
                "parameters": {},
                "rationale": "test",
            }],
        })
        engine = GroundingEngine(registry, MockBackend(mock_response))
        plan = engine.ground("do something fake")
        assert plan.feasible is False
        assert "hallucination" in plan.reason.lower() or "unknown" in plan.reason.lower()

    def test_invalid_json_from_llm(self, registry):
        engine = GroundingEngine(registry, MockBackend("this is not json"))
        plan = engine.ground("anything")
        assert plan.feasible is False
        assert "JSON" in plan.reason
