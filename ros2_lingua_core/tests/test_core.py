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
        tags=["locomotion", "navigation"],
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
        tags=["balance"],
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

# ------------------------------------------------------------------
# Tagging tests
# ------------------------------------------------------------------

class TestCapabilityTagging:
    def test_tags_in_schema(self):
        cap = Capability(
            name="navigate",
            description="navigates",
            ros_action="a/b",
            tags=["locomotion", "navigation"],
        )
        assert "locomotion" in cap.tags
        assert "navigation" in cap.tags

    def test_tags_serialization_roundtrip(self):
        cap = Capability(
            name="navigate",
            description="navigates",
            ros_action="a/b",
            tags=["locomotion", "outdoor"],
        )
        restored = Capability.from_json(cap.to_json())
        assert restored.tags == ["locomotion", "outdoor"]

    def test_tags_default_empty(self):
        cap = Capability(name="test", description="d", ros_action="a/b")
        assert cap.tags == []

    def test_get_by_tag(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b", tags=["locomotion"]))
        r.register(Capability(name="pick", description="d", ros_action="a/c", tags=["manipulation"]))
        r.register(Capability(name="say", description="d", ros_service="a/d", tags=["speech"]))
        loco = r.get_by_tag("locomotion")
        assert len(loco) == 1
        assert loco[0].name == "nav"

    def test_get_by_tags_any(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b", tags=["locomotion"]))
        r.register(Capability(name="pick", description="d", ros_action="a/c", tags=["manipulation"]))
        r.register(Capability(name="say", description="d", ros_service="a/d", tags=["speech"]))
        result = r.get_by_tags(["locomotion", "manipulation"], match="any")
        names = {c.name for c in result}
        assert names == {"nav", "pick"}

    def test_get_by_tags_all(self):
        r = CapabilityRegistry()
        r.register(Capability(name="wave", description="d", ros_action="a/b",
                               tags=["manipulation", "social"]))
        r.register(Capability(name="pick", description="d", ros_action="a/c",
                               tags=["manipulation"]))
        result = r.get_by_tags(["manipulation", "social"], match="all")
        assert len(result) == 1
        assert result[0].name == "wave"

    def test_get_all_tags(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b",
                               tags=["locomotion"]))
        r.register(Capability(name="pick", description="d", ros_action="a/c",
                               tags=["manipulation", "social"]))
        all_tags = r.get_all_tags()
        assert all_tags == sorted(["locomotion", "manipulation", "social"])

    def test_get_untagged(self):
        r = CapabilityRegistry()
        r.register(Capability(name="tagged", description="d", ros_action="a/b",
                               tags=["locomotion"]))
        r.register(Capability(name="untagged", description="d", ros_action="a/c"))
        untagged = r.get_untagged()
        assert len(untagged) == 1
        assert untagged[0].name == "untagged"

    def test_llm_context_tag_filter(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="navigates", ros_action="a/b",
                               tags=["locomotion"]))
        r.register(Capability(name="pick", description="picks", ros_action="a/c",
                               tags=["manipulation"]))
        context = r.to_llm_context(tags=["locomotion"])
        assert "nav" in context
        assert "pick" not in context

    def test_llm_context_no_filter_shows_all(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b",
                               tags=["locomotion"]))
        r.register(Capability(name="pick", description="d", ros_action="a/c",
                               tags=["manipulation"]))
        context = r.to_llm_context()
        assert "nav" in context
        assert "pick" in context

    def test_untagged_always_included_in_filtered_context(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b",
                               tags=["locomotion"]))
        r.register(Capability(name="untagged_cap", description="d", ros_action="a/c"))
        # Filter by manipulation — nav should be excluded, untagged_cap included
        context = r.to_llm_context(tags=["manipulation"])
        assert "nav" not in context
        assert "untagged_cap" in context

    def test_ground_with_tag_filter(self, registry):
        import json
        mock_response = json.dumps({
            "feasible": True, "reason": "", "steps": [{
                "capability_name": "navigate_to_location",
                "parameters": {"location_name": "table"},
                "rationale": "go",
            }]
        })
        from ros2_lingua_core import GroundingEngine, MockBackend
        engine = GroundingEngine(registry, MockBackend(mock_response), auto_chain=False)
        plan = engine.ground("go to table", tag_filter=["locomotion"])
        assert plan.feasible


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




# ------------------------------------------------------------------
# Robustness tests
# ------------------------------------------------------------------

class TestRobustness:
    def test_retry_config_defaults(self):
        from ros2_lingua_core import RetryConfig
        r = RetryConfig()
        assert r.max_retries == 3
        assert r.base_delay_sec == 1.0
        assert r.backoff_factor == 2.0

    def test_retry_config_no_retry(self):
        from ros2_lingua_core import RetryConfig
        r = RetryConfig.no_retry()
        assert r.max_retries == 0

    def test_mock_backend_failing_then_succeeds(self):
        import json
        from ros2_lingua_core import MockBackend, LLMTimeoutError
        success_response = json.dumps({
            "feasible": True, "reason": "", "steps": []
        })
        backend = MockBackend.failing(
            LLMTimeoutError("timed out"),
            retries_before_success=2,
            success_response=success_response,
        )
        # First 2 calls raise, 3rd succeeds
        for _ in range(2):
            try:
                backend.complete([])
            except LLMTimeoutError:
                pass
        result = backend.complete([])
        assert "feasible" in result

    def test_empty_registry_returns_infeasible(self):
        from ros2_lingua_core import CapabilityRegistry, GroundingEngine, MockBackend
        import json
        r = CapabilityRegistry()
        engine = GroundingEngine(r, MockBackend("{}"), auto_chain=False)
        plan = engine.ground("do something")
        assert not plan.feasible
        assert "registered" in plan.reason.lower()

    def test_error_hierarchy(self):
        from ros2_lingua_core import (
            LinguaError, LLMBackendError, LLMTimeoutError,
            LLMRateLimitError, GroundingError, HallucinationError,
            PlanningError, UnsatisfiablePreconditionError,
        )
        assert issubclass(LLMBackendError, LinguaError)
        assert issubclass(LLMTimeoutError, LLMBackendError)
        assert issubclass(LLMRateLimitError, LLMBackendError)
        assert issubclass(HallucinationError, GroundingError)
        assert issubclass(GroundingError, LinguaError)
        assert issubclass(UnsatisfiablePreconditionError, PlanningError)
        assert issubclass(PlanningError, LinguaError)

    def test_step_timeout_error_message(self):
        from ros2_lingua_core import StepTimeoutError
        e = StepTimeoutError("navigate_to_location", 10.0)
        assert "navigate_to_location" in str(e)
        assert "10.0" in str(e)

    def test_hallucination_error_message(self):
        from ros2_lingua_core import HallucinationError
        e = HallucinationError("ghost_capability")
        assert "ghost_capability" in str(e)
        assert "hallucinated" in str(e).lower()

    def test_unsatisfiable_precondition_error_message(self):
        from ros2_lingua_core import UnsatisfiablePreconditionError
        e = UnsatisfiablePreconditionError("robot_is_balanced", "navigate_to_location")
        assert "robot_is_balanced" in str(e)
        assert "navigate_to_location" in str(e)

    def test_grounding_engine_retry_params(self):
        from ros2_lingua_core import CapabilityRegistry, GroundingEngine, MockBackend, Capability
        import json
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b"))
        mock = MockBackend(json.dumps({"feasible": True, "reason": "", "steps": [
            {"capability_name": "nav", "parameters": {}, "rationale": "go"}
        ]}))
        # Verify max_retries and retry params are accepted
        engine = GroundingEngine(r, mock, auto_chain=False,
                                  max_retries=5, retry_delay=0.1, retry_backoff=1.5)
        plan = engine.ground("do nav")
        assert plan.feasible


# ------------------------------------------------------------------
# Parameter validation tests
# ------------------------------------------------------------------

class TestParameterValidation:
    """Tests for ParameterValidator and its integration into GroundingEngine."""

    def _make_cap(self, params):
        return Capability(
            name="test_cap", description="d", ros_action="a/b",
            parameters=params,
        )

    # ── Type coercion ──────────────────────────────────────────

    def test_string_coercion(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([CapabilityParameter("name", "string", "a name")])
        result = ParameterValidator().validate(cap, {"name": 42})
        assert result["name"] == "42"

    def test_float_from_int(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([CapabilityParameter("speed", "float", "speed")])
        result = ParameterValidator().validate(cap, {"speed": 1})
        assert result["speed"] == 1.0
        assert isinstance(result["speed"], float)

    def test_float_from_string(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([CapabilityParameter("speed", "float", "speed")])
        result = ParameterValidator().validate(cap, {"speed": "0.5"})
        assert result["speed"] == 0.5

    def test_float_invalid_string_fails(self):
        from ros2_lingua_core import ParameterValidator, ParameterValidationError
        cap = self._make_cap([CapabilityParameter("speed", "float", "speed")])
        with pytest.raises(ParameterValidationError) as exc_info:
            ParameterValidator().validate(cap, {"speed": "fast"})
        assert "speed" in str(exc_info.value)
        assert "fast" in str(exc_info.value)

    def test_int_from_float_whole_number(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([CapabilityParameter("count", "int", "count")])
        result = ParameterValidator().validate(cap, {"count": 3.0})
        assert result["count"] == 3
        assert isinstance(result["count"], int)

    def test_int_from_fractional_float_fails(self):
        from ros2_lingua_core import ParameterValidator, ParameterValidationError
        cap = self._make_cap([CapabilityParameter("count", "int", "count")])
        with pytest.raises(ParameterValidationError):
            ParameterValidator().validate(cap, {"count": 3.5})

    def test_int_from_string(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([CapabilityParameter("count", "integer", "count")])
        result = ParameterValidator().validate(cap, {"count": "5"})
        assert result["count"] == 5

    def test_bool_from_string_true(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([CapabilityParameter("enabled", "bool", "flag")])
        for val in ["true", "True", "TRUE", "1", "yes"]:
            result = ParameterValidator().validate(cap, {"enabled": val})
            assert result["enabled"] is True, f"Failed for: {val}"

    def test_bool_from_string_false(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([CapabilityParameter("enabled", "bool", "flag")])
        for val in ["false", "False", "FALSE", "0", "no"]:
            result = ParameterValidator().validate(cap, {"enabled": val})
            assert result["enabled"] is False, f"Failed for: {val}"

    def test_bool_invalid_string_fails(self):
        from ros2_lingua_core import ParameterValidator, ParameterValidationError
        cap = self._make_cap([CapabilityParameter("enabled", "boolean", "flag")])
        with pytest.raises(ParameterValidationError):
            ParameterValidator().validate(cap, {"enabled": "maybe"})

    def test_list_from_json_string(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([CapabilityParameter("items", "list", "items")])
        result = ParameterValidator().validate(cap, {"items": '["a", "b", "c"]'})
        assert result["items"] == ["a", "b", "c"]

    def test_list_invalid_json_fails(self):
        from ros2_lingua_core import ParameterValidator, ParameterValidationError
        cap = self._make_cap([CapabilityParameter("items", "list", "items")])
        with pytest.raises(ParameterValidationError):
            ParameterValidator().validate(cap, {"items": "not json"})

    # ── Required / optional parameters ────────────────────────

    def test_missing_required_fails(self):
        from ros2_lingua_core import ParameterValidator, ParameterValidationError
        cap = self._make_cap([CapabilityParameter("loc", "string", "location", required=True)])
        with pytest.raises(ParameterValidationError) as exc_info:
            ParameterValidator().validate(cap, {})
        assert "required" in str(exc_info.value)

    def test_missing_optional_uses_default(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([
            CapabilityParameter("speed", "float", "speed", required=False, default=0.5)
        ])
        result = ParameterValidator().validate(cap, {})
        assert result["speed"] == 0.5

    def test_all_failures_reported_at_once(self):
        from ros2_lingua_core import ParameterValidator, ParameterValidationError
        cap = self._make_cap([
            CapabilityParameter("speed",  "float",  "speed",  required=True),
            CapabilityParameter("name",   "string", "name",   required=True),
            CapabilityParameter("count",  "int",    "count",  required=True),
        ])
        with pytest.raises(ParameterValidationError) as exc_info:
            # speed is wrong type, name missing, count wrong type
            ParameterValidator().validate(cap, {"speed": "fast", "count": "three"})
        error_msg = str(exc_info.value)
        assert "speed" in error_msg
        assert "name" in error_msg
        assert "count" in error_msg

    # ── ROS message types pass through ────────────────────────

    def test_ros_message_type_passes_through(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([
            CapabilityParameter("pose", "geometry_msgs/Pose", "target pose")
        ])
        pose_val = {"position": {"x": 1.0}, "orientation": {"w": 1.0}}
        result = ParameterValidator().validate(cap, {"pose": pose_val})
        assert result["pose"] == pose_val

    # ── Strict mode ────────────────────────────────────────────

    def test_strict_mode_rejects_unknown_params(self):
        from ros2_lingua_core import ParameterValidator, ParameterValidationError
        cap = self._make_cap([CapabilityParameter("name", "string", "name")])
        with pytest.raises(ParameterValidationError):
            ParameterValidator().validate(
                cap, {"name": "table", "unknown_extra": "value"}, strict=True
            )

    def test_non_strict_mode_passes_unknown_params(self):
        from ros2_lingua_core import ParameterValidator
        cap = self._make_cap([CapabilityParameter("name", "string", "name")])
        result = ParameterValidator().validate(
            cap, {"name": "table", "extra": "value"}, strict=False
        )
        assert result["name"] == "table"
        assert result["extra"] == "value"

    # ── Integration with GroundingEngine ──────────────────────

    def test_grounding_engine_validates_params(self):
        import json
        r = CapabilityRegistry()
        r.register(Capability(
            name="navigate_to_location", description="d", ros_action="a/n",
            parameters=[
                CapabilityParameter("location_name", "string", "where", required=True),
                CapabilityParameter("speed", "float", "how fast", required=False, default=0.5),
            ],
        ))
        mock_response = json.dumps({
            "feasible": True, "reason": "", "steps": [{
                "capability_name": "navigate_to_location",
                "parameters": {"location_name": "table", "speed": "fast"},
                "rationale": "go",
            }]
        })
        engine = GroundingEngine(r, MockBackend(mock_response), auto_chain=False, validate_params=True)
        plan = engine.ground("go to table")
        assert not plan.feasible
        assert "speed" in plan.reason and "fast" in plan.reason

    def test_grounding_engine_coerces_valid_params(self):
        import json
        r = CapabilityRegistry()
        r.register(Capability(
            name="navigate_to_location", description="d", ros_action="a/n",
            parameters=[
                CapabilityParameter("location_name", "string", "where", required=True),
                CapabilityParameter("speed", "float", "how fast", required=False, default=0.5),
            ],
        ))
        mock_response = json.dumps({
            "feasible": True, "reason": "", "steps": [{
                "capability_name": "navigate_to_location",
                "parameters": {"location_name": "table", "speed": "0.5"},
                "rationale": "go",
            }]
        })
        engine = GroundingEngine(r, MockBackend(mock_response), auto_chain=False, validate_params=True)
        plan = engine.ground("go to table")
        assert plan.feasible
        assert plan.steps[0].parameters["speed"] == 0.5
        assert isinstance(plan.steps[0].parameters["speed"], float)

    def test_grounding_engine_validation_disabled(self, registry):
        import json
        mock_response = json.dumps({
            "feasible": True, "reason": "", "steps": [{
                "capability_name": "navigate_to_location",
                "parameters": {"location_name": "table", "speed": "fast"},
                "rationale": "go",
            }]
        })
        engine = GroundingEngine(
            registry, MockBackend(mock_response),
            auto_chain=False, validate_params=False
        )
        plan = engine.ground("go to table")
        # With validation off, bad types pass through
        assert plan.feasible
        assert plan.steps[0].parameters["speed"] == "fast"


# ------------------------------------------------------------------
# Recovery Planner tests
# ------------------------------------------------------------------

class TestRecoveryPlanner:
    """Tests for RecoveryPlanner, RecoveryConfig, and RecoveryDecision."""

    def _make_registry(self):
        """Create a registry with standard test capabilities."""
        r = CapabilityRegistry()
        r.register(Capability(
            name="stabilize_robot",
            description="Stabilizes the robot",
            ros_action="humanoid/stabilize",
            preconditions=[],
            postconditions=["robot_is_balanced"],
        ))
        r.register(Capability(
            name="navigate_to_location",
            description="Walks to a named location",
            ros_action="humanoid/navigate",
            parameters=[CapabilityParameter("location_name", "string", "where")],
            preconditions=["robot_is_balanced"],
            postconditions=["robot_at_location"],
        ))
        r.register(Capability(
            name="pick_up_object",
            description="Picks up an object",
            ros_action="humanoid/pick",
            parameters=[CapabilityParameter("object_name", "string", "what")],
            preconditions=["robot_at_location"],
            postconditions=["object_in_hand"],
        ))
        r.register(Capability(
            name="return_to_home",
            description="Returns the robot to home",
            ros_action="humanoid/home",
            preconditions=[],
            postconditions=["robot_at_home"],
        ))
        return r

    def _make_engine(self, registry, plan_steps):
        """Create a GroundingEngine with a MockBackend returning the given steps."""
        mock_response = json.dumps({
            "feasible": True, "reason": "",
            "steps": plan_steps,
        })
        return GroundingEngine(
            registry, MockBackend(mock_response),
            auto_chain=False, validate_params=False,
        )

    def _failed_step(self, cap_name="pick_up_object", params=None):
        return {
            "capability_name": cap_name,
            "parameters": params or {},
        }

    # ── RecoveryConfig tests ──────────────────────────────────

    def test_recovery_config_defaults(self):
        from ros2_lingua_core import RecoveryConfig
        config = RecoveryConfig()
        assert config.max_retries == 2
        assert config.enable_replan is True
        assert config.safe_fallback is None
        assert config.safe_fallback_params == {}
        assert config.abort_on_fallback_failure is True

    def test_recovery_config_no_replan(self):
        from ros2_lingua_core import RecoveryConfig
        config = RecoveryConfig(enable_replan=False)
        assert config.enable_replan is False

    # ── RecoveryDecision tests ────────────────────────────────

    def test_recovery_decision_dataclass(self):
        from ros2_lingua_core import RecoveryDecision
        d = RecoveryDecision(strategy="retry", reason="testing")
        assert d.strategy == "retry"
        assert d.new_plan is None
        assert d.reason == "testing"

    # ── Retry strategy ────────────────────────────────────────

    def test_retry_decision_on_first_failure(self):
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        planner = RecoveryPlanner(RecoveryConfig(max_retries=2))
        decision = planner.on_step_failed(
            failed_step=self._failed_step(),
            step_index=0,
            original_instruction="pick up the bottle",
            current_state=set(),
            error="gripper timeout",
        )
        assert decision.strategy == "retry"
        assert "attempt 1" in decision.reason

    def test_retry_exhaustion_leads_to_replan(self):
        """After max retries, should escalate to replan."""
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        registry = self._make_registry()
        engine = self._make_engine(registry, [
            {"capability_name": "pick_up_object",
             "parameters": {"object_name": "bottle"}, "rationale": "pick"},
        ])
        planner = RecoveryPlanner(
            RecoveryConfig(max_retries=1, enable_replan=True),
            grounding_engine=engine,
            registry=registry,
        )
        # First failure → retry
        d1 = planner.on_step_failed(
            self._failed_step(), 0, "pick up", set(), "fail"
        )
        assert d1.strategy == "retry"
        # Second failure → replan (retries exhausted)
        d2 = planner.on_step_failed(
            self._failed_step(), 0, "pick up", set(), "fail again"
        )
        assert d2.strategy == "replan"

    def test_retry_count_tracks_per_step(self):
        """Each step has independent retry counters."""
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        planner = RecoveryPlanner(RecoveryConfig(max_retries=1))
        # Step 0 fails once → retry
        d0 = planner.on_step_failed(
            self._failed_step("step_a"), 0, "instr", set(), "err"
        )
        assert d0.strategy == "retry"
        # Step 1 also gets its own retry
        d1 = planner.on_step_failed(
            self._failed_step("step_b"), 1, "instr", set(), "err"
        )
        assert d1.strategy == "retry"

    # ── Replan strategy ───────────────────────────────────────

    def test_replan_returns_new_plan(self):
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        registry = self._make_registry()
        engine = self._make_engine(registry, [
            {"capability_name": "navigate_to_location",
             "parameters": {"location_name": "kitchen"}, "rationale": "go"},
        ])
        planner = RecoveryPlanner(
            RecoveryConfig(max_retries=0, enable_replan=True),
            grounding_engine=engine,
            registry=registry,
        )
        decision = planner.on_step_failed(
            self._failed_step(), 0, "go to kitchen", set(), "nav fail"
        )
        assert decision.strategy == "replan"
        assert decision.new_plan is not None
        assert decision.new_plan.feasible
        assert len(decision.new_plan.steps) >= 1

    def test_replan_skips_completed_steps(self):
        """When replanning with updated state, backward chainer skips done work."""
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        registry = self._make_registry()
        # The mock response returns only pick_up — the engine with auto_chain=True
        # would insert stabilize + navigate, but since the state already has those
        # postconditions, they should be skipped
        engine_with_chain = GroundingEngine(
            registry,
            MockBackend(json.dumps({
                "feasible": True, "reason": "", "steps": [
                    {"capability_name": "pick_up_object",
                     "parameters": {"object_name": "bottle"}, "rationale": "pick"},
                ]
            })),
            auto_chain=True,
            validate_params=False,
        )
        planner = RecoveryPlanner(
            RecoveryConfig(max_retries=0, enable_replan=True),
            grounding_engine=engine_with_chain,
            registry=registry,
        )
        # State already has stabilize and navigate postconditions
        state = {"robot_is_balanced", "robot_at_location"}
        decision = planner.on_step_failed(
            self._failed_step(), 0,
            "pick up the bottle from the table",
            state, "gripper error",
        )
        assert decision.strategy == "replan"
        # The new plan should NOT include stabilize_robot or navigate_to_location
        # since their postconditions are already in state
        step_names = [s.capability_name for s in decision.new_plan.steps]
        assert "stabilize_robot" not in step_names
        assert "navigate_to_location" not in step_names
        assert "pick_up_object" in step_names

    def test_replan_disabled_goes_to_fallback(self):
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=False,
                safe_fallback="return_to_home",
            ),
        )
        decision = planner.on_step_failed(
            self._failed_step(), 0, "do thing", set(), "error"
        )
        assert decision.strategy == "fallback"

    def test_recovery_planner_without_engine(self):
        """Replan is skipped when no grounding engine is provided."""
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=True,  # enabled but no engine
                safe_fallback="return_to_home",
            ),
        )
        decision = planner.on_step_failed(
            self._failed_step(), 0, "instr", set(), "err"
        )
        # Should skip replan and go to fallback
        assert decision.strategy == "fallback"

    # ── Fallback strategy ─────────────────────────────────────

    def test_fallback_decision(self):
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=False,
                safe_fallback="return_to_home",
            ),
        )
        decision = planner.on_step_failed(
            self._failed_step(), 0, "instr", set(), "err"
        )
        assert decision.strategy == "fallback"
        assert "return_to_home" in decision.reason

    def test_fallback_with_registry_validation(self):
        """Fallback capability must exist in registry when registry is provided."""
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        registry = self._make_registry()
        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=False,
                safe_fallback="nonexistent_fallback",
            ),
            registry=registry,
        )
        decision = planner.on_step_failed(
            self._failed_step(), 0, "instr", set(), "err"
        )
        # Fallback not registered → abort
        assert decision.strategy == "abort"

    # ── Abort strategy ────────────────────────────────────────

    def test_no_fallback_goes_to_abort(self):
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=False,
                safe_fallback=None,
            ),
        )
        decision = planner.on_step_failed(
            self._failed_step(), 0, "instr", set(), "err"
        )
        assert decision.strategy == "abort"

    def test_abort_decision_has_reason(self):
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        planner = RecoveryPlanner(
            RecoveryConfig(max_retries=0, enable_replan=False),
        )
        decision = planner.on_step_failed(
            self._failed_step(), 0, "instr", set(), "some error"
        )
        assert decision.strategy == "abort"
        assert "exhausted" in decision.reason.lower()
        assert "some error" in decision.reason

    # ── RecoveryExhaustedError ────────────────────────────────

    def test_recovery_exhausted_error(self):
        from ros2_lingua_core import RecoveryExhaustedError
        e = RecoveryExhaustedError("pick_up_object", ["retry", "replan"])
        assert "pick_up_object" in str(e)
        assert "retry" in str(e)
        assert "replan" in str(e)
        assert e.capability_name == "pick_up_object"
        assert e.strategies_tried == ["retry", "replan"]

    # ── Reset ─────────────────────────────────────────────────

    def test_reset_clears_retry_counts(self):
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        planner = RecoveryPlanner(RecoveryConfig(max_retries=1))
        # Use up the retry for step 0
        d1 = planner.on_step_failed(
            self._failed_step(), 0, "instr", set(), "err"
        )
        assert d1.strategy == "retry"
        # Reset
        planner.reset()
        # Now step 0 should get a fresh retry
        d2 = planner.on_step_failed(
            self._failed_step(), 0, "instr", set(), "err"
        )
        assert d2.strategy == "retry"

    # ── Multiple step failures ────────────────────────────────

    def test_multiple_step_failures(self):
        """Different steps can fail and recover independently."""
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        planner = RecoveryPlanner(RecoveryConfig(max_retries=1))
        # Step 0 fails → retry
        assert planner.on_step_failed(
            self._failed_step("step_a"), 0, "instr", set(), "err"
        ).strategy == "retry"
        # Step 0 fails again → escalate (no engine, no fallback → abort)
        assert planner.on_step_failed(
            self._failed_step("step_a"), 0, "instr", set(), "err"
        ).strategy == "abort"
        # Step 2 fails → retry (independent counter)
        assert planner.on_step_failed(
            self._failed_step("step_c"), 2, "instr", set(), "err"
        ).strategy == "retry"

    # ── Replan with partial state ─────────────────────────────

    def test_replan_with_partial_state(self):
        """After 2/4 steps complete, replan only plans remaining work."""
        from ros2_lingua_core import RecoveryPlanner, RecoveryConfig
        registry = self._make_registry()
        # Mock engine returns the single remaining step
        engine = self._make_engine(registry, [
            {"capability_name": "pick_up_object",
             "parameters": {"object_name": "cup"}, "rationale": "pick"},
        ])
        planner = RecoveryPlanner(
            RecoveryConfig(max_retries=0, enable_replan=True),
            grounding_engine=engine,
            registry=registry,
        )
        # Simulate that stabilize and navigate already completed
        partial_state = {"robot_is_balanced", "robot_at_location"}
        decision = planner.on_step_failed(
            self._failed_step("pick_up_object", {"object_name": "cup"}),
            2,
            "pick up the cup from the table",
            partial_state,
            "gripper fault",
        )
        assert decision.strategy == "replan"
        assert len(decision.new_plan.steps) == 1
        assert decision.new_plan.steps[0].capability_name == "pick_up_object"

