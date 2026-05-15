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

    def _nav_cap(self):
        return Capability(
            name="navigate",
            description="navigates",
            ros_action="a/nav",
            parameters=[CapabilityParameter("loc", "string", "where")],
            tags=["locomotion"],
        )

    def _mock_plan(self, cap_name="navigate", loc="table"):
        import json
        return json.dumps({
            "feasible": True, "reason": "", "steps": [{
                "capability_name": cap_name,
                "parameters": {"loc": loc},
                "rationale": "go",
            }]
        })

    def test_empty_registry_raises(self):
        from ros2_lingua_core.errors import EmptyRegistryError
        r = CapabilityRegistry()
        engine = GroundingEngine(r, MockBackend(self._mock_plan()), auto_chain=False)
        with pytest.raises(EmptyRegistryError):
            engine.ground("go to table")

    def test_retry_on_connection_error(self):
        """Backend fails twice then succeeds — engine should retry and return plan."""
        from ros2_lingua_core.errors import BackendConnectionError
        r = CapabilityRegistry()
        r.register(self._nav_cap())
        backend = MockBackend(
            self._mock_plan(),
            fail_times=2,
            fail_with=BackendConnectionError("Simulated"),
        )
        engine = GroundingEngine(r, backend, auto_chain=False, max_retries=3, retry_delay=0.0)
        plan = engine.ground("go to table")
        assert plan.feasible
        assert plan.steps[0].capability_name == "navigate"

    def test_auth_error_not_retried(self):
        """Auth errors should raise immediately without retrying."""
        from ros2_lingua_core.errors import BackendAuthError, MaxRetriesExceededError
        r = CapabilityRegistry()
        r.register(self._nav_cap())
        backend = MockBackend(
            self._mock_plan(),
            fail_times=99,
            fail_with=BackendAuthError("Bad API key"),
        )
        engine = GroundingEngine(r, backend, auto_chain=False, max_retries=3, retry_delay=0.0)
        with pytest.raises(BackendAuthError):
            engine.ground("go to table")

    def test_max_retries_exceeded_raises(self):
        """If all retries fail, MaxRetriesExceededError should be raised."""
        from ros2_lingua_core.errors import BackendConnectionError, MaxRetriesExceededError
        r = CapabilityRegistry()
        r.register(self._nav_cap())
        backend = MockBackend(
            self._mock_plan(),
            fail_times=99,
            fail_with=BackendConnectionError("Always fails"),
        )
        engine = GroundingEngine(r, backend, auto_chain=False, max_retries=2, retry_delay=0.0)
        with pytest.raises(MaxRetriesExceededError) as exc_info:
            engine.ground("go to table")
        assert exc_info.value.last_error is not None

    def test_mock_backend_fail_then_succeed(self):
        """MockBackend fail_times works correctly."""
        from ros2_lingua_core.errors import BackendConnectionError
        backend = MockBackend("response", fail_times=2,
                              fail_with=BackendConnectionError("x"))
        calls = 0
        errors = 0
        for _ in range(3):
            try:
                result = backend.complete([])
                calls += 1
            except BackendConnectionError:
                errors += 1
        assert errors == 2
        assert calls == 1

    def test_typed_errors_hierarchy(self):
        """All backend errors should be catchable as BackendError."""
        from ros2_lingua_core.errors import (
            BackendError, BackendConnectionError, BackendAuthError,
            BackendTimeoutError, BackendRateLimitError,
        )
        for exc_class in [BackendConnectionError, BackendAuthError,
                          BackendTimeoutError, BackendRateLimitError]:
            instance = exc_class("test")
            assert isinstance(instance, BackendError)

    def test_grounding_error_hierarchy(self):
        """Grounding errors should be catchable as GroundingError."""
        from ros2_lingua_core.errors import (
            GroundingError, EmptyRegistryError, MaxRetriesExceededError,
        )
        assert isinstance(EmptyRegistryError("test"), GroundingError)
        assert isinstance(MaxRetriesExceededError("test"), GroundingError)


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
