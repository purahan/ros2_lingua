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
import sys
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

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
        parameters=[CapabilityParameter("location_name", "string", "Where to go")],
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
        cap = Capability(name="", description="desc", ros_action="test/action")
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

    def test_concurrent_reads_and_writes(self):
        from concurrent.futures import ThreadPoolExecutor
        from threading import Barrier
        from time import sleep

        registry = CapabilityRegistry()
        for index in range(64):
            registry.update(
                Capability(
                    name=f"cap_{index}",
                    description="concurrent capability",
                    ros_action="robot/action",
                    tags=["concurrency"],
                )
            )
        registry.update(Capability(name="goal", description="goal", ros_action="robot/goal"))

        start = Barrier(4)

        def writer():
            start.wait(timeout=5)
            for index in range(1000):
                name = f"cap_{index % 64}"
                registry.update(
                    Capability(
                        name=name,
                        description="updated capability",
                        ros_action="robot/action",
                        tags=["concurrency"],
                    )
                )
                if index % 2:
                    registry.unregister(name)
                registry.set_state(f"state_{index % 8}")
                if index % 3 == 0:
                    registry.clear_state(f"state_{index % 8}")
                if index % 10 == 0:
                    sleep(0)

        def reader():
            start.wait(timeout=5)
            for index in range(500):
                registry.get_all()
                registry.names()
                registry.get_by_tag("concurrency")
                registry.get_by_tags(["concurrency"])
                registry.get_all_tags()
                registry.get_untagged()
                registry.get_state()
                registry.is_satisfied(["state_1"])
                registry.to_llm_context(tags=["concurrency"])
                registry.resolve_chain("goal")
                if index % 10 == 0:
                    sleep(0)

        with ThreadPoolExecutor(max_workers=4) as executor:
            tasks = [executor.submit(writer)] + [executor.submit(reader) for _ in range(3)]
            for task in tasks:
                task.result(timeout=15)


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
        r.register(
            Capability(name="pick", description="d", ros_action="a/c", tags=["manipulation"])
        )
        r.register(Capability(name="say", description="d", ros_service="a/d", tags=["speech"]))
        loco = r.get_by_tag("locomotion")
        assert len(loco) == 1
        assert loco[0].name == "nav"

    def test_get_by_tags_any(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b", tags=["locomotion"]))
        r.register(
            Capability(name="pick", description="d", ros_action="a/c", tags=["manipulation"])
        )
        r.register(Capability(name="say", description="d", ros_service="a/d", tags=["speech"]))
        result = r.get_by_tags(["locomotion", "manipulation"], match="any")
        names = {c.name for c in result}
        assert names == {"nav", "pick"}

    def test_get_by_tags_all(self):
        r = CapabilityRegistry()
        r.register(
            Capability(
                name="wave", description="d", ros_action="a/b", tags=["manipulation", "social"]
            )
        )
        r.register(
            Capability(name="pick", description="d", ros_action="a/c", tags=["manipulation"])
        )
        result = r.get_by_tags(["manipulation", "social"], match="all")
        assert len(result) == 1
        assert result[0].name == "wave"

    def test_get_all_tags(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b", tags=["locomotion"]))
        r.register(
            Capability(
                name="pick", description="d", ros_action="a/c", tags=["manipulation", "social"]
            )
        )
        all_tags = r.get_all_tags()
        assert all_tags == sorted(["locomotion", "manipulation", "social"])

    def test_get_untagged(self):
        r = CapabilityRegistry()
        r.register(
            Capability(name="tagged", description="d", ros_action="a/b", tags=["locomotion"])
        )
        r.register(Capability(name="untagged", description="d", ros_action="a/c"))
        untagged = r.get_untagged()
        assert len(untagged) == 1
        assert untagged[0].name == "untagged"

    def test_llm_context_tag_filter(self):
        r = CapabilityRegistry()
        r.register(
            Capability(name="nav", description="navigates", ros_action="a/b", tags=["locomotion"])
        )
        r.register(
            Capability(name="pick", description="picks", ros_action="a/c", tags=["manipulation"])
        )
        context = r.to_llm_context(tags=["locomotion"])
        assert "nav" in context
        assert "pick" not in context

    def test_llm_context_no_filter_shows_all(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b", tags=["locomotion"]))
        r.register(
            Capability(name="pick", description="d", ros_action="a/c", tags=["manipulation"])
        )
        context = r.to_llm_context()
        assert "nav" in context
        assert "pick" in context

    def test_untagged_always_included_in_filtered_context(self):
        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b", tags=["locomotion"]))
        r.register(Capability(name="untagged_cap", description="d", ros_action="a/c"))
        # Filter by manipulation — nav should be excluded, untagged_cap included
        context = r.to_llm_context(tags=["manipulation"])
        assert "nav" not in context
        assert "untagged_cap" in context

    def test_ground_with_tag_filter(self, registry):
        import json

        mock_response = json.dumps(
            {
                "feasible": True,
                "reason": "",
                "steps": [
                    {
                        "capability_name": "navigate_to_location",
                        "parameters": {"location_name": "table"},
                        "rationale": "go",
                    }
                ],
            }
        )
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
        chain = registry.resolve_chain("navigate_to_location", current_state={"robot_is_balanced"})
        names = [c.name for c in chain]
        assert names == ["navigate_to_location"]

    def test_unsatisfiable_precondition_raises(self):
        r = CapabilityRegistry()
        r.register(
            Capability(
                name="do_thing",
                description="Does a thing",
                ros_action="robot/do_thing",
                preconditions=["impossible_condition"],
                postconditions=[],
            )
        )
        with pytest.raises(ValueError, match="Cannot satisfy precondition"):
            r.resolve_chain("do_thing", current_state=set())


# ------------------------------------------------------------------
# Grounding Engine tests
# ------------------------------------------------------------------


class TestGroundingEngine:
    def test_successful_grounding(self, registry):
        mock_response = json.dumps(
            {
                "feasible": True,
                "reason": "",
                "steps": [
                    {
                        "capability_name": "navigate_to_location",
                        "parameters": {"location_name": "table"},
                        "rationale": "Go to table",
                    }
                ],
            }
        )
        engine = GroundingEngine(registry, MockBackend(mock_response), auto_chain=False)
        plan = engine.ground("go to the table")
        assert plan.feasible is True
        assert len(plan.steps) == 1
        assert plan.steps[0].capability_name == "navigate_to_location"
        assert plan.steps[0].parameters["location_name"] == "table"

    def test_auto_chain_inserts_prerequisites(self, registry):
        # LLM suggests navigate_to_location but robot_is_balanced is not set
        mock_response = json.dumps(
            {
                "feasible": True,
                "reason": "",
                "steps": [
                    {
                        "capability_name": "navigate_to_location",
                        "parameters": {"location_name": "table"},
                        "rationale": "Go to table",
                    }
                ],
            }
        )
        engine = GroundingEngine(registry, MockBackend(mock_response), auto_chain=True)
        # State is empty — robot_is_balanced not satisfied
        plan = engine.ground("go to the table")
        assert plan.feasible is True
        names = [s.capability_name for s in plan.steps]
        # stabilize_robot should have been auto-inserted
        assert "stabilize_robot" in names
        assert names.index("stabilize_robot") < names.index("navigate_to_location")

    def test_infeasible_plan(self, registry):
        mock_response = json.dumps(
            {
                "feasible": False,
                "reason": "No capability for flying.",
                "steps": [],
            }
        )
        engine = GroundingEngine(registry, MockBackend(mock_response))
        plan = engine.ground("fly to the moon")
        assert plan.feasible is False
        assert "flying" in plan.reason

    def test_hallucinated_capability_caught(self, registry):
        mock_response = json.dumps(
            {
                "feasible": True,
                "reason": "",
                "steps": [
                    {
                        "capability_name": "nonexistent_capability",
                        "parameters": {},
                        "rationale": "test",
                    }
                ],
            }
        )
        engine = GroundingEngine(registry, MockBackend(mock_response))
        plan = engine.ground("do something fake")
        assert plan.feasible is False
        assert "hallucination" in plan.reason.lower() or "unknown" in plan.reason.lower()

    @pytest.mark.parametrize(
        ("response", "expected_reason"),
        [
            ('{"feasible": "false", "steps": []}', "'feasible' must be a boolean"),
            ('{"feasible": 1, "steps": []}', "'feasible' must be a boolean"),
            ('{"steps": {}}', "'steps' must be a list"),
            ('{"steps": [null]}', "steps[0] must be an object"),
            (
                '{"steps": [{"capability_name": "navigate_to_location", "parameters": []}]}',
                "steps[0].parameters must be an object",
            ),
            ("[]", "top-level JSON value must be an object"),
        ],
    )
    def test_malformed_response_shapes_are_infeasible(self, registry, response, expected_reason):
        engine = GroundingEngine(registry, MockBackend(response), auto_chain=False)
        plan = engine.ground("anything")

        assert plan.feasible is False
        assert plan.steps == []
        assert expected_reason in plan.reason

    def test_invalid_json_from_llm(self, registry):
        engine = GroundingEngine(registry, MockBackend("this is not json"))
        plan = engine.ground("anything")
        assert plan.feasible is False
        assert "JSON" in plan.reason


# ------------------------------------------------------------------
# Plan cache tests
# ------------------------------------------------------------------


@pytest.fixture
def cache_engine(registry, monkeypatch):
    clock = SimpleNamespace(now=1000.0)
    # Cover the old wall clock implementation as well as the monotonic clock.
    monkeypatch.setattr("ros2_lingua_core.grounding.time.time", lambda: clock.now)
    monkeypatch.setattr("ros2_lingua_core.grounding.time.monotonic", lambda: clock.now)
    response = json.dumps(
        {
            "feasible": True,
            "steps": [
                {
                    "capability_name": "navigate_to_location",
                    "parameters": {"location_name": "table", "route": {"points": [1, 2]}},
                    "rationale": "Go to table",
                }
            ],
        }
    )

    def make_engine(**kwargs):
        backend = Mock(spec=["complete"])
        backend.complete.return_value = response
        options = {"cache_ttl_sec": 10.0, "max_retries": 1}
        options.update(kwargs)
        engine = GroundingEngine(registry, backend, **options)
        return engine, backend, clock

    return make_engine


class TestPlanCache:
    def test_uncopyable_plan_is_returned_without_caching(self, cache_engine, basic_capability):
        # JSON and the prompt accept this depth, but recursive deepcopy exceeds
        # the Python recursion limit. The cache must not turn success into failure.
        value = []
        for _ in range(sys.getrecursionlimit() // 2 + 50):
            value = [value]
        basic_capability.parameters.append(
            CapabilityParameter("fallback", "object", "Fallback", required=False, default=value)
        )
        engine, backend, _ = cache_engine()

        assert engine.ground("go to the table").feasible
        assert engine.ground("go to the table").feasible
        assert backend.complete.call_count == 2
        assert not engine._plan_cache

    def test_failed_cache_hit_copy_regenerates_plan(self, cache_engine, monkeypatch):
        engine, backend, _ = cache_engine()
        expected = engine.ground("go to the table")
        copier = Mock(side_effect=[TypeError("cannot copy cached plan"), deepcopy(expected)])
        monkeypatch.setattr("ros2_lingua_core.grounding.deepcopy", copier)

        assert engine.ground("go to the table") == expected
        assert backend.complete.call_count == 2
        assert len(engine._plan_cache) == 1

        monkeypatch.setattr("ros2_lingua_core.grounding.deepcopy", deepcopy)
        assert engine.ground("go to the table") == expected
        assert backend.complete.call_count == 2

    @pytest.mark.parametrize("field", ["metadata", "default"])
    @pytest.mark.parametrize("value_kind", ["path", "mixed-keys", "cycle"])
    def test_unserializable_definitions_bypass_cache(
        self, cache_engine, basic_capability, field, value_kind
    ):
        parameter = CapabilityParameter("fallback", "object", "Fallback", required=False)
        basic_capability.parameters.append(parameter)
        engine, backend, _ = cache_engine()
        expected = engine.ground("go to the table")
        assert expected.feasible

        if value_kind == "path":
            value = Path("maps/floor.yaml")
        elif value_kind == "mixed-keys":
            value = {1: 0.5, "default": 0.2}
        else:
            value = []
            value.append(value)

        if field == "metadata":
            basic_capability.metadata = {"configuration": value}
        else:
            parameter.default = value

        for _ in range(2):
            plan = engine.ground("go to the table")
            assert plan.feasible
            if field == "default":
                fallback = plan.steps[-1].parameters["fallback"]
                if value_kind == "cycle":
                    assert fallback[0] is fallback
                else:
                    assert fallback == value
        assert backend.complete.call_count == 3
        assert len(engine._plan_cache) == 1

        # Bypassing an unsafe key must neither reuse nor overwrite an existing entry.
        basic_capability.metadata = {}
        parameter.default = None
        assert engine.ground("go to the table") == expected
        assert backend.complete.call_count == 3

    def test_identical_request_uses_cached_chained_plan(self, cache_engine):
        engine, backend, _ = cache_engine()
        first = engine.ground("go to the table")
        second = engine.ground("go to the table")

        assert first.feasible
        assert [step.capability_name for step in first.steps] == [
            "stabilize_robot",
            "navigate_to_location",
        ]
        assert second == first
        assert second is not first
        assert backend.complete.call_count == 1

    @pytest.mark.parametrize(
        "response",
        [
            pytest.param("not JSON", id="invalid-json"),
            pytest.param('{"feasible": false, "reason": "try again"}', id="infeasible"),
            pytest.param('{"steps": {}}', id="invalid-shape"),
            pytest.param('{"steps": [{"capability_name": "missing"}]}', id="unknown-capability"),
            pytest.param(
                '{"steps": [{"capability_name": "navigate_to_location", "parameters": {}}]}',
                id="invalid-parameters",
            ),
            pytest.param('{"steps": [{"capability_name": "blocked"}]}', id="chaining-failure"),
        ],
    )
    def test_failed_plan_is_retried_on_next_request(self, cache_engine, registry, response):
        registry.register(
            Capability(
                name="blocked",
                description="Has an unavailable prerequisite",
                ros_action="robot/blocked",
                preconditions=["unavailable"],
            )
        )
        engine, backend, _ = cache_engine()
        backend.complete.side_effect = [response, backend.complete.return_value]

        assert not engine.ground("go to the table").feasible
        assert engine.ground("go to the table").feasible
        assert engine.ground("go to the table").feasible
        assert backend.complete.call_count == 2

    def test_backend_error_is_retried_on_next_request(self, cache_engine):
        from ros2_lingua_core import LLMTimeoutError

        engine, backend, _ = cache_engine()
        backend.complete.side_effect = [LLMTimeoutError("timed out"), backend.complete.return_value]

        assert not engine.ground("go to the table").feasible
        assert engine.ground("go to the table").feasible
        assert engine.ground("go to the table").feasible
        assert backend.complete.call_count == 2

    @pytest.mark.parametrize(
        ("field", "value"),
        [
            ("description", "Navigate carefully to a named location"),
            ("ros_action", "robot/new_navigate"),
            ("ros_service", "robot/navigate_service"),
            ("preconditions", ["robot_is_balanced", "door_open"]),
            ("postconditions", ["robot_at_table"]),
            ("metadata", {"limits": {"speed": 0.5}}),
            ("tags", ["indoor"]),
        ],
    )
    def test_capability_definition_change_misses_cache(
        self, cache_engine, registry, basic_capability, field, value
    ):
        registry.set_state("door_open")
        engine, backend, _ = cache_engine()
        assert engine.ground("go to the table").feasible

        updated = deepcopy(basic_capability)
        setattr(updated, field, value)
        if field == "ros_service":
            updated.ros_action = None
        registry.update(updated)

        assert engine.ground("go to the table").feasible
        assert backend.complete.call_count == 2

    @pytest.mark.parametrize(
        ("field", "value"),
        [
            ("name", "destination"),
            ("type", "int"),
            ("description", "Destination name or number"),
            ("required", False),
            ("default", "door"),
        ],
    )
    def test_parameter_definition_change_misses_cache(
        self, cache_engine, basic_capability, field, value
    ):
        engine, backend, _ = cache_engine()
        assert engine.ground("go to the table").feasible

        # In-place changes must also be detected, without registry.update().
        setattr(basic_capability.parameters[0], field, value)
        engine.ground("go to the table")
        assert backend.complete.call_count == 2

    def test_filtered_out_capability_change_misses_cache(self, cache_engine, stabilize_capability):
        engine, backend, _ = cache_engine()
        assert engine.ground("go to the table", tag_filter=["locomotion"]).feasible
        stabilize_capability.description = "Balance carefully before navigating"
        assert engine.ground("go to the table", tag_filter=["locomotion"]).feasible
        assert backend.complete.call_count == 2

    def test_state_change_recomputes_prerequisites(self, cache_engine, registry):
        engine, backend, _ = cache_engine()
        assert len(engine.ground("go to the table").steps) == 2
        registry.set_state("robot_is_balanced")
        assert len(engine.ground("go to the table").steps) == 1
        assert backend.complete.call_count == 2

    @pytest.mark.parametrize(
        ("first_filter", "second_filter"),
        [(None, []), (None, ["locomotion"]), (["locomotion"], ["balance"])],
    )
    def test_tag_filter_change_misses_cache(self, cache_engine, first_filter, second_filter):
        engine, backend, _ = cache_engine()
        engine.ground("go to the table", tag_filter=first_filter)
        engine.ground("go to the table", tag_filter=second_filter)
        engine.ground("go to the table", tag_filter=second_filter)
        assert backend.complete.call_count == 2
        first_prompt = backend.complete.call_args_list[0].args[0][0]["content"]
        second_prompt = backend.complete.call_args_list[1].args[0][0]["content"]
        assert first_prompt != second_prompt

    def test_mutating_filter_list_misses_cache(self, cache_engine):
        engine, backend, _ = cache_engine()
        tags = ["locomotion"]
        engine.ground("go to the table", tag_filter=tags)
        tags[:] = ["balance"]
        engine.ground("go to the table", tag_filter=tags)
        assert backend.complete.call_count == 2

    @pytest.mark.parametrize("mutate_cache_hit", [False, True], ids=["first-result", "cache-hit"])
    def test_returned_plan_mutations_do_not_change_cache(self, cache_engine, mutate_cache_hit):
        engine, backend, _ = cache_engine()
        result = engine.ground("go to the table")
        expected = deepcopy(result)
        if mutate_cache_hit:
            result = engine.ground("go to the table")

        result.original_instruction = "changed"
        result.feasible = False
        result.reason = "changed"
        result.steps[-1].parameters["route"]["points"].append(3)
        result.steps[-1].rationale = "changed"
        result.steps[-1].capability_name = "changed"
        result.steps.pop(0)

        assert engine.ground("go to the table") == expected
        assert backend.complete.call_count == 1

    def test_ttl_expires_from_insertion_not_last_hit(self, cache_engine):
        engine, backend, clock = cache_engine()
        engine.ground("go to the table")
        clock.now += 9
        engine.ground("go to the table")
        assert backend.complete.call_count == 1

        clock.now += 1
        engine.ground("go to the table")
        assert backend.complete.call_count == 2
        clock.now += 9
        engine.ground("go to the table")
        assert backend.complete.call_count == 2

    def test_ttl_uses_completion_time(self, cache_engine):
        engine, backend, clock = cache_engine()

        def slow_response(messages):
            clock.now += 20
            return backend.complete.return_value

        backend.complete.side_effect = slow_response
        engine.ground("go to the table")
        clock.now += 9
        engine.ground("go to the table")
        assert backend.complete.call_count == 1

    def test_ttl_is_unaffected_by_wall_clock_changes(self, cache_engine, monkeypatch):
        engine, backend, clock = cache_engine()
        engine.ground("go to the table")
        monkeypatch.setattr("ros2_lingua_core.grounding.time.time", lambda: 0.0)
        clock.now += 11
        engine.ground("go to the table")
        assert backend.complete.call_count == 2

    @pytest.mark.parametrize("ttl", [0, -1])
    def test_nonpositive_ttl_disables_cache(self, cache_engine, ttl):
        engine, backend, _ = cache_engine(cache_ttl_sec=ttl)
        engine.ground("go to the table")
        engine.ground("go to the table")
        assert backend.complete.call_count == 2
        assert not engine._plan_cache

    def test_lru_hit_protects_entry_from_eviction(self, cache_engine):
        engine, backend, _ = cache_engine(cache_max_size=2)
        for instruction in ["A", "B", "A", "C", "A", "C"]:
            assert engine.ground(instruction).feasible
        assert backend.complete.call_count == 3
        assert len(engine._plan_cache) == 2

        engine.ground("B")
        assert backend.complete.call_count == 4
        assert len(engine._plan_cache) == 2

    def test_default_cache_size_is_bounded(self, cache_engine):
        engine, backend, _ = cache_engine()
        for index in range(129):
            assert engine.ground(f"request {index}").feasible
        assert len(engine._plan_cache) == 128
        engine.ground("request 0")
        assert backend.complete.call_count == 130

    @pytest.mark.parametrize("size", [0, -1])
    def test_nonpositive_size_disables_cache(self, cache_engine, size):
        engine, backend, _ = cache_engine(cache_max_size=size)
        engine.ground("go to the table")
        engine.ground("go to the table")
        assert backend.complete.call_count == 2
        assert not engine._plan_cache

    def test_failure_does_not_evict_successful_plan(self, cache_engine):
        engine, backend, _ = cache_engine(cache_max_size=1)
        engine.ground("go to the table")
        backend.complete.return_value = '{"feasible": false}'
        assert not engine.ground("impossible").feasible
        assert engine.ground("go to the table").feasible
        assert backend.complete.call_count == 2
        assert len(engine._plan_cache) == 1


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

        from ros2_lingua_core import LLMTimeoutError, MockBackend

        success_response = json.dumps({"feasible": True, "reason": "", "steps": []})
        backend = MockBackend.failing(
            LLMTimeoutError("timed out"),
            retries_before_success=2,
            success_response=success_response,
        )
        import contextlib

        # First 2 calls raise, 3rd succeeds
        for _ in range(2):
            with contextlib.suppress(LLMTimeoutError):
                backend.complete([])
        result = backend.complete([])
        assert "feasible" in result

    def test_empty_registry_returns_infeasible(self):

        from ros2_lingua_core import CapabilityRegistry, GroundingEngine, MockBackend

        r = CapabilityRegistry()
        engine = GroundingEngine(r, MockBackend("{}"), auto_chain=False)
        plan = engine.ground("do something")
        assert not plan.feasible
        assert "registered" in plan.reason.lower()

    def test_error_hierarchy(self):
        from ros2_lingua_core import (
            GroundingError,
            HallucinationError,
            LinguaError,
            LLMBackendError,
            LLMRateLimitError,
            LLMTimeoutError,
            PlanningError,
            UnsatisfiablePreconditionError,
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
        import json

        from ros2_lingua_core import Capability, CapabilityRegistry, GroundingEngine, MockBackend

        r = CapabilityRegistry()
        r.register(Capability(name="nav", description="d", ros_action="a/b"))
        mock = MockBackend(
            json.dumps(
                {
                    "feasible": True,
                    "reason": "",
                    "steps": [{"capability_name": "nav", "parameters": {}, "rationale": "go"}],
                }
            )
        )
        # Verify max_retries and retry params are accepted
        engine = GroundingEngine(
            r, mock, auto_chain=False, max_retries=5, retry_delay=0.1, retry_backoff=1.5
        )
        plan = engine.ground("do nav")
        assert plan.feasible

    def test_grounding_engine_does_not_retry_model_errors(self, registry):
        from ros2_lingua_core import LLMModelNotFoundError

        class FailingBackend:
            calls = 0

            def complete(self, messages):
                self.calls += 1
                raise LLMModelNotFoundError("model not found")

        backend = FailingBackend()
        engine = GroundingEngine(registry, backend, auto_chain=False, max_retries=3, retry_delay=0)

        plan = engine.ground("go to the table")

        assert not plan.feasible
        assert backend.calls == 1
        assert "model not found" in plan.reason

    def test_grounding_engine_retries_only_invalid_json(self, registry):
        import json

        class ResponseSequenceBackend:
            def __init__(self):
                self.responses = iter(
                    [
                        "not json",
                        json.dumps(
                            {
                                "feasible": True,
                                "reason": "",
                                "steps": [
                                    {
                                        "capability_name": "navigate_to_location",
                                        "parameters": {"location_name": "table"},
                                        "rationale": "go",
                                    }
                                ],
                            }
                        ),
                    ]
                )
                self.calls = 0

            def complete(self, messages):
                self.calls += 1
                return next(self.responses)

        backend = ResponseSequenceBackend()
        engine = GroundingEngine(registry, backend, auto_chain=False, max_retries=2, retry_delay=0)

        plan = engine.ground("go to the table")

        assert plan.feasible
        assert backend.calls == 2

    def test_openai_authentication_failure_is_not_retried(self, monkeypatch, registry):
        import sys
        import time
        from types import ModuleType, SimpleNamespace

        from ros2_lingua_core import OpenAIBackend, RetryConfig

        class AuthenticationError(Exception):
            pass

        calls = 0
        client_options = {}

        class FakeOpenAI:
            def __init__(self, **kwargs):
                client_options.update(kwargs)
                self.chat = SimpleNamespace(
                    completions=SimpleNamespace(create=self.create_completion)
                )

            def create_completion(self, **kwargs):
                nonlocal calls
                calls += 1
                raise AuthenticationError("invalid API key")

        sdk = ModuleType("openai")
        sdk.OpenAI = FakeOpenAI
        sdk.AuthenticationError = AuthenticationError
        monkeypatch.setitem(sys.modules, "openai", sdk)
        monkeypatch.setattr(time, "sleep", lambda _: None)

        backend = OpenAIBackend(
            api_key="invalid",
            retry=RetryConfig(max_retries=3, base_delay_sec=0),
        )
        engine = GroundingEngine(registry, backend, auto_chain=False, max_retries=3, retry_delay=0)

        plan = engine.ground("go to the table")

        assert not plan.feasible
        assert calls == 1
        assert client_options["max_retries"] == 0
        assert "invalid API key" in plan.reason

    def test_openai_retries_transient_backend_errors(self, monkeypatch):
        import sys
        import time
        from types import ModuleType, SimpleNamespace

        from ros2_lingua_core import OpenAIBackend, RetryConfig

        class APIConnectionError(Exception):
            pass

        calls = 0
        client_options = {}

        class FakeOpenAI:
            def __init__(self, **kwargs):
                client_options.update(kwargs)
                self.chat = SimpleNamespace(
                    completions=SimpleNamespace(create=self.create_completion)
                )

            def create_completion(self, **kwargs):
                nonlocal calls
                calls += 1
                if calls < 3:
                    raise APIConnectionError("connection reset")
                return SimpleNamespace(
                    choices=[SimpleNamespace(message=SimpleNamespace(content="ok"))]
                )

        sdk = ModuleType("openai")
        sdk.OpenAI = FakeOpenAI
        sdk.APIConnectionError = APIConnectionError
        monkeypatch.setitem(sys.modules, "openai", sdk)
        monkeypatch.setattr(time, "sleep", lambda _: None)

        backend = OpenAIBackend(
            api_key="test",
            retry=RetryConfig(max_retries=2, base_delay_sec=0),
        )

        assert backend.complete([]) == "ok"
        assert calls == 3
        assert client_options["max_retries"] == 0

    def test_openai_model_not_found_is_not_retried(self, monkeypatch):
        import sys
        import time
        from types import ModuleType, SimpleNamespace

        import pytest

        from ros2_lingua_core import LLMModelNotFoundError, OpenAIBackend, RetryConfig

        class NotFoundError(Exception):
            pass

        calls = 0

        class FakeOpenAI:
            def __init__(self, **kwargs):
                self.chat = SimpleNamespace(
                    completions=SimpleNamespace(create=self.create_completion)
                )

            def create_completion(self, **kwargs):
                nonlocal calls
                calls += 1
                raise NotFoundError("model not found")

        sdk = ModuleType("openai")
        sdk.OpenAI = FakeOpenAI
        sdk.NotFoundError = NotFoundError
        monkeypatch.setitem(sys.modules, "openai", sdk)
        monkeypatch.setattr(time, "sleep", lambda _: None)

        backend = OpenAIBackend(
            api_key="test",
            model="missing-model",
            retry=RetryConfig(max_retries=3, base_delay_sec=0),
        )

        with pytest.raises(LLMModelNotFoundError, match="missing-model.*not found"):
            backend.complete([])

        assert calls == 1

    def test_anthropic_authentication_failure_is_not_retried(self, monkeypatch):
        import sys
        import time
        from types import ModuleType, SimpleNamespace

        import pytest

        from ros2_lingua_core import AnthropicBackend, LLMAuthError, RetryConfig

        class AuthenticationError(Exception):
            pass

        calls = 0
        client_options = {}

        class FakeAnthropic:
            def __init__(self, **kwargs):
                client_options.update(kwargs)
                self.messages = SimpleNamespace(create=self.create_message)

            def create_message(self, **kwargs):
                nonlocal calls
                calls += 1
                raise AuthenticationError("invalid API key")

        sdk = ModuleType("anthropic")
        sdk.Anthropic = FakeAnthropic
        sdk.AuthenticationError = AuthenticationError
        monkeypatch.setitem(sys.modules, "anthropic", sdk)
        monkeypatch.setattr(time, "sleep", lambda _: None)

        backend = AnthropicBackend(
            api_key="invalid",
            retry=RetryConfig(max_retries=3, base_delay_sec=0),
        )

        with pytest.raises(LLMAuthError, match="authentication failed"):
            backend.complete([])

        assert calls == 1
        assert client_options["max_retries"] == 0

    def test_anthropic_overloaded_error_respects_rate_limit_policy(self, monkeypatch):
        import sys
        import time
        from types import ModuleType, SimpleNamespace

        import pytest

        from ros2_lingua_core import AnthropicBackend, LLMRateLimitError, RetryConfig

        class OverloadedError(Exception):
            pass

        calls = 0

        class FakeAnthropic:
            def __init__(self, **kwargs):
                self.messages = SimpleNamespace(create=self.create_message)

            def create_message(self, **kwargs):
                nonlocal calls
                calls += 1
                raise OverloadedError("temporarily overloaded")

        sdk = ModuleType("anthropic")
        sdk.Anthropic = FakeAnthropic
        sdk.OverloadedError = OverloadedError
        monkeypatch.setitem(sys.modules, "anthropic", sdk)
        monkeypatch.setattr(time, "sleep", lambda _: None)

        backend = AnthropicBackend(
            api_key="test",
            retry=RetryConfig(max_retries=3, retry_on_rate_limit=False),
        )

        with pytest.raises(LLMRateLimitError, match="rate limit"):
            backend.complete([])

        assert calls == 1

    def test_ollama_uses_client_bound_to_host_and_timeout(self, monkeypatch):
        from ros2_lingua_core import OllamaBackend, RetryConfig

        client = Mock()
        client.chat.return_value = {"message": {"content": "ok"}}
        sdk = SimpleNamespace(Client=Mock(return_value=client), chat=client.chat)
        monkeypatch.setitem(sys.modules, "ollama", sdk)

        backend = OllamaBackend(
            model="test-model",
            host="http://ollama.internal:11434",
            timeout_sec=12.5,
            retry=RetryConfig.no_retry(),
        )
        messages = [{"role": "user", "content": "hi"}]
        assert backend.complete(messages) == "ok"
        sdk.Client.assert_called_once_with(host="http://ollama.internal:11434", timeout=12.5)
        client.chat.assert_called_once_with(
            model="test-model", messages=messages, options={"temperature": 0.0}
        )

    def test_anthropic_passes_timeout_and_temperature(self, monkeypatch):
        from ros2_lingua_core import AnthropicBackend, RetryConfig

        client = Mock()
        client.messages.create.return_value = SimpleNamespace(content=[SimpleNamespace(text="ok")])
        sdk = SimpleNamespace(Anthropic=Mock(return_value=client))
        monkeypatch.setitem(sys.modules, "anthropic", sdk)

        backend = AnthropicBackend(
            api_key="test", model="test-model", timeout_sec=9.0, retry=RetryConfig.no_retry()
        )
        messages = [{"role": "user", "content": "hi"}]
        assert backend.complete([{"role": "system", "content": "be brief"}, *messages]) == "ok"
        sdk.Anthropic.assert_called_once_with(api_key="test", timeout=9.0, max_retries=0)
        client.messages.create.assert_called_once_with(
            model="test-model",
            max_tokens=1024,
            system="be brief",
            messages=messages,
            temperature=0.0,
        )

    def test_ollama_response_errors_use_typed_status_codes(self, monkeypatch):
        import sys
        from types import ModuleType

        import pytest

        from ros2_lingua_core import (
            LLMAuthError,
            LLMModelNotFoundError,
            LLMRateLimitError,
            LLMTimeoutError,
            OllamaBackend,
            RetryConfig,
        )

        class ResponseError(Exception):
            def __init__(self, message, status_code):
                super().__init__(message)
                self.status_code = status_code

        calls = 0
        current_status = 401

        def chat(**kwargs):
            nonlocal calls
            calls += 1
            raise ResponseError("request failed", current_status)

        sdk = ModuleType("ollama")
        sdk.ResponseError = ResponseError
        sdk.Client = Mock(return_value=SimpleNamespace(chat=chat))
        monkeypatch.setitem(sys.modules, "ollama", sdk)

        cases = [
            (401, LLMAuthError),
            (403, LLMAuthError),
            (404, LLMModelNotFoundError),
            (408, LLMTimeoutError),
            (429, LLMRateLimitError),
        ]
        for status_code, expected_error in cases:
            current_status = status_code
            calls = 0
            backend = OllamaBackend(
                retry=RetryConfig(
                    max_retries=3,
                    retry_on_rate_limit=False,
                    retry_on_timeout=False,
                )
            )

            with pytest.raises(expected_error):
                backend.complete([])

            assert calls == 1


# ------------------------------------------------------------------
# Parameter validation tests
# ------------------------------------------------------------------


class TestParameterValidation:
    """Tests for ParameterValidator and its integration into GroundingEngine."""

    def _make_cap(self, params):
        return Capability(
            name="test_cap",
            description="d",
            ros_action="a/b",
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
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

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
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

        cap = self._make_cap([CapabilityParameter("count", "int", "count")])
        with pytest.raises(ParameterValidationError):
            ParameterValidator().validate(cap, {"count": 3.5})

    def test_float_non_finite_fails(self):
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

        cap = self._make_cap([CapabilityParameter("speed", "float", "speed")])
        for val in [float("nan"), float("inf"), float("-inf"), "nan", "inf", "-inf"]:
            with pytest.raises(ParameterValidationError):
                ParameterValidator().validate(cap, {"speed": val})

    def test_float_rejects_bool(self):
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

        cap = self._make_cap([CapabilityParameter("speed", "float", "speed")])
        with pytest.raises(ParameterValidationError):
            ParameterValidator().validate(cap, {"speed": True})

    def test_int_non_finite_float_fails(self):
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

        cap = self._make_cap([CapabilityParameter("count", "int", "count")])
        for val in [float("inf"), float("-inf"), float("nan"), "inf", "-inf", "1e400"]:
            with pytest.raises(ParameterValidationError):
                ParameterValidator().validate(cap, {"count": val})

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
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

        cap = self._make_cap([CapabilityParameter("enabled", "boolean", "flag")])
        with pytest.raises(ParameterValidationError):
            ParameterValidator().validate(cap, {"enabled": "maybe"})

    def test_list_from_json_string(self):
        from ros2_lingua_core import ParameterValidator

        cap = self._make_cap([CapabilityParameter("items", "list", "items")])
        result = ParameterValidator().validate(cap, {"items": '["a", "b", "c"]'})
        assert result["items"] == ["a", "b", "c"]

    def test_list_invalid_json_fails(self):
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

        cap = self._make_cap([CapabilityParameter("items", "list", "items")])
        with pytest.raises(ParameterValidationError):
            ParameterValidator().validate(cap, {"items": "not json"})

    # ── Required / optional parameters ────────────────────────

    def test_missing_required_fails(self):
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

        cap = self._make_cap([CapabilityParameter("loc", "string", "location", required=True)])
        with pytest.raises(ParameterValidationError) as exc_info:
            ParameterValidator().validate(cap, {})
        assert "required" in str(exc_info.value)

    def test_missing_optional_uses_default(self):
        from ros2_lingua_core import ParameterValidator

        cap = self._make_cap(
            [CapabilityParameter("speed", "float", "speed", required=False, default=0.5)]
        )
        result = ParameterValidator().validate(cap, {})
        assert result["speed"] == 0.5

    def test_all_failures_reported_at_once(self):
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

        cap = self._make_cap(
            [
                CapabilityParameter("speed", "float", "speed", required=True),
                CapabilityParameter("name", "string", "name", required=True),
                CapabilityParameter("count", "int", "count", required=True),
            ]
        )
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

        cap = self._make_cap([CapabilityParameter("pose", "geometry_msgs/Pose", "target pose")])
        pose_val = {"position": {"x": 1.0}, "orientation": {"w": 1.0}}
        result = ParameterValidator().validate(cap, {"pose": pose_val})
        assert result["pose"] == pose_val

    # ── Strict mode ────────────────────────────────────────────

    def test_strict_mode_rejects_unknown_params(self):
        from ros2_lingua_core import ParameterValidationError, ParameterValidator

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
        r.register(
            Capability(
                name="navigate_to_location",
                description="d",
                ros_action="a/n",
                parameters=[
                    CapabilityParameter("location_name", "string", "where", required=True),
                    CapabilityParameter("speed", "float", "how fast", required=False, default=0.5),
                ],
            )
        )
        mock_response = json.dumps(
            {
                "feasible": True,
                "reason": "",
                "steps": [
                    {
                        "capability_name": "navigate_to_location",
                        "parameters": {"location_name": "table", "speed": "fast"},
                        "rationale": "go",
                    }
                ],
            }
        )
        engine = GroundingEngine(
            r, MockBackend(mock_response), auto_chain=False, validate_params=True
        )
        plan = engine.ground("go to table")
        assert not plan.feasible
        assert "speed" in plan.reason and "fast" in plan.reason

    def test_grounding_engine_coerces_valid_params(self):
        import json

        r = CapabilityRegistry()
        r.register(
            Capability(
                name="navigate_to_location",
                description="d",
                ros_action="a/n",
                parameters=[
                    CapabilityParameter("location_name", "string", "where", required=True),
                    CapabilityParameter("speed", "float", "how fast", required=False, default=0.5),
                ],
            )
        )
        mock_response = json.dumps(
            {
                "feasible": True,
                "reason": "",
                "steps": [
                    {
                        "capability_name": "navigate_to_location",
                        "parameters": {"location_name": "table", "speed": "0.5"},
                        "rationale": "go",
                    }
                ],
            }
        )
        engine = GroundingEngine(
            r, MockBackend(mock_response), auto_chain=False, validate_params=True
        )
        plan = engine.ground("go to table")
        assert plan.feasible
        assert plan.steps[0].parameters["speed"] == 0.5
        assert isinstance(plan.steps[0].parameters["speed"], float)

    def test_grounding_engine_validation_disabled(self, registry):
        import json

        mock_response = json.dumps(
            {
                "feasible": True,
                "reason": "",
                "steps": [
                    {
                        "capability_name": "navigate_to_location",
                        "parameters": {"location_name": "table", "speed": "fast"},
                        "rationale": "go",
                    }
                ],
            }
        )
        engine = GroundingEngine(
            registry, MockBackend(mock_response), auto_chain=False, validate_params=False
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
        r.register(
            Capability(
                name="stabilize_robot",
                description="Stabilizes the robot",
                ros_action="humanoid/stabilize",
                preconditions=[],
                postconditions=["robot_is_balanced"],
            )
        )
        r.register(
            Capability(
                name="navigate_to_location",
                description="Walks to a named location",
                ros_action="humanoid/navigate",
                parameters=[CapabilityParameter("location_name", "string", "where")],
                preconditions=["robot_is_balanced"],
                postconditions=["robot_at_location"],
            )
        )
        r.register(
            Capability(
                name="pick_up_object",
                description="Picks up an object",
                ros_action="humanoid/pick",
                parameters=[CapabilityParameter("object_name", "string", "what")],
                preconditions=["robot_at_location"],
                postconditions=["object_in_hand"],
            )
        )
        r.register(
            Capability(
                name="return_to_home",
                description="Returns the robot to home",
                ros_action="humanoid/home",
                preconditions=[],
                postconditions=["robot_at_home"],
            )
        )
        return r

    def _make_engine(self, registry, plan_steps):
        """Create a GroundingEngine with a MockBackend returning the given steps."""
        mock_response = json.dumps(
            {
                "feasible": True,
                "reason": "",
                "steps": plan_steps,
            }
        )
        return GroundingEngine(
            registry,
            MockBackend(mock_response),
            auto_chain=False,
            validate_params=False,
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
        assert config.max_replans == 1

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
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

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
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        registry = self._make_registry()
        engine = self._make_engine(
            registry,
            [
                {
                    "capability_name": "pick_up_object",
                    "parameters": {"object_name": "bottle"},
                    "rationale": "pick",
                },
            ],
        )
        planner = RecoveryPlanner(
            RecoveryConfig(max_retries=1, enable_replan=True),
            grounding_engine=engine,
            registry=registry,
        )
        # First failure → retry
        d1 = planner.on_step_failed(self._failed_step(), 0, "pick up", set(), "fail")
        assert d1.strategy == "retry"
        # Second failure → replan (retries exhausted)
        d2 = planner.on_step_failed(self._failed_step(), 0, "pick up", set(), "fail again")
        assert d2.strategy == "replan"

    def test_replan_limit_resets_retry_budget_for_new_plan(self):
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        engine = Mock()
        engine.ground.return_value = SimpleNamespace(feasible=True, steps=[self._failed_step()])
        planner = RecoveryPlanner(
            RecoveryConfig(max_retries=1, max_replans=1, safe_fallback="return_to_home"),
            grounding_engine=engine,
        )
        failed_step = self._failed_step()

        assert planner.on_step_failed(failed_step, 0, "instr", set(), "err").strategy == "retry"
        assert planner.on_step_failed(failed_step, 0, "instr", set(), "err").strategy == "replan"
        assert planner.on_step_failed(failed_step, 0, "instr", set(), "err").strategy == "retry"
        decision = planner.on_step_failed(failed_step, 0, "instr", set(), "err")
        assert decision.strategy == "fallback"
        assert "return_to_home" in decision.reason
        assert engine.ground.call_count == 1

        planner.reset()
        assert planner.on_step_failed(failed_step, 0, "instr", set(), "err").strategy == "retry"
        assert planner.on_step_failed(failed_step, 0, "instr", set(), "err").strategy == "replan"
        assert engine.ground.call_count == 2

    def test_failed_replan_attempt_counts_toward_limit(self):
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        engine = Mock()
        engine.ground.side_effect = RuntimeError("backend unavailable")
        planner = RecoveryPlanner(
            RecoveryConfig(max_retries=0, max_replans=1),
            grounding_engine=engine,
        )

        first = planner.on_step_failed(self._failed_step(), 0, "instr", set(), "err")
        second = planner.on_step_failed(self._failed_step(), 0, "instr", set(), "err")
        assert first.strategy == "abort"
        assert second.strategy == "abort"
        assert "limit reached" in second.reason
        assert engine.ground.call_count == 1

    def test_retry_count_tracks_per_step(self):
        """Each step has independent retry counters."""
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        planner = RecoveryPlanner(RecoveryConfig(max_retries=1))
        # Step 0 fails once → retry
        d0 = planner.on_step_failed(self._failed_step("step_a"), 0, "instr", set(), "err")
        assert d0.strategy == "retry"
        # Step 1 also gets its own retry
        d1 = planner.on_step_failed(self._failed_step("step_b"), 1, "instr", set(), "err")
        assert d1.strategy == "retry"

    # ── Replan strategy ───────────────────────────────────────

    def test_replan_returns_new_plan(self):
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        registry = self._make_registry()
        engine = self._make_engine(
            registry,
            [
                {
                    "capability_name": "navigate_to_location",
                    "parameters": {"location_name": "kitchen"},
                    "rationale": "go",
                },
            ],
        )
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
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        registry = self._make_registry()
        # The mock response returns only pick_up — the engine with auto_chain=True
        # would insert stabilize + navigate, but since the state already has those
        # postconditions, they should be skipped
        engine_with_chain = GroundingEngine(
            registry,
            MockBackend(
                json.dumps(
                    {
                        "feasible": True,
                        "reason": "",
                        "steps": [
                            {
                                "capability_name": "pick_up_object",
                                "parameters": {"object_name": "bottle"},
                                "rationale": "pick",
                            },
                        ],
                    }
                )
            ),
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
            self._failed_step(),
            0,
            "pick up the bottle from the table",
            state,
            "gripper error",
        )
        assert decision.strategy == "replan"
        # The new plan should NOT include stabilize_robot or navigate_to_location
        # since their postconditions are already in state
        step_names = [s.capability_name for s in decision.new_plan.steps]
        assert "stabilize_robot" not in step_names
        assert "navigate_to_location" not in step_names
        assert "pick_up_object" in step_names

    def test_replan_disabled_goes_to_fallback(self):
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=False,
                safe_fallback="return_to_home",
            ),
        )
        decision = planner.on_step_failed(self._failed_step(), 0, "do thing", set(), "error")
        assert decision.strategy == "fallback"

    def test_recovery_planner_without_engine(self):
        """Replan is skipped when no grounding engine is provided."""
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=True,  # enabled but no engine
                safe_fallback="return_to_home",
            ),
        )
        decision = planner.on_step_failed(self._failed_step(), 0, "instr", set(), "err")
        # Should skip replan and go to fallback
        assert decision.strategy == "fallback"

    # ── Fallback strategy ─────────────────────────────────────

    def test_fallback_decision(self):
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=False,
                safe_fallback="return_to_home",
            ),
        )
        decision = planner.on_step_failed(self._failed_step(), 0, "instr", set(), "err")
        assert decision.strategy == "fallback"
        assert "return_to_home" in decision.reason

    def test_fallback_with_registry_validation(self):
        """Fallback capability must exist in registry when registry is provided."""
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        registry = self._make_registry()
        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=False,
                safe_fallback="nonexistent_fallback",
            ),
            registry=registry,
        )
        decision = planner.on_step_failed(self._failed_step(), 0, "instr", set(), "err")
        # Fallback not registered → abort
        assert decision.strategy == "abort"

    # ── Abort strategy ────────────────────────────────────────

    def test_no_fallback_goes_to_abort(self):
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        planner = RecoveryPlanner(
            RecoveryConfig(
                max_retries=0,
                enable_replan=False,
                safe_fallback=None,
            ),
        )
        decision = planner.on_step_failed(self._failed_step(), 0, "instr", set(), "err")
        assert decision.strategy == "abort"

    def test_abort_decision_has_reason(self):
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        planner = RecoveryPlanner(
            RecoveryConfig(max_retries=0, enable_replan=False),
        )
        decision = planner.on_step_failed(self._failed_step(), 0, "instr", set(), "some error")
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
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        planner = RecoveryPlanner(RecoveryConfig(max_retries=1))
        # Use up the retry for step 0
        d1 = planner.on_step_failed(self._failed_step(), 0, "instr", set(), "err")
        assert d1.strategy == "retry"
        # Reset
        planner.reset()
        # Now step 0 should get a fresh retry
        d2 = planner.on_step_failed(self._failed_step(), 0, "instr", set(), "err")
        assert d2.strategy == "retry"

    # ── Multiple step failures ────────────────────────────────

    def test_multiple_step_failures(self):
        """Different steps can fail and recover independently."""
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        planner = RecoveryPlanner(RecoveryConfig(max_retries=1))
        # Step 0 fails → retry
        assert (
            planner.on_step_failed(self._failed_step("step_a"), 0, "instr", set(), "err").strategy
            == "retry"
        )
        # Step 0 fails again → escalate (no engine, no fallback → abort)
        assert (
            planner.on_step_failed(self._failed_step("step_a"), 0, "instr", set(), "err").strategy
            == "abort"
        )
        # Step 2 fails → retry (independent counter)
        assert (
            planner.on_step_failed(self._failed_step("step_c"), 2, "instr", set(), "err").strategy
            == "retry"
        )

    # ── Replan with partial state ─────────────────────────────

    def test_replan_with_partial_state(self):
        """After 2/4 steps complete, replan only plans remaining work."""
        from ros2_lingua_core import RecoveryConfig, RecoveryPlanner

        registry = self._make_registry()
        # Mock engine returns the single remaining step
        engine = self._make_engine(
            registry,
            [
                {
                    "capability_name": "pick_up_object",
                    "parameters": {"object_name": "cup"},
                    "rationale": "pick",
                },
            ],
        )
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


# ------------------------------------------------------------------
# Backend error handling tests (for coverage improvement)
# ------------------------------------------------------------------


class TestBackendErrorHandling:
    """Test SDK error classification and retry logic in backends."""

    def test_openai_missing_import_raises(self):
        """Test that missing openai package raises ImportError."""
        from unittest.mock import patch

        from ros2_lingua_core import OpenAIBackend

        with patch.dict("sys.modules", {"openai": None}), pytest.raises(
            ImportError, match="openai package is required"
        ):
            OpenAIBackend(api_key="test")

    def test_anthropic_missing_import_raises(self):
        """Test that missing anthropic package raises ImportError."""
        from unittest.mock import patch

        from ros2_lingua_core import AnthropicBackend

        with patch.dict("sys.modules", {"anthropic": None}), pytest.raises(
            ImportError, match="anthropic package is required"
        ):
            AnthropicBackend(api_key="test")

    def test_ollama_missing_import_raises(self):
        """Test that missing ollama package raises ImportError."""
        from unittest.mock import patch

        from ros2_lingua_core import OllamaBackend

        with patch.dict("sys.modules", {"ollama": None}), pytest.raises(
            ImportError, match="ollama package is required"
        ):
            OllamaBackend()

    def test_mock_backend_fixed_response(self):
        """Test MockBackend returns fixed response."""
        from ros2_lingua_core import MockBackend

        response = '{"feasible": true, "steps": []}'
        backend = MockBackend(response)
        result = backend.complete([])
        assert result == response

    def test_mock_backend_infeasible_keyword_detection(self):
        """Test MockBackend detects infeasibility keywords."""
        import json

        from ros2_lingua_core import MockBackend

        backend = MockBackend()
        result = backend.complete([{"role": "user", "content": "this is impossible"}])
        parsed = json.loads(result)
        assert not parsed["feasible"]

    def test_mock_backend_failing_before_success(self):
        """Test MockBackend.failing() raises then succeeds."""
        from ros2_lingua_core import LLMTimeoutError, MockBackend

        backend = MockBackend.failing(
            LLMTimeoutError("timeout"), retries_before_success=1, success_response="OK"
        )
        # First call fails
        with pytest.raises(LLMTimeoutError):
            backend.complete([])
        # Second call succeeds
        result = backend.complete([])
        assert result == "OK"

    def test_retry_config_aggressive(self):
        """Test RetryConfig.aggressive() creates appropriate config."""
        from ros2_lingua_core import RetryConfig

        r = RetryConfig.aggressive()
        assert r.max_retries == 5
        assert r.base_delay_sec == 2.0

    def test_with_retry_no_retries_when_disabled(self):
        """Test _with_retry when max_retries=0."""

        from ros2_lingua_core import LLMTimeoutError
        from ros2_lingua_core.backends import RetryConfig, _with_retry

        def failing_fn():
            raise LLMTimeoutError("timeout")

        retry_config = RetryConfig.no_retry()
        with pytest.raises(LLMTimeoutError):
            _with_retry(failing_fn, retry_config, "test")

    def test_with_retry_respects_retry_on_timeout_flag(self):
        """Test _with_retry skips retry when retry_on_timeout=False."""

        from ros2_lingua_core import LLMTimeoutError
        from ros2_lingua_core.backends import RetryConfig, _with_retry

        call_count = 0

        def failing_fn():
            nonlocal call_count
            call_count += 1
            raise LLMTimeoutError("timeout")

        retry_config = RetryConfig(max_retries=5, retry_on_timeout=False)
        with pytest.raises(LLMTimeoutError):
            _with_retry(failing_fn, retry_config, "test")
        assert call_count == 1  # Should not retry

    def test_with_retry_respects_retry_on_rate_limit_flag(self):
        """Test _with_retry skips retry when retry_on_rate_limit=False."""
        from ros2_lingua_core import LLMRateLimitError
        from ros2_lingua_core.backends import RetryConfig, _with_retry

        call_count = 0

        def failing_fn():
            nonlocal call_count
            call_count += 1
            raise LLMRateLimitError("rate limit")

        retry_config = RetryConfig(max_retries=5, retry_on_rate_limit=False)
        with pytest.raises(LLMRateLimitError):
            _with_retry(failing_fn, retry_config, "test")
        assert call_count == 1  # Should not retry

    def test_with_retry_never_retries_auth_error(self):
        """Test _with_retry never retries LLMAuthError."""
        from ros2_lingua_core import LLMAuthError
        from ros2_lingua_core.backends import RetryConfig, _with_retry

        call_count = 0

        def failing_fn():
            nonlocal call_count
            call_count += 1
            raise LLMAuthError("auth failed")

        retry_config = RetryConfig(max_retries=5)
        with pytest.raises(LLMAuthError):
            _with_retry(failing_fn, retry_config, "test")
        assert call_count == 1  # Should not retry

    def test_with_retry_never_retries_model_not_found(self):
        """Test _with_retry never retries LLMModelNotFoundError."""
        from ros2_lingua_core import LLMModelNotFoundError
        from ros2_lingua_core.backends import RetryConfig, _with_retry

        call_count = 0

        def failing_fn():
            nonlocal call_count
            call_count += 1
            raise LLMModelNotFoundError("model not found")

        retry_config = RetryConfig(max_retries=5)
        with pytest.raises(LLMModelNotFoundError):
            _with_retry(failing_fn, retry_config, "test")
        assert call_count == 1  # Should not retry

    def test_with_retry_succeeds_after_transient_errors(self):
        """Test _with_retry succeeds after transient backend errors."""
        from ros2_lingua_core import LLMBackendError
        from ros2_lingua_core.backends import RetryConfig, _with_retry

        call_count = 0

        def failing_then_succeeding_fn():
            nonlocal call_count
            call_count += 1
            if call_count < 3:
                raise LLMBackendError("transient error")
            return "success"

        retry_config = RetryConfig(max_retries=5)
        result = _with_retry(failing_then_succeeding_fn, retry_config, "test")
        assert result == "success"
        assert call_count == 3  # Should retry twice then succeed
