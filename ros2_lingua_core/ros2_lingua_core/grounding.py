"""
ros2_lingua_core.grounding
---------------------------
The GroundingEngine is the brain of ros2_lingua.

Given:
  - A natural language instruction (e.g. "pick up the red cup")
  - A CapabilityRegistry (all registered capabilities + current state)

It returns:
  - An ActionPlan: an ordered list of ActionStep objects, each describing
    which capability to call and with what parameter values

The engine is LLM-backend agnostic — you plug in any backend that
implements the LLMBackend protocol (OpenAI, Anthropic, Ollama, etc.)
"""

import hashlib
import json
import logging
import time
from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Protocol

from .errors import (
    LLMBackendError,
    ParameterValidationError,
)
from .registry import CapabilityRegistry
from .validator import ParameterValidator

logger = logging.getLogger(__name__)


# ------------------------------------------------------------------
# LLM Backend Protocol
# ------------------------------------------------------------------


class LLMBackend(Protocol):
    """
    Any object that implements this protocol can be used as the LLM
    backend for the GroundingEngine.

    The only requirement is a complete() method that takes a list of
    messages (OpenAI-style format) and returns the assistant reply string.
    """

    def complete(self, messages: list[dict[str, str]]) -> str:
        """
        Args:
            messages: List of {"role": "system"|"user"|"assistant", "content": str}

        Returns:
            The model's reply as a plain string.
        """
        ...


# ------------------------------------------------------------------
# Action Plan structures
# ------------------------------------------------------------------


@dataclass
class ActionStep:
    """
    A single step in an execution plan.

    capability_name: Which capability to call
    parameters: The resolved parameter values for this call
    rationale: Why the LLM chose this step (useful for debugging/demo)
    """

    capability_name: str
    parameters: dict[str, Any]
    rationale: str = ""

    def to_dict(self) -> dict:
        return {
            "capability_name": self.capability_name,
            "parameters": self.parameters,
            "rationale": self.rationale,
        }


@dataclass
class ActionPlan:
    """
    The complete execution plan returned by the GroundingEngine.

    steps: Ordered list of ActionStep objects to execute
    original_instruction: The natural language instruction that was grounded
    feasible: False if the engine determined the plan cannot be executed
    reason: If not feasible, explains why
    """

    steps: list[ActionStep]
    original_instruction: str
    feasible: bool = True
    reason: str = ""

    def to_dict(self) -> dict:
        return {
            "original_instruction": self.original_instruction,
            "feasible": self.feasible,
            "reason": self.reason,
            "steps": [s.to_dict() for s in self.steps],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)


# ------------------------------------------------------------------
# Grounding Engine
# ------------------------------------------------------------------

SYSTEM_PROMPT_TEMPLATE = """You are the action grounding engine for a ROS 2 robot.

Your job is to translate a natural language instruction into a structured
execution plan using ONLY the capabilities listed below.

{capability_context}

Rules:
1. Only use capabilities from the list above.
2. For each step, provide all required parameters. Use reasonable defaults for optional ones.
3. If the instruction cannot be fulfilled with the available capabilities, set feasible to false.
4. If multiple steps are needed, order them correctly based on their dependencies.
5. Respond ONLY with a valid JSON object — no explanation, no markdown.

Response format:
{{
  "feasible": true,
  "reason": "",
  "steps": [
    {{
      "capability_name": "<name>",
      "parameters": {{ "<param>": "<value>" }},
      "rationale": "<brief reason for this step>"
    }}
  ]
}}
"""


class GroundingEngine:
    """
    Translates natural language instructions into executable ActionPlans.

    Usage:
        backend = OpenAIBackend(api_key="...")   # or any LLMBackend
        engine = GroundingEngine(registry, backend)
        plan = engine.ground("pick up the bottle from the table")
        for step in plan.steps:
            print(step.capability_name, step.parameters)
    """

    def __init__(
        self,
        registry: CapabilityRegistry,
        backend: LLMBackend,
        auto_chain: bool = True,
        max_retries: int = 3,
        retry_delay: float = 1.0,
        retry_backoff: float = 2.0,
        validate_params: bool = True,
        strict_params: bool = False,
        cache_ttl_sec: float = 0.0,
        cache_max_size: int = 128,
    ):
        """
        Args:
            registry:        The CapabilityRegistry with all registered capabilities
            backend:         Any object implementing the LLMBackend protocol
            auto_chain:      Insert missing prerequisites automatically. Default True.
            max_retries:     Maximum attempts to obtain valid JSON. Default 3.
            retry_delay:     Initial delay between malformed-response attempts. Default 1.0.
            retry_backoff:   Multiply retry_delay by this each attempt. Default 2.0.
            validate_params: Validate and coerce LLM parameter values against the
                             capability schema before returning the plan. Default True.
                             Catches type errors (e.g. speed="fast") before dispatch.
            strict_params:   If True, reject unknown parameters in LLM responses.
                             If False (default), pass unknown params through unchanged.
            cache_ttl_sec:   How long to cache successful plans after completion, in seconds.
                             Cache hits do not extend the TTL. Nonpositive values disable
                             caching. Unserializable definitions and plans that cannot
                             be deep-copied bypass the cache.
                             Default 0.0 (disabled).
            cache_max_size: Maximum number of cached plans, evicting the least recently
                             used entry when full. Nonpositive values disable caching.
                             Default 128.
        """
        self._registry = registry
        self._backend = backend
        self._auto_chain = auto_chain
        self._max_retries = max_retries
        self._retry_delay = retry_delay
        self._retry_backoff = retry_backoff
        self._validate_params = validate_params
        self._strict_params = strict_params
        self._cache_ttl_sec = cache_ttl_sec
        self._cache_max_size = cache_max_size
        self._validator = ParameterValidator() if validate_params else None

        # Cache: (instruction_hash, registry_hash) -> (ActionPlan, timestamp)
        self._plan_cache: OrderedDict[tuple[str, str], tuple[ActionPlan, float]] = OrderedDict()

    def ground(self, instruction: str, tag_filter: list[str] | None = None) -> ActionPlan:
        """
        Ground a natural language instruction into an ActionPlan.

        Backend RetryConfig controls provider retries. The engine retries only
        malformed JSON responses.

        Args:
            instruction: Natural language command, e.g. "go to the kitchen"
            tag_filter: Optional list of tags to filter the capability context.

        Returns:
            ActionPlan with ordered steps ready for dispatch. Backend failures
            and exhausted malformed-JSON retries return an infeasible plan.
        """
        if not self._registry.get_all():
            return ActionPlan(
                steps=[],
                original_instruction=instruction,
                feasible=False,
                reason="No capabilities are registered. Nodes must register before grounding.",
            )

        # Check cache if both TTL and capacity are enabled.
        cache_key = None
        if self._cache_ttl_sec > 0 and self._cache_max_size > 0:
            instruction_hash = hashlib.md5(instruction.encode()).hexdigest()
            # Include all definitions: even filtered-out capabilities can be prerequisites.
            # Preserve registration order, which determines the first prerequisite producer.
            registry_state = {
                "caps": [cap.to_dict() for cap in self._registry.get_all()],
                "state": sorted(self._registry.get_state()),
                "tag_filter": tag_filter,
            }
            try:
                registry_json = json.dumps(registry_state, sort_keys=True)
            except (TypeError, ValueError, RecursionError):
                # Metadata and parameter defaults may contain arbitrary Python values.
                # An optional cache must not prevent otherwise valid grounding.
                logger.debug("Skipping plan cache: registry context cannot be serialized.")
            else:
                registry_hash = hashlib.md5(registry_json.encode()).hexdigest()
                cache_key = (instruction_hash, registry_hash)

            if cache_key is not None and cache_key in self._plan_cache:
                cached_plan, timestamp = self._plan_cache[cache_key]
                if time.monotonic() - timestamp < self._cache_ttl_sec:
                    try:
                        plan = deepcopy(cached_plan)
                    except Exception:
                        logger.debug("Discarding cached plan: it cannot be copied.")
                        del self._plan_cache[cache_key]
                    else:
                        self._plan_cache.move_to_end(cache_key)
                        logger.info(f"Cache hit for instruction: '{instruction}'")
                        return plan
                else:
                    # Expired
                    del self._plan_cache[cache_key]

        capability_context = self._registry.to_llm_context(tags=tag_filter)
        system_prompt = SYSTEM_PROMPT_TEMPLATE.format(capability_context=capability_context)
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": instruction},
        ]

        last_parse_error: str | None = None
        delay = self._retry_delay

        for attempt in range(1, self._max_retries + 1):
            try:
                raw_response = self._backend.complete(messages)
            except LLMBackendError as e:
                logger.error(f"LLM backend failed: {e}")
                return ActionPlan(
                    steps=[],
                    original_instruction=instruction,
                    feasible=False,
                    reason=f"LLM backend error: {e}",
                )
            except Exception as e:
                logger.error(f"Unexpected grounding error: {e}")
                return ActionPlan(
                    steps=[],
                    original_instruction=instruction,
                    feasible=False,
                    reason=f"Unexpected error: {e}",
                )

            try:
                plan = self._parse_response(raw_response, instruction)
            except json.JSONDecodeError as e:
                last_parse_error = (
                    f"LLM returned invalid JSON: {e}. Raw response: {raw_response[:200]}"
                )
                if attempt < self._max_retries:
                    logger.warning(
                        f"LLM returned invalid JSON on attempt {attempt}/{self._max_retries}. "
                        f"Retrying in {delay:.1f}s..."
                    )
                    time.sleep(delay)
                    delay *= self._retry_backoff
                    continue
                break

            except Exception as e:
                logger.error(f"Unexpected grounding error: {e}")
                return ActionPlan(
                    steps=[],
                    original_instruction=instruction,
                    feasible=False,
                    reason=f"Unexpected error: {e}",
                )

            try:
                if plan.feasible and self._auto_chain:
                    plan = self._apply_auto_chaining(plan, instruction)

                if cache_key is not None and plan.feasible:
                    try:
                        cached_plan = deepcopy(plan)
                    except Exception:
                        logger.debug("Skipping plan cache: plan cannot be copied.")
                    else:
                        self._plan_cache[cache_key] = (cached_plan, time.monotonic())
                        self._plan_cache.move_to_end(cache_key)
                        if len(self._plan_cache) > self._cache_max_size:
                            self._plan_cache.popitem(last=False)

                return plan
            except Exception as e:
                logger.error(f"Unexpected grounding error: {e}")
                return ActionPlan(
                    steps=[],
                    original_instruction=instruction,
                    feasible=False,
                    reason=f"Unexpected error: {e}",
                )

        return ActionPlan(
            steps=[],
            original_instruction=instruction,
            feasible=False,
            reason=last_parse_error or "No response attempts were configured.",
        )

    def _parse_response(self, raw: str, instruction: str) -> ActionPlan:
        """Parse the LLM JSON response into an ActionPlan."""
        # Strip any accidental markdown fences, including language tags like ```json
        cleaned = raw.strip()
        if cleaned.startswith("```"):
            # Drop the opening fence line entirely (handles ``` and ```json etc.)
            lines = cleaned.split("\n")
            cleaned = "\n".join(lines[1:])
        if cleaned.endswith("```"):
            lines = cleaned.split("\n")
            cleaned = "\n".join(lines[:-1])
        cleaned = cleaned.strip()

        data = json.loads(cleaned)

        def invalid_response(message: str) -> ActionPlan:
            return ActionPlan(
                steps=[],
                original_instruction=instruction,
                feasible=False,
                reason=f"LLM returned invalid response: {message}",
            )

        if not isinstance(data, dict):
            return invalid_response("the top-level JSON value must be an object")

        feasible = data.get("feasible", True)
        if type(feasible) is not bool:
            return invalid_response("'feasible' must be a boolean")

        reason = data.get("reason", "")
        response_steps = data.get("steps", [])
        if not isinstance(response_steps, list):
            return invalid_response("'steps' must be a list")

        steps = []

        for index, step_data in enumerate(response_steps):
            if not isinstance(step_data, dict):
                return invalid_response(f"steps[{index}] must be an object")
            cap_name = step_data.get("capability_name", "")
            parameters = step_data.get("parameters", {})
            if not isinstance(parameters, dict):
                return invalid_response(f"steps[{index}].parameters must be an object")
            # Validate that the capability actually exists
            if self._registry.get(cap_name) is None:
                return ActionPlan(
                    steps=[],
                    original_instruction=instruction,
                    feasible=False,
                    reason=(
                        f"LLM referenced unknown capability '{cap_name}'. "
                        "This is a hallucination — the capability is not registered."
                    ),
                )
            steps.append(
                ActionStep(
                    capability_name=cap_name,
                    parameters=parameters,
                    rationale=step_data.get("rationale", ""),
                )
            )

        # Parameter validation — coerce types and check required fields
        if self._validator and steps and feasible:
            validated_steps = []
            validation_failures = []

            for step in steps:
                cap = self._registry.get(step.capability_name)
                if cap is None or not cap.parameters:
                    validated_steps.append(step)
                    continue
                try:
                    coerced = self._validator.validate(
                        cap, step.parameters, strict=self._strict_params
                    )
                    validated_steps.append(
                        ActionStep(
                            capability_name=step.capability_name,
                            parameters=coerced,
                            rationale=step.rationale,
                        )
                    )
                except ParameterValidationError as e:
                    validation_failures.append(str(e))

            if validation_failures:
                return ActionPlan(
                    steps=[],
                    original_instruction=instruction,
                    feasible=False,
                    reason=("Parameter validation failed:\n" + "\n".join(validation_failures)),
                )
            steps = validated_steps

        return ActionPlan(
            steps=steps,
            original_instruction=instruction,
            feasible=feasible,
            reason=reason,
        )

    def _apply_auto_chaining(self, plan: ActionPlan, instruction: str) -> ActionPlan:
        """
        For each step in the plan, check if its preconditions are met.
        If not, use the registry's backward chainer to prepend the
        necessary prerequisite steps.
        """
        current_state = self._registry.get_state()
        final_steps: list[ActionStep] = []

        for step in plan.steps:
            cap = self._registry.get(step.capability_name)
            if cap is None:
                continue

            # Check if preconditions need satisfying
            unsatisfied = [p for p in cap.preconditions if p not in current_state]
            if unsatisfied:
                try:
                    chain = self._registry.resolve_chain(
                        step.capability_name, current_state=set(current_state)
                    )
                    # Prepend all prerequisite steps (exclude the goal itself,
                    # which is already in our plan as 'step')
                    for prerequisite_cap in chain[:-1]:
                        already_planned = any(
                            s.capability_name == prerequisite_cap.name for s in final_steps
                        )
                        if not already_planned:
                            final_steps.append(
                                ActionStep(
                                    capability_name=prerequisite_cap.name,
                                    parameters={},  # will use defaults
                                    rationale=f"Auto-inserted prerequisite for '{step.capability_name}'",
                                )
                            )
                except (ValueError, RecursionError) as e:
                    return ActionPlan(
                        steps=[],
                        original_instruction=instruction,
                        feasible=False,
                        reason=str(e),
                    )

            final_steps.append(step)

            # Update simulated state with postconditions
            for token in cap.postconditions:
                current_state.add(token)

        plan.steps = final_steps
        return plan
