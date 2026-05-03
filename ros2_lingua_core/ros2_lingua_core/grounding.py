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

import json
import logging
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Protocol

from .registry import CapabilityRegistry
from .schema import Capability
from .errors import (
    LLMBackendError,
    LLMTimeoutError,
    HallucinationError,
    GroundingError,
)

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

    def complete(self, messages: List[Dict[str, str]]) -> str:
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
    parameters: Dict[str, Any]
    rationale: str = ""

    def to_dict(self) -> Dict:
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
    steps: List[ActionStep]
    original_instruction: str
    feasible: bool = True
    reason: str = ""

    def to_dict(self) -> Dict:
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
    ):
        """
        Args:
            registry: The CapabilityRegistry with all registered capabilities
            backend: Any object implementing the LLMBackend protocol
            auto_chain: If True, automatically resolve and prepend prerequisite
                        capabilities before the LLM-chosen ones. Default True.
            max_retries: Maximum number of LLM call attempts before giving up.
                         Default 3. Set to 1 to disable retries.
            retry_delay: Initial delay in seconds between retry attempts.
                         Default 1.0s.
            retry_backoff: Multiplier applied to retry_delay after each attempt.
                           Default 2.0 (exponential backoff).
                           e.g. delays will be: 1s, 2s, 4s
        """
        self._registry = registry
        self._backend = backend
        self._auto_chain = auto_chain
        self._max_retries = max_retries
        self._retry_delay = retry_delay
        self._retry_backoff = retry_backoff

    def ground(self, instruction: str, tag_filter: Optional[List[str]] = None) -> ActionPlan:
        """
        Ground a natural language instruction into an ActionPlan.

        Automatically retries on transient backend failures (connection issues,
        timeouts, rate limits) with exponential backoff. Non-retryable errors
        (bad API key, empty registry, hallucinated capabilities) fail immediately.

        Args:
            instruction: Natural language command, e.g. "go to the kitchen"
            tag_filter: Optional list of tags to filter the capability context.

        Returns:
            ActionPlan with ordered steps ready for dispatch

        Raises:
            EmptyRegistryError: If no capabilities are registered
            BackendAuthError: If API key is invalid (not retried)
            MaxRetriesExceededError: If all retry attempts fail
        """
        if not self._registry.get_all():
            return ActionPlan(
                steps=[],
                original_instruction=instruction,
                feasible=False,
                reason="No capabilities are registered. Nodes must register before grounding.",
            )

        capability_context = self._registry.to_llm_context(tags=tag_filter)
        system_prompt = SYSTEM_PROMPT_TEMPLATE.format(
            capability_context=capability_context
        )
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": instruction},
        ]

        last_error = None
        delay = self._retry_delay

        for attempt in range(1, self._max_retries + 1):
            try:
                raw_response = self._backend.complete(messages)
                plan = self._parse_response(raw_response, instruction)

                if plan.feasible and self._auto_chain:
                    plan = self._apply_auto_chaining(plan, instruction)

                return plan

            except LLMTimeoutError as e:
                last_error = e
                if attempt < self._max_retries:
                    logger.warning(
                        f"LLM timeout on attempt {attempt}/{self._max_retries}. "
                        f"Retrying in {delay:.1f}s..."
                    )
                    import time; time.sleep(delay)
                    delay *= self._retry_backoff

            except LLMBackendError as e:
                last_error = e
                if attempt < self._max_retries:
                    logger.warning(
                        f"LLM backend error on attempt {attempt}/{self._max_retries}: {e}. "
                        f"Retrying in {delay:.1f}s..."
                    )
                    import time; time.sleep(delay)
                    delay *= self._retry_backoff
                else:
                    logger.error(f"LLM backend failed after {self._max_retries} attempts: {e}")

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
            reason=(
                f"Grounding failed after {self._max_retries} attempts. "
                f"Last error: {last_error}"
            ),
        )

    def _parse_response(self, raw: str, instruction: str) -> ActionPlan:
        """Parse the LLM JSON response into an ActionPlan."""
        # Strip any accidental markdown fences
        cleaned = raw.strip()
        if cleaned.startswith("```"):
            cleaned = "\n".join(cleaned.split("\n")[1:])
        if cleaned.endswith("```"):
            cleaned = "\n".join(cleaned.split("\n")[:-1])

        try:
            data = json.loads(cleaned)
        except json.JSONDecodeError as e:
            return ActionPlan(
                steps=[],
                original_instruction=instruction,
                feasible=False,
                reason=f"LLM returned invalid JSON: {e}. Raw response: {raw[:200]}",
            )

        feasible = data.get("feasible", True)
        reason = data.get("reason", "")
        steps = []

        for step_data in data.get("steps", []):
            cap_name = step_data.get("capability_name", "")
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
                    parameters=step_data.get("parameters", {}),
                    rationale=step_data.get("rationale", ""),
                )
            )

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
        final_steps: List[ActionStep] = []

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
                            s.capability_name == prerequisite_cap.name
                            for s in final_steps
                        )
                        if not already_planned:
                            final_steps.append(
                                ActionStep(
                                    capability_name=prerequisite_cap.name,
                                    parameters={},   # will use defaults
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
