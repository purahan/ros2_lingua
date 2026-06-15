"""
ros2_lingua_core.recovery
--------------------------
Error recovery planner for step failures during plan execution.

When a dispatcher step fails, the RecoveryPlanner decides what to do
next by walking through a prioritized strategy cascade:

    1. Retry    — re-execute the same step (up to max_retries)
    2. Replan   — call the grounding engine with updated state so the
                  backward chainer skips completed work
    3. Fallback — execute a safe fallback capability (e.g. return_to_home)
    4. Abort    — give up

Design:
- Pure Python, zero ROS 2 dependencies
- Independently unit-testable
- Grounding engine and registry are optional injected dependencies
- Per-step retry tracking (each step gets its own counter)
"""

import logging
from dataclasses import dataclass, field
from typing import Any

logger = logging.getLogger(__name__)


# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------


@dataclass
class RecoveryConfig:
    """
    Controls how the recovery planner responds to step failures.

    Attributes:
        max_retries:              Max retries per step before escalating.
        enable_replan:            Whether to attempt replanning from current state.
        safe_fallback:            Capability name to execute before aborting.
                                  e.g. "return_to_home". None disables fallback.
        safe_fallback_params:     Parameters for the fallback capability.
        abort_on_fallback_failure: If True, abort even if fallback itself fails.

    Example:
        RecoveryConfig(
            max_retries=2,
            enable_replan=True,
            safe_fallback="return_to_home",
        )
    """

    max_retries: int = 2
    enable_replan: bool = True
    safe_fallback: str | None = None
    safe_fallback_params: dict[str, Any] = field(default_factory=dict)
    abort_on_fallback_failure: bool = True


# ------------------------------------------------------------------
# Decision output
# ------------------------------------------------------------------


@dataclass
class RecoveryDecision:
    """
    The recovery planner's recommendation for what to do after a step failure.

    Attributes:
        strategy:  One of "retry", "replan", "fallback", "abort"
        new_plan:  If strategy is "replan", contains the replanned ActionPlan.
                   None otherwise.
        reason:    Human-readable explanation of why this strategy was chosen.
    """

    strategy: str  # "retry" | "replan" | "fallback" | "abort"
    new_plan: Any = None  # Optional ActionPlan (avoiding circular import)
    reason: str = ""


# ------------------------------------------------------------------
# Recovery Planner
# ------------------------------------------------------------------


class RecoveryPlanner:
    """
    Decides what to do when a dispatcher step fails.

    Usage:
        config = RecoveryConfig(max_retries=2, enable_replan=True,
                                safe_fallback="return_to_home")
        planner = RecoveryPlanner(config, grounding_engine=engine, registry=registry)

        # When step 2 fails:
        decision = planner.on_step_failed(
            failed_step={"capability_name": "pick_up_object", "parameters": {...}},
            step_index=2,
            original_instruction="pick up the bottle",
            current_state={"robot_is_balanced", "robot_at_location"},
            error="Gripper timeout",
            tag_filter=None,
        )

        if decision.strategy == "retry":
            # re-execute the same step
        elif decision.strategy == "replan":
            # execute decision.new_plan
        elif decision.strategy == "fallback":
            # execute the fallback capability, then abort
        elif decision.strategy == "abort":
            # give up
    """

    def __init__(
        self,
        config: RecoveryConfig = None,
        grounding_engine=None,
        registry=None,
    ):
        """
        Args:
            config:           RecoveryConfig controlling strategy selection.
            grounding_engine: Optional GroundingEngine for replanning.
                              If not provided, replan is skipped even if enabled.
            registry:         Optional CapabilityRegistry for state inspection.
                              Used to verify fallback capability exists.
        """
        self._config = config or RecoveryConfig()
        self._engine = grounding_engine
        self._registry = registry

        # Track retry counts per step index
        # Key: step_index, Value: number of retries attempted
        self._retry_counts: dict[int, int] = {}

    @property
    def config(self) -> RecoveryConfig:
        """The active recovery configuration."""
        return self._config

    def reset(self) -> None:
        """Reset retry counters. Call this at the start of a new plan execution."""
        self._retry_counts.clear()

    def on_step_failed(
        self,
        failed_step: dict[str, Any],
        step_index: int,
        original_instruction: str,
        current_state: set[str],
        error: str = "",
        tag_filter: list[str] | None = None,
    ) -> RecoveryDecision:
        """
        Decide what to do after a step failure.

        Args:
            failed_step:          The step dict that failed (capability_name, parameters).
            step_index:           Index of the failed step in the plan.
            original_instruction: The natural language instruction being executed.
            current_state:        Current symbolic state of the robot.
            error:                Error message from the failure.
            tag_filter:           Tag filter used in the original grounding call.

        Returns:
            A RecoveryDecision with the recommended strategy.
        """
        cap_name = failed_step.get("capability_name", "unknown")
        strategies_tried = []

        # ── Strategy 1: Retry ─────────────────────────────────────
        retries_used = self._retry_counts.get(step_index, 0)
        if retries_used < self._config.max_retries:
            self._retry_counts[step_index] = retries_used + 1
            attempt = retries_used + 1
            logger.info(
                f"Recovery: retrying '{cap_name}' (attempt {attempt}/{self._config.max_retries})"
            )
            return RecoveryDecision(
                strategy="retry",
                reason=(
                    f"Retrying '{cap_name}' — attempt {attempt} of "
                    f"{self._config.max_retries}. Error was: {error}"
                ),
            )
        strategies_tried.append("retry")

        # ── Strategy 2: Replan ────────────────────────────────────
        if self._config.enable_replan and self._engine is not None:
            logger.info(
                f"Recovery: replanning from current state after "
                f"'{cap_name}' failed. State: {current_state}"
            )
            try:
                # Sync registry state so the backward chainer sees
                # which postconditions are already satisfied
                if self._registry is not None:
                    self._registry.update_state(current_state)
                new_plan = self._engine.ground(original_instruction, tag_filter=tag_filter)
                if new_plan.feasible and new_plan.steps:
                    return RecoveryDecision(
                        strategy="replan",
                        new_plan=new_plan,
                        reason=(
                            f"Replanned after '{cap_name}' failed. "
                            f"New plan has {len(new_plan.steps)} step(s). "
                            f"The backward chainer skipped already-completed work."
                        ),
                    )
                else:
                    logger.warning(f"Recovery: replan returned infeasible plan — {new_plan.reason}")
                    strategies_tried.append("replan")
            except Exception as e:
                logger.error(f"Recovery: replanning failed — {e}")
                strategies_tried.append("replan")
        elif self._config.enable_replan:
            # Replan enabled but no engine available
            logger.warning(
                "Recovery: replan enabled but no grounding engine provided. Skipping replan."
            )
            strategies_tried.append("replan (no engine)")
        else:
            strategies_tried.append("replan (disabled)")

        # ── Strategy 3: Safe fallback ─────────────────────────────
        if self._config.safe_fallback is not None:
            # Verify fallback capability exists if registry is available
            if self._registry is not None:
                fallback_cap = self._registry.get(self._config.safe_fallback)
                if fallback_cap is None:
                    logger.error(
                        f"Recovery: fallback capability "
                        f"'{self._config.safe_fallback}' not registered."
                    )
                    strategies_tried.append("fallback (not registered)")
                else:
                    logger.info(f"Recovery: executing fallback '{self._config.safe_fallback}'")
                    return RecoveryDecision(
                        strategy="fallback",
                        reason=(
                            f"All retries exhausted for '{cap_name}'. "
                            f"Executing safe fallback: "
                            f"'{self._config.safe_fallback}'."
                        ),
                    )
            else:
                # No registry — trust that the fallback exists
                logger.info(
                    f"Recovery: executing fallback "
                    f"'{self._config.safe_fallback}' (registry not available)"
                )
                return RecoveryDecision(
                    strategy="fallback",
                    reason=(
                        f"All retries exhausted for '{cap_name}'. "
                        f"Executing safe fallback: "
                        f"'{self._config.safe_fallback}'."
                    ),
                )
        else:
            strategies_tried.append("fallback (not configured)")

        # ── Strategy 4: Abort ─────────────────────────────────────
        logger.error(f"Recovery: all strategies exhausted for '{cap_name}'. Aborting.")
        return RecoveryDecision(
            strategy="abort",
            reason=(
                f"All recovery strategies exhausted for '{cap_name}'. "
                f"Tried: {', '.join(strategies_tried)}. "
                f"Last error: {error}"
            ),
        )
