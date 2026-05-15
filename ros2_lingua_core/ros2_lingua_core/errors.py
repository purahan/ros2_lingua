"""
ros2_lingua_core.errors
------------------------
Custom exception hierarchy for ros2_lingua.

Using specific exception types instead of bare Exception means:
- Callers can catch exactly what they care about
- Error messages are consistent and informative
- The grounding node can return meaningful error codes to clients

Hierarchy:
    LinguaError                     (base)
    ├── LLMBackendError             (LLM call failed)
    │   ├── LLMTimeoutError         (LLM took too long)
    │   ├── LLMRateLimitError       (API rate limit hit)
    │   └── LLMModelNotFoundError   (model doesn't exist)
    ├── GroundingError              (grounding engine logic failed)
    │   ├── HallucinationError      (LLM referenced unknown capability)
    │   ├── InfeasibleError         (instruction cannot be executed)
    │   └── ParameterValidationError (LLM returned bad parameter values)
    └── PlanningError               (backward chaining failed)
        ├── UnsatisfiablePreconditionError
        └── CircularDependencyError
"""


class LinguaError(Exception):
    """Base class for all ros2_lingua errors."""
    pass


# --- LLM Backend Errors ---

class LLMBackendError(LinguaError):
    """Raised when an LLM backend call fails for any reason."""
    def __init__(self, message: str, original: Exception = None):
        super().__init__(message)
        self.original = original

class LLMTimeoutError(LLMBackendError):
    """Raised when the LLM backend does not respond within the timeout."""
    pass

class LLMRateLimitError(LLMBackendError):
    """Raised when the LLM API rate limit is exceeded."""
    pass

class LLMModelNotFoundError(LLMBackendError):
    """Raised when the specified model does not exist on the backend."""
    pass


# --- Grounding Errors ---

class GroundingError(LinguaError):
    """Raised when the grounding engine cannot produce a valid plan."""
    pass

class HallucinationError(GroundingError):
    """Raised when the LLM references a capability that is not registered."""
    def __init__(self, capability_name: str):
        self.capability_name = capability_name
        super().__init__(
            f"LLM hallucinated capability '{capability_name}' — "
            "it is not registered in the CapabilityRegistry."
        )

class InfeasibleError(GroundingError):
    """Raised when the LLM determines the instruction cannot be executed."""
    def __init__(self, reason: str):
        self.reason = reason
        super().__init__(f"Instruction is not feasible: {reason}")


class ParameterValidationError(GroundingError):
    """
    Raised when one or more parameters in an LLM-generated step
    fail validation against the capability's schema.

    Contains a list of all validation failures so the caller gets
    the full picture in one error rather than failing one at a time.

    Example:
        ParameterValidationError(
            capability_name="navigate_to_location",
            failures=[
                "speed: expected float, got str ('fast')",
                "location_name: required parameter is missing",
            ]
        )
    """
    def __init__(self, capability_name: str, failures: list):
        self.capability_name = capability_name
        self.failures = failures
        failures_str = "\n  ".join(failures)
        super().__init__(
            f"Parameter validation failed for '{capability_name}':\n"
            f"  {failures_str}"
        )


# --- Planning Errors ---

class PlanningError(LinguaError):
    """Raised when the backward-chaining planner cannot build a valid plan."""
    pass

class UnsatisfiablePreconditionError(PlanningError):
    """Raised when a precondition cannot be satisfied by any registered capability."""
    def __init__(self, precondition: str, required_by: str):
        self.precondition = precondition
        self.required_by = required_by
        super().__init__(
            f"Cannot satisfy precondition '{precondition}' "
            f"required by '{required_by}'. "
            "No registered capability produces this state token."
        )

class CircularDependencyError(PlanningError):
    """Raised when capability dependencies form a cycle."""
    pass


# --- Dispatcher Errors ---

class DispatchError(LinguaError):
    """Raised when a capability cannot be dispatched to the robot."""
    pass

class StepTimeoutError(DispatchError):
    """Raised when a capability step does not complete within the timeout."""
    def __init__(self, capability_name: str, timeout_sec: float):
        self.capability_name = capability_name
        self.timeout_sec = timeout_sec
        super().__init__(
            f"Step '{capability_name}' did not complete within {timeout_sec}s."
        )

class StepFailedError(DispatchError):
    """Raised when a capability step returns a failure result."""
    def __init__(self, capability_name: str, reason: str = ""):
        self.capability_name = capability_name
        self.reason = reason
        super().__init__(
            f"Step '{capability_name}' failed" + (f": {reason}" if reason else ".")
        )
