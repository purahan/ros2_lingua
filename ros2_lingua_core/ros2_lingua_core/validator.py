"""
ros2_lingua_core.validator
---------------------------
Validates and coerces parameter values supplied by the LLM against
a Capability's parameter schema before the plan reaches the dispatcher.

Design principles:
- Be lenient on types where safe (coerce "0.5" → 0.5 for float fields)
- Be strict on missing required parameters
- Collect ALL failures before reporting, not just the first one
- Never silently drop a value — either coerce it or fail clearly
- ROS message types (geometry_msgs/Pose etc.) pass through unchecked
  since we can't validate their structure without ROS

Type coercion rules:
  "string"          → str (always coercible)
  "float"/"double"  → float (coerces int, numeric str; rejects non-numeric)
  "int"/"integer"   → int (coerces float if whole number, numeric str)
  "bool"/"boolean"  → bool (coerces "true"/"false"/"1"/"0")
  "list"/"array"    → list (coerces JSON array string)
  anything else     → pass through unchanged (ROS message types etc.)
"""

from typing import Any, Dict, List, Optional, Tuple

from .schema import Capability, CapabilityParameter
from .errors import ParameterValidationError


# Types that ros2_lingua validates natively.
# Anything not in this set is treated as a ROS message type and passed through.
_NATIVE_TYPES = {"string", "str", "float", "double", "int", "integer", "bool", "boolean", "list", "array"}


class ParameterValidator:
    """
    Validates and coerces a parameter dict against a Capability's schema.

    Usage:
        validator = ParameterValidator()

        # Validate and get coerced values back
        coerced = validator.validate(capability, {"speed": "0.5", "location_name": "table"})

        # Raises ParameterValidationError if anything is wrong
    """

    def validate(
        self,
        capability: Capability,
        parameters: Dict[str, Any],
        strict: bool = False,
    ) -> Dict[str, Any]:
        """
        Validate and coerce parameters against the capability schema.

        Args:
            capability:  The Capability whose schema to validate against
            parameters:  The raw parameter dict from the LLM
            strict:      If True, reject unknown parameters.
                         If False (default), pass unknown params through.

        Returns:
            A new dict with coerced values and defaults filled in.

        Raises:
            ParameterValidationError: if any required param is missing,
                                      or any value cannot be coerced.
        """
        failures = []
        result = {}

        for param_def in capability.parameters:
            value = parameters.get(param_def.name)

            if value is None:
                if param_def.required:
                    failures.append(
                        f"'{param_def.name}': required parameter is missing"
                    )
                    continue
                else:
                    # Use default — skip validation
                    result[param_def.name] = param_def.default
                    continue

            coerced, error = self._coerce(value, param_def)
            if error:
                failures.append(f"'{param_def.name}': {error}")
            else:
                result[param_def.name] = coerced

        if strict:
            defined_names = {p.name for p in capability.parameters}
            for key in parameters:
                if key not in defined_names:
                    failures.append(
                        f"'{key}': unknown parameter "
                        f"(not defined in capability schema)"
                    )
        else:
            # Pass through any extra parameters not in the schema
            defined_names = {p.name for p in capability.parameters}
            for key, val in parameters.items():
                if key not in defined_names:
                    result[key] = val

        if failures:
            raise ParameterValidationError(capability.name, failures)

        return result

    def validate_plan_steps(
        self,
        steps: list,
        capability_lookup: dict,
        strict: bool = False,
    ) -> Tuple[list, list]:
        """
        Validate parameters for all steps in a plan.

        Args:
            steps:             List of ActionStep-like dicts with
                               capability_name and parameters fields
            capability_lookup: Dict mapping capability name → Capability
            strict:            Passed through to validate()

        Returns:
            (validated_steps, failures) where:
            - validated_steps: steps with coerced parameter values
            - failures: list of (step_index, capability_name, error_message)
              for any steps that failed validation
        """
        validated_steps = []
        all_failures = []

        for i, step in enumerate(steps):
            cap_name = step.get("capability_name", "")
            params   = step.get("parameters", {})

            cap = capability_lookup.get(cap_name)
            if cap is None:
                # Capability not found — leave step unchanged, let
                # hallucination detection handle it
                validated_steps.append(step)
                continue

            if not cap.parameters:
                # Capability takes no parameters — nothing to validate
                validated_steps.append(step)
                continue

            try:
                coerced_params = self.validate(cap, params, strict=strict)
                validated_step = dict(step)
                validated_step["parameters"] = coerced_params
                validated_steps.append(validated_step)
            except ParameterValidationError as e:
                all_failures.append((i, cap_name, str(e)))
                validated_steps.append(step)  # keep original on failure

        return validated_steps, all_failures

    # ------------------------------------------------------------------
    # Type coercion
    # ------------------------------------------------------------------

    def _coerce(
        self, value: Any, param_def: CapabilityParameter
    ) -> Tuple[Any, Optional[str]]:
        """
        Attempt to coerce value to the expected type.

        Returns (coerced_value, None) on success.
        Returns (None, error_message) on failure.
        """
        expected = param_def.type.lower().strip()

        # ROS message types — pass through unchecked
        if "/" in param_def.type or expected not in _NATIVE_TYPES:
            return value, None

        if expected == "string" or expected == "str":
            return self._to_string(value, param_def)

        if expected in ("float", "double"):
            return self._to_float(value, param_def)

        if expected in ("int", "integer"):
            return self._to_int(value, param_def)

        if expected in ("bool", "boolean"):
            return self._to_bool(value, param_def)

        if expected in ("list", "array"):
            return self._to_list(value, param_def)

        # Unknown primitive — pass through
        return value, None

    def _to_string(self, value, param_def):
        # Everything is coercible to string
        return str(value), None

    def _to_float(self, value, param_def):
        if isinstance(value, float):
            return value, None
        if isinstance(value, int):
            return float(value), None
        if isinstance(value, str):
            try:
                return float(value), None
            except ValueError:
                return None, (
                    f"expected float, got str ('{value}'). "
                    f"Examples of valid values: 0.5, 1.0, 2"
                )
        return None, (
            f"expected float, got {type(value).__name__} ('{value}')"
        )

    def _to_int(self, value, param_def):
        if isinstance(value, int) and not isinstance(value, bool):
            return value, None
        if isinstance(value, float):
            if value == int(value):
                return int(value), None
            return None, (
                f"expected int, got float ({value}) — "
                f"fractional values are not allowed"
            )
        if isinstance(value, str):
            try:
                f = float(value)
                if f == int(f):
                    return int(f), None
                return None, (
                    f"expected int, got str ('{value}') — "
                    f"fractional values are not allowed"
                )
            except ValueError:
                return None, (
                    f"expected int, got str ('{value}'). "
                    f"Examples of valid values: 1, 5, 10"
                )
        return None, (
            f"expected int, got {type(value).__name__} ('{value}')"
        )

    def _to_bool(self, value, param_def):
        if isinstance(value, bool):
            return value, None
        if isinstance(value, int):
            return bool(value), None
        if isinstance(value, str):
            lower = value.lower().strip()
            if lower in ("true", "1", "yes", "on"):
                return True, None
            if lower in ("false", "0", "no", "off"):
                return False, None
            return None, (
                f"expected bool, got str ('{value}'). "
                f"Valid values: true, false, 1, 0"
            )
        return None, (
            f"expected bool, got {type(value).__name__} ('{value}')"
        )

    def _to_list(self, value, param_def):
        if isinstance(value, list):
            return value, None
        if isinstance(value, str):
            import json
            try:
                parsed = json.loads(value)
                if isinstance(parsed, list):
                    return parsed, None
                return None, (
                    f"expected list, got str ('{value}') — "
                    f"JSON parsed to {type(parsed).__name__}, not list"
                )
            except json.JSONDecodeError:
                return None, (
                    f"expected list, got str ('{value}'). "
                    f"Must be a valid JSON array, e.g. [\"a\", \"b\"]"
                )
        return None, (
            f"expected list, got {type(value).__name__} ('{value}')"
        )


# Module-level singleton for convenience
_default_validator = ParameterValidator()


def validate_parameters(
    capability: Capability,
    parameters: Dict[str, Any],
    strict: bool = False,
) -> Dict[str, Any]:
    """
    Module-level convenience function. Equivalent to:
        ParameterValidator().validate(capability, parameters)
    """
    return _default_validator.validate(capability, parameters, strict=strict)
