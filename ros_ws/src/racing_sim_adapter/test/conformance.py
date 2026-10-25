"""Capability vocabulary and gating shared by the conformance launch files.

The names mirror ``contract.hpp``; ADAPT-1020 fails if they drift.
"""

CLOCK_PARAMETER = "capability.clock"
RESET_PARAMETER = "capability.reset"
STEP_MODE_PARAMETER = "capability.step_mode"
TIME_SCALE_PARAMETER = "capability.time_scale"
START_HELD_PARAMETER = "capability.start_held"

CAPABILITY_PARAMETERS = (
    CLOCK_PARAMETER,
    RESET_PARAMETER,
    STEP_MODE_PARAMETER,
    TIME_SCALE_PARAMETER,
    START_HELD_PARAMETER,
)
BOOLEAN_PARAMETERS = (
    STEP_MODE_PARAMETER,
    TIME_SCALE_PARAMETER,
    START_HELD_PARAMETER,
)

OWNS_CLOCK = "owns_clock"
FREE_RUNNING = "free_running"
CLOCK_VALUES = (OWNS_CLOCK, FREE_RUNNING)

SEEDED_POSE = "seeded_pose"
INITIAL_POSE_ONLY = "initial_pose_only"
RESET_VALUES = (SEEDED_POSE, INITIAL_POSE_ONLY)


def _value(parameter_value):
    """The Python value of an rcl_interfaces ParameterValue."""
    from rcl_interfaces.msg import ParameterType

    if parameter_value.type == ParameterType.PARAMETER_BOOL:
        return parameter_value.bool_value
    if parameter_value.type == ParameterType.PARAMETER_STRING:
        return parameter_value.string_value
    return None


def declared_capabilities(node, backend="racing_sim", timeout=10.0):
    """Read the backend's five capability parameters through ROS.

    Returns ``{name: value}``; a parameter the backend did not declare
    reads as ``None``.
    """
    import rclpy
    from rclpy.parameter_client import AsyncParameterClient

    client = AsyncParameterClient(node, backend)
    if not client.wait_for_services(timeout_sec=timeout):
        raise RuntimeError(f"{backend} parameter services unavailable")
    future = client.get_parameters(list(CAPABILITY_PARAMETERS))
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f"{backend} did not answer get_parameters")
    # An undeclared parameter makes the service answer with fewer values.
    values = list(future.result().values)
    values += [None] * (len(CAPABILITY_PARAMETERS) - len(values))
    return {
        name: None if value is None else _value(value)
        for name, value in zip(CAPABILITY_PARAMETERS, values, strict=True)
    }
