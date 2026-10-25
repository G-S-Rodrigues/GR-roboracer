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


# Test id -> {capability parameter: the value the test needs}. A test whose
# need the backend does not declare is not applicable (named, never skipped).
REQUIREMENTS = {
    "ADAPT-2050": {RESET_PARAMETER: SEEDED_POSE},
    "ADAPT-2070": {CLOCK_PARAMETER: OWNS_CLOCK},
    "ADAPT-2075": {CLOCK_PARAMETER: OWNS_CLOCK},
    "ADAPT-2085": {CLOCK_PARAMETER: OWNS_CLOCK},
    "ADAPT-2080": {START_HELD_PARAMETER: True, CLOCK_PARAMETER: OWNS_CLOCK},
    "ADAPT-2090": {TIME_SCALE_PARAMETER: True, CLOCK_PARAMETER: OWNS_CLOCK},
    "ADAPT-2120": {START_HELD_PARAMETER: True},
}
# Every other conformance test applies to every backend.
UNGATED = frozenset(
    {
        "ADAPT-2010",
        "ADAPT-2020",
        "ADAPT-2030",
        "ADAPT-2040",
        "ADAPT-2100",
        "ADAPT-2110",
        "ADAPT-2130",
        "ADAPT-2140",
        "ADAPT-2145",
        "ADAPT-2150",
    }
)
# What the backend the four launch files start is expected to lack. They
# launch the gym, which has every capability.
BACKEND_EXPECTED_NOT_APPLICABLE = frozenset()


def not_applicable(requirements, declared):
    """Test id -> reason, for every test the declaration cannot satisfy."""
    missing = {}
    for test_id, needs in requirements.items():
        lacking = [
            f"{name}={want} (backend declares {declared.get(name)})"
            for name, want in needs.items()
            if declared.get(name) != want
        ]
        if lacking:
            missing[test_id] = f"{test_id} needs " + ", ".join(lacking)
    return missing


def applicable(testcase, node, test_id):
    """True when the running backend can run ``test_id``.

    Otherwise prints ``NOT APPLICABLE <reason>`` and returns False, and fails
    the test unless the id is in BACKEND_EXPECTED_NOT_APPLICABLE: an
    unexpected not-applicable must never read as a pass.
    """
    cls = type(testcase)
    if getattr(cls, "_declared", None) is None:
        cls._declared = declared_capabilities(node)
    reason = not_applicable(
        {test_id: REQUIREMENTS[test_id]}, cls._declared
    ).get(test_id)
    if reason is None:
        return True
    print(f"NOT APPLICABLE {reason}")
    testcase.assertIn(test_id, BACKEND_EXPECTED_NOT_APPLICABLE, reason)
    return False
