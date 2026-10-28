"""Capability vocabulary, gating and per-backend expectations shared by the
conformance launch files.

The names mirror ``contract.hpp``; ADAPT-1020 fails if they drift. Which
backend a launch file starts, and what it is expected to lack, is the
``BACKENDS`` row that ``RACING_SIM_BACKEND`` names (default the gym).
"""

import os
import sys
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from pathlib import Path

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


def declared_capabilities(node, backend="racing_sim", timeout=20.0):
    """Read the backend's five capability parameters through ROS.

    Returns ``{name: value}``; a parameter the backend did not declare
    reads as ``None``.
    """
    import rclpy
    from rcl_interfaces.srv import GetParameters

    # A bare service client, not AsyncParameterClient: Jazzy's has no destroy,
    # and one left behind per call keeps its six clients on the node.
    client = node.create_client(GetParameters, f"{backend}/get_parameters")
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"{backend} parameter services unavailable")
        request = GetParameters.Request()
        request.names = list(CAPABILITY_PARAMETERS)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f"{backend} did not answer get_parameters")
    finally:
        node.destroy_client(client)
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
    "ADAPT-2160": {STEP_MODE_PARAMETER: True, CLOCK_PARAMETER: OWNS_CLOCK},
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
REPO = Path(__file__).resolve().parents[4]
GYM = "f1tenth_gym_jax"
AUTODRIVE = "autodrive"


@dataclass(frozen=True)
class Backend:
    """One backend's launch and what the conformance tests expect of it.

    ``actions`` builds the launch actions for a launch file from the gym
    parameter list that file would pass (a path or a dict); a backend takes
    from it what it has (the autodrive row takes only ``capability.*``).
    ``odom_max_median_period_s`` is None for the gym, whose ADAPT-2030 keeps
    its literal 100 Hz +- 20 % check as its own code path.
    """

    name: str
    actions: Callable[[Sequence], list]
    expected_not_applicable: frozenset
    odom_max_median_period_s: float | None
    expected_h: Callable[[], float]
    expected_clock: str
    clock_override: str


def _gym_actions(parameters=None):
    import launch_ros.actions

    arguments = {}
    if parameters:
        arguments["parameters"] = list(parameters)
    return [
        launch_ros.actions.Node(
            package="racing_sim_gym_jax",
            executable="racing_sim_gym_jax_node",
            output="screen",
            **arguments,
        )
    ]


def _gym_h():
    from f1tenth_gym_jax.envs.utils import Param

    return Param().h


# Part 4's files, not copies (plan Step 0 / D9). h and the steering limit are
# not single keys there (see racing_sim_autodrive/test/_server_launch.py);
# kp/ki are UNTUNED.
AUTODRIVE_H = 0.07354
AUTODRIVE_REPLAY_START_FRAME = 90


def _autodrive_vehicle() -> dict:
    import yaml

    directory = REPO / "config" / "vehicles" / "autodrive_roboracer"
    source = yaml.safe_load((directory / "source.yaml").read_text())["values"]
    identified = yaml.safe_load((directory / "identified.yaml").read_text())
    return {
        "lr": float(source["com_x_lr_m"]["value"]),
        "h": AUTODRIVE_H,
        "wheel_radius": float(source["wheel_radius_m"]["value"]),
        "throttle_gain": float(
            identified["measurements"]["throttle_gain"]["value"]
        ),
        "max_steering_angle": 0.523633,
        "kp": 0.02,
        "ki": 0.05,
    }


def _autodrive_actions(parameters=None):
    """The server on an ephemeral port (never the sim's 4567), then the
    replay as the sim once the server says it is listening."""
    import socket

    import launch
    import launch.actions
    import launch.event_handlers
    import launch_ros.actions

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    capabilities = {}
    for entry in parameters or ():
        if isinstance(entry, dict):
            capabilities.update(
                {k: v for k, v in entry.items() if k.startswith("capability.")}
            )
    server = launch_ros.actions.Node(
        package="racing_sim_autodrive",
        executable="racing_sim_autodrive_server",
        name="racing_sim",
        parameters=[
            {
                **_autodrive_vehicle(),
                "world_to_map.x": 0.0,
                "world_to_map.y": 0.0,
                "world_to_map.yaw": 0.0,
                "track_path": str(REPO / "config" / "tracks" / "porto.yaml"),
                "scenario_id": "porto",
                "port": port,
                "host": "127.0.0.1",
                **capabilities,
            }
        ],
        output="screen",
    )
    # --start-frame is inside the recorded driving segment (frames ~90-355),
    # so the 2 s ADAPT-2110 window cannot reach the loop wrap or the reset
    # segment (frames ~470-480) that re-anchors dead reckoning.
    replay = launch.actions.ExecuteProcess(
        cmd=[
            sys.executable,
            "-m",
            "racing_sim_autodrive.replay",
            "--fixture",
            str(
                REPO
                / "ros_ws/src/racing_sim_autodrive/test/fixtures"
                / "porto_stream.jsonl.gz"
            ),
            "--url",
            f"http://127.0.0.1:{port}",
            "--pace",
            "recorded",
            "--loop",
            "--start-frame",
            str(AUTODRIVE_REPLAY_START_FRAME),
        ],
        output="screen",
    )

    def start_replay(event):
        text = event.text.decode(errors="replace")
        if "listening on" in text:
            return [replay]
        return None

    return [
        server,
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessIO(
                target_action=server,
                on_stdout=start_replay,
                on_stderr=start_replay,
            )
        ),
    ]


BACKENDS = {
    GYM: Backend(
        name=GYM,
        actions=_gym_actions,
        # The gym has every capability.
        expected_not_applicable=frozenset(),
        odom_max_median_period_s=None,
        expected_h=_gym_h,
        expected_clock=OWNS_CLOCK,
        clock_override=FREE_RUNNING,
    ),
    AUTODRIVE: Backend(
        name=AUTODRIVE,
        actions=_autodrive_actions,
        expected_not_applicable=frozenset(
            {
                "ADAPT-2050",
                "ADAPT-2070",
                "ADAPT-2075",
                "ADAPT-2080",
                "ADAPT-2085",
                "ADAPT-2090",
                "ADAPT-2120",
                "ADAPT-2160",
            }
        ),
        # >= 20 Hz, the spec's nominal. One-sided, and on replay it measures
        # the replay's pacing (one publish per frame).
        odom_max_median_period_s=0.05,
        expected_h=lambda: AUTODRIVE_H,
        expected_clock=FREE_RUNNING,
        # Not the declared value, or ADAPT-2145 would be vacuous.
        clock_override=OWNS_CLOCK,
    ),
}


def backend() -> Backend:
    """The row ``RACING_SIM_BACKEND`` names; an unknown name raises."""
    name = os.environ.get("RACING_SIM_BACKEND", GYM)
    if name not in BACKENDS:
        raise ValueError(
            f"RACING_SIM_BACKEND={name!r} is not one of {sorted(BACKENDS)}"
        )
    return BACKENDS[name]


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
    the test unless the id is in the backend row's expected_not_applicable: an
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
    testcase.assertIn(test_id, backend().expected_not_applicable, reason)
    return False
