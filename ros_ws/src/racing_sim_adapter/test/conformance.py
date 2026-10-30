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

# The frame /scan is stamped in: mirrors contract.hpp kLaserFrame (CHECK-2020
# guards it), and the AutoDRIVE devkit publishes its LaserScan in it too.
SCAN_FRAME = "lidar"

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


def declared_capabilities(node, backend=None, timeout=20.0):
    """Read the backend's five capability parameters through ROS.

    ``backend`` is the node's name; None means the running row's
    ``node_fqn()``.

    Returns ``{name: value}``; a parameter the backend did not declare
    reads as ``None``.
    """
    import rclpy
    from rcl_interfaces.srv import GetParameters

    if backend is None:
        backend = node_fqn()
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
        "ADAPT-2170",
    }
)
REPO = Path(__file__).resolve().parents[4]
ADAPTER_NAME = "racing_sim"
# The absolute names a backend creates; a row remaps them under its prefix
# (a namespace alone does not move "/x"). A second copy of
# racing_sim_autodrive/test/_server_launch.py ABSOLUTE_NAMES: ADAPT-2170 and
# SIMAD-2100 each fail by name on a name their copy misses.
CONTRACT_TOPICS = (
    "/scan",
    "/ground_truth/scan",
    "/imu",
    "/odom",
    "/ground_truth/odom",
    "/ground_truth/track_relative_state",
    "/tf",
    "/drive",
)
CLOCK_TOPIC = "/clock"
GYM_NAMESPACE = "/test_adapt_gym"
AUTODRIVE_NAMESPACE = "/test_adapt_autodrive"


def _prefixed(namespace: str, name: str) -> str:
    return name if namespace == "/" else f"{namespace}{name}"


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
    # The row's namespace ("/" = the root, the leak ADAPT-2170 forbids) and
    # the absolute names it remaps under it.
    namespace: str = "/"
    remapped: tuple[str, ...] = ()


def _gym_actions(parameters=None):
    import launch_ros.actions

    arguments = {}
    if parameters:
        arguments["parameters"] = list(parameters)
    # colcon runs other packages' tier-2 tests in parallel on one domain, so
    # the node's absolute names (and its /clock, which every use_sim_time
    # node on the domain would follow) must not land on the shared graph
    # (ADAPT-2170, gotcha #39). The prefix is test-only: production launches
    # keep the global names.
    return [
        launch_ros.actions.Node(
            package="racing_sim_gym_jax",
            executable="racing_sim_gym_jax_node",
            namespace=GYM_NAMESPACE,
            remappings=[
                (n, _prefixed(GYM_NAMESPACE, n))
                for n in CONTRACT_TOPICS + (CLOCK_TOPIC,)
            ],
            output="screen",
            **arguments,
        )
    ]


def _gym_h():
    from f1tenth_gym_jax.envs.utils import Param

    return Param().h


# One source with the server tests: racing_sim_autodrive/test/_vehicle_values.py
# (part 4's files; h and the steering limit anchored there; kp/ki UNTUNED).
def _load_replay_loop():
    directory = str(REPO / "ros_ws/src/racing_sim_autodrive/test")
    if directory not in sys.path:
        sys.path.append(directory)
    import _replay_loop

    return _replay_loop.PORTO_DRIVING_LOOP


AUTODRIVE_REPLAY_START_FRAME, AUTODRIVE_REPLAY_END_FRAME = _load_replay_loop()


def _autodrive_vehicle() -> dict:
    directory = str(REPO / "ros_ws/src/racing_sim_autodrive/test")
    if directory not in sys.path:
        sys.path.insert(0, directory)
    import _vehicle_values

    return _vehicle_values.vehicle()


def autodrive_replay_args(port: int) -> list[str]:
    """The replay's argv after `-m racing_sim_autodrive.replay`: ROS-free, so
    SIMAD-1320 parses what the conformance launch really runs."""
    return [
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
        "--end-frame",
        str(AUTODRIVE_REPLAY_END_FRAME),
    ]


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
        # Parallel colcon tests share the domain (ADAPT-2170, gotcha #39).
        namespace=AUTODRIVE_NAMESPACE,
        remappings=[
            (n, _prefixed(AUTODRIVE_NAMESPACE, n)) for n in CONTRACT_TOPICS
        ],
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
    # The replay loops only the driving frames [132, 338), so any 2 s window,
    # wherever the test's wall clock puts it, moves the car at least 1.82 m
    # (CoG) at recorded pace (SIMAD-1320). The wrap jumps truth about 3.5 m
    # back while odom holds; that cannot fake "moved" (every looped frame is
    # moving) nor "drift" (an odom that copies truth jumps with it). The
    # window still assumes the server handles frames at the recorded pace;
    # under load it may not (fix part 7.1 PR, part 8 rate fix).
    replay = launch.actions.ExecuteProcess(
        cmd=[
            sys.executable,
            "-m",
            "racing_sim_autodrive.replay",
            *autodrive_replay_args(port),
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
        namespace=GYM_NAMESPACE,
        remapped=CONTRACT_TOPICS + (CLOCK_TOPIC,),
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
        expected_h=lambda: _autodrive_vehicle()["h"],
        expected_clock=FREE_RUNNING,
        # Not the declared value, or ADAPT-2145 would be vacuous.
        clock_override=OWNS_CLOCK,
        namespace=AUTODRIVE_NAMESPACE,
        remapped=CONTRACT_TOPICS,
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


def topic(name: str) -> str:
    """``name`` (absolute) as the running row's backend publishes it."""
    return _prefixed(backend().namespace, name)


def node_fqn() -> str:
    return topic(f"/{ADAPTER_NAME}")


def service(name: str) -> str:
    """A private service of the backend."""
    return f"{node_fqn()}/{name}"


def adapter_present(node) -> bool:
    """The backend's node is in the graph under the row's namespace (the bare
    name would also match another package's ``racing_sim``)."""
    ns = backend().namespace
    return (ADAPTER_NAME, ns) in node.get_node_names_and_namespaces()
