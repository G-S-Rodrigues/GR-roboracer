"""Tier 1: `simulator:=` and `autodrive_bridge:=` on the Pure Pursuit launch
(#44 part 8). Every case runs `_launch_nodes` without a graph, like
BRINGUP-1090/1170; none starts a process.

A separate file from test_bringup_contract.py so the launch edits of the other
open branches (V1, SLAM-8) do not collide with these tests' hunks.
"""

import ast
from pathlib import Path

import pytest
import yaml
from test_bringup_contract import (
    LAUNCH_FILE,
    NOT_GYM_CAST,
    REPOSITORY_ROOT,
    _launch_context,
    _merged,
)

AUTODRIVE_VEHICLE = "config/vehicles/autodrive_roboracer.yaml"
PORTO = "config/scenarios/porto.yaml"
AUTODRIVE = {
    "simulator": "autodrive",
    "vehicle": AUTODRIVE_VEHICLE,
    "scenario": PORTO,
}
PORTO_START = REPOSITORY_ROOT / "config" / "autodrive" / "porto_frame.yaml"


@pytest.fixture(autouse=True)
def _repository_root(monkeypatch):
    monkeypatch.chdir(REPOSITORY_ROOT)


def _nodes(module, context) -> list:
    return [
        node
        for node in module._launch_nodes(context, "", "")
        if hasattr(node, "_Node__package")
    ]


def _packages(nodes) -> list[str]:
    return [node._Node__package for node in nodes]


def _declared(module) -> dict:
    return {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if hasattr(entity, "default_value")
    }


def _roles() -> dict:
    return yaml.safe_load(
        (REPOSITORY_ROOT / "config" / "reference_stack.yaml").read_text(
            encoding="utf-8"
        )
    )["roles"]


def test_bringup_1600_simulator_choices_are_the_reference_stack_ones() -> None:
    """BRINGUP-1600: `simulator` is a launch argument whose choices are
    exactly the implementations `reference_stack.yaml` names for the role,
    the default being the reference one; every alternative's package is a
    literal `Node(package=...)` in the launch (BRINGUP-1030 checks only the
    default's), and the pointer to the image digests resolves to the file
    that carries them (plan D2)."""
    module, _ = _launch_context()
    argument = _declared(module)["simulator"]
    simulator = _roles()["simulator"]
    named = [
        simulator["implementation"],
        *(a["implementation"] for a in simulator.get("alternatives", [])),
    ]
    assert sorted(argument.choices) == sorted(named)
    default = "".join(part.text for part in argument.default_value)
    assert default == simulator["implementation"] == "f1tenth_gym_jax"
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    literal_packages = {
        keyword.value.value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call) and getattr(call.func, "id", "") == "Node"
        for keyword in call.keywords
        if keyword.arg == "package" and isinstance(keyword.value, ast.Constant)
    }
    for alternative in simulator.get("alternatives", []):
        assert alternative["package"] in literal_packages, alternative
    images = REPOSITORY_ROOT / simulator["alternatives"][0]["images"]
    text = images.read_text(encoding="utf-8")
    assert text.count("@sha256:") >= 2
    port = yaml.safe_load(text)["x-autodrive"]["bridge_port"]
    assert "".join(
        p.text for p in _declared(module)["autodrive_port"].default_value
    ) == str(port)


def test_bringup_1610_autodrive_bridge_offers_ours_and_official() -> None:
    """BRINGUP-1610: `autodrive_bridge` offers `ours` (default) and
    `official`; both launch (`official` is built by BRINGUP-1690, which
    replaces this case's old refusal of it)."""
    module, context = _launch_context(**AUTODRIVE)
    argument = _declared(module)["autodrive_bridge"]
    assert list(argument.choices) == ["ours", "official"]
    assert "".join(p.text for p in argument.default_value) == "ours"
    _nodes(module, context)
    module, context = _launch_context(
        autodrive_bridge="official", autodrive_devkit_domain="7", **AUTODRIVE
    )
    assert "racing_sim_autodrive" in _packages(_nodes(module, context))


def test_bringup_1620_autodrive_launches_our_server_and_no_gym() -> None:
    """BRINGUP-1620: `simulator:=autodrive` starts racing_sim_autodrive_server
    (node name racing_sim) with the server values file, the Porto track and
    frame, the scenario's id and `autodrive_port`, and no gym node; the
    default launches the gym and no server."""
    module, context = _launch_context()
    default = _packages(_nodes(module, context))
    assert "racing_sim_gym_jax" in default
    assert "racing_sim_autodrive" not in default
    # The server's own inputs are read only when it launches: a bad port
    # cannot break a gym launch that never uses it.
    module, context = _launch_context(autodrive_port="not-a-port")
    assert _packages(_nodes(module, context)) == default

    module, context = _launch_context(autodrive_port="4999", **AUTODRIVE)
    nodes = _nodes(module, context)
    assert "racing_sim_gym_jax" not in _packages(nodes)
    servers = [n for n in nodes if n._Node__package == "racing_sim_autodrive"]
    assert len(servers) == 1
    server = servers[0]
    assert server._Node__node_executable == "racing_sim_autodrive_server"
    assert server._Node__node_name == "racing_sim"
    parameters = _merged(context, server)
    frame = yaml.safe_load(PORTO_START.read_text(encoding="utf-8"))
    assert parameters["world_to_map.x"] == frame["world_to_map"]["x"]
    assert parameters["world_to_map.y"] == frame["world_to_map"]["y"]
    assert parameters["world_to_map.yaw"] == frame["world_to_map"]["yaw"]
    assert parameters["track_path"] == str(
        REPOSITORY_ROOT / "config" / "tracks" / "porto.yaml"
    )
    assert parameters["scenario_id"] == "porto"
    assert int(parameters["port"]) == 4999
    files = [
        "".join(part.text for part in entry.param_file)
        for entry in server._Node__parameters
        if not isinstance(entry, dict)
    ]
    assert [Path(name).name for name in files] == ["server.yaml"]
    assert Path(files[0]).parent.name == "autodrive_roboracer"


def _sim_time(context, node) -> object:
    return _merged(context, node).get("use_sim_time", "absent")


def test_bringup_1630_use_sim_time_follows_the_clock_capability() -> None:
    """BRINGUP-1630: every node but the simulator reads `/clock` exactly when
    the simulator owns it (the gym: yes; AutoDRIVE, free-running: no), the
    simulator itself setting no `use_sim_time` (ADR 0006). Replaces the
    literal-`True` reading of the launch source; `time_scale` is still a
    launch-time control."""
    module, context = _launch_context()
    gym = _nodes(module, context)
    # The exact graph (the removed test pinned the 9-node set): the gym
    # launch, and the autodrive one with its server in the gym's place.
    gym_packages = sorted(_packages(gym))
    assert gym_packages == [
        "racing_bringup",
        "racing_controller_baseline",
        "racing_evaluation",
        "racing_metrics",
        "racing_recording",
        "racing_safety_supervisor",
        "racing_sim_gym_jax",
        "robot_state_publisher",
        "rviz2",
    ]
    for node in gym:
        if node._Node__package == "racing_sim_gym_jax":
            assert _sim_time(context, node) == "absent"
        else:
            assert _sim_time(context, node) is True, node._Node__package

    module, context = _launch_context(**AUTODRIVE)
    autodrive = _nodes(module, context)
    assert sorted(_packages(autodrive)) == sorted(
        "racing_sim_autodrive" if name == "racing_sim_gym_jax" else name
        for name in gym_packages
    )
    for node in autodrive:
        if node._Node__package == "racing_sim_autodrive":
            assert _sim_time(context, node) == "absent"
        else:
            assert _sim_time(context, node) is False, node._Node__package

    assert 'DeclareLaunchArgument("time_scale"' in LAUNCH_FILE.read_text(
        encoding="utf-8"
    )


def test_bringup_1640_estimators_follow_the_simulator_clock() -> None:
    """BRINGUP-1640: slam_toolbox, nav2_amcl, MOLA's launch arguments and its
    map alignment node take `use_sim_time` from the caller (default True,
    which is what the gym's goldens ran on)."""
    from racing_bringup.estimators import (
        amcl,
        mola,
        mola_launch_arguments,
        slam_toolbox,
    )
    from racing_bringup.scenario_parameters import track_parameters

    config = REPOSITORY_ROOT / "config" / "localization"
    for node in slam_toolbox(
        "mapping", [str(config / "slam_toolbox_online.yaml")]
    ):
        if hasattr(node, "_Node__package"):
            assert _sim_time_of(node) is True
    for node in slam_toolbox(
        "mapping",
        [str(config / "slam_toolbox_online.yaml")],
        use_sim_time=False,
    ):
        if hasattr(node, "_Node__package"):
            assert _sim_time_of(node) is False

    scenario = REPOSITORY_ROOT / "config/scenarios/spielberg_1080.yaml"
    grid = track_parameters(scenario)["map_server"]["yaml_filename"]
    for use_sim_time in (True, False):
        for node in amcl(
            [str(config / "amcl.yaml")], grid, use_sim_time=use_sim_time
        ):
            assert _sim_time_of(node) is use_sim_time, node._Node__package
        static, _include = mola(
            str(config / "mola_lidar2d.yaml"), grid, use_sim_time=use_sim_time
        )
        assert _sim_time_of(static) is use_sim_time
    assert (
        mola_launch_arguments(
            str(config / "mola_lidar2d.yaml"), "x.mm", use_sim_time=False
        )["use_sim_time"]
        == "False"
    )

    module, context = _launch_context(pose_source="slam_toolbox", **AUTODRIVE)
    slam = [
        n for n in _nodes(module, context) if n._Node__package == "slam_toolbox"
    ]
    assert len(slam) == 1
    assert _sim_time(context, slam[0]) is False


def _sim_time_of(node) -> object:
    """`use_sim_time` of a Node whose parameters are files and dicts: the
    last dict that names it wins, as in the launch's list order."""
    from launch import LaunchContext

    context = LaunchContext()
    return _merged(context, node).get("use_sim_time", "absent")


def test_bringup_1650_the_gym_refuses_a_vehicle_it_cannot_cast() -> None:
    """BRINGUP-1650: `vehicle:=autodrive_roboracer.yaml` with the gym is an
    error naming `simulator:=autodrive` (the gym would cast /scan from a
    mount that is not that car's); the production list of such vehicles
    equals the contract test's NOT_GYM_CAST."""
    module, context = _launch_context(vehicle=AUTODRIVE_VEHICLE)
    with pytest.raises(RuntimeError, match=r"simulator:=autodrive"):
        module._launch_nodes(context, "", "")
    assert frozenset(module.GYM_INCOMPATIBLE_VEHICLES) == NOT_GYM_CAST


@pytest.mark.parametrize(
    "overrides, message",
    [
        ({"vehicle": "config/vehicles/f1tenth_default.yaml"}, "vehicle"),
        ({"scenario": "config/scenarios/contract_test.yaml"}, "scenario"),
        ({"start_held": "true"}, "start_held"),
        ({"time_scale": "2.0"}, "time_scale"),
    ],
)
def test_bringup_1660_autodrive_refuses_what_it_cannot_do(
    overrides, message
) -> None:
    """BRINGUP-1660: `simulator:=autodrive` refuses a vehicle other than
    autodrive_roboracer, a scenario other than porto (the build has one
    active track), `start_held:=true` and `time_scale` other than 1, each
    naming the argument; the valid combination launches."""
    module, context = _launch_context(**{**AUTODRIVE, **overrides})
    with pytest.raises(RuntimeError, match=message):
        module._launch_nodes(context, "", "")
    module, context = _launch_context(**AUTODRIVE)
    assert module._launch_nodes(context, "", "")


def test_bringup_1670_server_values_are_part_4s_numbers() -> None:
    """BRINGUP-1670: config/vehicles/autodrive_roboracer/server.yaml carries
    the server's seven vehicle values, and each is the number its source
    states (lr, wheel_radius: source.yaml; throttle_gain, h, the steering
    limit: identified.yaml; kp and ki, untuned until part 10, are pinned
    here), so the launch cannot start the server on a drifted copy."""
    directory = REPOSITORY_ROOT / "config" / "vehicles" / "autodrive_roboracer"
    server = yaml.safe_load((directory / "server.yaml").read_text())[
        "racing_sim"
    ]["ros__parameters"]
    source = yaml.safe_load((directory / "source.yaml").read_text())["values"]
    identified = yaml.safe_load((directory / "identified.yaml").read_text())
    assert set(server) == {
        "lr",
        "h",
        "wheel_radius",
        "throttle_gain",
        "max_steering_angle",
        "kp",
        "ki",
    }
    assert server["lr"] == source["com_x_lr_m"]["value"]
    assert server["wheel_radius"] == source["wheel_radius_m"]["value"]
    assert (
        server["throttle_gain"]
        == identified["measurements"]["throttle_gain"]["value"]
    )
    assert server["h"] == identified["measurements"]["rest_height"]["value"]
    assert (
        server["max_steering_angle"]
        == identified["measurements"]["steering"]["limit"]["value"]
    )
    # No source states the gains: they are part 10's to tune, and a change
    # here is a reviewed one.
    assert (server["kp"], server["ki"]) == (0.02, 0.05)


def test_bringup_1680_trajectory_speed_is_refused_on_a_map_with_no_raceline():
    """BRINGUP-1680: `speed_source:=trajectory` on a map whose raceline_path
    resolves empty (Porto has no `<map>_raceline.csv`, so `track_centerline`
    drives the centerline at speed 0) is an error naming the map, instead of
    a car that never moves; the curvature schedule and a map with a raceline
    still launch."""
    module, context = _launch_context(speed_source="trajectory", **AUTODRIVE)
    with pytest.raises(RuntimeError, match=r"porto.*no raceline") as refused:
        module._launch_nodes(context, "", "")
    assert "speed_source" in str(refused.value)
    module, context = _launch_context(**AUTODRIVE)
    assert module._launch_nodes(context, "", "")
    module, context = _launch_context(
        speed_source="trajectory",
        scenario="config/scenarios/spielberg.yaml",
    )
    assert module._launch_nodes(context, "", "")


def test_bringup_1690_official_bridge_launches_the_devkit_node(
    monkeypatch,
) -> None:
    """BRINGUP-1690: `autodrive_bridge:=official` starts
    racing_sim_autodrive_devkit (node name racing_sim, no server) with the
    server values file, the Porto frame, track and scenario id, and
    `devkit_domain_id` = `autodrive_devkit_domain`; `autodrive_port` is
    not read. The argument defaults to $AUTODRIVE_ROS_DOMAIN_ID, and each
    refusal names it: missing, not an integer 0-101, or equal to the
    launch's own `ROS_DOMAIN_ID` (unset = 0), which would put the devkit's
    global /tf (world -> roboracer_1 -> lidar) in the stack's TF tree.
    Replaces BRINGUP-1610's old refusal of `official`: that one guarded
    "official never silently runs nothing"; this asserts the exact node it
    runs, its parameters and domain, and every way it refuses.
    """
    monkeypatch.setenv("ROS_DOMAIN_ID", "62")
    official = {**AUTODRIVE, "autodrive_bridge": "official"}
    module, context = _launch_context(
        autodrive_devkit_domain="64", autodrive_port="not-a-port", **official
    )
    nodes = _nodes(module, context)
    packages = _packages(nodes)
    assert packages.count("racing_sim_autodrive") == 1
    assert "racing_sim_gym_jax" not in packages
    node = next(n for n in nodes if n._Node__package == "racing_sim_autodrive")
    assert node._Node__node_executable == "racing_sim_autodrive_devkit"
    assert node._Node__node_name == "racing_sim"
    parameters = _merged(context, node)
    frame = yaml.safe_load(PORTO_START.read_text(encoding="utf-8"))
    assert parameters["world_to_map.x"] == frame["world_to_map"]["x"]
    assert parameters["track_path"] == str(
        REPOSITORY_ROOT / "config" / "tracks" / "porto.yaml"
    )
    assert parameters["scenario_id"] == "porto"
    assert int(parameters["devkit_domain_id"]) == 64
    assert "port" not in parameters
    files = [
        "".join(part.text for part in entry.param_file)
        for entry in node._Node__parameters
        if not isinstance(entry, dict)
    ]
    assert [Path(name).name for name in files] == ["server.yaml"]

    declared = _declared(module)["autodrive_devkit_domain"]
    monkeypatch.setenv("AUTODRIVE_ROS_DOMAIN_ID", "71")
    module, _ = _launch_context()
    default = _declared(module)["autodrive_devkit_domain"].default_value
    assert "".join(p.text for p in default) == "71"
    assert declared is not None

    for value, why in (
        ("", "missing"),
        ("abc", "not an integer"),
        ("102", "above 101"),
        ("-1", "negative"),
        ("62", "equal to the launch's ROS_DOMAIN_ID"),
    ):
        module, context = _launch_context(
            autodrive_devkit_domain=value, **official
        )
        with pytest.raises(RuntimeError, match="autodrive_devkit_domain"):
            module._launch_nodes(context, "", "")
        print(why)

    # unused under `ours`: a bad domain cannot break a launch that never
    # reads it
    module, context = _launch_context(
        autodrive_devkit_domain="abc", **AUTODRIVE
    )
    assert "racing_sim_autodrive" in _packages(_nodes(module, context))
