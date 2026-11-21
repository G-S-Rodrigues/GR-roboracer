"""slam.launch.py: a hand-driven map of the track with one SLAM method."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.utilities import perform_substitutions
from racing_bringup import (
    autodrive,
    estimators,
    slam,
    track_data,
    visualization,
)
from test_bringup_contract import _merged

REPOSITORY_ROOT = Path(__file__).parents[1].parents[2]
SERIALIZE_COMMAND = (
    "ros2 service call /slam_toolbox/serialize_map "
    "slam_toolbox/srv/SerializePoseGraph"
)


def test_bringup_1991_far_lio_is_refused_as_not_integrated() -> None:
    """BRINGUP-1991: `method:=far_lio` is refused in either mode with the
    reason, and the note it points to exists."""
    for mode in slam.MODES:
        with pytest.raises(RuntimeError) as raised:
            slam.check("far_lio", mode, "")
        assert str(raised.value) == (
            "method:=far_lio is not integrated yet (no launch, no map save; "
            "see docs/agents_knowledge/localization-slam.md)"
        )
    assert (
        REPOSITORY_ROOT / "docs" / "agents_knowledge" / "localization-slam.md"
    ).is_file()


def test_bringup_1991_unknown_method_or_mode_is_refused() -> None:
    """BRINGUP-1991: an unknown method or mode is refused, naming the
    argument and its choices."""
    with pytest.raises(RuntimeError) as raised:
        slam.check("cartographer", "mapping", "")
    message = str(raised.value)
    assert "method:=cartographer" in message
    assert all(method in message for method in slam.METHODS)

    with pytest.raises(RuntimeError) as raised:
        slam.check("slam_toolbox", "racing", "")
    message = str(raised.value)
    assert "mode:=racing" in message
    assert all(mode in message for mode in slam.MODES)


def test_bringup_1991_mapping_needs_no_map() -> None:
    """BRINGUP-1991: every integrated method maps with no `map:=`."""
    for method in ("slam_toolbox", "mola"):
        slam.check(method, "mapping", "")


def test_bringup_1993_slam_toolbox_localization_needs_a_map() -> None:
    """BRINGUP-1993: slam_toolbox localization with an empty `map:=` is
    refused, and the message names the serialize_map command and says
    `map:=` is its base path with no extension."""
    with pytest.raises(RuntimeError) as raised:
        slam.check("slam_toolbox", "localization", "")
    message = str(raised.value)
    assert "map:=" in message
    assert SERIALIZE_COMMAND in message
    assert "no extension" in message


def test_bringup_1993_slam_toolbox_map_must_name_both_graph_files(
    tmp_path: Path,
) -> None:
    """BRINGUP-1993: slam_toolbox's `map:=` is the base path serialize_map
    wrote `<base>.posegraph` and `<base>.data` under; a missing one is
    refused by name, both present is accepted."""
    base = tmp_path / "porto_manual"
    with pytest.raises(RuntimeError) as raised:
        slam.check("slam_toolbox", "localization", str(base))
    assert f"{base}.posegraph" in str(raised.value)

    Path(f"{base}.posegraph").write_bytes(b"")
    with pytest.raises(RuntimeError) as raised:
        slam.check("slam_toolbox", "localization", str(base))
    assert f"{base}.data" in str(raised.value)

    Path(f"{base}.data").write_bytes(b"")
    slam.check("slam_toolbox", "localization", str(base))


def test_bringup_1993_mola_localization_defaults_or_needs_the_file(
    tmp_path: Path,
) -> None:
    """BRINGUP-1993: MOLA localization with an empty `map:=` is accepted
    (the committed Porto `.mm` is the default); a `map:=` naming a missing
    `.mm` is refused by name, an existing one is accepted."""
    slam.check("mola", "localization", "")

    missing = tmp_path / "missing.mm"
    with pytest.raises(RuntimeError) as raised:
        slam.check("mola", "localization", str(missing))
    assert str(missing) in str(raised.value)

    present = tmp_path / "porto.mm"
    present.write_bytes(b"")
    slam.check("mola", "localization", str(present))


RVIZ_CONFIG = "/share/racing_vehicle_description/rviz/slam.rviz"


def test_bringup_1995_no_viewer_is_no_node() -> None:
    """BRINGUP-1995: `visualization:=none` starts nothing."""
    assert visualization.nodes("none", RVIZ_CONFIG, True) == []


@pytest.mark.parametrize("use_sim_time", [True, False])
def test_bringup_1995_rviz_is_one_rviz2_node(use_sim_time: bool) -> None:
    """BRINGUP-1995: `visualization:=rviz` is exactly one rviz2 node on the
    given config, whose `use_sim_time` follows the simulator's clock."""
    started = visualization.nodes("rviz", RVIZ_CONFIG, use_sim_time)
    assert len(started) == 1
    (node,) = started
    assert node._Node__package == "rviz2"
    assert node._Node__node_executable == "rviz2"
    assert node._Node__arguments == ["-d", RVIZ_CONFIG]
    parameters = _merged(LaunchContext(), node)
    assert parameters["use_sim_time"] is use_sim_time


def test_bringup_1995_an_unknown_viewer_is_refused() -> None:
    """BRINGUP-1995: a viewer with no branch is refused, naming the argument
    and the choices."""
    with pytest.raises(RuntimeError) as raised:
        visualization.nodes("rqt", RVIZ_CONFIG, False)
    message = str(raised.value)
    assert "visualization:=rqt" in message
    assert all(viewer in message for viewer in visualization.VIEWERS)


def test_bringup_1995_every_viewer_has_a_branch() -> None:
    """BRINGUP-1995: every member of VIEWERS is accepted, so a value added
    to the choices without a branch fails here, not at launch."""
    for viewer in visualization.VIEWERS:
        assert isinstance(visualization.nodes(viewer, RVIZ_CONFIG, False), list)


# slam.launch.py itself: every case builds the launch's actions from the
# declared defaults and the given arguments, without a graph; none starts a
# process.

LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "slam.launch.py"
ONLINE_1080 = "config/localization/slam_toolbox_online_1080.yaml"
LOCALIZATION_1080 = "config/localization/slam_toolbox_localization_1080.yaml"
MOLA_LOCALIZATION = "config/localization/mola_lidar2d.yaml"
TELEOP_CONFIG = "config/teleop/autodrive_keyboard.yaml"
PORTO_TRACK = (
    REPOSITORY_ROOT / "third_party" / "GR-roboracer-data" / "tracks" / "Porto"
)
DRIVING_PACKAGES = (
    "racing_controller_baseline",
    "racing_safety_supervisor",
    "racing_bringup",
)


@pytest.fixture
def _repository_root(monkeypatch):
    monkeypatch.chdir(REPOSITORY_ROOT)


def _module():
    spec = importlib.util.spec_from_file_location(
        "racing_slam_launch", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _declared(module) -> dict:
    return {
        entity.name: entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, DeclareLaunchArgument)
    }


def _default(argument: DeclareLaunchArgument) -> str:
    return "".join(part.text for part in argument.default_value)


def _context(module, **arguments: str) -> LaunchContext:
    """A context holding every declared argument's default, then
    `arguments`: what `ros2 launch` hands the OpaqueFunction."""
    context = LaunchContext()
    for name, argument in _declared(module).items():
        context.launch_configurations[name] = _default(argument)
    context.launch_configurations.update(arguments)
    return context


def _launched(**arguments: str) -> tuple[LaunchContext, list]:
    """The actions the launch's OpaqueFunction returns."""
    module = _module()
    context = _context(module, **arguments)
    actions: list = []
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, OpaqueFunction):
            actions.extend(entity.execute(context) or [])
    return context, actions


def _nodes(actions) -> list:
    return [a for a in actions if hasattr(a, "_Node__package")]


def _started(nodes) -> set[tuple[str, str]]:
    return {(n._Node__package, n._Node__node_executable) for n in nodes}


def _parameter_files(node) -> list[str]:
    return [
        "".join(part.text for part in entry.param_file)
        for entry in node._Node__parameters
        if not isinstance(entry, dict)
    ]


def _text(item) -> str:
    if isinstance(item, str):
        return item
    return "".join(part.text for part in item)


def _include_arguments(actions) -> dict:
    (include,) = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    return {
        _text(name): _text(value) for name, value in include.launch_arguments
    }


def _alignment(actions) -> list[str]:
    (static,) = [
        n
        for n in _nodes(actions)
        if n._Node__node_executable == "static_transform_publisher"
    ]
    return list(static._Node__arguments)


def _pose_graph(tmp_path: Path) -> str:
    base = tmp_path / "porto_manual"
    for suffix in slam.POSE_GRAPH_SUFFIXES:
        Path(f"{base}{suffix}").write_bytes(b"")
    return str(base)


def test_bringup_1990_every_argument_is_described_with_its_choices(
    _repository_root,
) -> None:
    """BRINGUP-1990: `ros2 launch racing_bringup slam.launch.py -s` lists
    every argument with a description; method, mode and visualization offer
    exactly slam.METHODS, slam.MODES and visualization.VIEWERS; the defaults
    map with slam_toolbox, on the AutoDRIVE car, at the bridge's port; `drive`
    offers exactly drive.DRIVES (window by default) and `teleop_config` names
    the committed keyboard limits."""
    from racing_bringup import drive

    declared = _declared(_module())
    assert set(declared) == {
        "method",
        "mode",
        "map",
        "slam_toolbox_config",
        "simplemap_output",
        "vehicle",
        "port",
        "visualization",
        "drive",
        "teleop_config",
    }
    for name, argument in declared.items():
        assert argument.description.strip(), name
        assert argument.description != "no description given", name
    assert list(declared["method"].choices) == list(slam.METHODS)
    assert list(declared["mode"].choices) == list(slam.MODES)
    assert list(declared["visualization"].choices) == list(
        visualization.VIEWERS
    )
    assert list(declared["drive"].choices) == list(drive.DRIVES)
    assert "Manual" in declared["method"].description
    defaults = {name: _default(a) for name, a in declared.items()}
    assert defaults == {
        "method": "slam_toolbox",
        "mode": "mapping",
        "map": "",
        "slam_toolbox_config": "",
        "simplemap_output": "log/maps/mola_manual.simplemap",
        "vehicle": "config/vehicles/autodrive_roboracer.yaml",
        "port": autodrive.AUTODRIVE_PORT,
        "visualization": "rviz",
        "drive": "window",
        "teleop_config": TELEOP_CONFIG,
    }


def test_bringup_1992_slam_toolbox_mapping_starts_no_driving_stack(
    _repository_root,
) -> None:
    """BRINGUP-1992: slam_toolbox mapping starts the AutoDRIVE server,
    robot_state_publisher, the online slam_toolbox node and rviz2, and
    nothing else: no controller, supervisor or support node, so nothing
    publishes /drive and the car is driven by hand."""
    context, actions = _launched()
    nodes = _nodes(actions)
    assert _started(nodes) == {
        ("racing_sim_autodrive", "racing_sim_autodrive_server"),
        ("robot_state_publisher", "robot_state_publisher"),
        ("slam_toolbox", "async_slam_toolbox_node"),
        ("rviz2", "rviz2"),
    }
    assert not {n._Node__package for n in nodes} & set(DRIVING_PACKAGES)
    for node in nodes:
        remapped = [_text(to) for _, to in node._Node__remappings or []]
        assert "/drive" not in remapped, node._Node__package
    slam_node = next(n for n in nodes if n._Node__package == "slam_toolbox")
    assert _parameter_files(slam_node) == [ONLINE_1080]

    (server,) = [n for n in nodes if n._Node__package == "racing_sim_autodrive"]
    assert server._Node__node_name == "racing_sim"
    merged = _merged(context, server)
    assert merged["port"] == int(autodrive.AUTODRIVE_PORT)
    assert merged["scenario_id"] == autodrive.AUTODRIVE_SCENARIO


def test_bringup_1992_slam_toolbox_config_overrides_the_default(
    _repository_root,
) -> None:
    """BRINGUP-1992: `slam_toolbox_config:=` is the file the node gets."""
    _, actions = _launched(slam_toolbox_config=LOCALIZATION_1080)
    (node,) = [n for n in _nodes(actions) if n._Node__package == "slam_toolbox"]
    assert _parameter_files(node) == [LOCALIZATION_1080]


def test_bringup_1993_slam_toolbox_localization_reads_the_pose_graph(
    _repository_root, tmp_path: Path
) -> None:
    """BRINGUP-1993: slam_toolbox localization starts
    localization_slam_toolbox_node on the localization file, with
    `map_file_name` the `map:=` base path."""
    base = _pose_graph(tmp_path)
    context, actions = _launched(mode="localization", map=base)
    (node,) = [n for n in _nodes(actions) if n._Node__package == "slam_toolbox"]
    assert node._Node__node_executable == "localization_slam_toolbox_node"
    assert _parameter_files(node) == [LOCALIZATION_1080]
    assert _merged(context, node)["map_file_name"] == base


def test_bringup_1993_the_launch_refuses_what_slam_check_refuses(
    _repository_root,
) -> None:
    """BRINGUP-1993: the launch runs slam.check (far_lio, a localization
    with no map) and autodrive.check (a gym vehicle) before any node."""
    with pytest.raises(RuntimeError, match="far_lio"):
        _launched(method="far_lio")
    with pytest.raises(RuntimeError, match="serialize_map"):
        _launched(mode="localization")
    with pytest.raises(RuntimeError, match="f1tenth_default"):
        _launched(vehicle="config/vehicles/f1tenth_default.yaml")


def test_bringup_1994_mola_mapping_writes_the_simplemap_asked_for(
    _repository_root, tmp_path: Path
) -> None:
    """BRINGUP-1994: MOLA mapping runs MOLA-LO's mapping pipeline with
    MOLA_SIMPLEMAP_OUTPUT = `simplemap_output`, whose folder the launch
    creates (MOLA would fail to write into a missing one), and no
    slam_toolbox."""
    output = tmp_path / "maps" / "porto.simplemap"
    context, actions = _launched(method="mola", simplemap_output=str(output))
    (variable,) = [a for a in actions if isinstance(a, SetEnvironmentVariable)]
    assert perform_substitutions(context, variable.name) == (
        "MOLA_SIMPLEMAP_OUTPUT"
    )
    assert perform_substitutions(context, variable.value) == str(output)
    assert output.parent.is_dir()
    arguments = _include_arguments(actions)
    assert arguments["start_mapping_enabled"] == "True"
    assert "mola_initial_map_mm_file" not in arguments
    assert "slam_toolbox" not in {n._Node__package for n in _nodes(actions)}


def test_bringup_1994_mola_localization_defaults_to_the_porto_map(
    _repository_root,
) -> None:
    """BRINGUP-1994: MOLA localization with no `map:=` localizes on the
    committed Porto .mm with the localization pipeline, and map -> mola_map
    is its provenance's T."""
    _, actions = _launched(method="mola", mode="localization")
    mm, provenance = track_data.mola_files(PORTO_TRACK)
    arguments = _include_arguments(actions)
    assert Path(arguments["mola_initial_map_mm_file"]).resolve() == (
        Path(mm).resolve()
    )
    assert arguments["start_mapping_enabled"] == "False"
    assert arguments == estimators.mola_launch_arguments(
        MOLA_LOCALIZATION, arguments["mola_initial_map_mm_file"], False
    )
    assert _alignment(actions) == estimators.mola_alignment_arguments(
        provenance
    )


def test_bringup_1994_a_given_mm_takes_its_sibling_provenance(
    _repository_root, tmp_path: Path
) -> None:
    """BRINGUP-1994: `map:=<x>.mm` localizes on that file, and
    map -> mola_map is the T of `<x>.yaml` beside it."""
    mm = tmp_path / "porto_manual.mm"
    mm.write_bytes(b"")
    (tmp_path / "porto_manual.yaml").write_text(
        "T_map_mola: {x: 1.5, y: -2.25, z: 0.074, yaw: 0.5}\n",
        encoding="utf-8",
    )
    _, actions = _launched(method="mola", mode="localization", map=str(mm))
    assert _include_arguments(actions)["mola_initial_map_mm_file"] == str(mm)
    alignment = _alignment(actions)
    assert alignment == estimators.mola_alignment_arguments(
        str(tmp_path / "porto_manual.yaml")
    )
    assert alignment[1] == "1.5" and alignment[7] == "0.5"


def test_bringup_1994_a_given_mm_without_provenance_is_the_identity(
    _repository_root, tmp_path: Path
) -> None:
    """BRINGUP-1994: a `.mm` with no `<x>.yaml` beside it (a hand-driven
    mapping lap) is aligned by the identity: `map` is where that lap
    started."""
    mm = tmp_path / "porto_manual.mm"
    mm.write_bytes(b"")
    _, actions = _launched(method="mola", mode="localization", map=str(mm))
    assert _include_arguments(actions)["mola_initial_map_mm_file"] == str(mm)
    alignment = _alignment(actions)
    values = dict(zip(alignment[0::2], alignment[1::2], strict=True))
    assert set(values) == {
        "--x",
        "--y",
        "--z",
        "--yaw",
        "--frame-id",
        "--child-frame-id",
    }
    for key in ("--x", "--y", "--z", "--yaw"):
        assert float(values[key]) == 0.0, key
    assert values["--frame-id"] == "map"
    assert values["--child-frame-id"] == "mola_map"


def test_bringup_1995_the_launch_starts_rviz_on_slam_rviz_or_nothing(
    _repository_root,
) -> None:
    """BRINGUP-1995: `visualization:=rviz` (the default) starts rviz2 on the
    installed slam.rviz; `visualization:=none` starts no rviz2."""
    rviz = (
        Path(get_package_share_directory("racing_vehicle_description"))
        / "rviz"
        / "slam.rviz"
    )
    assert rviz.is_file()
    _, actions = _launched()
    (node,) = [n for n in _nodes(actions) if n._Node__package == "rviz2"]
    assert node._Node__arguments == ["-d", str(rviz)]

    _, actions = _launched(visualization="none")
    assert "rviz2" not in {n._Node__package for n in _nodes(actions)}


def test_bringup_1995_the_launch_refuses_an_unknown_viewer(
    _repository_root,
) -> None:
    """BRINGUP-1995: `visualization:=foxglove` is refused by the declared
    choices before the OpaqueFunction runs."""
    argument = _declared(_module())["visualization"]
    context = LaunchContext()
    context.launch_configurations["visualization"] = "foxglove"
    with pytest.raises(RuntimeError, match="foxglove"):
        argument.execute(context)


def test_bringup_1995_slam_rviz_shows_both_methods_maps() -> None:
    """BRINGUP-1995: slam.rviz is fixed in `map` and shows the grid, /scan
    (best effort, as the server publishes it), the robot, slam_toolbox's
    graph and MOLA's local map."""
    document = yaml.safe_load(
        (
            REPOSITORY_ROOT
            / "ros_ws"
            / "src"
            / "racing_vehicle_description"
            / "rviz"
            / "slam.rviz"
        ).read_text(encoding="utf-8")
    )
    manager = document["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "map"
    displays = {d["Class"].split("/")[-1]: d for d in manager["Displays"]}
    assert set(displays) == {
        "Grid",
        "Map",
        "LaserScan",
        "RobotModel",
        "MarkerArray",
        "PointCloud2",
    }
    assert displays["Map"]["Topic"]["Value"] == "/map"
    assert displays["Map"]["Topic"]["Durability Policy"] == "Transient Local"
    assert displays["Map"]["Topic"]["Reliability Policy"] == "Reliable"
    assert displays["LaserScan"]["Topic"]["Value"] == "/scan"
    assert displays["LaserScan"]["Topic"]["Reliability Policy"] == (
        "Best Effort"
    )
    assert displays["RobotModel"]["Description Topic"] == "/robot_description"
    assert displays["MarkerArray"]["Topic"]["Value"] == (
        "/slam_toolbox/graph_visualization"
    )
    assert displays["PointCloud2"]["Topic"]["Value"] == (
        "/lidar_odometry/localmap_points"
    )


@pytest.mark.parametrize(
    "arguments",
    [
        {},
        {"mode": "localization"},
        {"method": "mola"},
        {"method": "mola", "mode": "localization"},
        {"drive": "teleop"},
    ],
    ids=["slam_toolbox", "slam_toolbox_loc", "mola", "mola_loc", "teleop"],
)
def test_bringup_1996_every_node_runs_on_wall_time(
    _repository_root, tmp_path: Path, arguments: dict
) -> None:
    """BRINGUP-1996: AutoDRIVE publishes no /clock (ADR 0006), so every node
    the launch starts, and MOLA's include, runs with use_sim_time False (the
    server sets none, which is ROS's default False)."""
    arguments = dict(arguments)
    if arguments.get("method") == "mola":
        arguments["simplemap_output"] = str(tmp_path / "out.simplemap")
    elif arguments.get("mode") == "localization":
        arguments["map"] = _pose_graph(tmp_path)
    context, actions = _launched(**arguments)
    nodes = _nodes(actions)
    assert nodes
    for node in nodes:
        assert _merged(context, node).get("use_sim_time", False) is False, (
            node._Node__package
        )
    if arguments.get("method") == "mola":
        assert _include_arguments(actions)["use_sim_time"] == "False"


def test_bringup_2000_the_default_drive_starts_no_teleop_node(
    _repository_root,
) -> None:
    """BRINGUP-2000: `drive:=window` (the default) starts no
    racing_bringup_teleop, and BRINGUP-1992's set is unchanged."""
    _, actions = _launched()
    assert ("racing_bringup", "racing_bringup_teleop") not in _started(
        _nodes(actions)
    )
    _, actions = _launched(drive="window")
    assert len(_nodes(actions)) == 4


def test_bringup_2000_drive_teleop_adds_exactly_the_teleop_node(
    _repository_root,
) -> None:
    """BRINGUP-2000: `drive:=teleop` adds exactly one racing_bringup_teleop
    with the keyboard limits as its parameter file; still no support node,
    controller or supervisor."""
    _, actions = _launched(drive="teleop")
    nodes = _nodes(actions)
    started = _started(nodes)
    assert _started(_nodes(_launched()[1])) <= started
    assert started - _started(_nodes(_launched()[1])) == {
        ("racing_bringup", "racing_bringup_teleop")
    }
    assert ("racing_bringup", "racing_bringup_support") not in started
    packages = {n._Node__package for n in nodes}
    assert not packages & {
        "racing_controller_baseline",
        "racing_safety_supervisor",
    }
    (teleop_node,) = [
        n for n in nodes if n._Node__node_executable == "racing_bringup_teleop"
    ]
    assert _parameter_files(teleop_node) == [TELEOP_CONFIG]
    assert teleop_node._Node__node_name == "racing_bringup_teleop"


def test_bringup_2000_an_unknown_drive_is_refused_by_name(
    _repository_root,
) -> None:
    """BRINGUP-2000: `drive:=joystick` is refused by the declared choices
    before the OpaqueFunction runs."""
    argument = _declared(_module())["drive"]
    context = LaunchContext()
    context.launch_configurations["drive"] = "joystick"
    with pytest.raises(RuntimeError, match="joystick"):
        argument.execute(context)


def test_bringup_2000_teleop_limits_above_the_vehicle_caps_are_refused(
    _repository_root, tmp_path: Path
) -> None:
    """BRINGUP-2000: a teleop_config with max_speed 5.0 is refused by
    check_caps for the default vehicle, naming the key."""
    document = yaml.safe_load(
        (REPOSITORY_ROOT / TELEOP_CONFIG).read_text(encoding="utf-8")
    )
    document["racing_bringup_teleop"]["ros__parameters"]["max_speed"] = 5.0
    fast = tmp_path / "fast.yaml"
    fast.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(RuntimeError, match="max_speed"):
        _launched(drive="teleop", teleop_config=str(fast))
