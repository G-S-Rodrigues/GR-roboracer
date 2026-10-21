"""Structural contract for the complete Pure Pursuit bringup graph."""

import ast
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "sim_pure_pursuit.launch.py"
VEHICLE_CONFIG = (
    REPOSITORY_ROOT / "config" / "vehicles" / "f1tenth_default.yaml"
)
VEHICLE_FILES = sorted((REPOSITORY_ROOT / "config" / "vehicles").glob("*.yaml"))


def _literal_keyword(call: ast.Call, name: str) -> str | None:
    for keyword in call.keywords:
        if keyword.arg == name and isinstance(keyword.value, ast.Constant):
            return keyword.value.value
    return None


def test_launch_composes_the_complete_runtime_graph() -> None:
    """The canonical launch wires every vertical-slice process."""
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    nodes = {
        (
            _literal_keyword(node, "package"),
            _literal_keyword(node, "executable"),
        )
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    }

    assert {
        ("racing_sim_gym_jax", "racing_sim_gym_jax_node"),
        ("racing_controller_baseline", "racing_controller_baseline_node"),
        ("racing_safety_supervisor", "racing_safety_supervisor_node"),
        ("racing_metrics", "racing_metrics_node"),
        ("racing_evaluation", "racing_evaluation_node"),
        ("racing_recording", "racing_recording_node"),
        ("racing_bringup", "racing_bringup_support"),
        ("robot_state_publisher", "robot_state_publisher"),
        ("rviz2", "rviz2"),
    }.issubset(nodes)


def test_launch_exposes_reproducibility_and_headless_controls() -> None:
    """Scenario, seed, recording and RViz are launch-time controls."""
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    for argument in (
        "scenario",
        "seed",
        "recording_path",
        "use_rviz",
        "vehicle",
    ):
        assert f'DeclareLaunchArgument("{argument}"' in source
    assert "IfCondition" in source
    assert "f1tenth_default.yaml" in source


def test_launch_runs_on_simulated_time() -> None:
    """ADR 0006: every node but sim_node takes time from /clock; sim_node
    owns the clock and its rate is a launch-time control."""
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    nodes_by_variable = {
        node.targets[0].id: node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and isinstance(node.value, ast.Call)
        and isinstance(node.value.func, ast.Name)
        and node.value.func.id == "Node"
    }

    def _uses_sim_time(call: ast.Call) -> bool:
        for keyword in call.keywords:
            if keyword.arg != "parameters" or not isinstance(
                keyword.value, ast.List
            ):
                continue
            for element in keyword.value.elts:
                if not isinstance(element, ast.Dict):
                    continue
                for key, value in zip(
                    element.keys, element.values, strict=True
                ):
                    if (
                        isinstance(key, ast.Constant)
                        and key.value == "use_sim_time"
                        and isinstance(value, ast.Constant)
                        and value.value is True
                    ):
                        return True
        return False

    assert set(nodes_by_variable) == {
        "sim_node",
        "support_node",
        "controller_node",
        "supervisor_node",
        "metrics_node",
        "evaluation_node",
        "recording_node",
        "robot_state_publisher_node",
        "rviz_node",
    }
    assert not _uses_sim_time(nodes_by_variable["sim_node"])
    for variable, call in nodes_by_variable.items():
        if variable == "sim_node":
            continue
        assert _uses_sim_time(call), f"{variable} must run with use_sim_time"

    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("time_scale"' in source


def test_launch_can_hold_the_sim_until_reset() -> None:
    """A caller that defines its own t=0 (scenario_runner) can start the sim
    held; by default it runs, so a hand-driven launch still moves."""
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("start_held", default_value="false")' in (
        source
    )
    assert '"start_held": start_held' in source


def test_default_vehicle_configuration_covers_each_configurable_node() -> None:
    """One versioned vehicle file owns every node's physical/control limits."""
    source = VEHICLE_CONFIG.read_text(encoding="utf-8")
    assert "vehicle_parameter_version: f1tenth-default-v1" in source
    for node_name in (
        "racing_controller_baseline",
        "racing_safety_supervisor",
        "racing_bringup_support",
        "racing_metrics",
        "racing_recording",
    ):
        assert f"{node_name}:\n  ros__parameters:" in source
    for parameter in (
        "wheelbase: 0.33",
        "vehicle_half_width: 0.15",
        "maximum_steering_angle: 0.4",
        "maximum_speed: 3.0",
    ):
        assert parameter in source


def test_support_node_declares_the_visualization_contract() -> None:
    """The support node owns the missing trajectory, boundary and TF outputs."""
    support = (PACKAGE_ROOT / "racing_bringup" / "support_node.py").read_text(
        encoding="utf-8"
    )
    for contract in (
        '"/trajectory"',
        '"/track/boundaries"',
        '"/visualization/track"',
        '"/ground_truth/odom"',
        '"map"',
        '"odom"',
    ):
        assert contract in support


def test_bringup_1010_the_scenario_selects_every_nodes_track() -> None:
    """BRINGUP-1010: the scenario file, not the vehicle file, decides which
    track every track-aware node loads.

    Otherwise a scenario selects the sim's world while the supervisor, the
    support node's trajectory and racing_metrics' lap length stay on the
    vehicle file's track - repo-gotchas #15's shape: the run completes,
    publishes metrics and passes every structural check on the wrong track.
    """
    from racing_bringup.scenario_parameters import track_parameters

    spielberg = track_parameters(
        REPOSITORY_ROOT / "config" / "scenarios" / "spielberg.yaml"
    )
    track = str(REPOSITORY_ROOT / "config" / "tracks" / "spielberg.yaml")
    raceline = str(
        REPOSITORY_ROOT
        / "config"
        / "scenarios"
        / "maps"
        / "Spielberg"
        / "Spielberg_raceline.csv"
    )
    assert spielberg["racing_safety_supervisor"]["track_path"] == track
    assert spielberg["racing_metrics"]["track_path"] == track
    assert spielberg["racing_evaluation"]["track_path"] == track
    assert spielberg["racing_bringup_support"]["track_path"] == track
    assert spielberg["racing_bringup_support"]["raceline_path"] == raceline
    for reporter in ("racing_metrics", "racing_recording"):
        assert spielberg[reporter]["scenario_id"] == "spielberg"
        assert spielberg[reporter]["track_version"] == "spielberg-v1"

    # A scenario that names no reporting identity keeps the vehicle file's,
    # so the analytic_circle golden's "baseline" / "analytic_circle-v1"
    # records are unchanged.
    circle = track_parameters(
        REPOSITORY_ROOT / "config" / "scenarios" / "contract_test.yaml"
    )
    assert circle["racing_metrics"]["track_path"] == str(
        REPOSITORY_ROOT / "config" / "tracks" / "analytic_circle.yaml"
    )
    assert "scenario_id" not in circle["racing_metrics"]
    assert "track_version" not in circle["racing_recording"]


def test_bringup_1030_the_launch_composes_the_reference_stack() -> None:
    """BRINGUP-1030: `config/reference_stack.yaml` and the launch file agree.

    ADR 0005 defines the reference stack in exactly one place. The launch
    file is what actually runs, so a second, silently diverging description
    is the failure this prevents: every implementation named in the file is
    a node the launch starts, and the pose source it names is the pose the
    controller drives on and racing_evaluation scores by default.
    """
    import yaml

    reference = yaml.safe_load(
        (REPOSITORY_ROOT / "config" / "reference_stack.yaml").read_text(
            encoding="utf-8"
        )
    )
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    launched = {
        _literal_keyword(node, "package")
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    }
    constants = {
        node.targets[0].id: node.value.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and isinstance(node.value, ast.Constant)
    }

    roles = reference["roles"]
    for role, entry in roles.items():
        assert entry["package"] in launched, f"{role}: {entry['package']}"
    assert roles["pose_source"]["implementation"] == "ground_truth"
    assert constants["GROUND_TRUTH_POSE"] == roles["pose_source"]["topic"]
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    assert 'remappings=[("/odom", GROUND_TRUTH_POSE)]' in source
    assert '"estimate_topic", default_value=GROUND_TRUTH_POSE' in source
    # The planner role: the default implementation is the launch's default
    # `planner`, and the implementations named agree with the resolver's.
    from racing_bringup import scenario_parameters

    assert roles["planner"]["implementation"] == "track_centerline"
    assert '"planner", default_value="track_centerline"' in " ".join(
        source.split()
    )
    named = {
        roles["planner"]["implementation"],
        *(
            a["implementation"]
            for a in roles["planner"].get("alternatives", [])
        ),
    }
    assert named == set(scenario_parameters.PLANNERS)


def test_bringup_1080_every_system_and_acceptance_test_has_an_owner() -> None:
    """BRINGUP-1080: the reference-stack map and the tier-3/4 tests agree.

    `check.sh --ready` runs only the tests the map selects (ADR 0007), so a
    test no implementation claims, a claim on a test that does not exist, or
    a Robot `impl:` tag that disagrees with the claims is a test the scoped
    gate skips or runs for the wrong reason. `load_stack` also raises on a
    test it cannot discover (no ID in the name or the documentation, or a
    duplicate), which fails this test the same way.
    """
    import importlib.util
    import sys

    module_path = REPOSITORY_ROOT / "scripts" / "changed_scope.py"
    spec = importlib.util.spec_from_file_location(
        "racing_changed_scope", module_path
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)

    stack = module.load_stack(REPOSITORY_ROOT)

    assert [i.key for i in stack.implementations if not i.verified_by] == []
    discovered = set(stack.system_tests) | set(stack.acceptance_tags)
    claimed = {t for i in stack.implementations for t in i.verified_by}
    assert sorted(discovered - claimed - {stack.reference_lap}) == []
    assert sorted(claimed - discovered) == []
    for test_id, tags in stack.acceptance_tags.items():
        expected = {
            i.tag for i in stack.implementations if test_id in i.verified_by
        }
        assert tags == expected, test_id


@pytest.mark.parametrize("vehicle_file", VEHICLE_FILES, ids=lambda p: p.stem)
def test_bringup_1050_the_sim_casts_scan_from_the_urdf_laser(
    vehicle_file: Path,
) -> None:
    """BRINGUP-1050: the sim's laser mount is the URDF's rear_axle_to_laser.

    /scan is stamped in frame `laser` and every estimator places its beams
    through TF rear_axle -> laser, which robot_state_publisher takes from the
    URDF. The vehicle file states the mount from the rear axle (the frame
    every source measures the lidar from: AutoDRIVE roboracer_1, the
    f1tenth_simulator scan_distance_to_base_link); the sim converts it to
    the CoG by the gym's lr, which SIMJAX-1140 pins to the URDF, so yaml,
    URDF and cast are one chain.
    Two copies of one geometry number drift silently (repo-gotchas #15's
    shape): a mismatch shifts every scan point by the difference, and reads
    as localization error, never as a fault. The sim is planar, so a mount
    it cannot represent (roll, pitch) fails here too.
    """
    import xml.etree.ElementTree as ElementTree

    import yaml

    urdf = REPOSITORY_ROOT / (
        "ros_ws/src/racing_vehicle_description/urdf/f1tenth.urdf"
    )
    joint = (
        ElementTree.parse(urdf)
        .getroot()
        .find("./joint[@name='rear_axle_to_laser']")
    )
    assert joint is not None, "no joint rear_axle_to_laser"
    assert joint.find("parent").attrib["link"] == "rear_axle"
    origin = joint.find("origin")
    assert origin is not None
    x, y, _z = (float(value) for value in origin.attrib["xyz"].split())
    roll, pitch, yaw = (float(value) for value in origin.attrib["rpy"].split())
    assert (roll, pitch) == (0.0, 0.0)

    mount = yaml.safe_load(vehicle_file.read_text(encoding="utf-8"))[
        "racing_sim"
    ]["ros__parameters"]
    assert (mount["laser_x"], mount["laser_y"], mount["laser_yaw"]) == (
        x,
        y,
        yaw,
    )

    # ...and the launch hands the sim that file, or the mount never arrives
    # and the sim casts from the rear axle.
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    sim_node = next(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == "sim_node"
    )
    parameters = next(
        keyword.value
        for keyword in sim_node.keywords
        if keyword.arg == "parameters"
    )
    assert any(
        isinstance(element, ast.Name) and element.id == "vehicle"
        for element in parameters.elts
    )


def _launch_context(**overrides: str):
    """The launch module and a LaunchContext holding every declared
    argument's default, then `overrides`. Paths are cwd-relative."""
    import importlib.util

    from launch import LaunchContext

    spec = importlib.util.spec_from_file_location(
        "racing_sim_pure_pursuit_launch", LAUNCH_FILE
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if hasattr(entity, "default_value"):
            context.launch_configurations[entity.name] = "".join(
                part.text for part in entity.default_value
            )
    context.launch_configurations.update(overrides)
    return module, context


def test_bringup_1090_the_vehicle_is_a_launch_argument(monkeypatch) -> None:
    """BRINGUP-1090: `vehicle:=` selects the vehicle file for every node that
    reads one, defaults to today's file, and a missing file is an error.

    Six nodes take the vehicle file. A node left on the constant would run
    one vehicle's caps beside another's, which reads as a plausible lap and
    nothing else.
    """
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    constants = {
        node.targets[0].id: node.value.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and isinstance(node.targets[0], ast.Name)
        and isinstance(node.value, ast.Constant)
    }
    assert constants["VEHICLE_CONFIG"] == "config/vehicles/f1tenth_default.yaml"

    declared = {
        call.args[0].value: call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
    }
    default = next(
        keyword.value
        for keyword in declared["vehicle"].keywords
        if keyword.arg == "default_value"
    )
    assert isinstance(default, ast.Name) and default.id == "VEHICLE_CONFIG"

    takes_vehicle = set()
    for call in ast.walk(tree):
        if not (
            isinstance(call, ast.Call)
            and getattr(call.func, "id", "") == "Node"
        ):
            continue
        parameters = next(
            (k.value for k in call.keywords if k.arg == "parameters"), None
        )
        names = {
            element.id
            for element in getattr(parameters, "elts", [])
            if isinstance(element, ast.Name)
        }
        assert "VEHICLE_CONFIG" not in names
        if "vehicle" in names:
            takes_vehicle.add(
                next(k.value.value for k in call.keywords if k.arg == "package")
            )
    assert takes_vehicle == {
        "racing_sim_gym_jax",
        "racing_bringup",
        "racing_controller_baseline",
        "racing_safety_supervisor",
        "racing_metrics",
        "racing_recording",
    }

    # A file that does not exist raises first, whatever the other arguments
    # hold. The launch's paths are cwd-relative.
    monkeypatch.chdir(REPOSITORY_ROOT)
    module, context = _launch_context(vehicle="config/vehicles/no_such.yaml")
    with pytest.raises(RuntimeError, match="no_such.yaml"):
        module._launch_nodes(context, "", "")


def test_bringup_1100_every_vehicle_file_covers_the_defaults_keys() -> None:
    """BRINGUP-1100: a vehicle file has every section and key the default has.

    A key missing from a file silently falls back to the node's
    `declare_parameter` default (the supervisor's 2.0 m/s^2, say) while the
    file claims a different vehicle.
    """
    import yaml

    def sections(path: Path) -> dict:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        return {
            node: set(body["ros__parameters"])
            for node, body in document.items()
        }

    default = sections(VEHICLE_CONFIG)
    versions = []
    for path in VEHICLE_FILES:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
        assert sections(path) == default, path.name
        metrics = document["racing_metrics"]["ros__parameters"]
        recording = document["racing_recording"]["ros__parameters"]
        assert (
            metrics["vehicle_parameter_version"]
            == recording["vehicle_parameter_version"]
        ), path.name
        versions.append(metrics["vehicle_parameter_version"])
    assert len(set(versions)) == len(versions)


def test_bringup_1060_the_scenario_selects_the_occupancy_grid() -> None:
    """BRINGUP-1060: the occupancy grid nav2_amcl localizes against is the
    one that belongs to the scenario's own map, and it exists.

    The same trap as BRINGUP-1010, one artifact further out: AMCL matching
    scans against another track's walls does not fail, it diverges - and a
    divergence is exactly what a localization test is meant to be able to
    attribute. The grid is a committed survey of the scenario's map
    (scripts/map_track.sh), so the path is derived from the scenario rather
    than hand-set anywhere.
    """
    from racing_bringup.scenario_parameters import track_parameters

    spielberg = track_parameters(
        REPOSITORY_ROOT / "config" / "scenarios" / "spielberg.yaml"
    )
    grid = (
        REPOSITORY_ROOT
        / "config"
        / "scenarios"
        / "maps"
        / "Spielberg"
        / "slam"
        / "Spielberg.yaml"
    )
    assert spielberg["map_server"]["yaml_filename"] == str(grid)
    assert grid.is_file(), "the committed occupancy grid is missing"


SPIELBERG_SCENARIO = REPOSITORY_ROOT / "config" / "scenarios" / "spielberg.yaml"
DEFAULT_VEHICLE = (
    REPOSITORY_ROOT / "config" / "vehicles" / "f1tenth_default.yaml"
)
GYM_LIMITS_VEHICLE = (
    REPOSITORY_ROOT / "config" / "vehicles" / "f1tenth_gym_limits.yaml"
)


@pytest.mark.parametrize(
    ("vehicle", "version"),
    [
        (DEFAULT_VEHICLE, "f1tenth-default-v1"),
        (GYM_LIMITS_VEHICLE, "f1tenth-gym-limits-v1"),
    ],
)
def test_bringup_1150_the_planner_selects_the_committed_raceline(
    vehicle: Path, version: str
) -> None:
    """BRINGUP-1150: `planner=optimized_raceline` loads
    `config/racelines/<track>/<vehicle version>/<optimizer>.csv`, whichever
    vehicle is named; the default planner still loads the scenario's own
    `<map>_raceline.csv`, byte for byte as before.
    """
    from racing_bringup.scenario_parameters import track_parameters

    expected = (
        REPOSITORY_ROOT
        / "config"
        / "racelines"
        / "spielberg"
        / version
        / "tum_mincurv_iqp.csv"
    )
    optimized = track_parameters(
        SPIELBERG_SCENARIO,
        planner="optimized_raceline",
        raceline_optimizer="tum_mincurv_iqp",
        vehicle_path=vehicle,
    )
    assert optimized["racing_bringup_support"]["raceline_path"] == str(expected)

    legacy = track_parameters(SPIELBERG_SCENARIO, vehicle_path=vehicle)
    assert legacy == track_parameters(SPIELBERG_SCENARIO)
    assert legacy["racing_bringup_support"]["raceline_path"].endswith(
        "maps/Spielberg/Spielberg_raceline.csv"
    )


def _raceline_world(tmp_path: Path) -> tuple[Path, Path]:
    """A copy of the Spielberg config tree small enough to corrupt one field
    of: scenario, track, vehicle and the default vehicle's raceline."""
    import shutil

    config = tmp_path / "config"
    for name in ("scenarios", "tracks", "vehicles"):
        (config / name).mkdir(parents=True)
    shutil.copy(SPIELBERG_SCENARIO, config / "scenarios")
    shutil.copy(
        REPOSITORY_ROOT / "config" / "tracks" / "spielberg.yaml",
        config / "tracks",
    )
    shutil.copy(DEFAULT_VEHICLE, config / "vehicles")
    target = config / "racelines" / "spielberg" / "f1tenth-default-v1"
    target.mkdir(parents=True)
    source = (
        REPOSITORY_ROOT
        / "config"
        / "racelines"
        / "spielberg"
        / "f1tenth-default-v1"
    )
    for suffix in (".csv", ".yaml"):
        shutil.copy(source / f"tum_mincurv_iqp{suffix}", target)
    return config / "scenarios" / "spielberg.yaml", config / "vehicles" / (
        "f1tenth_default.yaml"
    )


def _resolve(scenario: Path, vehicle: Path | None, **keywords):
    from racing_bringup.scenario_parameters import track_parameters

    return track_parameters(
        scenario,
        planner=keywords.pop("planner", "optimized_raceline"),
        vehicle_path=vehicle,
        **keywords,
    )


def test_bringup_1160_the_resolver_fails_closed(tmp_path: Path) -> None:
    """BRINGUP-1160: a raceline that is not exactly the one asked for is an
    error, never the centerline. The scenario, vehicle and sidecar can each
    disagree with the file name; every disagreement raises.
    """
    import yaml

    scenario, vehicle = _raceline_world(tmp_path)
    sidecar = (
        scenario.parents[1]
        / "racelines"
        / "spielberg"
        / ("f1tenth-default-v1/tum_mincurv_iqp.yaml")
    )
    # The unmodified copy resolves, so every raise below is the one change.
    assert _resolve(scenario, vehicle)

    with pytest.raises(ValueError, match="planner"):
        _resolve(scenario, vehicle, planner="no_such_planner")
    with pytest.raises(ValueError, match="raceline_optimizer"):
        _resolve(scenario, vehicle, raceline_optimizer="no_such_optimizer")
    # An unknown optimizer is an error under the default planner as well, so
    # a typo never passes silently.
    with pytest.raises(ValueError, match="raceline_optimizer"):
        _resolve(
            scenario,
            vehicle,
            planner="track_centerline",
            raceline_optimizer="no_such_optimizer",
        )
    with pytest.raises(ValueError, match="vehicle"):
        _resolve(scenario, None)

    original = yaml.safe_load(sidecar.read_text(encoding="utf-8"))
    for key, value in (
        ("track", "monza"),
        ("vehicle_version", "f1tenth-gym-limits-v1"),
        ("optimizer", "tum_mintime"),
    ):
        sidecar.write_text(
            yaml.safe_dump(dict(original, **{key: value})), encoding="utf-8"
        )
        with pytest.raises(ValueError, match=key):
            _resolve(scenario, vehicle)
    sidecar.write_text(
        yaml.safe_dump(dict(original, verdict={"ok": False})), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="verdict"):
        _resolve(scenario, vehicle)
    sidecar.write_text(yaml.safe_dump(original), encoding="utf-8")

    vehicle_document = yaml.safe_load(vehicle.read_text(encoding="utf-8"))
    without = {name: {"ros__parameters": {}} for name in ("racing_metrics",)}
    vehicle.write_text(yaml.safe_dump(without), encoding="utf-8")
    with pytest.raises(ValueError, match="vehicle_parameter_version"):
        _resolve(scenario, vehicle)
    disagreeing = yaml.safe_load(yaml.safe_dump(vehicle_document))
    disagreeing["racing_recording"]["ros__parameters"][
        "vehicle_parameter_version"
    ] = "f1tenth-gym-limits-v1"
    vehicle.write_text(yaml.safe_dump(disagreeing), encoding="utf-8")
    with pytest.raises(ValueError, match="vehicle_parameter_version"):
        _resolve(scenario, vehicle)
    vehicle.write_text(yaml.safe_dump(vehicle_document), encoding="utf-8")

    # No committed raceline for the track: the circle has none.
    contract = REPOSITORY_ROOT / "config" / "scenarios" / "contract_test.yaml"
    with pytest.raises(FileNotFoundError, match="analytic_circle"):
        _resolve(contract, DEFAULT_VEHICLE)


def _node_parameters(context, node) -> list:
    """A launch_ros Node's parameters, decoded.

    launch_ros has no public accessor; Jazzy keeps the normalized list as
    `node._Node__parameters` (launch_ros/actions/node.py), whose dict keys and
    string values are tuples of Substitutions.
    """
    import yaml
    from launch.utilities import perform_substitutions

    def decode(value):
        if isinstance(value, tuple) and all(
            hasattr(v, "perform") for v in value
        ):
            # launch_ros stores a string as its YAML text: "x\n...\n".
            return yaml.safe_load(perform_substitutions(context, list(value)))
        return value

    decoded = []
    for entry in node._Node__parameters:
        if isinstance(entry, dict):
            entry = {decode(key): decode(value) for key, value in entry.items()}
        decoded.append(entry)
    return decoded


def _merged(context, node) -> dict:
    merged: dict = {}
    for entry in _node_parameters(context, node):
        if isinstance(entry, dict):
            merged.update(entry)
    return merged


def _launched(context, module, package: str) -> dict:
    nodes = [
        node
        for node in module._launch_nodes(context, "", "")
        if getattr(node, "_Node__package", None) == package
    ]
    assert len(nodes) == 1
    return _merged(context, nodes[0])


def test_bringup_1170_the_launch_selects_the_planner_and_speed_source(
    monkeypatch,
) -> None:
    """BRINGUP-1170: `planner:=`, `raceline_optimizer:=` and `speed_source:=`
    reach the nodes that act on them, default to today's behaviour, and an
    unknown value is an error rather than a silent centerline.
    """
    monkeypatch.chdir(REPOSITORY_ROOT)
    scenario = "config/scenarios/spielberg.yaml"

    module, context = _launch_context(scenario=scenario)
    support = _launched(context, module, "racing_bringup")
    controller = _launched(context, module, "racing_controller_baseline")
    assert support["raceline_path"].endswith(
        "maps/Spielberg/Spielberg_raceline.csv"
    )
    assert controller.get("speed_source") == "curvature_schedule"

    module, context = _launch_context(
        scenario=scenario,
        planner="optimized_raceline",
        raceline_optimizer="tum_mincurv_iqp",
        speed_source="trajectory",
    )
    support = _launched(context, module, "racing_bringup")
    controller = _launched(context, module, "racing_controller_baseline")
    assert support["raceline_path"] == str(
        REPOSITORY_ROOT
        / "config"
        / "racelines"
        / "spielberg"
        / "f1tenth-default-v1"
        / "tum_mincurv_iqp.csv"
    )
    assert controller.get("speed_source") == "trajectory"

    for overrides in (
        {"planner": "no_such_planner"},
        {"raceline_optimizer": "no_such_optimizer"},
        {"speed_source": "no_such_source"},
        # No committed raceline for the circle: never the centerline instead.
        {"planner": "optimized_raceline"},
    ):
        module, context = _launch_context(**overrides)
        with pytest.raises(RuntimeError):
            module._launch_nodes(context, "", "")
