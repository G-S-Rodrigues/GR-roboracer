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


def test_bringup_1530_launch_pose_sources_are_the_reference_ones() -> None:
    """BRINGUP-1530: the `pose_source` values the launch accepts are exactly
    the implementations `reference_stack.yaml` names for that role (the
    default plus its alternatives). A guard: it passes when the two agree
    and names the difference when one gains an estimator the other lacks."""
    import yaml

    roles = yaml.safe_load(
        (REPOSITORY_ROOT / "config" / "reference_stack.yaml").read_text(
            encoding="utf-8"
        )
    )["roles"]
    pose_source = roles["pose_source"]
    named = {
        pose_source["implementation"],
        *(a["implementation"] for a in pose_source.get("alternatives", [])),
    }
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    configs = next(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == "POSE_SOURCE_CONFIGS"
    )
    accepted = {
        key.value for key in configs.keys if isinstance(key, ast.Constant)
    } | {pose_source["implementation"]}
    assert accepted == named, (
        f"launch only: {sorted(accepted - named)}; "
        f"reference_stack only: {sorted(named - accepted)}"
    )


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


def _effective(vehicle: Path, forwarded: dict, name: str):
    """What the controller sees: the vehicle file's value, overlaid by what
    the launch forwards after it (launch parameters apply in list order)."""
    import yaml

    document = yaml.safe_load(vehicle.read_text(encoding="utf-8"))
    values = document["racing_controller_baseline"]["ros__parameters"]
    return {**values, **forwarded}.get(name)


def test_bringup_1170_the_launch_selects_the_planner_and_speed_source(
    monkeypatch, tmp_path
) -> None:
    """BRINGUP-1170: `planner:=`, `raceline_optimizer:=` and `speed_source:=`
    reach the nodes that act on them, default to today's behaviour, and an
    unknown value is an error rather than a silent centerline. `speed_source`
    is forwarded only when given: argument > vehicle file > node default.
    """
    monkeypatch.chdir(REPOSITORY_ROOT)
    scenario = "config/scenarios/spielberg.yaml"

    module, context = _launch_context(scenario=scenario)
    support = _launched(context, module, "racing_bringup")
    controller = _launched(context, module, "racing_controller_baseline")
    assert support["raceline_path"].endswith(
        "maps/Spielberg/Spielberg_raceline.csv"
    )
    # Precedence: a given argument > the vehicle file > the node default.
    # Not given, nothing is forwarded, so the node default applies.
    assert "speed_source" not in controller

    # A vehicle file may set speed_source; with no argument it is preserved.
    import yaml

    document = yaml.safe_load(DEFAULT_VEHICLE.read_text(encoding="utf-8"))
    document["racing_controller_baseline"]["ros__parameters"][
        "speed_source"
    ] = "trajectory"
    vehicle = tmp_path / "vehicle_trajectory.yaml"
    vehicle.write_text(yaml.safe_dump(document), encoding="utf-8")
    module, context = _launch_context(scenario=scenario, vehicle=str(vehicle))
    controller = _launched(context, module, "racing_controller_baseline")
    assert "speed_source" not in controller
    assert _effective(vehicle, controller, "speed_source") == "trajectory"

    # A given argument wins over the vehicle file.
    module, context = _launch_context(
        scenario=scenario,
        vehicle=str(vehicle),
        speed_source="curvature_schedule",
    )
    controller = _launched(context, module, "racing_controller_baseline")
    assert (
        _effective(vehicle, controller, "speed_source") == "curvature_schedule"
    )

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


MOLA_CONFIGURATION = (
    REPOSITORY_ROOT / "config" / "localization" / "mola_lidar2d.yaml"
)


def test_bringup_1520_mola_runs_localization_only_on_the_derived_map() -> None:
    """BRINGUP-1520: `pose_source:=mola` starts MOLA-LO's lidar2d pipeline in
    localization-only mode on the scenario's own committed map, headless, on
    simulated time, with its frames renamed so that it owns exactly
    `mola_map -> odom` and a static `map -> mola_map` (the provenance's T)
    puts that into the world frame. No value reads `/ground_truth` (AC6), and
    describing the nodes starts no process."""
    import yaml
    from racing_bringup.estimators import (
        mola,
        mola_alignment_arguments,
        mola_launch_arguments,
        mola_map_files,
    )
    from racing_bringup.scenario_parameters import track_parameters

    grid = track_parameters(SPIELBERG_SCENARIO)["map_server"]["yaml_filename"]
    mm, provenance = mola_map_files(grid)
    arguments = mola_launch_arguments(str(MOLA_CONFIGURATION), mm)

    assert arguments["lidar_topic_name"] == "/scan"
    assert arguments["lidar_topic_type"] == "LaserScan"
    assert Path(arguments["mola_lo_pipeline"]).is_file()
    assert Path(arguments["mola_lo_pipeline"]).name == (
        "mola_lidar2d_localization_pipeline.yaml"
    )
    assert arguments["start_mapping_enabled"] == "False"
    assert arguments["use_rviz"] == "False"
    assert arguments["use_mola_gui"] == "False"
    assert arguments["use_sim_time"] == "True"
    assert arguments["mola_state_estimator_reference_frame"] == "mola_map"
    assert arguments["mola_lo_reference_frame"] == "mola_map"
    assert arguments["mola_initial_map_mm_file"] == mm
    assert all("ground_truth" not in str(v) for v in arguments.values())

    transform = yaml.safe_load(Path(provenance).read_text(encoding="utf-8"))[
        "T_map_mola"
    ]
    assert mola_alignment_arguments(provenance) == [
        "--x",
        str(transform["x"]),
        "--y",
        str(transform["y"]),
        "--z",
        str(transform["z"]),
        "--yaw",
        str(transform["yaw"]),
        "--frame-id",
        "map",
        "--child-frame-id",
        "mola_map",
    ]

    static, include = mola(str(MOLA_CONFIGURATION), grid)
    assert static.node_package == "tf2_ros"
    assert static.node_executable == "static_transform_publisher"
    assert type(include).__name__ == "IncludeLaunchDescription"


def test_bringup_1570_warns_when_mola_is_driven_on_its_own_estimate(
    monkeypatch,
) -> None:
    """BRINGUP-1570: `pose_source:=mola` with `drive_on_estimate:=true` (the
    launch default) is launched, not refused, with a warning: MOLA is backed
    open loop only, and the controller would latch a stop before the hold
    releases (repo-gotchas #19). Open loop and the other sources say nothing."""
    monkeypatch.chdir(REPOSITORY_ROOT)

    def warnings_for(**overrides) -> list[str]:
        module, context = _launch_context(
            scenario="config/scenarios/spielberg_1080.yaml", **overrides
        )
        recorded: list[str] = []

        class Recorder:
            def warning(self, message, *args, **kwargs):
                recorded.append(str(message))

        monkeypatch.setattr(module, "LOGGER", Recorder())
        module._launch_nodes(context, "", "")
        return recorded

    closed = warnings_for(pose_source="mola", drive_on_estimate="true")
    assert len(closed) == 1 and "drive_on_estimate" in closed[0]
    assert warnings_for(pose_source="mola", drive_on_estimate="false") == []
    assert warnings_for(pose_source="amcl", drive_on_estimate="true") == []


SLAM_TOOLBOX_64 = (
    REPOSITORY_ROOT / "config" / "localization" / "slam_toolbox_online.yaml"
)


def _slam_toolbox_configuration(module, context) -> Path:
    """The parameter file the launch hands slam_toolbox.

    `_launched` keeps dict entries only, so a file path is invisible to it:
    read it from the node's own parameter list instead.
    """
    from launch.utilities import perform_substitutions

    nodes = [
        node
        for node in module._launch_nodes(context, "", "")
        if getattr(node, "_Node__package", None) == "slam_toolbox"
    ]
    assert len(nodes) == 1
    files = []
    for entry in _node_parameters(context, nodes[0]):
        if isinstance(entry, dict):
            continue
        if hasattr(entry, "evaluate"):
            entry = entry.evaluate(context)
        elif isinstance(entry, tuple):
            entry = perform_substitutions(context, list(entry))
        files.append(Path(entry).resolve())
    assert len(files) == 1, files
    return files[0]


def test_bringup_1800_slam_toolbox_config_is_a_launch_argument(
    monkeypatch, tmp_path
) -> None:
    """BRINGUP-1800: `slam_toolbox_config:=<file>` is the parameter file
    slam_toolbox loads, empty means the one the launch chooses, and a wrong
    value is an error rather than a fallback.

    The argument exists so the 64-beam file stays runnable on any scenario
    (ADR 0005), so a car with no scenario can name its own, and so a variant
    can be measured without editing a tracked file. It is resolved only for
    `pose_source:=slam_toolbox`: given to another source it would be ignored
    without a word.
    """
    import shutil

    monkeypatch.chdir(REPOSITORY_ROOT)
    scenario = "config/scenarios/spielberg.yaml"

    module, context = _launch_context(
        scenario=scenario, pose_source="slam_toolbox"
    )
    assert _slam_toolbox_configuration(module, context) == SLAM_TOOLBOX_64

    copy = tmp_path / "variant.yaml"
    shutil.copy(SLAM_TOOLBOX_64, copy)
    module, context = _launch_context(
        scenario=scenario,
        pose_source="slam_toolbox",
        slam_toolbox_config=str(copy),
    )
    assert _slam_toolbox_configuration(module, context) == copy.resolve()

    missing = tmp_path / "no_such_file.yaml"
    module, context = _launch_context(
        scenario=scenario,
        pose_source="slam_toolbox",
        slam_toolbox_config=str(missing),
    )
    with pytest.raises(RuntimeError, match="no_such_file.yaml"):
        module._launch_nodes(context, "", "")

    module, context = _launch_context(
        scenario=scenario,
        pose_source="amcl",
        slam_toolbox_config=str(copy),
    )
    with pytest.raises(RuntimeError, match="slam_toolbox_config"):
        module._launch_nodes(context, "", "")
