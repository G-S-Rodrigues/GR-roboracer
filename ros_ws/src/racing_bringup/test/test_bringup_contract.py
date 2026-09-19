"""Structural contract for the complete Pure Pursuit bringup graph."""

import ast
from pathlib import Path

PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "sim_pure_pursuit.launch.py"
VEHICLE_CONFIG = (
    REPOSITORY_ROOT / "config" / "vehicles" / "f1tenth_default.yaml"
)


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
        ("racing_recording", "racing_recording_node"),
        ("racing_bringup", "racing_bringup_support"),
        ("robot_state_publisher", "robot_state_publisher"),
        ("rviz2", "rviz2"),
    }.issubset(nodes)


def test_launch_exposes_reproducibility_and_headless_controls() -> None:
    """Scenario, seed, recording and RViz are launch-time controls."""
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    for argument in ("scenario", "seed", "recording_path", "use_rviz"):
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
        '"/odom"',
        '"map"',
        '"base_link"',
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
