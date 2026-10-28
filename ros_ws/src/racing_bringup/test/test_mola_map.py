"""The committed MOLA map and its provenance (part 1 of the MOLA plan)."""

import hashlib
import re
from pathlib import Path

import pytest
import yaml

PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
SCENARIOS = REPOSITORY_ROOT / "config" / "scenarios"
MOLA_DIR = SCENARIOS / "maps" / "Spielberg" / "mola"
MOLA_PREF = REPOSITORY_ROOT / "docker" / "apt-preferences" / "mola.pref"
# SIM-3510 localizes at this seed; the map must not share its noise draw.
LOCALIZATION_SEED = 42
REQUIRED_KEYS = (
    "command",
    "scenario",
    "seed",
    "time_scale",
    "mola_version",
    "mp2p_icp_version",
    "T_map_mola",
    "T_source",
    "mm_sha256",
    "mm_bytes",
)


def _pinned_version(package: str) -> str:
    """The version `mola.pref` pins `package` to, as a 'x.y.z' prefix."""
    for stanza in MOLA_PREF.read_text().split("\n\n"):
        names = re.search(r"^Package: (.*)$", stanza, re.MULTILINE)
        pin = re.search(r"^Pin: version (\S+)$", stanza, re.MULTILINE)
        if names and pin and package in names.group(1).split():
            return pin.group(1).split("-")[0]
    raise AssertionError(f"{package} has no stanza in mola.pref")


def test_bringup_1500_the_scenario_selects_its_mola_map() -> None:
    """BRINGUP-1500: the MOLA map is derived from the scenario's own grid
    (maps/<map>/slam/<map>.yaml -> maps/<map>/mola/<map>.{mm,yaml}), so a
    run can only be localized on its own track, and both files exist. A map
    that was never built says which script builds it."""
    from racing_bringup.estimators import mola_map_files
    from racing_bringup.scenario_parameters import track_parameters

    grid = track_parameters(SCENARIOS / "spielberg.yaml")["map_server"][
        "yaml_filename"
    ]
    mm, provenance = mola_map_files(grid)
    assert Path(mm) == MOLA_DIR / "Spielberg.mm"
    assert Path(provenance) == MOLA_DIR / "Spielberg.yaml"
    assert Path(mm).is_file() and Path(provenance).is_file()

    with pytest.raises(FileNotFoundError, match="mola_map_track.sh"):
        mola_map_files(str(SCENARIOS / "maps" / "Nowhere" / "slam" / "x.yaml"))


def test_bringup_1510_the_provenance_is_complete_and_true_to_the_files() -> (
    None
):
    """BRINGUP-1510: the provenance yaml names how the map was made and
    agrees with the committed .mm and with the pinned MOLA build."""
    provenance = yaml.safe_load((MOLA_DIR / "Spielberg.yaml").read_text())
    missing = [key for key in REQUIRED_KEYS if key not in provenance]
    assert not missing, f"provenance lacks {missing}"

    mm = (MOLA_DIR / "Spielberg.mm").read_bytes()
    assert provenance["mm_sha256"] == hashlib.sha256(mm).hexdigest()
    assert provenance["mm_bytes"] == len(mm)

    # A pin bump without regenerating the map fails here.
    assert provenance["mola_version"] == _pinned_version(
        "ros-jazzy-mola-lidar-odometry"
    )
    assert provenance["mp2p_icp_version"] == _pinned_version(
        "ros-jazzy-mp2p-icp"
    )

    transform = provenance["T_map_mola"]
    assert set(transform) == {"x", "y", "z", "yaw"}
    if provenance["T_source"] == "identity_no_ground_truth":
        assert all(transform[key] == 0 for key in transform)
    else:
        assert provenance["T_source"] == "first_processed_scan"

    assert provenance["seed"] != LOCALIZATION_SEED
    assert not provenance["scenario"].endswith("_survey")
    assert (REPOSITORY_ROOT / provenance["scenario"]).is_file()


def test_bringup_1540_the_mapping_script_never_fails_silently() -> None:
    """Under `set -e` a bare failing test left the script exiting with no
    message after the lap (seen once). Every exit names its line, and the
    shutdown wait follows MOLA's own pid rather than a process-name match."""
    script = (REPOSITORY_ROOT / "scripts" / "mola_map_track.sh").read_text()
    assert re.search(r"^trap 'echo .*LINENO.*' ERR$", script, re.MULTILINE)
    assert 'wait "$mola"' in script
    assert "pgrep" not in script
    # `ros2 launch` was measured to leave mola-cli running for 15 minutes
    # after a SIGINT (it never saved the simplemap): the signal goes to
    # mola-cli itself, which saves on SIGINT.
    assert "pkill -INT -f lib/mola_launcher/mola-cli" in script
    # The recording is read for T only after the stack that writes it has
    # exited: read while it ran, its last line was cut mid-write and the
    # provenance step died on a JSONDecodeError.
    assert script.index('wait "$launch"') < script.index('mola_mapping.jsonl"')
    # Under `set -e` a failing kill ended the EXIT trap before its pkills ran
    # and left the stack running (gotcha #10); SIGINT to a background job is
    # ignored, so the launch is stopped with SIGTERM.
    assert 'kill -INT "$launch"' not in script
    assert 'kill -TERM "$mola" "$launch" 2>/dev/null || true' in script
    # The stack is stopped mid-write, so the recording's last line is cut.
    assert "json.JSONDecodeError" in script
    # The metrics subscriber starts while the sim is held, not after t=0.
    assert script.index("topic echo") < script.index(
        "ros2 service call /racing_sim/reset"
    )


def test_bringup_1550_the_alignment_publisher_is_killable_by_name() -> None:
    """The stock `static_transform_publisher` lives under /opt/ros, so
    `pkill -f install/racing_` never matched it and it outlived the stack
    (gotcha #10). It carries a racing_ node name, which is on its command
    line, and the pattern is in the documented kill lists."""
    from racing_bringup.estimators import mola
    from racing_bringup.scenario_parameters import track_parameters

    scenario = REPOSITORY_ROOT / "config/scenarios/spielberg.yaml"
    grid = track_parameters(str(scenario))["map_server"]["yaml_filename"]
    configuration = REPOSITORY_ROOT / "config/localization/mola_lidar2d.yaml"
    static, _ = mola(str(configuration), grid)
    name = static._Node__node_name  # not expanded before launch
    assert name == "racing_mola_map_alignment"
    script = (REPOSITORY_ROOT / "scripts" / "mola_map_track.sh").read_text()
    assert "pkill -f racing_mola_map_alignment" in script
    gotchas = (REPOSITORY_ROOT / "docs/agents/repo-gotchas.md").read_text()
    assert "racing_mola_map_alignment" in gotchas


def test_bringup_1560_localization_pipeline_differs_by_two_values() -> None:
    """The mapping pipeline made the committed map and stays unchanged; the
    localization copy narrows ICP's search window and nothing else, so the
    map keeps coming from the committed mapping pipeline."""

    def values(name: str) -> list[str]:
        text = (REPOSITORY_ROOT / "config" / "localization" / name).read_text()
        lines = (line.split(" #")[0].rstrip() for line in text.splitlines())
        return [
            line
            for line in lines
            if line.strip() and not line.lstrip().startswith("#")
        ]

    mapping = values("mola_lidar2d_pipeline.yaml")
    localization = values("mola_lidar2d_localization_pipeline.yaml")
    assert len(mapping) == len(localization)
    differing = [
        (a.strip(), b.strip())
        for a, b in zip(mapping, localization, strict=True)
        if a != b
    ]
    assert differing == [
        ("maximum_sigma: 1.0", "maximum_sigma: 0.5"),
        (
            'threshold: "2.0*ADAPTIVE_THRESHOLD_SIGMA"',
            'threshold: "1.0*ADAPTIVE_THRESHOLD_SIGMA"',
        ),
    ]


MAPPING_CONFIGURATION = (
    REPOSITORY_ROOT / "config" / "localization" / "mola_lidar2d_mapping.yaml"
)


def test_bringup_1710_mapping_arguments_name_no_initial_map() -> None:
    """BRINGUP-1710: `mola_launch_arguments(cfg, None)` is the mapping form: it
    has no `mola_initial_map_mm_file` (a mapping lap starts from nothing), and
    the localization form (cfg, mm) is unchanged."""
    from racing_bringup.estimators import mola_launch_arguments

    mapping = mola_launch_arguments(str(MAPPING_CONFIGURATION), None)
    assert "mola_initial_map_mm_file" not in mapping
    assert mapping["start_mapping_enabled"] == "True"
    localization = mola_launch_arguments(
        str(REPOSITORY_ROOT / "config/localization/mola_lidar2d.yaml"),
        "/x/m.mm",
    )
    assert localization["mola_initial_map_mm_file"] == "/x/m.mm"


def test_bringup_1700_mola_mapping_is_the_mapping_yaml_plus_an_output() -> None:
    """BRINGUP-1700: `mola_mapping` includes MOLA-LO's launch file with the
    mapping yaml's arguments (mapping on, simplemap generated, MOLA's frames
    renamed, simulated time) and sets MOLA_SIMPLEMAP_OUTPUT, the environment
    variable the pipeline reads, to the given path. No value reads
    `/ground_truth` (acceptance 6)."""
    from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
    from racing_bringup.estimators import mola_mapping

    actions = mola_mapping(str(MAPPING_CONFIGURATION), "/x/out.simplemap")
    environment = [a for a in actions if isinstance(a, SetEnvironmentVariable)]
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    assert len(actions) == 2 and len(environment) == 1 and len(includes) == 1
    assert environment[0].name[0].text == "MOLA_SIMPLEMAP_OUTPUT"
    assert environment[0].value[0].text == "/x/out.simplemap"
    arguments = dict(includes[0].launch_arguments)
    assert arguments["start_mapping_enabled"] == "True"
    assert arguments["generate_simplemap"] == "True"
    assert arguments["mola_bridge_odometry_frame"] == "mola_odom"
    assert arguments["use_sim_time"] == "True"
    assert "mola_initial_map_mm_file" not in arguments
    assert all("ground_truth" not in v for v in arguments.values())


MAPPING_LAUNCH = PACKAGE_ROOT / "launch" / "mola_mapping.launch.py"


def test_bringup_1720_the_mapping_launch_starts_only_mola() -> None:
    """BRINGUP-1720: `mola_mapping.launch.py` declares `simplemap_output` and
    `configuration`, and what it starts is exactly the output variable and
    MOLA's include: no stack, so the script's SIGINT-to-mola-cli shutdown
    owns a process group of one launch."""
    import importlib.util

    from launch import LaunchContext
    from launch.actions import (
        DeclareLaunchArgument,
        IncludeLaunchDescription,
        OpaqueFunction,
        SetEnvironmentVariable,
    )

    assert MAPPING_LAUNCH.is_file(), f"{MAPPING_LAUNCH} does not exist"
    spec = importlib.util.spec_from_file_location(
        "mola_mapping", MAPPING_LAUNCH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities
    declared = {
        e.name for e in entities if isinstance(e, DeclareLaunchArgument)
    }
    assert declared == {"simplemap_output", "configuration"}
    (opaque,) = [e for e in entities if isinstance(e, OpaqueFunction)]
    context = LaunchContext()
    context.launch_configurations["simplemap_output"] = "/x/out.simplemap"
    context.launch_configurations["configuration"] = str(MAPPING_CONFIGURATION)
    actions = opaque.execute(context)
    assert [type(a) for a in actions] == [
        SetEnvironmentVariable,
        IncludeLaunchDescription,
    ]
    text = MAPPING_LAUNCH.read_text()
    assert "sim_pure_pursuit" not in text.replace("sim_pure_pursuit.launch", "")
    assert "config/localization/mola_lidar2d_mapping.yaml" in text


def test_bringup_1730_the_script_drives_the_mapping_launch() -> None:
    """BRINGUP-1730: `mola_map_track.sh` starts MOLA through
    `racing_bringup mola_mapping.launch.py` (no yaml parsed in a heredoc, no
    direct `mola_lidar_odometry` launch), and its order is lap, simplemap,
    .mm, T, grid, provenance. The seed is a positional, so the recorded
    command's stem stays at `split()[-2]`."""
    script = (REPOSITORY_ROOT / "scripts" / "mola_map_track.sh").read_text()
    assert "ros2 launch racing_bringup mola_mapping.launch.py" in script
    assert 'simplemap_output:="$simplemap"' in script
    assert "ros2 launch mola_lidar_odometry" not in script
    assert "config/localization/mola_lidar2d_mapping.yaml" not in script
    assert (
        "usage: mola_map_track.sh <scenario.yaml> <output-stem> <seed>"
        in script
    )
    order = [
        'wait "$mola"',
        'sm2mm -i "$simplemap" -o "$output.mm"',
        "mola_mapping_T.txt",
        "sm-cli tf",
        "config/localization/mola_sm2grid.yaml",
        "mm2grid",
        "grid_sha256",
    ]
    code = "\n".join(
        line for line in script.splitlines() if not line.startswith("#")
    )
    positions = [code.index(item) for item in order]
    assert positions == sorted(positions), dict(
        zip(order, positions, strict=True)
    )
    # Every tool step runs under `set -e` and the ERR trap names its line.
    assert re.search(r"^trap 'echo .*LINENO.*' ERR$", script, re.MULTILINE)


# BRINGUP-1740..1760: the committed grid. Bounds are measured (Step 5/6 of
# the plan) and written beside the constants; None means not measured yet.
GRID_YAML = MOLA_DIR / "Spielberg_grid.yaml"
GRID_PNG = MOLA_DIR / "Spielberg_grid.png"
GRID_PROVENANCE_KEYS = (
    "loop_closure",
    "grid_image",
    "grid_sha256",
    "grid_bytes",
    "grid_pipeline",
)
MAP_SERVER_KEYS = {
    "image",
    "resolution",
    "origin",
    "negate",
    "occupied_thresh",
    "free_thresh",
}
# FRAME_P95_BOUND is SIM-3530's P95_BOUND copied as a number (this test owns
# it): five seed-42 laps, wall p95 0.518 0.860 0.810 0.810 0.988 m, worst
# 0.9885 x 1.25 = 1.236 -> 1.24 (2026-10-28, slam-p2, no loop closure).
# GRID_VOXEL_P95_BOUND: the same five laps' grid -> voxel / voxel -> grid
# nearest-neighbour p95 under each run's T (m):
#   0.0329/0.0455  0.0330/0.0466  0.0330/0.0452  0.0330/0.0455  0.0329/0.0442
# worst 0.0466 x 1.25 = 0.0583 -> 0.06 (about one 0.05 m cell).
FRAME_P95_BOUND = 1.24
GRID_VOXEL_P95_BOUND = 0.06


def check_grid_provenance(provenance_path: Path, directory: Path) -> None:
    """The provenance's grid keys agree with the grid files in `directory`
    and the grid yaml has map_server's key set; raises AssertionError naming
    what disagrees."""
    from racing_evaluation.map_accuracy import load_grid

    provenance = yaml.safe_load(provenance_path.read_text())
    for key in GRID_PROVENANCE_KEYS:
        assert key in provenance, f"provenance lacks {key}"
    png = directory / provenance["grid_image"]
    assert png.is_file(), f"{png} is missing"
    data = png.read_bytes()
    assert provenance["grid_bytes"] == len(data), "grid_bytes disagrees"
    assert provenance["grid_sha256"] == hashlib.sha256(data).hexdigest(), (
        "grid_sha256 disagrees with the grid image"
    )
    grid_yaml = directory / (png.stem + ".yaml")
    document = yaml.safe_load(grid_yaml.read_text())
    assert set(document) >= MAP_SERVER_KEYS, set(document)
    assert document["image"] == png.name
    assert document["resolution"] == pytest.approx(0.05)
    assert load_grid(grid_yaml).occupied.any()


def test_bringup_1740_the_provenance_agrees_with_the_grid_files(
    tmp_path,
) -> None:
    """BRINGUP-1740: `loop_closure`, `grid_image`, `grid_sha256`,
    `grid_bytes` and `grid_pipeline` agree with the committed grid; the grid
    yaml has map_server's keys, resolution 0.05, and `load_grid` reads it.
    Bite proof: one hex digit of `grid_sha256` changed goes red naming it."""
    check_grid_provenance(MOLA_DIR / "Spielberg.yaml", MOLA_DIR)
    document = yaml.safe_load((MOLA_DIR / "Spielberg.yaml").read_text())
    assert document["loop_closure"] is False
    assert (REPOSITORY_ROOT / document["grid_pipeline"]).is_file()
    sha = document["grid_sha256"]
    document["grid_sha256"] = ("0" if sha[0] != "0" else "1") + sha[1:]
    mutated = tmp_path / "Spielberg.yaml"
    mutated.write_text(yaml.safe_dump(document))
    with pytest.raises(AssertionError, match="grid_sha256"):
        check_grid_provenance(mutated, MOLA_DIR)


def _rotated_90(grid):
    """`grid` rotated 90 degrees about its centre."""
    import numpy as np
    from racing_evaluation.map_accuracy import Grid

    height, width = grid.occupied.shape
    centre = (
        grid.origin[0] + width * grid.resolution / 2,
        grid.origin[1] + height * grid.resolution / 2,
    )
    return Grid(
        np.rot90(grid.occupied).copy(),
        np.rot90(grid.free).copy(),
        grid.resolution,
        (
            centre[0] - height * grid.resolution / 2,
            centre[1] - width * grid.resolution / 2,
        ),
    )


def test_bringup_1750_the_grid_is_in_the_world_frame() -> None:
    """BRINGUP-1750: the committed grid's wall p95 against the gym map is
    within `FRAME_P95_BOUND`, so a grid left in `mola_map`, or placed with a
    wrong T, is caught in seconds. The bound is this test's own, copied as a
    number from SIM-3530's five seed-42 laps. Bite proof: the grid rotated
    90 degrees about its centre is at least 3x the bound."""
    from racing_evaluation.map_accuracy import (
        load_grid,
        map_to_wall,
        scenario_truth_grid,
    )

    assert FRAME_P95_BOUND is not None, "FRAME_P95_BOUND is not measured yet"
    truth = scenario_truth_grid(SCENARIOS / "spielberg_1080.yaml")
    grid = load_grid(GRID_YAML)
    assert map_to_wall(grid, truth).p95 <= FRAME_P95_BOUND
    rotated = map_to_wall(_rotated_90(grid), truth).p95
    assert rotated >= 3 * FRAME_P95_BOUND, rotated


def _frame_deltas(grid, points, transform) -> tuple[float, float]:
    """(grid -> voxels, voxels -> grid) nearest-neighbour p95 in metres,
    the voxel centres moved into the world by `transform` (x, y, yaw)."""
    import numpy as np
    from racing_evaluation.map_accuracy import nearest_p95

    x, y, yaw = transform
    c, s = np.cos(yaw), np.sin(yaw)
    world = np.column_stack(
        [
            c * points[:, 0] - s * points[:, 1] + x,
            s * points[:, 0] + c * points[:, 1] + y,
        ]
    )
    rows, cols = np.nonzero(grid.occupied)
    cells = np.column_stack(
        [
            grid.origin[0] + (cols + 0.5) * grid.resolution,
            grid.origin[1] + (rows + 0.5) * grid.resolution,
        ]
    )
    return nearest_p95(cells, world), nearest_p95(world, cells)


def test_bringup_1760_the_grid_and_the_mm_agree_under_the_provenance_t(
    tmp_path,
) -> None:
    """BRINGUP-1760 (no ground truth): the grid's occupied cell centres and
    the `.mm` `localmap` voxel centres moved by the provenance's T agree
    within `GRID_VOXEL_P95_BOUND`, in both directions. On a real car T is the
    identity and the check holds unchanged. Bite proofs: T taken as the
    identity, and T with its yaw's sign flipped, are at least 3x the bound."""
    from racing_evaluation.map_accuracy import load_grid, mm_layer_points

    assert GRID_VOXEL_P95_BOUND is not None, "bound is not measured yet"
    provenance = yaml.safe_load((MOLA_DIR / "Spielberg.yaml").read_text())
    t = provenance["T_map_mola"]
    grid = load_grid(GRID_YAML)
    points = mm_layer_points(MOLA_DIR / "Spielberg.mm", "localmap", tmp_path)
    right = _frame_deltas(grid, points, (t["x"], t["y"], t["yaw"]))
    print(f"BRINGUP-1760 grid->voxel, voxel->grid p95: {right}")
    assert max(right) <= GRID_VOXEL_P95_BOUND, right
    for name, wrong in (
        ("identity", (0.0, 0.0, 0.0)),
        ("flipped yaw", (t["x"], t["y"], -t["yaw"])),
    ):
        deltas = _frame_deltas(grid, points, wrong)
        print(f"BRINGUP-1760 bite {name}: {deltas}")
        assert min(deltas) >= 3 * GRID_VOXEL_P95_BOUND, (name, deltas)


def test_bringup_1770_a_watchdog_never_holds_the_callers_pipe() -> None:
    """BRINGUP-1770: the script's kill watchdogs do not keep its stdout open.

    Nightly 40663334967 (SIM-5520): the script exited 0 but
    `subprocess.run(capture_output=True)` waited the full 900 s timeout,
    because `( sleep 900; kill ... ) &` left its `sleep` alive after the
    watchdog subshell was killed, and that `sleep` inherited the pipe.
    The functions are run as the script defines them, under a capture, with
    a 30 s watchdog stopped at once.
    """
    import subprocess
    import time

    text = (REPOSITORY_ROOT / "scripts" / "mola_map_track.sh").read_text()
    functions = re.findall(
        r"^(?:start_watchdog|stop_watchdog)\(\) \{.*?^\}\n",
        text,
        flags=re.M | re.S,
    )
    assert len(functions) == 2, "start_watchdog and stop_watchdog"
    assert "start_watchdog " in text and "stop_watchdog" in text
    harness = "\n".join(functions) + (
        "\nstart_watchdog 30 $$\nsleep 0.5\nstop_watchdog\necho done\n"
    )
    started = time.monotonic()
    completed = subprocess.run(
        ["bash", "-c", harness],
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    )
    elapsed = time.monotonic() - started
    assert completed.stdout == "done\n"
    assert elapsed < 5, f"capture held open {elapsed:.1f} s by a watchdog"
