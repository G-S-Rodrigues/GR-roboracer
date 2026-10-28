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
