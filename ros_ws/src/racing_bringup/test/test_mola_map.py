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
