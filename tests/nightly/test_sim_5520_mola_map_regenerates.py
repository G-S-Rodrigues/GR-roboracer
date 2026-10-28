"""SIM-5520: the committed MOLA map regenerates from its recorded command.

`config/scenarios/maps/Spielberg/mola/Spielberg.{mm,yaml}` is a committed
artifact produced by `scripts/mola_map_track.sh` (a lap plus MOLA-LO's
shutdown save and `sm2mm`). MOLA is multi-threaded and which scans it keeps
is not deterministic, so two runs are close and never byte-identical: what
is asserted is that the inputs are exact, T is the same, and the voxels land
in the same places (nearest-neighbour p95, both directions).

Tier 5 because it is a lap, and because what it catches is slow: a pipeline
parameter, a scan geometry or a package bump that changes what the map means.
The assertion logic is `check_regenerated`, which takes the committed
provenance path so the tier-1 test below can feed it a mutated copy.
"""

import math
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml
from racing_evaluation.map_accuracy import mm_layer_points, nearest_p95

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
COMMITTED = (
    REPOSITORY_ROOT / "config/scenarios/maps/Spielberg/mola/Spielberg.yaml"
)
# A mapping lap at 1x, MOLA's save and sm2mm.
TIMEOUT_SECONDS = 900.0
# Inputs that must be identical; `command` is compared with the output stem
# taken out, since the nightly writes to a temporary one.
EXACT_INPUTS = (
    "scenario",
    "seed",
    "time_scale",
    "mola_version",
    "mp2p_icp_version",
    "T_source",
)
# Rule: each tolerance is 2x the spread measured between same-seed laps,
# with a floor of one voxel (0.05 m) for distances and 0.2 deg for yaw.
# Measured, three same-seed laps (2026-10-28, dev container, 1x): T identical
# to the 4th decimal in all three (spread 0 -> the floors); occupied-voxel
# nearest-neighbour p95 between laps 0.224-1.154 m (p50 0.05-0.14 m), spread
# 0.93 m -> 1.9 m. The p95 is that large because a lap with no loop closure
# drifts differently each time: the bound guards against a different map,
# not against a different drift. (An older committed map, built with a
# different scan rate, was 68 m p95 from these.)
MEASUREMENT = (
    "three same-seed laps: T spread 0, nearest-neighbour p95 0.224-1.154 m"
)
POSITION_TOLERANCE = 0.05
YAW_TOLERANCE_DEGREES = 0.2
NEAREST_NEIGHBOUR_P95_TOLERANCE = 1.9


def _document(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def check_regenerated(
    committed_yaml: Path, regenerated_stem: Path, work: Path | None = None
) -> dict:
    """Compare a regenerated map (`<stem>.mm` and `.yaml`) with the one
    committed beside `committed_yaml`; raise AssertionError naming what
    differs, and return the measured deltas."""
    committed = _document(committed_yaml)
    regenerated = _document(regenerated_stem.with_suffix(".yaml"))
    for key in EXACT_INPUTS:
        assert regenerated[key] == committed[key], (
            f"input mismatch on {key}: committed {committed[key]!r}, "
            f"regenerated {regenerated[key]!r}"
        )
    stem = str(committed["command"]).split()[-2]
    regenerated_command = str(regenerated["command"]).replace(
        str(regenerated["command"]).split()[-2], stem
    )
    assert regenerated_command == committed["command"], (
        f"input mismatch on command: committed {committed['command']!r}, "
        f"regenerated {regenerated['command']!r}"
    )
    a, b = committed["T_map_mola"], regenerated["T_map_mola"]
    position = math.hypot(a["x"] - b["x"], a["y"] - b["y"])
    yaw = abs(math.degrees(a["yaw"] - b["yaw"]))
    work = work or regenerated_stem.parent
    old = mm_layer_points(
        committed_yaml.with_suffix(".mm"), "localmap", work / "committed"
    )
    new = mm_layer_points(
        regenerated_stem.with_suffix(".mm"), "localmap", work / "regenerated"
    )
    deltas = {
        "T_position_m": position,
        "T_yaw_deg": yaw,
        "p95_regenerated_to_committed_m": nearest_p95(new, old),
        "p95_committed_to_regenerated_m": nearest_p95(old, new),
    }
    print(f"SIM-5520 deltas: {deltas}; measured: {MEASUREMENT}")
    assert position <= POSITION_TOLERANCE, deltas
    assert yaw <= YAW_TOLERANCE_DEGREES, deltas
    for name in (
        "p95_regenerated_to_committed_m",
        "p95_committed_to_regenerated_m",
    ):
        assert deltas[name] <= NEAREST_NEIGHBOUR_P95_TOLERANCE, deltas
    return deltas


def test_sim_5520_input_mismatch_is_named(tmp_path: Path) -> None:
    """The committed provenance with `seed: 8` is not the one a seed-7 lap
    regenerates: the failure names the input, not a distance."""
    mutated = tmp_path / "Spielberg.yaml"
    document = _document(COMMITTED)
    document["seed"] = 8
    mutated.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(AssertionError, match=r"seed.*8.*7"):
        check_regenerated(mutated, COMMITTED.with_suffix(""), tmp_path)


def test_sim_5520_unmutated_copy_passes(tmp_path: Path) -> None:
    copy = tmp_path / "Spielberg.yaml"
    shutil.copy(COMMITTED, copy)
    shutil.copy(COMMITTED.with_suffix(".mm"), copy.with_suffix(".mm"))
    deltas = check_regenerated(copy, COMMITTED.with_suffix(""), tmp_path)
    assert deltas["T_position_m"] == 0


def test_sim_5520_committed_map_regenerates(tmp_path: Path) -> None:
    stem = tmp_path / "Spielberg"
    command = _document(COMMITTED)["command"].split()
    command[-2] = str(stem)
    completed = subprocess.run(
        command,
        cwd=REPOSITORY_ROOT,
        capture_output=True,
        text=True,
        timeout=TIMEOUT_SECONDS,
        check=False,
    )
    assert completed.returncode == 0, (
        completed.stdout[-2000:] + completed.stderr
    )
    check_regenerated(COMMITTED, stem)
