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

import numpy as np
import pytest
import yaml
from PIL import Image
from racing_evaluation.map_accuracy import (
    load_grid,
    mm_layer_points,
    nearest_p95,
)

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
    "loop_closure",
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
    "three same-seed laps: T spread 0, nearest-neighbour p95 0.224-1.154 m; "
    "final pipeline, committed + three laps: T spread 0, voxel p95 0.35-0.80 "
    "m, grid p95 0.40-0.81 m"
)
POSITION_TOLERANCE = 0.05
YAW_TOLERANCE_DEGREES = 0.2
NEAREST_NEIGHBOUR_P95_TOLERANCE = 1.9
# Re-measured 2026-10-30 (slam-p2, 1x, load 7.7-7.9) on the final mapping
# pipeline: the committed map and three same-seed regenerations, every pair:
# T identical to the 4th decimal (spread 0 -> the floors above stand);
# voxel p95 0.35-0.80 m and grid p95 0.40-0.81 m (the two agree within 0.05
# m in each pair; spread 0.46 m, the existing 1.9 m stands as 2.4x the
# worst). The grid tolerance is a new constant: 1.62 m, 2x the worst grid
# p95 (0.81 m), the rule above. A
# 90 degree rotated copy of the committed grid scores 32.2 / 61.2 m (>= 3x).
# No existing tolerance moved.
GRID_NEAREST_NEIGHBOUR_P95_TOLERANCE = 1.62


def _document(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _occupied_centres(map_yaml: Path) -> np.ndarray:
    """Occupied cell centres of the grid beside `<stem>.yaml`."""
    grid = load_grid(map_yaml.with_name(map_yaml.stem + "_grid.yaml"))
    return grid.cell_centres(grid.occupied)


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
    old_cells = _occupied_centres(committed_yaml)
    new_cells = _occupied_centres(regenerated_stem.with_suffix(".yaml"))
    deltas = {
        "T_position_m": position,
        "T_yaw_deg": yaw,
        "p95_regenerated_to_committed_m": nearest_p95(new, old),
        "p95_committed_to_regenerated_m": nearest_p95(old, new),
        "grid_p95_regenerated_to_committed_m": nearest_p95(
            new_cells, old_cells
        ),
        "grid_p95_committed_to_regenerated_m": nearest_p95(
            old_cells, new_cells
        ),
    }
    print(f"SIM-5520 deltas: {deltas}; measured: {MEASUREMENT}")
    assert position <= POSITION_TOLERANCE, deltas
    assert yaw <= YAW_TOLERANCE_DEGREES, deltas
    for name in (
        "p95_regenerated_to_committed_m",
        "p95_committed_to_regenerated_m",
    ):
        assert deltas[name] <= NEAREST_NEIGHBOUR_P95_TOLERANCE, deltas
    for name in (
        "grid_p95_regenerated_to_committed_m",
        "grid_p95_committed_to_regenerated_m",
    ):
        assert deltas[name] <= GRID_NEAREST_NEIGHBOUR_P95_TOLERANCE, (
            f"grid p95 differs ({name})",
            deltas,
        )
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


def _copy_map(directory: Path) -> Path:
    """The committed map (.yaml, .mm, grid yaml and png) in `directory`."""
    directory.mkdir(parents=True, exist_ok=True)
    copy = directory / "Spielberg.yaml"
    shutil.copy(COMMITTED, copy)
    shutil.copy(COMMITTED.with_suffix(".mm"), copy.with_suffix(".mm"))
    for name in ("Spielberg_grid.yaml", "Spielberg_grid.png"):
        shutil.copy(COMMITTED.parent / name, directory / name)
    return copy


def test_sim_5520_rotated_grid_is_named(tmp_path: Path) -> None:
    """The committed map with its occupancy grid ROTATED 90 degrees about
    its centre (np.rot90 of the PNG, the yaml origin recomputed so the centre
    stays put) is named by the grid check. A rotation, not a shift: a shift
    folds on a 2.2 m corridor, while a rotation moves every wall away from
    its old place (plan Step 6.5 (c), R3). The .mm is untouched, so only
    the grid check can see it."""
    copy = _copy_map(tmp_path / "rotated")
    grid_yaml = copy.with_name("Spielberg_grid.yaml")
    document = yaml.safe_load(grid_yaml.read_text(encoding="utf-8"))
    image = np.asarray(Image.open(grid_yaml.with_name(document["image"])))
    rows, cols = image.shape[:2]
    resolution = float(document["resolution"])
    centre_x = document["origin"][0] + cols * resolution / 2
    centre_y = document["origin"][1] + rows * resolution / 2
    Image.fromarray(np.rot90(image).copy()).save(
        grid_yaml.with_name(document["image"])
    )
    # rot90 swaps the extents: rows x cols becomes cols x rows.
    document["origin"] = [
        centre_x - rows * resolution / 2,
        centre_y - cols * resolution / 2,
        0.0,
    ]
    grid_yaml.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(AssertionError, match=r"grid.*p95"):
        check_regenerated(COMMITTED, copy.with_suffix(""), tmp_path)


def test_sim_5520_unmutated_copy_passes(tmp_path: Path) -> None:
    copy = _copy_map(tmp_path / "copy")
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
