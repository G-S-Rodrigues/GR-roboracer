"""SIM-5010: the committed occupancy grid regenerates from the survey.

`config/scenarios/maps/Spielberg/slam/Spielberg.{png,yaml}` is a committed
artifact - the map nav2_map_server serves to nav2_amcl - produced by
`scripts/map_track.sh` from an exact-sensor survey lap. A committed binary
that nobody can reproduce is a fact nobody can check, and the survey is not
bit-reproducible (which scans the async mapper takes is not deterministic),
so what is asserted here is that a fresh survey puts the walls in the same
*places*, not that it writes the same bytes.

Tier 5 because it is a lap plus a mapping run, and because what it catches
is slow: a mapper parameter, a scan geometry or a saver flag that changes
what the grid means, at a moment when no one is looking at the grid.

The plan named SIM-5010 "the committed pose graph regenerates". No pose
graph is committed (plan D7 - slam_toolbox maps online and a matcher-off
survey graph has no vertices), so the committed artifact that survey
actually produces is what this covers instead.
"""

import subprocess
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
SURVEY_SCENARIO = "config/scenarios/spielberg_survey.yaml"
COMMITTED = (
    REPOSITORY_ROOT / "config/scenarios/maps/Spielberg/slam/Spielberg.yaml"
)
# A survey lap at 1x plus the mapper's last update and the save.
SURVEY_TIMEOUT_SECONDS = 600.0
# Half a cell at 0.05 m resolution, rounded up to a full cell: a wall that
# regenerates one cell over is the same wall.
NEIGHBOURHOOD_CELLS = 1
# Measured, two regenerated surveys against the committed grid. Before the
# floor fix (2 min 21 s): every one of the fresh grid's 20 000 wall cells
# has a committed wall cell beside it (1.0000), and 0.9937 of the
# committed grid's 20 234 do in the fresh one. After it (2 min 17 s):
# 1.0000 and 0.9999, 20 251 fresh wall cells. The two runs differ because
# which scans the async mapper takes is not deterministic, so the fix and
# the run-to-run spread are not separable from two runs; the bound keeps
# headroom under the worse direction and is not a number to lower
# (repo-gotchas #14).
MEASUREMENT = (
    "two regenerated surveys vs the committed grid: fresh->committed "
    "1.0000 both times, committed->fresh 0.9937 (20000 wall cells) then "
    "0.9999 (20251 wall cells, after the floor fix), against 20234 committed"
)
AGREEMENT_MINIMUM = 0.95


def _load_grid(yaml_path: Path) -> tuple[dict, np.ndarray]:
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = (yaml_path.parent / document["image"]).resolve()
    image = np.asarray(Image.open(image_path).convert("L"))
    # map_server's convention for a non-negated grid: dark is occupied.
    occupancy = (255.0 - image.astype(float)) / 255.0
    occupied = occupancy > document["occupied_thresh"]
    return document, occupied


def _world_of_occupied(document: dict, occupied: np.ndarray) -> np.ndarray:
    rows, columns = np.nonzero(occupied)
    resolution = document["resolution"]
    origin_x, origin_y = document["origin"][0], document["origin"][1]
    # Row 0 is the top of the image, which is the *last* row of the grid.
    x = origin_x + (columns + 0.5) * resolution
    y = origin_y + (occupied.shape[0] - rows - 0.5) * resolution
    return np.column_stack((x, y))


def _fraction_within(
    points: np.ndarray, document: dict, occupied: np.ndarray
) -> float:
    """Fraction of `points` with an occupied cell of `occupied` beside them."""
    resolution = document["resolution"]
    # floor, not a bare int cast: that truncates toward zero, folding a
    # point just outside the grid (cell -0.5) into boundary cell 0.
    columns = np.floor((points[:, 0] - document["origin"][0]) / resolution)
    columns = columns.astype(int)
    rows = (
        occupied.shape[0]
        - np.floor((points[:, 1] - document["origin"][1]) / resolution).astype(
            int
        )
        - 1
    )
    hit = np.zeros(len(points), dtype=bool)
    span = range(-NEIGHBOURHOOD_CELLS, NEIGHBOURHOOD_CELLS + 1)
    for row_offset in span:
        for column_offset in span:
            r = np.clip(rows + row_offset, 0, occupied.shape[0] - 1)
            c = np.clip(columns + column_offset, 0, occupied.shape[1] - 1)
            inside = (
                (rows + row_offset >= 0)
                & (rows + row_offset < occupied.shape[0])
                & (columns + column_offset >= 0)
                & (columns + column_offset < occupied.shape[1])
            )
            hit |= occupied[r, c] & inside
    return float(hit.mean())


def test_sim_5010_committed_grid_regenerates(tmp_path: Path) -> None:
    stem = tmp_path / "Spielberg"
    completed = subprocess.run(
        [
            "./scripts/map_track.sh",
            SURVEY_SCENARIO,
            str(stem),
        ],
        cwd=REPOSITORY_ROOT,
        capture_output=True,
        text=True,
        timeout=SURVEY_TIMEOUT_SECONDS,
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr

    committed_document, committed_occupied = _load_grid(COMMITTED)
    fresh_document, fresh_occupied = _load_grid(stem.with_suffix(".yaml"))

    # The metadata is the part that is not allowed to drift: a resolution
    # or a threshold that moved silently changes what every cell means.
    assert fresh_document["resolution"] == committed_document["resolution"]
    assert (
        fresh_document["occupied_thresh"]
        == (committed_document["occupied_thresh"])
    )
    assert fresh_document["free_thresh"] == committed_document["free_thresh"]
    assert fresh_document["negate"] == committed_document["negate"]

    fresh_walls = _world_of_occupied(fresh_document, fresh_occupied)
    committed_walls = _world_of_occupied(committed_document, committed_occupied)
    fresh_in_committed = _fraction_within(
        fresh_walls, committed_document, committed_occupied
    )
    committed_in_fresh = _fraction_within(
        committed_walls, fresh_document, fresh_occupied
    )
    print(
        f"SIM-5010 agreement: fresh->committed {fresh_in_committed:.4f}, "
        f"committed->fresh {committed_in_fresh:.4f} "
        f"({len(fresh_walls)} vs {len(committed_walls)} wall cells)"
    )

    assert fresh_in_committed >= AGREEMENT_MINIMUM, fresh_in_committed
    assert committed_in_fresh >= AGREEMENT_MINIMUM, committed_in_fresh
