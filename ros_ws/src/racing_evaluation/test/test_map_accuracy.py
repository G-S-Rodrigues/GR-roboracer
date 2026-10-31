"""EVAL-1500..1540: how far a produced map's walls are from the true ones.

The truth is the gym map image's free/occupied boundary (what the sim lidar
hits). Each occupied cell of the produced grid is measured by the signed
distance from its CENTRE to the nearest truth edge, positive when the centre
is on the truth's occupied side. The raw centre distance is kept (owner
ruling O2): a perfect map scores res/2 (2.5 cm at 5 cm), not 0.
"""

import os
import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest
from PIL import Image
from racing_evaluation.map_accuracy import (
    Grid,
    grid_from_occupancy,
    load_grid,
    map_to_wall,
    mm_layer_points,
    nearest_p95,
    scenario_truth_grid,
    wall_edges,
)

REPOSITORY_ROOT = Path(__file__).parents[4]
SPIELBERG_1080 = REPOSITORY_ROOT / "config/scenarios/spielberg_1080.yaml"

RES = 0.05
ORIGIN = (-3.0, 2.0)


def _grid(occupied, resolution=RES, origin=ORIGIN) -> Grid:
    occupied = np.asarray(occupied, dtype=bool)
    return Grid(occupied, ~occupied, resolution, origin)


def _corridor(cols=20, rows=10, free=(8, 12)) -> Grid:
    """Vertical corridor: occupied outside columns [free[0], free[1])."""
    occupied = np.ones((rows, cols), dtype=bool)
    occupied[:, free[0] : free[1]] = False
    return _grid(occupied)


def _with_cells(truth: Grid, cells) -> Grid:
    occupied = np.zeros_like(truth.occupied)
    for row, col in cells:
        occupied[row, col] = True
    return _grid(occupied)


def _wall_cells(rows, left, right):
    return [(r, c) for r in rows for c in (left, right)]


def test_eval_1500_loader_follows_map_server_convention(tmp_path) -> None:
    image = np.full((3, 4), 254, dtype=np.uint8)
    image[0, 0] = 0  # image row 0 is the TOP of the map
    Image.fromarray(image).save(tmp_path / "m.pgm")
    (tmp_path / "m.yaml").write_text(
        "image: m.pgm\nresolution: 0.1\norigin: [1.0, 2.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    grid = load_grid(tmp_path / "m.yaml")
    assert grid.occupied.shape == (3, 4)
    assert grid.occupied[2, 0] and grid.occupied.sum() == 1
    assert not grid.occupied[0, 0]
    assert grid.free.sum() == 11
    assert grid.resolution == pytest.approx(0.1)
    assert grid.origin == pytest.approx((1.0, 2.0))
    centre = grid.cell_centres(grid.occupied)[0]
    assert centre == pytest.approx((1.0 + 0.5 * 0.1, 2.0 + 2.5 * 0.1))


def test_eval_1500_occupancy_grid_thresholds() -> None:
    values = [100, 64, 65, -1, 0, 19, 20]
    grid = grid_from_occupancy(values, 7, 1, 0.05, (0.0, 0.0))
    assert grid.occupied[0].tolist() == [True, False, True] + [False] * 4
    assert grid.free[0].tolist() == [False] * 4 + [True, True, False]


def test_eval_1510_a_flush_grid_scores_half_a_cell() -> None:
    truth = _corridor()
    # Every truth wall cell that touches the corridor, nothing else.
    flush = _with_cells(truth, _wall_cells(range(10), 7, 12))
    score = map_to_wall(flush, truth)
    half = RES / 2
    assert score.wall_cell_count == 20
    for value in (
        score.maximum,
        score.p95,
        score.median,
        score.signed_median,
        score.signed_mean,
    ):
        assert value == pytest.approx(half, abs=1e-12)


def test_eval_1520_shifted_walls_read_in_metres_with_sign() -> None:
    truth = _corridor()
    half = RES / 2
    outward = map_to_wall(
        _with_cells(truth, _wall_cells(range(10), 5, 14)), truth
    )
    assert outward.signed_median == pytest.approx(half + 2 * RES)
    assert outward.signed_mean == pytest.approx(half + 2 * RES)
    inward = map_to_wall(
        _with_cells(truth, _wall_cells(range(10), 8, 11)), truth
    )
    # One cell into the corridor: the centre is on the free side.
    assert inward.signed_median == pytest.approx(half - RES)
    assert inward.signed_mean == pytest.approx(-half)
    assert inward.median == pytest.approx(half)  # unsigned
    # 20 flush cells (0.025) and two stray ones 0.125: p95 interpolates.
    cells = _wall_cells(range(10), 7, 12) + [(4, 5), (5, 5)]
    mixed = map_to_wall(_with_cells(truth, cells), truth)
    assert mixed.wall_cell_count == 22
    assert mixed.maximum == pytest.approx(0.125)
    assert mixed.median == pytest.approx(0.025)
    assert mixed.p95 == pytest.approx(0.025 + 0.95 * 0.1)
    assert mixed.signed_mean == pytest.approx((20 * 0.025 + 2 * 0.125) / 22)


def test_eval_1530_a_grid_with_no_walls_is_unscored_not_zero() -> None:
    truth = _corridor()
    empty = _with_cells(truth, [])
    score = map_to_wall(empty, truth)
    assert score.wall_cell_count == 0
    assert score.maximum is None
    assert score.p95 is None
    assert score.median is None
    assert score.signed_median is None
    assert score.signed_mean is None
    # Unknown (neither free nor occupied) cells are not walls either.
    unknown = Grid(empty.occupied, empty.occupied.copy(), RES, ORIGIN)
    assert map_to_wall(unknown, truth).wall_cell_count == 0


def test_eval_1540_grids_are_compared_in_metres_not_cells() -> None:
    truth = _corridor()
    fine = 0.025
    origin = (ORIGIN[0] + 0.013, ORIGIN[1] - 0.0071)
    occupied = np.zeros((40, 80), dtype=bool)
    occupied[8, 13] = True  # centre x = ox + 0.013 + 13.5 * 0.025
    produced = _grid(occupied, resolution=fine, origin=origin)
    centre = produced.cell_centres(produced.occupied)[0]
    wall_x = ORIGIN[0] + 8 * RES
    expected = wall_x - centre[0]
    assert 0.0 < expected < 0.06
    score = map_to_wall(produced, truth)
    assert score.signed_median == pytest.approx(expected, abs=1e-9)
    assert score.maximum == pytest.approx(expected, abs=1e-9)


def test_eval_1540_wall_edges_are_the_free_occupied_faces() -> None:
    edges = wall_edges(_corridor())
    assert edges.shape == (20, 2, 2)  # 10 rows x 2 faces
    lengths = np.linalg.norm(edges[:, 1] - edges[:, 0], axis=1)
    assert lengths == pytest.approx(RES)


def test_eval_1550_the_scenarios_truth_grid_is_its_gym_map() -> None:
    """EVAL-1550: `scenario_truth_grid(scenario.yaml)` loads the gym map image
    the scenario names (`environment.map` under `map_directory`), the grid
    the sim lidar hits."""
    expected = load_grid(
        REPOSITORY_ROOT / "config/scenarios/maps/Spielberg/Spielberg.yaml"
    )
    truth = scenario_truth_grid(SPIELBERG_1080)
    assert np.array_equal(truth.occupied, expected.occupied)
    assert np.array_equal(truth.free, expected.free)
    assert truth.resolution == expected.resolution
    assert truth.origin == expected.origin


def test_eval_1560_nearest_p95_is_the_95th_percentile_of_the_distance() -> None:
    """EVAL-1560: identical point sets are 0 apart; a set shifted by
    (0.3, 0.4) is 0.5 from its original, in either direction."""
    rng = np.random.default_rng(0)
    points = rng.uniform(0, 10, size=(200, 2))
    assert nearest_p95(points, points) == 0.0
    shifted = points + np.array([0.3, 0.4])
    assert nearest_p95(shifted, points) < 0.5 + 1e-9
    one = np.array([[0.0, 0.0]])
    assert nearest_p95(one + np.array([0.3, 0.4]), one) == pytest.approx(0.5)


def test_eval_1560_mm_layer_points_reads_the_layers_text_export(
    tmp_path,
) -> None:
    """EVAL-1560: `mm_layer_points(mm, layer, work)` is the (x, y) columns of
    what `mm2txt` writes for the layer, read from `<stem>_<layer>.txt`. The
    reference is a direct mm2txt run in a second directory (no count pinned:
    the committed map is regenerated)."""
    mm = REPOSITORY_ROOT / "config/scenarios/maps/Spielberg/mola/Spielberg.mm"
    points = mm_layer_points(mm, "localmap", tmp_path / "a")
    assert points.ndim == 2 and points.shape[1] == 2 and len(points) > 0
    other = tmp_path / "b"
    other.mkdir()
    subprocess.run(
        [shutil.which("mm2txt"), str(mm)],
        cwd=other,
        check=True,
        capture_output=True,
        env=os.environ,
    )
    expected = np.loadtxt(other / "Spielberg_localmap.txt")[:, :2]
    assert np.array_equal(points, expected)
