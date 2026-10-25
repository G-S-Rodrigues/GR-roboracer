"""Tier 1: AutoDRIVE's Porto track in the tree (#44 part 3).

The map is AutoDRIVE-RoboRacer-Racetracks' ``Legacy Tracks/Porto Track``
at a pinned commit; ``config/tracks/porto.yaml`` is extracted from it by
``python -m raceline.track_from_map``. The start (0, 0, 0) and the ``ccw``
direction are an assumption that part 10 checks against the simulator.
"""

import hashlib
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[2]
MAP_DIR = REPO / "config" / "scenarios" / "maps" / "Porto"
MAP_YAML = MAP_DIR / "Porto.yaml"
TRACK = REPO / "config" / "tracks" / "porto.yaml"
SCENARIO = REPO / "config" / "scenarios" / "porto.yaml"

UPSTREAM_SHA = "cf3cf1e5d37489f4768c8041706e53977e4d6bc7"
SHA256 = {
    "Porto.pgm": (
        "404a72551715fbf48ebcbd5b3b4093fcdf62934fb815898b144a017fe2c02efe"
    ),
    "Porto.yaml": (
        "b78d687bae1758fdd62070b6da30c77a817bb102926630d973503035584d4d0d"
    ),
    "LICENSE": (
        "928db06fa55c94014ac8cc807b7d4ac884ca96ca2977f0f8dc49d7e551f158bb"
    ),
}
START = (0.0, 0.0, 0.0)  # assumed AutoDRIVE spawn: map origin, heading +x
DIRECTION = "ccw"
# Measured at part 3 step 1 (dev, 2026-10-25): 327 points, 32.7105 m.
LENGTH_M = 32.71
# Measured: the centerline's smallest clearance to an occupied cell is
# 0.489 m (p5 0.589 m). The floor is that rounded down to a 0.05 m cell.
CLEARANCE_FLOOR_M = 0.45
MIN_TOTAL_WIDTH_M = 0.90  # the rules' minimum; measured min 1.176 m


def _sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def test_raceline_1180_official_map_is_byte_identical_to_upstream():
    """RACELINE-1180: map files and licence text equal upstream @cf3cf1e5."""
    for name in ("Porto.pgm", "Porto.yaml"):
        got = _sha((MAP_DIR / name).read_bytes())
        assert got == SHA256[name], f"{name}: {got} != {SHA256[name]}"
    notice = (MAP_DIR / "NOTICE.md").read_bytes()
    begin = b"<!-- LICENSE begin -->\n"
    end = b"<!-- LICENSE end -->"
    block = notice.split(begin, 1)[1].split(end, 1)[0]
    got = _sha(block)
    assert got == SHA256["LICENSE"], f"licence block {got}"
    assert UPSTREAM_SHA.encode() in notice


def _cli_args(out, start=("0", "0", "0")):
    return [
        "--map", str(MAP_YAML),
        "--start-pose", *start,
        "--direction", DIRECTION,
        "--walls", "observed",
        "--name", "Porto",
        "--out", str(out),
    ]  # fmt: skip


def test_raceline_1185_cli_writes_what_the_extractor_extracts(tmp_path, capsys):
    """RACELINE-1185: the CLI is write_track(extract_track(...)), nothing more.

    The header names the command, the upstream commit and the assumed start.
    A refusal exits 3 with the extractor's message on stderr.
    """
    from raceline.extract import Pose2D, extract_track, load_map
    from raceline.track import write_track

    from raceline import track_from_map

    out = tmp_path / "porto.yaml"
    note = "start (0, 0, 0) ccw assumed; checked in #44 part 10"
    argv = _cli_args(out) + ["--source-note", note]
    assert track_from_map.main(argv) == 0
    text = out.read_text()
    header = [ln for ln in text.splitlines() if ln.startswith("#")]
    joined = "\n".join(header)
    assert "raceline.track_from_map" in joined
    assert "--start-pose 0.0 0.0 0.0" in joined and "--direction ccw" in joined
    assert note in joined
    grid = load_map(MAP_YAML)
    track = extract_track(
        grid, grid.resolution, grid.origin, Pose2D(*START), DIRECTION,
        name="Porto", walls="observed",
    )  # fmt: skip
    expected = write_track(track, tmp_path / "expected.yaml", source="\n".join(
        s[2:] for s in header))  # fmt: skip
    assert out.read_text() == expected.read_text()

    capsys.readouterr()
    bad = tmp_path / "bad.yaml"
    assert track_from_map.main(_cli_args(bad, ("5", "5", "0"))) == 3
    assert not bad.exists()
    assert capsys.readouterr().err.strip()


@pytest.fixture(scope="module")
def extracted():
    from raceline.extract import Pose2D, extract_track, load_map

    grid = load_map(MAP_YAML)
    return extract_track(
        grid,
        grid.resolution,
        grid.origin,
        Pose2D(*START),
        DIRECTION,
        name="Porto",
        walls="observed",
    )


@pytest.fixture(scope="module")
def grid():
    from raceline.extract import load_map

    return load_map(MAP_YAML)


def _outside_walls(track, grid, floor):
    """Indices of centerline points off a free cell or under the floor.

    Independent of the extractor: the free cells and the distance to the
    nearest occupied cell both come from the official map.
    """
    from raceline.extract import _dist_to_occupied

    dist = _dist_to_occupied(grid)
    col = np.floor((track.xy[:, 0] - grid.origin.x) / grid.resolution)
    row = np.floor((track.xy[:, 1] - grid.origin.y) / grid.resolution)
    h, w = dist.shape
    bad = []
    cells = zip(col.astype(int), row.astype(int), strict=True)
    for i, (c, r) in enumerate(cells):
        if not (0 <= c < w and 0 <= r < h) or (
            grid.data[r, c] != 0 or dist[r, c] < floor
        ):
            bad.append(i)
    return bad


def _too_narrow(track, minimum=MIN_TOTAL_WIDTH_M):
    """Indices where ``w_left + w_right`` is under ``minimum``."""
    total = track.w_left + track.w_right
    return [int(i) for i in np.flatnonzero(total < minimum)]


def test_raceline_1190_porto_track_regenerates_from_the_map(extracted):
    """RACELINE-1190: porto.yaml is the extraction of the committed map.

    Same point count, values within 1e-6 (the file prints xy and curvature
    at 1e-9 and widths at 1e-6).
    """
    from raceline.track import load_track

    got = load_track(TRACK)
    assert len(got.xy) == len(extracted.xy)
    np.testing.assert_allclose(got.xy, extracted.xy, atol=1e-6)
    np.testing.assert_allclose(got.curvature, extracted.curvature, atol=1e-6)
    np.testing.assert_allclose(got.w_left, extracted.w_left, atol=1e-6)
    np.testing.assert_allclose(got.w_right, extracted.w_right, atol=1e-6)


def test_raceline_1195_porto_track_is_a_closed_lap_of_known_length():
    """RACELINE-1195: Porto's lap is 32.71 m (repo-gotchas #15).

    Lap distance comes from the track, so this file is Porto's lap length:
    within 1% of the measured 32.71 m, closed, starting within
    ``MAX_START_OFFSET`` of the assumed spawn.
    """
    from raceline.extract import MAX_START_OFFSET, STEP_M
    from raceline.track import load_track

    track = load_track(TRACK)
    assert track.name == "Porto"
    seg = np.roll(track.xy, -1, axis=0) - track.xy
    step = np.hypot(seg[:, 0], seg[:, 1])
    assert step[-1] <= 1.5 * STEP_M, f"gap {step[-1]:.3f} m closing the loop"
    assert abs(step.sum() - LENGTH_M) <= 0.01 * LENGTH_M, step.sum()
    assert np.hypot(*(track.xy[0] - START[:2])) <= MAX_START_OFFSET


def test_raceline_1200_porto_centerline_stays_inside_the_walls(grid):
    """RACELINE-1200: every point is a free cell >= 0.45 m from a wall.

    The negative case moves the committed line 1.0 m along its left normal,
    out of the 1.18-2.6 m corridor: the check must flag it, or it is vacuous.
    """
    from raceline.geometry import tangents
    from raceline.track import Track, load_track

    track = load_track(TRACK)
    assert _outside_walls(track, grid, CLEARANCE_FLOOR_M) == []

    tan = tangents(track.xy)
    left = np.column_stack([-tan[:, 1], tan[:, 0]])
    shifted = Track(
        track.name,
        track.xy + 1.0 * left,
        track.curvature,
        track.w_left,
        track.w_right,
    )
    bad = _outside_walls(shifted, grid, CLEARANCE_FLOOR_M)
    assert bad, "a line shifted 1.0 m sideways passed the inside-walls check"


def test_raceline_1205_porto_is_wide_enough_everywhere():
    """RACELINE-1205: w_left + w_right >= 0.90 m at every point.

    Measured min 1.176 m, the duct gaps included. The negative case scales
    the widths by 0.7 (min ~0.82 m): the check must flag it.
    """
    from raceline.track import Track, load_track

    track = load_track(TRACK)
    assert _too_narrow(track) == []
    thin = Track(
        track.name,
        track.xy,
        track.curvature,
        0.7 * track.w_left,
        0.7 * track.w_right,
    )
    assert _too_narrow(thin), "widths x0.7 passed the minimum-width check"


def test_raceline_1207_porto_scenario_points_at_the_porto_files():
    """RACELINE-1207: the scenario resolves to Porto's track and map.

    Reads the YAML only (no ROS import). SIMJAX-1060 additionally checks, for
    every scenario, that ``parameters:`` set no gym ``Param`` field.
    """
    import yaml

    doc = yaml.safe_load(SCENARIO.read_text())
    assert doc["scenario_id"] == "porto"
    assert doc["metrics"] == {
        "scenario_id": "porto",
        "track_version": "porto-v1",
    }
    track = (SCENARIO.parent / doc["track"]).resolve()
    assert track == TRACK
    from raceline.track import load_track

    assert load_track(track).name == "Porto"
    maps = (SCENARIO.parent / doc["map_directory"]).resolve()
    assert (maps / doc["environment"]["map"] / "Porto.yaml").is_file()
    assert "noise" not in doc
