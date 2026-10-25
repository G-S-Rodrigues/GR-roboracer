"""Tier 1: AutoDRIVE's Porto track in the tree (#44 part 3).

The map is AutoDRIVE-RoboRacer-Racetracks' ``Legacy Tracks/Porto Track``
at a pinned commit; ``config/tracks/porto.yaml`` is extracted from it by
``python -m raceline.track_from_map``. The start (0, 0, 0) and the ``ccw``
direction are an assumption that part 10 checks against the simulator.
"""

import hashlib
from pathlib import Path

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
