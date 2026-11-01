"""Tier 1: the Porto grid our own stack mapped is extractable (#44 part 11a).

`config/scenarios/maps/Porto/slam/Porto.{png,yaml}` is what slam_toolbox's
survey mapper laid from a follow_the_gap lap of AutoDRIVE's Porto, on the
sim's dead-reckoned (perfect-gyro) odometry. 11b extracts the committed
track from it; this proves only that the corridor closes and the header
carries what 11b and a regeneration need.
"""

import re
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SLAM_DIR = REPO / "config" / "scenarios" / "maps" / "Porto" / "slam"
MAP_YAML = SLAM_DIR / "Porto.yaml"


def test_raceline_1520_the_mapped_porto_grid_is_extractable(tmp_path, capsys):
    """RACELINE-1520: `track_from_map --walls observed --direction cw` on the
    committed grid, from the start pose its own header records, exits 0 (the
    corridor closes and is wide enough). The header names the regenerate
    command, the perfect-gyro qualifier, and both mapping runs' numbers."""
    from raceline import track_from_map

    assert MAP_YAML.is_file(), f"{MAP_YAML} not committed"
    assert (SLAM_DIR / "Porto.png").is_file()
    header = "\n".join(
        line
        for line in MAP_YAML.read_text(encoding="utf-8").splitlines()
        if line.startswith("#")
    )
    assert "scripts/autodrive_qualify.sh --case mapping" in header
    assert "PERFECT-GYRO" in header
    for key in ("loop_closure:", "shape:", "agreement:"):
        assert key in header, key
    match = re.search(r"start_pose[^:]*: \[([^\]]+)\]", header)
    assert match, "the header records no start_pose"
    start = [v.strip() for v in match.group(1).split(",")]
    assert len(start) == 3
    out = tmp_path / "porto.yaml"
    code = track_from_map.main(
        [
            "--map", str(MAP_YAML),
            "--start-pose", *start,
            "--direction", "cw",
            "--walls", "observed",
            "--name", "probe",
            "--out", str(out),
        ]
    )  # fmt: skip
    assert code == 0, capsys.readouterr().err
    assert out.is_file()
