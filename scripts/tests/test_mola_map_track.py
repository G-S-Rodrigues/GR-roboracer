"""CHECK-2484: `scripts/mola_map_track.sh --from-simplemap <file> --dry-run`.

The script drives a live lap; here it only has to parse its options. The test
runs a COPY of the script in a tmp tree that has no `install/setup.bash` and
no `ros2`, so that a script which does not yet know the options fails at its
`source` (the ERR trap's line) before it can `rm` or launch anything, and a
script that does know them prints its plan and touches no file.
"""

from __future__ import annotations

import json
import os
import shutil
import stat
import subprocess
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "mola_map_track.sh"


def _tree(tmp_path: Path) -> Path:
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    shutil.copy(SCRIPT, scripts / "mola_map_track.sh")
    return tmp_path


def _snapshot(root: Path) -> list[str]:
    return sorted(str(p.relative_to(root)) for p in root.rglob("*"))


def test_check_2484_from_simplemap_dry_run_touches_nothing(tmp_path) -> None:
    """CHECK-2484: the script passes `bash -n`; with `--from-simplemap F
    --dry-run <scenario> <stem> <seed>` (options before the positionals) it
    exits 0 before the `cd`, the `source`, the `rm` and any launch, prints
    that it will skip the lap and start from F, and creates no file in the
    tree (the stem's directory included)."""
    assert subprocess.run(["bash", "-n", str(SCRIPT)]).returncode == 0
    root = _tree(tmp_path)
    simplemap = tmp_path / "lap.simplemap"
    simplemap.write_bytes(b"x")
    before = _snapshot(root)
    done = subprocess.run(
        [
            "bash",
            str(root / "scripts" / "mola_map_track.sh"),
            "--from-simplemap",
            str(simplemap),
            "--dry-run",
            "config/scenarios/porto.yaml",
            "out/Porto",
            "7",
        ],
        capture_output=True,
        text=True,
        timeout=60,
        cwd=root,
    )
    assert done.returncode == 0, (done.stdout, done.stderr)
    assert "failed at line" not in done.stderr, done.stderr
    assert f"from simplemap {simplemap}" in done.stdout, done.stdout
    assert _snapshot(root) == before


_FAKES = {
    # `sm-cli export-keyframes <simplemap> --output <tum>` writes the TUM the
    # test chose; `sm-cli tf <in> <out> <pose>` copies.
    "sm-cli": """#!/bin/bash
case "$1" in
export-keyframes) cp "$KFS_TUM" "$4" ;;
tf) cp "$2" "$3" ;;
esac
""",
    "sm2mm": """#!/bin/bash
while [ $# -gt 0 ]; do
    [ "$1" = -o ] && echo mm >"$2"
    shift
done
""",
    "mm2grid": """#!/bin/bash
while [ $# -gt 0 ]; do
    [ "$1" = -o ] && { echo png >"$2.png"; echo yaml >"$2.yaml"; }
    shift
done
""",
    "dpkg-query": "#!/bin/bash\necho -n 1.0-1\n",
}


def _truth(stamp: float, x: float) -> dict:
    sec = int(stamp)
    return {
        "record_type": "event",
        "stamp": {"sec": sec, "nanosec": round((stamp - sec) * 1e9)},
        "topic": "/ground_truth/odom",
        "payload": {"x": x, "y": 0.0, "qz": 0.0, "qw": 1.0},
    }


def _run_from_simplemap(
    tmp_path: Path,
    truth: list[tuple[float, float]],
    keyframe: tuple[float, float],
    env_extra: dict | None = None,
):
    """Run the copied script `--from-simplemap` over fakes: `truth` is
    (stamp, x) records, `keyframe` the first keyframe's (stamp, x)."""
    tmp_path.mkdir(parents=True, exist_ok=True)
    root = _tree(tmp_path)
    (root / "install").mkdir()
    (root / "install" / "setup.bash").write_text("")
    fakes = tmp_path / "fakes"
    fakes.mkdir()
    for name, body in _FAKES.items():
        fake = fakes / name
        fake.write_text(body)
        fake.chmod(fake.stat().st_mode | stat.S_IXUSR)
    recording = tmp_path / "rec.jsonl"
    recording.write_text(
        "\n".join(json.dumps(_truth(t, x)) for t, x in truth) + "\n"
    )
    tum = tmp_path / "kfs.tum"
    tum.write_text(
        f"{keyframe[0]:.6f} {keyframe[1]} 0 0 0 0 0 1\n"
        f"{keyframe[0] + 0.05:.6f} {keyframe[1] + 0.2} 0 0 0 0 0 1\n"
    )
    simplemap = tmp_path / "lap.simplemap"
    simplemap.write_bytes(b"x")
    done = subprocess.run(
        [
            "bash",
            str(root / "scripts" / "mola_map_track.sh"),
            "--from-simplemap",
            str(simplemap),
            "config/scenarios/porto.yaml",
            "out/Porto",
            "7",
        ],
        capture_output=True,
        text=True,
        timeout=60,
        cwd=root,
        env={
            **os.environ,
            "PATH": f"{fakes}:{os.environ['PATH']}",
            "KFS_TUM": str(tum),
            "MOLA_RECORDING": str(recording),
            **(env_extra or {}),
        },
    )
    return root, done


def test_check_2486_t_is_taken_at_the_first_keyframe_stamp(tmp_path) -> None:
    """CHECK-2486 (a), (b): with truth at 100.0 (A, x = 1) and at 102.56 (B,
    x = 2) and the simplemap's first keyframe, at the origin, stamped
    102.56, `T_map_mola` is B, `T_stamp` is 102.56, `T_stamp_source` is
    `simplemap_first_keyframe` and the yaml header names it; a `FIRST_STAMP`
    in the environment (the old input) changes nothing."""
    for name, env in (("a", None), ("b", {"FIRST_STAMP": "100.0"})):
        root, done = _run_from_simplemap(
            tmp_path / name,
            [(100.0, 1.0), (102.56, 2.0)],
            (102.56, 0.0),
            env,
        )
        assert done.returncode == 0, (name, done.stdout, done.stderr)
        text = (root / "out" / "Porto.yaml").read_text()
        provenance = yaml.safe_load(text)
        assert provenance["T_map_mola"]["x"] == 2.0, (name, provenance)
        assert provenance["T_stamp"] == 102.56, (name, provenance)
        assert provenance["T_stamp_source"] == "simplemap_first_keyframe"
        assert provenance["T_source"] == "first_processed_scan"
        header = "".join(
            line for line in text.splitlines() if line.startswith("#")
        )
        assert "first keyframe" in header, text


def test_check_2486_no_truth_within_1_ms_fails_loudly(tmp_path) -> None:
    """CHECK-2486 (c): truth only at 102.50 and 102.62 (no record within 1 ms
    of the keyframe at 102.56): exit != 0, stderr names 102.56 and no
    provenance is written."""
    root, done = _run_from_simplemap(
        tmp_path, [(102.5, 1.0), (102.62, 2.0)], (102.56, 0.0)
    )
    assert done.returncode != 0, (done.stdout, done.stderr)
    assert "102.56" in done.stderr, done.stderr
    assert not (root / "out" / "Porto.yaml").exists()


def test_check_2486_first_keyframe_off_the_origin_fails(tmp_path) -> None:
    """CHECK-2486 (d): a first keyframe at x = 0.5 is not MOLA's origin, and
    composition is not built: exit != 0, stderr names the keyframe."""
    root, done = _run_from_simplemap(tmp_path, [(102.56, 2.0)], (102.56, 0.5))
    assert done.returncode != 0, (done.stdout, done.stderr)
    assert "origin" in done.stderr and "0.5" in done.stderr, done.stderr
    assert not (root / "out" / "Porto.yaml").exists()
