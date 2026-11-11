"""CHECK-2484: `scripts/mola_map_track.sh --from-simplemap <file> --dry-run`.

The script drives a live lap; here it only has to parse its options. The test
runs a COPY of the script in a tmp tree that has no `install/setup.bash` and
no `ros2`, so that a script which does not yet know the options fails at its
`source` (the ERR trap's line) before it can `rm` or launch anything, and a
script that does know them prints its plan and touches no file.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

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
