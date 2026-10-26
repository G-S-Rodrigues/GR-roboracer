"""SIM-5060: each committed minimum-time line regenerates from its committed
reference, but only on a night the optimizer code changed (Q8, owner E-3).

A min-time solve is up to ~20 minutes (`MINTIME_MAX_WALL_S`), and its
determinism is a property of the optimizer code and the pinned fork, so the
test asks first whether that code differs from what the committed yaml was
proven with: the `tum:` pins and the blobs of `MINTIME_CODE_FILES`
(`mintime_code_changed`, CHECK-1570).

- **Unchanged:** nothing is run. The test records
  `{"regenerated": false, "reason": ...}` per line in
  `log/raceline-report/regeneration-mintime.json` and passes. This is a
  deliberate skip by ruling, written in the test body (no `skip`/`xfail`),
  and the record says so.
- **Changed:** the recorded command is re-run with `--reference <the committed
  reference>`, so the solve starts from the same file the line was seeded
  from. The inputs must match exactly, `tum:` and `code:` excepted (a fork-pin
  bump makes the fresh `tum:` differ by construction; both are reported), and
  the line must lie within `REGEN_XY_TOL` / `REGEN_V_TOL` of the committed one.

Every run prints the IPOPT outcome (`solver:`) and the changed files.
"""

from __future__ import annotations

import json
import subprocess
import time
from pathlib import Path

import pytest
import yaml

from scripts.raceline_report import (
    compare_lines,
    mintime_code_changed,
    provenance_diff,
    regeneration_argv,
)

REPO = Path(__file__).resolve().parents[2]
REPORT_DIR = REPO / "log" / "raceline-report"
RACELINES = sorted(
    path
    for path in (REPO / "config" / "racelines").glob("*/*/*.yaml")
    if yaml.safe_load(path.read_text())["optimizer"] == "tum_mintime"
)
# MINTIME_MAX_WALL_S (1200) for the solve plus the min-curv stages that make
# the reference and the verifier; with --reference only the solve remains, so
# this is the solve bound plus a margin (E-5).
SOLVE_TIMEOUT_SECONDS = 1260
# Same bounds as SIM-5040 (owner item 6); the procedure's measurement for the
# min-time lines is Step 9's SIM-5040 rerun of the same code, which printed
# max dxy 0 and max dv 0 on both lines (bitwise determinism, also 1-thread
# Nehalem in 8F). Not to be widened to green a run (repo-gotchas #14).
MEASUREMENT = (
    "min-time lines regenerated bitwise identical in dev (8F/Step 1: 1-thread "
    "Nehalem too); max dxy 0, max dv 0"
)
REGEN_XY_TOL = 0.01
REGEN_V_TOL = 0.01

RECORDS: dict[str, dict] = {}


def current_fingerprint() -> dict:
    """The `tum:` pins and `MINTIME_CODE_FILES` blobs as the CLI would record
    them now. Module-level so the RED test can replace it."""
    program = (
        "import json\n"
        "from raceline import __main__ as m\n"
        "print(json.dumps({'tum': m._tum_pin(), 'code': {p: m._blob("
        "m.PACKAGE / p) for p in m.MINTIME_CODE_FILES}}))\n"
    )
    done = subprocess.run(
        [
            "env",
            "-u",
            "PYTHONPATH",
            "uv",
            "run",
            "--project",
            "raceline",
            "--locked",
            "python",
            "-c",
            program,
        ],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=True,
    )
    return json.loads(done.stdout.strip().splitlines()[-1])


def regenerate(doc: dict, out_dir: Path) -> tuple[Path, dict, float]:
    """Run the recorded command from the committed reference; returns
    (fresh csv, fresh yaml, seconds). Module-level so the RED mutation can
    replace it with a stand-in that pays for no solve."""
    argv = regeneration_argv(doc, out_dir) + [
        "--reference",
        doc["parameters"]["reference"]["path"],
    ]
    started = time.monotonic()
    done = subprocess.run(
        [
            "env",
            "-u",
            "PYTHONPATH",
            "uv",
            "run",
            "--project",
            "raceline",
            "--locked",
            *argv,
        ],
        cwd=REPO,
        timeout=SOLVE_TIMEOUT_SECONDS,
        capture_output=True,
        text=True,
        check=False,
    )
    seconds = time.monotonic() - started
    assert done.returncode == 0, (
        f"regeneration exited {done.returncode} (the CLI's verifier rejected "
        f"the line or the solve failed):\n{done.stderr[-2000:]}"
    )
    folder = out_dir / doc["track"] / doc["vehicle_version"]
    fresh_csv = folder / f"{doc['optimizer']}.csv"
    fresh_doc = yaml.safe_load(
        (folder / f"{doc['optimizer']}.yaml").read_text(encoding="utf-8")
    )
    return fresh_csv, fresh_doc, seconds


def _write_records() -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    (REPORT_DIR / "regeneration-mintime.json").write_text(
        json.dumps(RECORDS, indent=2, sort_keys=True), encoding="utf-8"
    )


def test_sim_5060_has_mintime_lines_to_regenerate() -> None:
    assert len(RACELINES) >= 2, f"found only {RACELINES}"


@pytest.mark.parametrize(
    "yaml_path",
    RACELINES,
    ids=[str(p.relative_to(REPO / "config" / "racelines")) for p in RACELINES],
)
def test_sim_5060_mintime_regenerates(yaml_path: Path, tmp_path: Path) -> None:
    committed = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    name = str(yaml_path.relative_to(REPO / "config" / "racelines"))
    changed = mintime_code_changed(committed, current_fingerprint())
    print(f"\nSIM-5060 {name}: committed solver {committed.get('solver')}")
    print(f"SIM-5060 {name}: optimizer code changed: {changed}")

    if not changed:
        RECORDS[name] = {
            "regenerated": False,
            "reason": "optimizer code unchanged since the committed line (Q8)",
            "changed": [],
        }
        _write_records()
        return

    fresh_csv, fresh_doc, seconds = regenerate(committed, tmp_path / "regen")
    measured = compare_lines(yaml_path.with_suffix(".csv"), fresh_csv)
    diff = provenance_diff(committed, fresh_doc)
    # `tum:` differs by construction when the trigger is a fork-pin bump;
    # reported below, never the failure.
    inputs = [d for d in diff["differences"] if not d.startswith("tum")]
    tum_diff = [d for d in diff["differences"] if d.startswith("tum")]

    RECORDS[name] = {
        "regenerated": True,
        "changed": changed,
        "solve_seconds": seconds,
        "solver": fresh_doc.get("solver"),
        "max_xy_m": measured["max_xy_m"],
        "max_dv_mps": measured["max_dv_mps"],
        "verdict_deltas": diff["verdict_deltas"],
        "code_drift": diff["code_drift"],
        "tum_differences": tum_diff,
    }
    _write_records()
    print(
        f"SIM-5060 {name}: solve {seconds:.0f} s, solver "
        f"{fresh_doc.get('solver')}, max dxy {measured['max_xy_m']:.6f} m, "
        f"max dv {measured['max_dv_mps']:.6f} m/s, tum differences "
        f"{tum_diff}, code drift {sorted(diff['code_drift'])}"
    )

    assert inputs == [], inputs
    assert measured["max_xy_m"] <= REGEN_XY_TOL, (
        f"{name}: regenerated line is {measured['max_xy_m']:.4f} m from the "
        f"committed one (bound {REGEN_XY_TOL} m)"
    )
    assert measured["max_dv_mps"] <= REGEN_V_TOL, (
        f"{name}: regenerated speed differs by {measured['max_dv_mps']:.4f} "
        f"m/s (bound {REGEN_V_TOL} m/s)"
    )
