"""Quality reports and the catastrophic bounds (ADR 0008).

A tier-3/4/5 test gates on "it works" and reports how well. Every quality
number goes through :func:`write_report` to ``log/reports/<ID>.json`` and a
``REPORT <ID> <json>`` line; the only quality assert left is a catastrophic
bound from :data:`CATASTROPHIC_BOUNDS`, one value per metric, argued from
geometry and never from a measured spread.

This module imports no ROS, so tier 1 can test it.
"""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
from collections.abc import Mapping
from pathlib import Path
from typing import Any

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
REPORT_ROOT = REPOSITORY_ROOT / "log" / "reports"

_ID_PATTERN = re.compile(r"^(SIM|ACC)-\d{4}(-[a-z0-9_.]+)*$")

# One value per metric, shared by every implementation, track and seed.
# Maximum/minimum are inclusive, as compare_localization's are. The reason is
# ADR 0008 section 2; a bound that fires on a working method is loosened by
# argument in a reviewed PR, never re-measured.
CATASTROPHIC_BOUNDS: dict[str, dict[str, Any]] = {
    "ate_rmse": {
        "maximum": 5.0,
        "measurement": (
            "about 4.5 Spielberg half-widths (1.1 m): an estimate that far "
            "off is lost; every working run measured is <= 2.94 m, measured "
            "failures are 6.93-8.17 m and 37.7 m (ADR 0008)"
        ),
    },
    "availability": {
        "minimum": 0.90,
        "measurement": (
            "an estimate missing for more than a tenth of the lap is a "
            "stalled estimator; every working run measured is >= 0.994 "
            "(ADR 0008)"
        ),
    },
    "map_median": {
        "maximum": 1.0,
        "measurement": (
            "under a Spielberg half-width (1.1 m); measured 0.20-0.56 m "
            "(ADR 0008)"
        ),
    },
    "wall_cells": {
        "minimum": 21800,
        "measurement": (
            "half the fewest measured (43 631): a map with half its walls "
            "missing is not a map (ADR 0008)"
        ),
    },
}


def catastrophic(
    metric: str, field: str | None = None
) -> dict[str, dict[str, Any]]:
    """A `compare_localization` bounds dict for one catastrophic metric.

    Only for the offline score fields (`ate_rmse`, `availability`): the
    comparator refuses any other field name."""
    return {field or metric: dict(CATASTROPHIC_BOUNDS[metric])}


def catastrophic_violations(
    actual: Mapping[str, Any], fields: Mapping[str, str]
) -> list[str]:
    """Messages for each `actual[key]` outside the catastrophic bound of
    `fields[key]`. Same rule as `compare_localization`: the bound itself is
    inside, and NaN is outside every bound."""
    messages: list[str] = []
    for key, metric in fields.items():
        bound = CATASTROPHIC_BOUNDS[metric]
        value = float(actual[key])
        for kind in ("maximum", "minimum"):
            if kind not in bound:
                continue
            limit = bound[kind]
            inside = (
                value <= limit if kind == "maximum" else value >= limit
            ) and not math.isnan(value)
            if not inside:
                messages.append(
                    f"{key} = {value!r} is outside the catastrophic "
                    f"{kind} {limit} of {metric}"
                )
    return messages


def _git_revision() -> str | None:
    def run(env: Mapping[str, str]) -> str | None:
        try:
            done = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=REPOSITORY_ROOT,
                env=dict(env),
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        value = done.stdout.strip()
        return value if done.returncode == 0 and value else None

    found = run(os.environ)
    if found:
        return found
    # A worktree made on the host: plain git fails in dev (gotcha #23), so
    # reuse changed_scope's trust-gated locator (CHECK-1350..1370).
    scripts = str(REPOSITORY_ROOT / "scripts")
    try:
        if scripts not in sys.path:
            sys.path.insert(0, scripts)
        import changed_scope

        location = changed_scope.locate_git_dir(REPOSITORY_ROOT, os.environ)
    except Exception:
        return None
    if location is None:
        return None
    env = dict(os.environ)
    env["GIT_DIR"] = str(location.git_dir)
    env["GIT_WORK_TREE"] = str(location.work_tree)
    return run(env)


def revision() -> str:
    """The code revision a report was made at, never raising: RACING_REVISION,
    GITHUB_SHA, git, git behind changed_scope's locator, then "unknown"."""
    for name in ("RACING_REVISION", "GITHUB_SHA"):
        value = os.environ.get(name, "").strip()
        if value:
            return value
    try:
        return _git_revision() or "unknown"
    except Exception:
        return "unknown"


def _host_load() -> dict[str, float]:
    try:
        from .scenario_runner import read_host_load
    except Exception:
        return {}
    return read_host_load()


def _strict(value: Any) -> Any:
    """JSON-safe: non-finite floats become None, containers recurse."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(key): _strict(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_strict(item) for item in value]
    if hasattr(value, "item") and not isinstance(value, (str, bytes)):
        try:
            return _strict(value.item())
        except (TypeError, ValueError):
            pass
    return value


def write_report(
    report_id: str,
    numbers: Mapping[str, Any],
    *,
    seed: int | None,
    scenario: str,
    implementation: str,
    baseline: Mapping[str, Any] | None = None,
) -> Path:
    """Write ``log/reports/<report_id>.json`` and print one REPORT line.

    Called right after the run and before the first quality assert, so a red
    run keeps its record. Nothing here ever fails a test on a number."""
    if not _ID_PATTERN.match(report_id):
        raise ValueError(f"not a path-safe report id: {report_id!r}")
    record = _strict(
        {
            "id": report_id,
            "numbers": numbers,
            "baseline": baseline,
            "seed": seed,
            "scenario": scenario,
            "implementation": implementation,
            "revision": revision(),
            "host_load": _host_load(),
        }
    )
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    path = REPORT_ROOT / f"{report_id}.json"
    path.write_text(json.dumps(record, indent=2, allow_nan=False) + "\n")
    print(
        f"REPORT {report_id} "
        + json.dumps(record, separators=(",", ":"), allow_nan=False),
        flush=True,
    )
    return path
