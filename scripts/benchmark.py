#!/usr/bin/env python3
"""The benchmark command: implementations of one role, side by side.

``run`` drives a role's matrix (implementation x seed) and prints the tables,
``render`` re-renders a saved run, ``roles`` names the roles a PR's new
implementations belong to, and ``history`` tabulates the nightly's quality
reports. It ranks nothing and gates nothing (ADR 0005, ADR 0008 section 5):
the only failure it reports is structural, meaning the data is broken. It is
run on demand, and by the Benchmark workflow on a PR that adds an
implementation to ``config/reference_stack.yaml``; never on a schedule.

Everything above ``run_role`` is pure (stdlib plus a lazy ``yaml``); the code
that launches the stack lives in ``scripts/benchmark_runs.py``.
"""

from __future__ import annotations

import sys
from collections.abc import Mapping
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
# `python3 scripts/benchmark.py` puts scripts/ first, not the repo root, so
# `from scripts.x import ...` and the probes' own `scripts.` import need both.
for _path in (REPO, REPO / "tests" / "lib"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from scripts import benchmark_runs  # noqa: E402
from scripts.changed_scope import stack_from_dict  # noqa: E402
from scripts.pose_source_report import cell  # noqa: E402

# reference_stack role -> the benchmark role that compares its implementations.
BENCHMARKED_ROLES = {"pose_source": "localization", "planner": "planning"}
# Implementations of a benchmarked role that are deliberately not rows.
NOT_BENCHMARKED = {
    "pose_source": {
        "ground_truth": (
            "not an estimator; publishes no composed /localization/odom "
            "(SIM-5510 docstring)"
        )
    }
}
# planner implementation -> the lines of a vehicle parameter version it
# drives: the track centerline, or each committed raceline.
PLANNER_RUNNABLE = {
    "track_centerline": lambda version: ["centerline"],
    "optimized_raceline": lambda version: [
        line
        for line in benchmark_runs.lines_of(version)
        if line != "centerline"
    ],
}
# The benchmark roles in the order they run and print.
ROLE_ORDER = ("localization", "planning")


def _implementations(stack: Mapping) -> set[tuple[str, str]]:
    parsed = stack_from_dict(stack, {}, {})
    return {(impl.role, impl.name) for impl in parsed.implementations}


def added_implementations(
    base: Mapping, head: Mapping
) -> list[tuple[str, str]]:
    """(role, implementation) pairs in ``head`` and not in ``base``, in the
    order ``head`` lists them. A promotion or a paths edit is not an addition;
    a rename is."""
    before = _implementations(base)
    parsed = stack_from_dict(head, {}, {})
    return [
        (impl.role, impl.name)
        for impl in parsed.implementations
        if (impl.role, impl.name) not in before
    ]


def benchmark_roles(
    added: list[tuple[str, str]],
) -> tuple[list[str], list[tuple[str, str]]]:
    """(benchmark roles to run, additions in a role with no benchmark)."""
    wanted = {BENCHMARKED_ROLES[r] for r, _ in added if r in BENCHMARKED_ROLES}
    unbenchmarked = [(r, n) for r, n in added if r not in BENCHMARKED_ROLES]
    return [role for role in ROLE_ORDER if role in wanted], unbenchmarked


def dispatch_roles(role: str) -> list[str]:
    """What a manual dispatch of ``role`` runs: ``all`` is both roles."""
    if role == "all":
        return list(ROLE_ORDER)
    if role in ROLE_ORDER:
        return [role]
    raise ValueError(
        f"unknown role {role!r}: expected one of {ROLE_ORDER} or all"
    )


def lap_cell(run) -> str:
    """A planning run as a table cell: the lap time, ``DNF <m> m`` with the
    distance reached, or the outcome word. A DNF is never a time."""
    if run is None:
        return "-"
    if run["outcome"] == "completed":
        return f"{run['metrics']['lap_time']:.2f}"
    if run["outcome"] == "dnf":
        return f"DNF {run['distance_m']:.1f} m"
    return str(run["outcome"])


def pivot(table: Mapping, row_keys, seeds, cell_of=cell) -> str:
    """An implementation x seed markdown table.

    ``table`` maps ``(row_key, seed)`` to a value; ``cell_of`` formats it. Rows
    appear in ``row_keys`` order and columns in ``seeds`` order: never sorted
    by value, because nothing here ranks. A missing (row, seed) is ``-``."""
    header = ["implementation", *map(str, seeds)]
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    for key in row_keys:
        label = " / ".join(key) if isinstance(key, tuple) else str(key)
        cells = [
            cell_of(table[(key, seed)]) if (key, seed) in table else "-"
            for seed in seeds
        ]
        lines.append("| " + " | ".join([label, *cells]) + " |")
    return "\n".join(lines) + "\n"
