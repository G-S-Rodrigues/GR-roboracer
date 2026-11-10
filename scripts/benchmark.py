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

import argparse
import dataclasses
import json
import sys
import tempfile
import time
from collections.abc import Mapping, Sequence
from datetime import UTC, datetime
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
# `python3 scripts/benchmark.py` puts scripts/ first, not the repo root, so
# `from scripts.x import ...` and the probes' own `scripts.` import need both.
for _path in (REPO, REPO / "tests" / "lib"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from scripts import (  # noqa: E402
    benchmark_runs,  # noqa: E402
    pose_source_report,
    raceline_report,
)
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


# -- run ----------------------------------------------------------------------

SCORED_AGAINST = "simulator ground truth (f1tenth_gym_jax)"
LEGACY_SCORED_AGAINST = "simulator ground truth (legacy nightly artifact)"
# Localization pivots: one table per scenario for each of these record fields.
LOCALIZATION_FIELDS = ("ate_rmse", "availability", "map_wall_median")
DEFAULT_OUT = REPO / "log" / "benchmark"


@dataclasses.dataclass(frozen=True)
class Matrix:
    roles: list[str]
    seeds: list[int]
    scenarios: list[str]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    run = commands.add_parser("run", help="drive a role's matrix")
    run.add_argument("--role", choices=(*ROLE_ORDER, "all"), default="all")
    run.add_argument(
        "--seeds",
        nargs="+",
        type=int,
        default=list(benchmark_runs.SEEDS),
    )
    run.add_argument(
        "--scenarios",
        nargs="+",
        default=list(pose_source_report.SCENARIOS),
        help="localization only",
    )
    run.add_argument("--out", type=Path, default=DEFAULT_OUT)
    render = commands.add_parser("render", help="re-render a saved report.json")
    render.add_argument("report", type=Path)
    history = commands.add_parser(
        "history",
        help="tabulate downloaded nightly quality reports (write_report)",
    )
    history.add_argument("dirs", nargs="+", type=Path)
    history.add_argument(
        "--field",
        action="append",
        default=None,
        help="a flattened numbers key, e.g. live.ate_rmse; repeatable",
    )
    return parser


def plan_matrix(args: argparse.Namespace) -> Matrix:
    """Validate ``run``'s arguments; exits 2 with the reason on a bad one."""
    parser = build_parser()
    if len(set(args.seeds)) != len(args.seeds):
        parser.error(f"--seeds repeats a seed: {args.seeds}")
    for scenario in args.scenarios:
        if scenario == "analytic_circle":
            parser.error(
                "localization on analytic_circle is refused: it is a "
                "rotationally symmetric annulus, so scan matching cannot "
                "observe position along the track and its error is "
                "unmeasurable (gotcha #18)"
            )
        if scenario not in pose_source_report.SCENARIOS:
            parser.error(
                f"unknown scenario {scenario!r}: expected one of "
                f"{list(pose_source_report.SCENARIOS)}"
            )
    return Matrix(
        roles=dispatch_roles(args.role),
        seeds=list(args.seeds),
        scenarios=list(args.scenarios),
    )


def _host_load() -> dict:
    try:
        from racing_test_keywords.scenario_runner import (  # noqa: PLC0415
            read_host_load,
        )

        return read_host_load()
    except Exception:  # noqa: BLE001 - a header field, never a failure
        return {}


def _revision() -> str:
    from racing_test_keywords import report  # noqa: PLC0415

    return report.revision()


def _write(out: Path, role: str, document: dict) -> None:
    folder = Path(out) / role
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "report.md").write_text(render_report(document), encoding="utf-8")
    (folder / "report.json").write_text(
        json.dumps(document, indent=2, default=str), encoding="utf-8"
    )


def _block(role, started, seeds, scenarios) -> dict:
    return {
        "role": role,
        "revision": _revision(),
        "started_utc": started.isoformat(),
        "host_load": _host_load(),
        "scored_against": SCORED_AGAINST,
        "seeds": list(seeds),
        "scenarios": list(scenarios),
    }


def _run_localization(seeds, scenarios, out, started, tmp) -> list[str]:
    keys = benchmark_runs.launch_keys()
    present, unknown = pose_source_report.present_sources(keys)
    length = benchmark_runs.track_length()
    expected = {"launch_keys": keys, "scenarios": scenarios, "seeds": seeds}
    records: list[dict] = []
    try:
        for source in pose_source_report.METHODS:
            for scenario in scenarios:
                for seed in seeds:
                    if source in present:
                        row = benchmark_runs.localization_run(
                            source, scenario, seed, length, tmp
                        )
                    else:
                        row = pose_source_report.absent_record(
                            source, scenario, seed
                        )
                    if row is None:
                        continue
                    records.append(row)
                    print(
                        f"localization {source}/{scenario}/seed {seed}: "
                        f"{row['outcome']}, ate_rmse {row['ate_rmse']}, "
                        f"pose {row['pose_rate_hz']} Hz, "
                        f"cpu {row['cpu_ms_per_scan']} ms/scan, "
                        f"map median {row['map_wall_median']}",
                        flush=True,
                    )
    finally:
        _write(
            out,
            "localization",
            {
                "benchmark": _block("localization", started, seeds, scenarios),
                "records": records,
                "unknown": unknown,
                "constants": {
                    "seeds": list(seeds),
                    "time_scale": benchmark_runs.TIME_SCALE,
                    "timeout_seconds": (
                        benchmark_runs.LOCALIZATION_TIMEOUT_SECONDS
                    ),
                    "scenarios": {
                        name: pose_source_report.SCENARIOS[name]
                        for name in scenarios
                    },
                },
                "reference": benchmark_runs.reference_row(),
            },
        )
    return pose_source_report.structural_failures(records, expected)


def _run_planning(seeds, out, started, tmp, session_start) -> list[str]:
    vehicles = benchmark_runs.VEHICLES
    expected = {
        "vehicles": {name: version for name, (_, version) in vehicles.items()},
        "default_vehicle": benchmark_runs.DEFAULT_VEHICLE,
        "seeds": list(seeds),
        "rows": {
            name: benchmark_runs.lines_of(version)
            for name, (_, version) in vehicles.items()
        },
    }
    length = benchmark_runs.track_length()
    runs: list[dict] = []
    try:
        for vehicle, lines in expected["rows"].items():
            for line in lines:
                for seed in seeds:
                    run = benchmark_runs.planning_run(
                        vehicle, line, seed, length, tmp
                    )
                    if run is None:
                        continue
                    runs.append(run)
                    print(
                        f"planning {vehicle}/{line}/seed {seed}: "
                        f"{run['outcome']}, lap "
                        f"{run.get('metrics', {}).get('lap_time')}, "
                        f"distance {run.get('distance_m')}",
                        flush=True,
                    )
    finally:
        regeneration = benchmark_runs.regeneration_record(session_start)
        statics = {}
        for vehicle, (_, version) in vehicles.items():
            for line in expected["rows"][vehicle]:
                fields = benchmark_runs.static_fields(version, line)
                key = f"spielberg/{version}/{line}.yaml"
                if key in regeneration:
                    fields["solve_seconds"] = regeneration[key]["solve_seconds"]
                statics[(vehicle, line)] = fields
        _write(
            out,
            "planning",
            {
                "benchmark": _block("planning", started, seeds, ["spielberg"]),
                "rows": raceline_report.summarise(runs, statics),
                "runs": runs,
                "regeneration": regeneration,
                "reference": raceline_report.upstream_reference(
                    benchmark_runs.UPSTREAM, "Spielberg"
                ),
            },
        )
    return raceline_report.structural_failures(runs, expected)


def run_role(
    role: str, seeds: Sequence[int], scenarios: Sequence[str], out: Path
) -> list[str]:
    """Drive ``role``'s matrix, write ``<out>/<role>/report.{md,json}`` (also
    when a launch raises) and return ``structural_failures``: why the data is
    broken, empty when it is not. Never judges a value."""
    started = datetime.now(UTC)
    session_start = time.time()
    with tempfile.TemporaryDirectory(prefix="benchmark-") as folder:
        tmp = Path(folder)
        if role == "localization":
            return _run_localization(
                list(seeds), list(scenarios), out, started, tmp
            )
        if role == "planning":
            return _run_planning(list(seeds), out, started, tmp, session_start)
    raise ValueError(f"unknown role {role!r}")


# -- render -------------------------------------------------------------------


def _unique(values) -> list:
    return list(dict.fromkeys(values))


def _header(role: str, block: dict) -> list[str]:
    load = block.get("host_load") or {}
    return [
        f"# Benchmark: {role}",
        "",
        "Research output, not a gate (ADR 0005, ADR 0008 section 5): nothing "
        "here ranks or bounds a value.",
        "",
        f"- revision: {block.get('revision', 'unknown')}",
        f"- started (UTC): {block.get('started_utc', 'unknown')}",
        f"- host load: {json.dumps(load) if load else 'not recorded'}",
        f"- scored against: {block['scored_against']}",
        "",
    ]


def _render_localization(document: dict, block: dict) -> str:
    records = document["records"]
    seeds = block.get("seeds") or _unique(r["seed"] for r in records)
    scenarios = block.get("scenarios") or _unique(
        r["scenario"] for r in records
    )
    out = _header("localization", block)
    for scenario in scenarios:
        for field in LOCALIZATION_FIELDS:
            table = {
                (r["source"], r["seed"]): r[field]
                for r in records
                if r["scenario"] == scenario
            }
            out += [
                f"## {scenario}: {field}",
                "",
                pivot(table, list(pose_source_report.METHODS), seeds),
            ]
    out += ["## Every record", "", pose_source_report.render(records)]
    out.append(
        "Reference (committed survey grid, this metric): "
        + json.dumps(document.get("reference"))
    )
    return "\n".join(out) + "\n"


def _render_planning(document: dict, block: dict) -> str:
    runs = document["runs"]
    seeds = block.get("seeds") or _unique(r["seed"] for r in runs)
    keys = [(r["vehicle"], r["line"]) for r in document["rows"]]
    table = {((r["vehicle"], r["line"]), r["seed"]): r for r in runs}
    out = _header("planning", block)
    out += [
        "Regeneration is measured by the nightly's SIM-5040, not here.",
        "",
        "## Lap time s (DNF = distance reached)",
        "",
        pivot(table, keys, seeds, lap_cell),
    ]
    out.append(
        raceline_report.render(
            document["rows"],
            document.get("regeneration") or {},
            document.get("reference"),
        )
    )
    return "\n".join(out) + "\n"


def render_report(document: dict) -> str:
    """Markdown for a saved run: this command's own ``report.json`` or a
    legacy nightly ``pose-source-report`` / ``raceline-report`` artifact."""
    block = dict(document.get("benchmark") or {})
    block.setdefault("scored_against", LEGACY_SCORED_AGAINST)
    if "records" in document:
        return _render_localization(document, block)
    if "runs" in document and "rows" in document:
        return _render_planning(document, block)
    raise ValueError("not a benchmark report: no records, or rows and runs")


# -- history ------------------------------------------------------------------


def _flatten(value, prefix: str = "") -> dict:
    """Scalar leaves of nested dicts, keyed ``a.b``; lists are skipped."""
    if isinstance(value, Mapping):
        out: dict = {}
        for key, item in value.items():
            out.update(_flatten(item, f"{prefix}.{key}" if prefix else key))
        return out
    if isinstance(value, (list, tuple)):
        return {}
    return {prefix: value}


def history(
    dirs: Sequence[Path], fields: Sequence[str] | None = None
) -> tuple[str, list[Path]]:
    """(markdown, skipped files): one table per report ID from the
    ``write_report`` records under each run directory, one row per record.

    Rows are ordered by run directory (as given), revision and seed, never by
    value. Reads files only. A JSON file that is not a report is skipped."""
    by_id: dict[str, list[tuple]] = {}
    skipped: list[Path] = []
    for order, folder in enumerate(dirs):
        for path in sorted(Path(folder).rglob("*.json")):
            try:
                doc = json.loads(path.read_text(encoding="utf-8"))
                report_id, numbers = doc["id"], doc["numbers"]
                if not isinstance(numbers, Mapping):
                    raise TypeError("numbers is not a mapping")
            except (OSError, ValueError, KeyError, TypeError):
                skipped.append(path)
                continue
            seed = doc.get("seed")
            sort = (
                order,
                str(doc.get("revision")),
                -1 if seed is None else seed,
                str(doc.get("implementation")),
            )
            row = (
                Path(folder).name,
                str(doc.get("revision", "unknown"))[:7],
                seed,
                doc.get("implementation"),
            )
            by_id.setdefault(report_id, []).append(
                (sort, row, _flatten(numbers))
            )
    out: list[str] = []
    for report_id in by_id:
        entries = sorted(by_id[report_id], key=lambda e: e[0])
        columns = (
            list(fields)
            if fields
            else _unique(key for _, _, leaves in entries for key in leaves)
        )
        head = ["run", "revision", "seed", "implementation", *columns]
        out += [
            f"## {report_id}",
            "",
            "| " + " | ".join(head) + " |",
            "|" + "---|" * len(head),
        ]
        for _, row, leaves in entries:
            cells = [*row, *(leaves.get(c) for c in columns)]
            out.append("| " + " | ".join(cell(c) for c in cells) + " |")
        out.append("")
    return "\n".join(out), skipped


# -- main ---------------------------------------------------------------------


def _command_run(args: argparse.Namespace) -> int:
    matrix = plan_matrix(args)
    broken = False
    for role in matrix.roles:
        failures = run_role(role, matrix.seeds, matrix.scenarios, args.out)
        print((Path(args.out) / role / "report.md").read_text(encoding="utf-8"))
        if failures:
            broken = True
            print(f"STRUCTURAL FAILURES ({role}):")
            print("\n".join(failures))
    return 1 if broken else 0


def _command_render(args: argparse.Namespace) -> int:
    try:
        document = json.loads(args.report.read_text(encoding="utf-8"))
        text = render_report(document)
    except (OSError, ValueError, KeyError, TypeError) as error:
        print(f"benchmark render: {args.report}: {error}", file=sys.stderr)
        raise SystemExit(2) from error
    print(text, end="")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "run":
        return _command_run(args)
    if args.command == "render":
        return _command_render(args)
    if args.command == "history":
        text, skipped = history(args.dirs, args.field)
        for path in skipped:
            print(f"skipped (not a quality report): {path}", file=sys.stderr)
        print(text, end="")
        return 0
    raise SystemExit(2)


if __name__ == "__main__":
    raise SystemExit(main())
