"""The pose-source data report (SIM-5510): record schema, structure, render.

ROS-free and pure: functions over dicts, no rclpy, no disk, no clock. One
record per (pose source, scenario, seed). The report records numbers and
fails only on structure (a missing record, a latch, a budget or
infrastructure outcome, a probe that heard nothing): it ranks nothing and
bounds no error, because ADR 0005 forbids pass/fail between methods and each
method's own tier-3 test owns its bound.

``laps_completed`` is 0 or 1: ``run_scenario`` ends at the first
``/scenario/metrics``, so a run never counts a second lap.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Method:
    """What the harness needs to know about one pose source."""

    process: str  # substring of its /proc/<pid>/cmdline, matched exactly once
    map_kind: str  # "online" (it builds /map) or "prior" (it is handed one)
    gpu: bool = False  # part 5 sets this for its GPU variant


# Fixed order = render order; never sorted by value. A None entry is a method
# whose part has not landed (parts 1 and 5 replace them).
METHODS: dict[str, Method | None] = {
    "slam_toolbox": Method(process="lib/slam_toolbox/", map_kind="online"),
    "amcl": Method(process="lib/nav2_amcl/amcl", map_kind="prior"),
    "mola": None,
    "far_lio": None,
}
SCENARIOS = {
    "spielberg": "config/scenarios/spielberg.yaml",
    "spielberg_1080": "config/scenarios/spielberg_1080.yaml",
}
RECORD_FIELDS = (
    "source",
    "scenario",
    "seed",
    "outcome",
    "time_scale",
    "drive_on_estimate",
    "ate_rmse",
    "ate_maximum",
    "availability",
    "maximum_gap",
    "pose_rate_hz",
    "tf_rate_hz",
    "scan_rate_hz",
    "cpu_ms_per_scan",
    "cpu_error",
    "gpu",
    "map_kind",
    "map_wall_cells",
    "map_wall_maximum",
    "map_wall_p95",
    "map_wall_median",
    "map_wall_signed_median",
    "map_wall_signed_mean",
    "latched",
    "laps_completed",
    "lap_time",
    "collision_count",
    "host_load_delta",
    "error",
)
OUTCOMES = ("absent", "completed", "dnf", "latched", "budget", "infrastructure")
FAILING_OUTCOMES = ("latched", "budget", "infrastructure")
# /proc/<pid>/stat: the fields after the closing ")" start at field 3 (state);
# utime and stime are fields 14 and 15.
_UTIME_AFTER_COMM = 14 - 3
_STIME_AFTER_COMM = 15 - 3


def present_sources(launch_keys) -> tuple[list[str], list[str]]:
    """(present, unknown): METHODS entries the launch has, in METHODS order,
    and launch keys the harness has no entry for."""
    keys = set(launch_keys)
    present = [n for n, method in METHODS.items() if method and n in keys]
    unknown = sorted(k for k in keys if METHODS.get(k) is None)
    return present, unknown


def _blank(source, scenario, seed) -> dict:
    row = dict.fromkeys(RECORD_FIELDS)
    row.update(source=source, scenario=scenario, seed=seed)
    return row


def absent_record(source, scenario, seed) -> dict:
    """A method the build does not have: every value None."""
    row = _blank(source, scenario, seed)
    row["outcome"] = "absent"
    return row


def record(
    source, scenario, seed, run, score, rates, cpu, map_accuracy, cpu_error=None
):
    """One run's record. ``run`` carries outcome, time_scale,
    drive_on_estimate, host_load_delta, error and (completed) metrics;
    ``score`` is a ``score_recording`` dict; ``rates`` holds the three
    ``*_rate_hz``; ``map_accuracy`` is ``dataclasses.asdict(MapAccuracy)``;
    ``cpu_error`` is the CPU probe's error text (a doubled process, say)."""
    row = _blank(source, scenario, seed)
    method = METHODS.get(source)
    metrics = run.get("metrics") or {}
    row.update(
        outcome=run["outcome"],
        time_scale=run.get("time_scale"),
        drive_on_estimate=run.get("drive_on_estimate"),
        host_load_delta=run.get("host_load_delta"),
        error=run.get("error"),
        gpu=bool(method.gpu) if method else False,
        map_kind=method.map_kind if method else None,
        latched=run["outcome"] == "latched",
        laps_completed=int(bool(metrics.get("lap_completed"))),
        lap_time=metrics.get("lap_time"),
        collision_count=metrics.get("collision_count"),
        cpu_ms_per_scan=cpu,
        cpu_error=cpu_error,
    )
    for key in ("ate_rmse", "ate_maximum", "availability", "maximum_gap"):
        row[key] = (score or {}).get(key)
    for key in ("pose_rate_hz", "tf_rate_hz", "scan_rate_hz"):
        row[key] = (rates or {}).get(key)
    accuracy = map_accuracy or {}
    row.update(
        map_wall_cells=accuracy.get("wall_cell_count"),
        map_wall_maximum=accuracy.get("maximum"),
        map_wall_p95=accuracy.get("p95"),
        map_wall_median=accuracy.get("median"),
        map_wall_signed_median=accuracy.get("signed_median"),
        map_wall_signed_mean=accuracy.get("signed_mean"),
    )
    return row


def pose_rate(samples, duration_s) -> dict:
    """Pose and TF rate from ``[(stamp, (x, y, yaw))]`` map->odom samples.

    ``tf_rate_hz`` counts every sample. ``pose_rate_hz`` counts only samples
    whose value differs from the previous one, because slam_toolbox
    republishes an unchanged transform every 20 ms: a raw count would call
    that a 50 Hz pose.
    """
    if duration_s <= 0:
        return {"pose_rate_hz": None, "tf_rate_hz": None}
    changed = 0
    previous = None
    for _stamp, value in samples:
        if value != previous:
            changed += 1
        previous = value
    return {
        "pose_rate_hz": changed / duration_s,
        "tf_rate_hz": len(samples) / duration_s,
    }


def parse_proc_stat(text: str) -> int:
    """utime + stime in clock ticks. ``comm`` may hold spaces and
    parentheses, so split after the LAST ")"."""
    fields = text[text.rindex(")") + 1 :].split()
    return int(fields[_UTIME_AFTER_COMM]) + int(fields[_STIME_AFTER_COMM])


def single_match(cmdlines: dict[int, str], pattern: str) -> int:
    """The pid whose cmdline holds ``pattern``; ValueError unless exactly
    one (a stale or doubled process must not be measured silently)."""
    found = [
        pid
        for pid, cmdline in cmdlines.items()
        if pattern in cmdline.replace("\0", " ")
    ]
    if len(found) != 1:
        which = "no process" if not found else f"{len(found)} processes"
        raise ValueError(f"{which} match {pattern!r}: {sorted(found)}")
    return found[0]


def cpu_ms_per_scan(ticks_before, ticks_after, clk_tck, scans):
    """CPU milliseconds per scan from tick deltas; None for zero scans."""
    if scans <= 0:
        return None
    return (ticks_after - ticks_before) / clk_tck * 1000.0 / scans


def _finite(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def structural_failures(records, expected) -> list[str]:
    """Why the data is structurally broken; empty when it is not.

    ``expected``: ``launch_keys``, ``scenarios`` and ``seeds``. A record is
    expected for every METHODS entry (absent ones as ``absent``). Values are
    never bounded: an enormous error or a terrible map is data.
    """
    failures = []
    _present, unknown = present_sources(expected["launch_keys"])
    failures += [f"{name}: in the build but not in METHODS" for name in unknown]
    by_key = {(r["source"], r["scenario"], r["seed"]): r for r in records}
    for source in METHODS:
        for scenario in expected["scenarios"]:
            for seed in expected["seeds"]:
                label = f"{source}/{scenario}/seed {seed}"
                row = by_key.get((source, scenario, seed))
                if row is None:
                    failures.append(f"missing record: {label}")
                    continue
                if tuple(row) != RECORD_FIELDS:
                    failures.append(f"{label}: record fields differ")
                outcome = row["outcome"]
                if outcome in FAILING_OUTCOMES:
                    failures.append(
                        f"{label}: {outcome}"
                        + (f": {row['error']}" if row.get("error") else "")
                    )
                elif outcome == "completed":
                    if not _finite(row["ate_rmse"]):
                        failures.append(
                            f"{label}: ate_rmse is not finite: "
                            f"{row['ate_rmse']!r}"
                        )
                    if row.get("cpu_error"):
                        failures.append(
                            f"{label}: cpu probe: {row['cpu_error']}"
                        )
                    if row["pose_rate_hz"] is None or (
                        row["scan_rate_hz"] is None
                    ):
                        failures.append(f"{label}: probe heard nothing")
    return failures


def _cell(value) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.3g}"
    return str(value)


RENDER_COLUMNS = (
    "source",
    "scenario",
    "seed",
    "outcome",
    "ate_rmse",
    "availability",
    "pose_rate_hz",
    "scan_rate_hz",
    "cpu_ms_per_scan",
    "map_wall_median",
    "map_wall_p95",
    "map_wall_signed_mean",
    "lap_time",
)


def render(records) -> str:
    """A markdown table in fixed METHODS x SCENARIOS x seed order."""
    sources = list(METHODS) + sorted(
        {r["source"] for r in records} - set(METHODS)
    )
    scenarios = list(SCENARIOS) + sorted(
        {r["scenario"] for r in records} - set(SCENARIOS)
    )
    lines = [
        "| " + " | ".join(RENDER_COLUMNS) + " |",
        "|" + "---|" * len(RENDER_COLUMNS),
    ]
    for source in sources:
        for scenario in scenarios:
            rows = sorted(
                (
                    r
                    for r in records
                    if r["source"] == source and r["scenario"] == scenario
                ),
                key=lambda r: r["seed"],
            )
            for row in rows:
                if row["outcome"] == "absent":
                    cells = [source, scenario, row["seed"], "absent"]
                    cells += ["-"] * (len(RENDER_COLUMNS) - 4)
                else:
                    cells = [row[c] for c in RENDER_COLUMNS]
                lines.append("| " + " | ".join(_cell(c) for c in cells) + " |")
    return "\n".join(lines) + "\n"
