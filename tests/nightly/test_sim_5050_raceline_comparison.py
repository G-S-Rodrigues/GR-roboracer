"""SIM-5050: the raceline comparison report, on both vehicles.

Every line (the track centerline plus each committed raceline of the
vehicle's parameter version) is driven by the reference stack on every seed
and reported side by side. This is research output, not a gate (ADR 0005):
there is no lap-time, ranking or collision bound here, and a gym-limits run
that does not finish is *recorded* with the distance it reached.

What is asserted is structure only (`structural_failures`):

- every (vehicle, line, seed) has a record;
- every completed run has finite metrics;
- every run's recording manifest, and every completed run's metrics, carry
  that vehicle's `vehicle_parameter_version`;
- no run is `latched` or `budget`;
- every default-vehicle run completes (ruling Q2: SIM-3110 gates that
  combination, so a default-vehicle DNF is a regression, not data).

The gym's collision freezes the car and the episode does not end, so a crash
surfaces as a metrics timeout like a latched stop or a slow run does; the
recording (flushed at teardown) tells them apart (`run_outcome`).

The upstream reference row reads `third_party/f1tenth_racetracks` when the
submodule is initialised and says "not available" otherwise.
"""

from __future__ import annotations

import json
import time
from pathlib import Path

from scripts.benchmark_runs import (
    DEFAULT_VEHICLE,
    REPORT_DIR,
    SEEDS,
    UPSTREAM,
    VEHICLES,
    lines_of,
    planning_run,
    regeneration_record,
    static_fields,
)
from scripts.benchmark_runs import (
    track_length as committed_track_length,
)
from scripts.raceline_report import (
    render,
    structural_failures,
    summarise,
    upstream_reference,
)

# Collection time, before SIM-5040 writes regeneration.json in this session:
# an older file is a previous session's and renders "not measured" (N2).
SESSION_START = time.time()


def test_sim_5050_raceline_comparison(tmp_path: Path) -> None:
    track_length = committed_track_length()
    expected = {
        "vehicles": {name: version for name, (_, version) in VEHICLES.items()},
        "default_vehicle": DEFAULT_VEHICLE,
        "seeds": list(SEEDS),
        "rows": {
            name: lines_of(version) for name, (_, version) in VEHICLES.items()
        },
    }
    runs: list[dict] = []
    try:
        for vehicle, lines in expected["rows"].items():
            for line in lines:
                for seed in SEEDS:
                    run = planning_run(
                        vehicle, line, seed, track_length, tmp_path
                    )
                    runs.append(run)
                    lap = run.get("metrics", {}).get("lap_time")
                    print(
                        f"\nSIM-5050 {vehicle}/{line}/seed {seed}: "
                        f"{run['outcome']}, lap {lap}, "
                        f"distance {run.get('distance_m')}"
                    )
    finally:
        regeneration = regeneration_record(SESSION_START)
        statics = {}
        for vehicle, (_, version) in VEHICLES.items():
            for line in lines_of(version):
                fields = static_fields(version, line)
                key = f"spielberg/{version}/{line}.yaml"
                if key in regeneration:
                    fields["solve_seconds"] = regeneration[key]["solve_seconds"]
                statics[(vehicle, line)] = fields
        rows = summarise(runs, statics)
        reference = upstream_reference(UPSTREAM, "Spielberg")
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        (REPORT_DIR / "report.md").write_text(
            render(rows, regeneration, reference), encoding="utf-8"
        )
        (REPORT_DIR / "report.json").write_text(
            json.dumps(
                {
                    "rows": rows,
                    "runs": runs,
                    "regeneration": regeneration,
                    "reference": reference,
                },
                indent=2,
                default=str,
            ),
            encoding="utf-8",
        )

    failures = structural_failures(runs, expected)
    assert failures == [], "\n".join(failures)
