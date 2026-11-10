"""SIM-5510: every pose source's data on 64 and 1080 beams, one record each.

Research output, not a gate (ADR 0005, and the lidar-SLAM spec's non-goal of
method-vs-method pass/fail). For every pose source the build has
(`METHODS` ∩ the launch's `POSE_SOURCE_CONFIGS`), on `spielberg` (64 beams)
and `spielberg_1080` (1080 beams, 270 degrees) and every seed, it records
ATE and availability, the rate of *changed* map -> odom transforms, TF and
scan rates, the estimator's CPU per scan, and how far the produced `/map`'s
wall cells sit from the gym PNG's walls (`racing_evaluation.map_accuracy`).
A method the build does not have gets an `absent` record, so a part that
lands later joins the table without touching this file (it adds a `METHODS`
entry; a launch key with no entry fails CHECK-1820 on purpose).

What is asserted is structure only (`structural_failures`): every record
exists, no run is `latched`, `budget` or `infrastructure`, every completed
row has a finite ATE and heard its probes. No error, rate, CPU or map value
is bounded: each method's own tier-3 test (SIM-3070/3080, later SIM-3510/
3520) owns its bound, and SIM-5020 stays the bounded combination check over
the same 64-beam laps. This one is the unbounded data.

Open loop throughout (`drive_on_estimate=False`, owner ruling O3), as SIM-5020
/5030: every row then drives the same reference lap, so a row that differs is
the estimator, and `latched`/`laps_completed` say nothing about the method
(ACC-4030/4040-style closed-loop laps cover driving). `ground_truth` is not
a row, for SIM-5020's reason (it is not an estimator and publishes no
composed `/localization/odom`).

The 1080 rows measure the shipped configuration: `amcl.yaml`'s `max_beams: 64`
stays. Tuning a method for 1080 beams is improvement work, not harness work.

The map metric is the raw cell-centre distance to the gym PNG's wall faces: a
perfect 5 cm map scores 2.5 cm (owner ruling O2). The reference row in the
report is the committed survey grid (amcl's prior) measured the same way.
`slam_toolbox`'s `/map` is up to `map_update_interval` (5 s) stale at lap end,
and a method that maps in its own start frame scores metres off until its
part transforms the grid first.

Timing: both scenarios lapped open loop with no latch in diagnostic
runs (host load 15-28, so no wall-time figure is recorded here). 5x is the
`time_scale` ceiling the sim does not reach there (step-bound, about 2x), so
slam_toolbox's latency is not gated by sim time (SIM-3070); the 400 s
timeout is SIM-5020's budget. The nightly time of this test is reported in
the PR from a run at host load below 8.
"""

from __future__ import annotations

import json
from pathlib import Path

from scripts.benchmark_runs import (
    LOCALIZATION_TIMEOUT_SECONDS as TIMEOUT_SECONDS,
)
from scripts.benchmark_runs import (
    SEEDS,
    TIME_SCALE,
    launch_keys,
    localization_run,
    reference_row,
)
from scripts.benchmark_runs import track_length as committed_track_length
from scripts.pose_source_report import (
    METHODS,
    SCENARIOS,
    absent_record,
    present_sources,
    render,
    structural_failures,
)

REPO = Path(__file__).resolve().parents[2]
REPORT_DIR = REPO / "log" / "pose-source-report"


def test_sim_5510_pose_source_data(tmp_path: Path) -> None:
    keys = launch_keys()
    present, unknown = present_sources(keys)
    track_length = committed_track_length()
    expected = {
        "launch_keys": keys,
        "scenarios": list(SCENARIOS),
        "seeds": list(SEEDS),
    }
    records: list[dict] = []
    try:
        for source in METHODS:
            for scenario in SCENARIOS:
                for seed in SEEDS:
                    if source in present:
                        row = localization_run(
                            source, scenario, seed, track_length, tmp_path
                        )
                    else:
                        row = absent_record(source, scenario, seed)
                    records.append(row)
                    print(
                        f"\nSIM-5510 {source}/{scenario}/seed {seed}: "
                        f"{row['outcome']}, ate_rmse {row['ate_rmse']}, "
                        f"pose {row['pose_rate_hz']} Hz, "
                        f"cpu {row['cpu_ms_per_scan']} ms/scan, "
                        f"map median {row['map_wall_median']}"
                    )
    finally:
        reference = reference_row()
        REPORT_DIR.mkdir(parents=True, exist_ok=True)
        (REPORT_DIR / "report.md").write_text(
            render(records)
            + "\nReference (committed survey grid, this metric): "
            + json.dumps(reference)
            + "\n",
            encoding="utf-8",
        )
        (REPORT_DIR / "report.json").write_text(
            json.dumps(
                {
                    "records": records,
                    "unknown": unknown,
                    "constants": {
                        "seeds": list(SEEDS),
                        "time_scale": TIME_SCALE,
                        "timeout_seconds": TIMEOUT_SECONDS,
                        "scenarios": SCENARIOS,
                    },
                    "reference": reference,
                },
                indent=2,
                default=str,
            ),
            encoding="utf-8",
        )

    failures = structural_failures(records, expected)
    assert failures == [], "\n".join(failures)
