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

import contextlib
import dataclasses
import importlib.util
import json
from pathlib import Path

import racing_common
from racing_evaluation.map_accuracy import (
    load_grid,
    map_to_wall,
    scenario_truth_grid,
)
from racing_test_keywords.probes import LastMapProbe, ProcessCpuProbe, RateProbe
from racing_test_keywords.scenario_runner import (
    BRINGUP_LAUNCH_FILE,
    ESTIMATE_TOPIC,
    host_load_delta,
    read_host_load,
    run_scenario,
)

from scripts.compare_localization import read_recording, score_recording
from scripts.pose_source_report import (
    METHODS,
    SCENARIOS,
    absent_record,
    present_sources,
    record,
    render,
    structural_failures,
)
from scripts.raceline_report import read_recording_tolerant, run_outcome

REPO = Path(__file__).resolve().parents[2]
TRACK = REPO / "config" / "tracks" / "spielberg.yaml"
REPORT_DIR = REPO / "log" / "pose-source-report"
SURVEY = (
    REPO / "config/scenarios/maps/Spielberg/slam/Spielberg.yaml"
)  # amcl's prior
# Owner ruling O4: four seeds, as SIM-5050.
SEEDS = (42, 1030, 7, 2026)
# A ceiling, not the rate reached: the sim is step-bound (4-5 ms CPU per step
# against a 2 ms period at 5x, diagnostic host), so it runs nearer 2x. Both
# scenarios lap with no latch at this setting.
TIME_SCALE = 5.0
# SIM-5020's budget: the longest measured row was 220 s on a loaded host.
TIMEOUT_SECONDS = 400.0
FROZEN_WINDOW_SECONDS = 1.0
NO_METRICS = "no message on /scenario/metrics"


def launch_keys() -> list[str]:
    """The pose sources the installed launch file accepts."""
    spec = importlib.util.spec_from_file_location(
        "sim_pure_pursuit_launch", BRINGUP_LAUNCH_FILE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return list(module.POSE_SOURCE_CONFIGS)


def one_run(source: str, scenario: str, seed: int, track_length, tmp: Path):
    recording = tmp / f"{source}-{scenario}-{seed}.jsonl"
    rate, cpu, last_map = (
        RateProbe(),
        ProcessCpuProbe(METHODS[source].process),
        LastMapProbe(),
    )
    run = {
        "time_scale": TIME_SCALE,
        "drive_on_estimate": False,
        "error": None,
    }
    load_before = read_host_load()
    try:
        metrics = run_scenario(
            seed,
            str(REPO / SCENARIOS[scenario]),
            timeout=TIMEOUT_SECONDS,
            time_scale=TIME_SCALE,
            pose_source=source,
            drive_on_estimate=False,
            launch_arguments={"recording_path": str(recording)},
            probes=[rate, cpu, last_map],
        )
        run.update(outcome="completed", metrics=metrics)
    except TimeoutError as error:
        text = str(error)
        if text.startswith(NO_METRICS) and recording.exists():
            events = read_recording_tolerant(recording)
            run.update(
                run_outcome(events, text, track_length, FROZEN_WINDOW_SECONDS)
            )
            run["error"] = text
        else:
            run.update(outcome="infrastructure", error=text)
    except Exception as error:  # noqa: BLE001 - recorded, never hidden
        run.update(outcome="infrastructure", error=f"{error!r}")
    run["host_load_delta"] = host_load_delta(load_before, read_host_load())

    score = None
    if recording.exists():
        with contextlib.suppress(Exception):
            score = score_recording(
                read_recording(recording), estimate_topic=ESTIMATE_TOPIC
            )
    rates = rate.results() if rate.samples or rate.scans else None
    accuracy = None
    grid = last_map.grid()
    if grid is not None and grid.occupied.any():
        accuracy = dataclasses.asdict(
            map_to_wall(grid, scenario_truth_grid(REPO / SCENARIOS[scenario]))
        )
    return record(
        source,
        scenario,
        seed,
        run,
        score,
        rates,
        cpu.cpu_ms_per_scan(rate.scans),
        accuracy,
        cpu_error=cpu.error,
    )


def reference_row() -> dict:
    """The committed survey grid, measured with this metric (O2)."""
    accuracy = map_to_wall(
        load_grid(SURVEY), scenario_truth_grid(REPO / SCENARIOS["spielberg"])
    )
    return {
        "map": str(SURVEY.relative_to(REPO)),
        "definition": (
            "raw cell-centre distance to the gym PNG's wall faces; a "
            "perfect 5 cm map scores 2.5 cm"
        ),
        **dataclasses.asdict(accuracy),
    }


def test_sim_5510_pose_source_data(tmp_path: Path) -> None:
    keys = launch_keys()
    present, unknown = present_sources(keys)
    track_length = racing_common.Track.from_yaml(str(TRACK)).length()
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
                        row = one_run(
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
