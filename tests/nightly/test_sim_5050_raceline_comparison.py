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

import contextlib
import json
from pathlib import Path

import racing_common
import yaml
from racing_test_keywords.scenario_runner import run_scenario

from scripts.raceline_report import (
    raceline_geometry,
    read_manifest,
    read_recording_tolerant,
    render,
    run_outcome,
    structural_failures,
    summarise,
    upstream_reference,
)

REPO = Path(__file__).resolve().parents[2]
SCENARIO = str(REPO / "config" / "scenarios" / "spielberg.yaml")
TRACK = REPO / "config" / "tracks" / "spielberg.yaml"
REPORT_DIR = REPO / "log" / "raceline-report"
RACELINES = REPO / "config" / "racelines" / "spielberg"
UPSTREAM = REPO / "third_party" / "f1tenth_racetracks"

# vehicle name -> (vehicle file, vehicle_parameter_version)
VEHICLES = {
    "gym_limits": ("f1tenth_gym_limits.yaml", "f1tenth-gym-limits-v1"),
    "default": ("f1tenth_default.yaml", "f1tenth-default-v1"),
}
DEFAULT_VEHICLE = "default"
# O5: four seeds; revisit if the seed-42 leg passes 150 min.
SEEDS = (42, 1030, 7, 2026)
TIME_SCALE = 5.0
# SIM-3110's 3 x 65 s. A run that does not finish costs all of it.
TIMEOUT_SECONDS = 195.0
# Sim seconds over which s and d must not move for a pose to be frozen.
# Measured (part 7 step 2, gym-limits, seed 42, 100 Hz recordings): the car is
# frozen from sim 2.98 s (centerline) and 2.53 s (min-curv) to the end, while
# before that the longest stretch with s and d still is 0.02 s. 1.0 s is 50x
# that, and a creeping car moves more than the 1 mm threshold in it.
FROZEN_WINDOW_SECONDS = 1.0
NO_METRICS = "no message on /scenario/metrics"


def lines_of(version: str) -> list[str]:
    stems = sorted(p.stem for p in (RACELINES / version).glob("*.csv"))
    return ["centerline", *stems]


def launch_arguments(vehicle_file: str, line: str, recording: Path) -> dict:
    arguments = {
        "vehicle": str(REPO / "config" / "vehicles" / vehicle_file),
        "recording_path": str(recording),
    }
    if line == "centerline":
        arguments["planner"] = "track_centerline"
        arguments["speed_source"] = "curvature_schedule"
    else:
        arguments["planner"] = "optimized_raceline"
        arguments["raceline_optimizer"] = line
        arguments["speed_source"] = "trajectory"
    return arguments


def one_run(vehicle: str, line: str, seed: int, track_length: float, tmp: Path):
    vehicle_file, _ = VEHICLES[vehicle]
    recording = tmp / f"{vehicle}-{line}-{seed}.jsonl"
    run = {"vehicle": vehicle, "line": line, "seed": seed}
    try:
        metrics = run_scenario(
            seed,
            SCENARIO,
            timeout=TIMEOUT_SECONDS,
            time_scale=TIME_SCALE,
            launch_arguments=launch_arguments(vehicle_file, line, recording),
        )
        run.update(outcome="completed", metrics=metrics)
    except TimeoutError as error:
        text = str(error)
        if text.startswith(NO_METRICS) and recording.exists():
            events = read_recording_tolerant(recording)
            run.update(
                run_outcome(events, text, track_length, FROZEN_WINDOW_SECONDS)
            )
        else:
            run.update(outcome="infrastructure", error=text)
    except Exception as error:  # noqa: BLE001 - recorded, never hidden
        run.update(outcome="infrastructure", error=f"{error!r}")
    if recording.exists():
        with contextlib.suppress(ValueError, KeyError):
            manifest = read_manifest(recording)
            run["manifest_version"] = manifest["vehicle_parameter_version"]
    return run


def static_fields(version: str, line: str) -> dict:
    if line == "centerline":
        return {"length_m": racing_common.Track.from_yaml(str(TRACK)).length()}
    geometry = raceline_geometry(RACELINES / version / f"{line}.csv")
    document = yaml.safe_load(
        (RACELINES / version / f"{line}.yaml").read_text(encoding="utf-8")
    )
    clearance = document["verdict"]["min_clearance"]
    fields = dict(geometry)
    fields["verifier_min_clearance"] = clearance.get("ground_truth_judged")
    return fields


def regeneration_record() -> dict:
    path = REPORT_DIR / "regeneration.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def test_sim_5050_raceline_comparison(tmp_path: Path) -> None:
    track_length = racing_common.Track.from_yaml(str(TRACK)).length()
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
                    run = one_run(vehicle, line, seed, track_length, tmp_path)
                    runs.append(run)
                    lap = run.get("metrics", {}).get("lap_time")
                    print(
                        f"\nSIM-5050 {vehicle}/{line}/seed {seed}: "
                        f"{run['outcome']}, lap {lap}, "
                        f"distance {run.get('distance_m')}"
                    )
    finally:
        regeneration = regeneration_record()
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
