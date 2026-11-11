"""The launches behind ``scripts/benchmark.py``: one run, one record.

Moved out of the nightly's SIM-5050 (raceline comparison) and SIM-5510 (pose
source data), which are retired; their measurement comments come with the
code. ``planning_run`` drives one (vehicle, line, seed) and
``localization_run`` one (pose source, scenario, seed), each through the real
ROS graph with ``run_scenario``. Research output, not a gate (ADR 0005): no
value is bounded here, and a run that does not finish is *recorded*.

The gym's collision freezes the car and the episode does not end, so a crash
surfaces as a metrics timeout like a latched stop or a slow run does; the
recording (flushed at teardown) tells them apart (``run_outcome``).

ROS-free to import: rclpy, ``scenario_runner``, the probes and
``racing_evaluation`` are imported inside the functions that launch.
"""

from __future__ import annotations

import contextlib
import dataclasses
import importlib.util
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
for _path in (REPO, REPO / "tests" / "lib"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

from scripts.frozen_pose import FROZEN_WINDOW_SECONDS  # noqa: E402,F401
from scripts.pose_source_report import (  # noqa: E402
    METHODS,
    SCENARIOS,
    record,
)
from scripts.raceline_report import (  # noqa: E402
    raceline_geometry,
    read_manifest,
    read_recording_tolerant,
    read_regeneration,
    run_outcome,
)

SCENARIO = str(REPO / "config" / "scenarios" / "spielberg.yaml")
TRACK = REPO / "config" / "tracks" / "spielberg.yaml"
REPORT_DIR = REPO / "log" / "raceline-report"
TRACKS = REPO / "third_party" / "GR-roboracer-data" / "tracks"
RACELINES = TRACKS / "Spielberg" / "racelines"
UPSTREAM = REPO / "third_party" / "f1tenth_racetracks"
SURVEY = (
    TRACKS / "Spielberg" / "maps" / "Spielberg" / "slam" / "Spielberg.yaml"
)  # amcl's prior

# vehicle name -> (vehicle file, vehicle_parameter_version)
VEHICLES = {
    "gym_limits": ("f1tenth_gym_limits.yaml", "f1tenth-gym-limits-v1"),
    "default": ("f1tenth_default.yaml", "f1tenth-default-v1"),
}
DEFAULT_VEHICLE = "default"
# O5 / O4: four seeds; revisit if the seed-42 leg passes 150 min.
SEEDS = (42, 1030, 7, 2026)
# A ceiling, not the rate reached: the sim is step-bound (4-5 ms CPU per step
# against a 2 ms period at 5x, diagnostic host), so it runs nearer 2x. Both
# scenarios lap with no latch at this setting.
TIME_SCALE = 5.0
# Planning: SIM-3110's 3 x 65 s. A run that is still moving at the end costs
# all of it; one whose pose froze (a DNF or a latch) is stopped by the
# frozen-pose hook 2 x FROZEN_WINDOW_SECONDS of sim time after it stops, 0.4 s
# of wall at x5.
PLANNING_TIMEOUT_SECONDS = 195.0
# Localization: SIM-5020's budget: the longest measured row was 220 s on a
# loaded host.
LOCALIZATION_TIMEOUT_SECONDS = 400.0
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


def track_length() -> float:
    import racing_common  # noqa: PLC0415

    return racing_common.Track.from_yaml(str(TRACK)).length()


def planning_run(
    vehicle: str, line: str, seed: int, track_length: float, tmp: Path
) -> dict:
    from racing_test_keywords.scenario_runner import (  # noqa: PLC0415
        run_scenario,
    )

    vehicle_file, _ = VEHICLES[vehicle]
    recording = tmp / f"{vehicle}-{line}-{seed}.jsonl"
    run = {"vehicle": vehicle, "line": line, "seed": seed}
    try:
        metrics = run_scenario(
            seed,
            SCENARIO,
            timeout=PLANNING_TIMEOUT_SECONDS,
            time_scale=TIME_SCALE,
            launch_arguments=launch_arguments(vehicle_file, line, recording),
            # Twice the window: run_outcome classifies from the recording,
            # whose subscriber is not the hook's, and a stop at exactly one
            # window can leave a recorded still tail short of it (`budget`).
            stop_on_frozen_pose=2 * FROZEN_WINDOW_SECONDS,
            track_length=track_length,
        )
        if metrics.get("lap_completed"):
            run.update(outcome="completed", metrics=metrics)
        else:
            # The hook stopped a frozen pose; finalize published partial
            # metrics. Classify from the recording, as for a timeout.
            run["partial_metrics"] = metrics
            events = read_recording_tolerant(recording)
            run.update(
                run_outcome(
                    events,
                    "frozen-pose stop",
                    track_length,
                    FROZEN_WINDOW_SECONDS,
                )
            )
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
        # StartMissed carries the lap it paid for: keep it (CHECK-2574).
        partial = getattr(error, "outcome", None)
        if partial is not None:
            run["partial_metrics"] = dict(partial)
    if recording.exists():
        with contextlib.suppress(ValueError, KeyError):
            manifest = read_manifest(recording)
            run["manifest_version"] = manifest["vehicle_parameter_version"]
    return run


def static_fields(version: str, line: str) -> dict:
    import racing_common  # noqa: PLC0415
    import yaml  # noqa: PLC0415

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


def regeneration_record(session_start: float) -> dict:
    """SIM-5040's record, or {} when it is older than ``session_start``: an
    older file is a previous session's and renders "not measured" (N2)."""
    return read_regeneration(REPORT_DIR / "regeneration.json", session_start)


def launch_keys() -> list[str]:
    """The pose sources the installed launch file accepts."""
    from racing_test_keywords.scenario_runner import (  # noqa: PLC0415
        BRINGUP_LAUNCH_FILE,
    )

    spec = importlib.util.spec_from_file_location(
        "sim_pure_pursuit_launch", BRINGUP_LAUNCH_FILE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return list(module.POSE_SOURCE_CONFIGS)


def localization_run(
    source: str, scenario: str, seed: int, track_length: float, tmp: Path
) -> dict:
    from racing_evaluation.map_accuracy import (  # noqa: PLC0415
        map_to_wall,
        scenario_truth_grid,
    )
    from racing_test_keywords.probes import (  # noqa: PLC0415
        LastMapProbe,
        ProcessCpuProbe,
        RateProbe,
    )
    from racing_test_keywords.scenario_runner import (  # noqa: PLC0415
        ESTIMATE_TOPIC,
        host_load_delta,
        read_host_load,
        run_scenario,
    )

    from scripts.compare_localization import (  # noqa: PLC0415
        read_recording,
        score_recording,
    )

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
            timeout=LOCALIZATION_TIMEOUT_SECONDS,
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
        # StartMissed carries the lap it paid for: keep it (CHECK-2574).
        partial = getattr(error, "outcome", None)
        if partial is not None:
            run["partial_metrics"] = dict(partial)
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
    from racing_evaluation.map_accuracy import (  # noqa: PLC0415
        load_grid,
        map_to_wall,
        scenario_truth_grid,
    )

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
