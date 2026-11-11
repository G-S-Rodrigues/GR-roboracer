"""What SIM-3770..3781 share (not a test): the fast lap on an estimator.

Imports SIM-3750's launch list and RACELINE rather than copying them, so the
estimator lap and the ground-truth lap cannot drift apart.
"""

import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
from _porto_drive import SPAWN_MAP, SPAWN_TOL_M, SPAWN_TOL_RAD, drive_porto
from racing_test_keywords.autodrive_live import judge_stale_input
from test_sim_3750_porto_fast_mintime_lap import (
    BUDGET_S,
    END_LATCH_WAIT_S,
    LAPS,
    LAUNCH,
    RACELINE,
)

from scripts.compare_localization import (
    last_live_error,
    read_recording,
    score_recording,
)


def fast_lap_on_estimator(
    pose_source: str, case: str, host_command: str, tmp_path: Path
) -> None:
    """SIM-3750's lap, driven on `pose_source`'s estimate. The gates are
    SIM-3750's (no latch, the spawn, both laps, no collision on either
    side); lap times and the live localization error are REPORTED, never
    bounded (owner 2026-11-08). A run that fails still writes its JSON."""
    out = Path(os.environ.get("AUTODRIVE_OUT", tmp_path))
    out.mkdir(parents=True, exist_ok=True)
    recording = out / "recording.jsonl"
    run = drive_porto(
        [*LAUNCH, f"pose_source:={pose_source}"],
        LAPS,
        out,
        budget_s=BUDGET_S,
        host_command=host_command,
        node_name=f"{case}_driver",
        pose_source=pose_source,
        recording_path=recording,
    )
    m = run.metrics
    stack_laps = list(m.lap_times) if m else []
    drive = run.speeds[run.first_after :]
    speeds = np.array(drive) if drive else np.zeros(1)
    stamps = run.stamps[run.first_after :]
    lap2 = [
        v
        for t, v in zip(stamps, drive, strict=True)
        if stack_laps and t >= stamps[-1] - stack_laps[-1]
    ]
    localization: dict = {"error": "no recording"}
    if recording.exists():
        try:
            events = read_recording(recording)
            live = last_live_error(events)
            localization = {
                "live": live,
                "offline": score_recording(
                    events, estimate_topic=live["estimate_topic"]
                ),
            }
        except (ValueError, KeyError) as error:
            localization = {"error": f"{type(error).__name__}: {error}"}
    result = {
        "pose_source": pose_source,
        "laps_stack_s": stack_laps,
        "laps_sim_s": [i[1] for i in run.increments],
        "collisions_stack": int(m.collision_count) if m else None,
        "collisions_sim": run.sim_collisions,
        "peak_speed_mps": float(speeds.max()),
        "mean_speed_lap2_mps": float(np.mean(lap2)) if lap2 else None,
        "safety_reasons": dict(run.reasons),
        "stale_input": run.stale,
        "host_load": run.load,
        "wall_s": run.wall_s,
        "localization": localization,
        "raceline_sha256": hashlib.sha256(RACELINE.read_bytes()).hexdigest(),
    }
    (out / f"{case}.json").write_text(json.dumps(result, indent=2, default=str))
    print(case.upper(), json.dumps(result, default=str))

    verdict = judge_stale_input(
        run.latch, run.stamps, 0.1, run.stale, run.trace, END_LATCH_WAIT_S
    )
    assert run.stale == 0, f"STALE_INPUT x{run.stale}: {verdict}"
    assert run.reasons.get("REASON_ESTOP_LATCHED", 0) == 0, (
        f"emergency stop latched: {dict(run.reasons)}"
    )
    assert run.spawn_error is not None, "no odom frame after the reset"
    assert (
        run.spawn_error[0] <= SPAWN_TOL_M
        and run.spawn_error[1] <= SPAWN_TOL_RAD
    ), f"first pose {run.first_pose} vs {SPAWN_MAP}: {run.spawn_error}"
    assert m is not None, "metrics never published"
    assert run.last_count >= LAPS, f"sim lap_count {run.last_count} < {LAPS}"
    assert m.laps_completed == LAPS, m.laps_completed
    assert all(math.isfinite(t) for t in stack_laps)
    assert m.collision_count == 0, f"stack collisions {m.collision_count}"
    assert run.sim_collisions == 0, f"sim collisions {run.sim_collisions}"
