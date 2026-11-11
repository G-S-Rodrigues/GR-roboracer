"""SIM-3750: the fast min-time line on the real AutoDRIVE sim, Porto, two
laps (#44 part 1): both complete, with no collision and no latched stop.

LIVE: needs the AutoDRIVE sim container connected to this launch's server, so
it lives in tests/autodrive/ and never under tests/system/ (no gate has a sim,
ADR 0007). Run it with `scripts/autodrive_qualify.sh --domain <60-69> --case
fast_lap`. With no sim it FAILS within the frame wait, naming that command;
it never skips.

The car is config/vehicles/autodrive_roboracer_fast.yaml (4.5 m/s, 4.0
m/s^2, steering rate 1.5) driving the committed minimum-time line, speed from
the line. Lap 1 is a standing start from the post-reset spawn; lap 2 is the
flying lap. Lap times are REPORTED (fast_lap.json), never bounded (owner
ruling 2026-11-08). The gates are: no latch (no STALE_INPUT status and no
ESTOP_LATCHED status; COMMAND_LIMIT is a non-latching clamp and is only
counted), the spawn, both laps counted by the stack and the sim, and zero
collisions on both sides.
"""

import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np
from _porto_drive import SPAWN_MAP, SPAWN_TOL_M, SPAWN_TOL_RAD, drive_porto
from racing_test_keywords.autodrive_live import judge_stale_input

REPO = Path(__file__).resolve().parents[2]
LAPS = 2  # owner ruling 2026-11-08: lap 2 is the flying lap
BUDGET_S = 300.0
VEHICLE = "config/vehicles/autodrive_roboracer_fast.yaml"
RACELINE = (
    REPO
    / "third_party/GR-roboracer-data/tracks/Porto/racelines"
    / "autodrive-roboracer-fast-v1/tum_mintime.csv"
)
LAUNCH = [
    "ros2",
    "launch",
    "racing_bringup",
    "sim_pure_pursuit.launch.py",
    "simulator:=autodrive",
    f"vehicle:={VEHICLE}",
    "scenario:=config/scenarios/porto.yaml",
    "use_rviz:=false",
    "planner:=optimized_raceline",
    "raceline_optimizer:=tum_mintime",
    "speed_source:=trajectory",
    f"laps:={LAPS}",
    "autodrive_port:=4567",
]
HOST_COMMAND = "scripts/autodrive_qualify.sh --domain <60-69> --case fast_lap"
END_LATCH_WAIT_S = 1.0


def test_sim_3750_porto_fast_mintime_lap(tmp_path):
    out = Path(os.environ.get("AUTODRIVE_OUT", tmp_path))
    run = drive_porto(
        LAUNCH,
        LAPS,
        out,
        budget_s=BUDGET_S,
        host_command=HOST_COMMAND,
        node_name="sim_3750_driver",
    )
    m = run.metrics
    stack_laps = list(m.lap_times) if m else []
    sim_laps = [i[1] for i in run.increments]
    # Only the drive: frames before the reset answers are the parked car.
    drive = run.speeds[run.first_after :]
    speeds = np.array(drive) if drive else np.zeros(1)
    # Lap 2 is the last stack lap: the drive ends when it is counted.
    stamps = run.stamps[run.first_after :]
    lap2 = [
        v
        for t, v in zip(stamps, drive, strict=True)
        if stack_laps and t >= stamps[-1] - stack_laps[-1]
    ]
    result = {
        "note": "lap 1 is a standing start; lap 2 is the flying lap",
        "laps_stack_s": stack_laps,
        "laps_sim_s": sim_laps,
        "collisions_stack": int(m.collision_count) if m else None,
        "collisions_sim": run.sim_collisions,
        "peak_speed_mps": float(speeds.max()),
        "mean_speed_mps": float(speeds.mean()),
        "mean_speed_lap2_mps": float(np.mean(lap2)) if lap2 else None,
        "speed_source": "/ground_truth/odom twist.twist.linear (x, y)",
        "safety_reasons": dict(run.reasons),
        "stale_input": run.stale,
        "host_load": run.load,
        "wall_s": run.wall_s,
        "raceline_sha256": hashlib.sha256(RACELINE.read_bytes()).hexdigest(),
    }
    (out / "fast_lap.json").write_text(json.dumps(result, indent=2))
    print("FAST_LAP", json.dumps(result))

    verdict = judge_stale_input(
        run.latch,
        run.stamps,
        0.1,
        run.stale,
        run.trace,
        END_LATCH_WAIT_S,
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
