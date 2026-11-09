"""SIM-3730: the stack qualifies on the real AutoDRIVE sim, Porto, 12 laps,
and the stack's metrics agree with the sim's own counters (#44 part 10).

LIVE: needs the AutoDRIVE sim container connected to this launch's server, so
it lives in tests/autodrive/ and never under tests/system/ (no gate has a sim,
ADR 0007). Run it with `scripts/autodrive_qualify.sh --domain <60-69>`, which
recreates the single-use sim container, runs this in `dev` and stops the sim.
With no sim it FAILS within the frame wait, naming that command; it never
skips.

t = 0 is the first frame after `/racing_sim/reset` (gotcha 19). The run ends
when metrics publishes (laps:=12) and the sim's own `lap_count` reaches 12.
"""

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest
from _porto_drive import SPAWN_MAP, SPAWN_TOL_M, SPAWN_TOL_RAD, drive_porto
from racing_test_keywords.autodrive_live import judge_stale_input

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "ros_ws/src/racing_evaluation"))
from racing_evaluation.qualification import score  # noqa: E402

LAPS = 12
BUDGET_S = 600.0
VEHICLE = "config/vehicles/autodrive_roboracer.yaml"
LAUNCH = [
    "ros2",
    "launch",
    "racing_bringup",
    "sim_pure_pursuit.launch.py",
    "simulator:=autodrive",
    f"vehicle:={VEHICLE}",
    "scenario:=config/scenarios/porto.yaml",
    "use_rviz:=false",
    f"laps:={LAPS}",
    "autodrive_port:=4567",
]
HOST_COMMAND = "scripts/autodrive_qualify.sh --domain <60-69>"
# Per-lap |stack lap_time - sim last_lap_time| allowed, seconds. Run 1
# (2026-10-31, quiet host, load 2.3): laps 2..12 disagreed by at most 0.050 s
# (median 0.034), the stack's clock being the odom stamps and the sim's its
# own Unity time; twice that, under the 0.3 s cap (gotcha 14).
LAP_TOL_S = 0.1
END_LATCH_WAIT_S = 1.0


def test_sim_3730_porto_qualification(tmp_path):
    out = Path(os.environ.get("AUTODRIVE_OUT", tmp_path))
    run = drive_porto(
        LAUNCH,
        LAPS,
        out,
        budget_s=BUDGET_S,
        host_command=HOST_COMMAND,
        node_name="sim_3730_driver",
    )
    periods = run.periods
    m = run.metrics
    stack_laps = list(m.lap_times) if m else []
    sim_laps = [i[1] for i in run.increments]
    sim_collisions = run.sim_collisions
    last_count = run.last_count
    result = {
        "laps_sim": sim_laps,
        "laps_stack": stack_laps,
        "collisions_sim": sim_collisions,
        "collisions_stack": int(m.collision_count) if m else None,
        "period_s": {
            "median": float(np.median(periods)),
            "p99": float(np.percentile(periods, 99)),
            "max": float(periods.max()),
        },
        "stale_input": run.stale,
        "host_load": run.load,
        "wall_s": run.wall_s,
    }
    if len(stack_laps) >= LAPS:
        result["score_stack"] = score(stack_laps, sim_collisions).total
    if len(sim_laps) >= LAPS:
        result["score_sim"] = score(sim_laps, sim_collisions).total
    (out / "qualification.json").write_text(json.dumps(result, indent=2))
    print("QUALIFICATION", json.dumps(result))

    spawn_error = run.spawn_error
    first = (None, *(run.first_pose or ()))
    verdict = judge_stale_input(
        run.latch,
        run.stamps,
        0.1,
        run.stale,
        run.trace,
        END_LATCH_WAIT_S,
    )
    stale = run.stale
    assert stale == 0, f"STALE_INPUT x{stale}: {verdict}"
    assert spawn_error is not None, "no odom frame after the reset"
    assert spawn_error[0] <= SPAWN_TOL_M and spawn_error[1] <= SPAWN_TOL_RAD, (
        f"first pose after reset {first[1:]} vs {SPAWN_MAP}: {spawn_error}"
    )
    assert m is not None, "metrics never published"
    if last_count == 0 and m.laps_completed > 0:
        pytest.fail(
            "the sim's lap_count stayed 0 while metrics counted"
            f" {m.laps_completed} laps: direction or lap line"
        )
    assert last_count >= LAPS, f"sim lap_count {last_count} < {LAPS}"
    assert m.laps_completed == LAPS, m.laps_completed
    assert sim_collisions <= 10, sim_collisions
    for k in range(1, LAPS):  # laps 2..12; lap 1 starts off the line
        assert abs(stack_laps[k] - sim_laps[k]) <= LAP_TOL_S, (
            f"lap {k + 1}: stack {stack_laps[k]:.3f} s vs sim"
            f" {sim_laps[k]:.3f} s (tol {LAP_TOL_S})"
        )
