"""SIM-3110: one seeded Spielberg lap on the minimum-curvature raceline
completes through the real launch, collision-free, at its own measured lap
time.

The default vehicle follows `config/racelines/spielberg/f1tenth-default-v1/
tum_mincurv_iqp.csv` with `speed_source: trajectory`, so the line's own speed
(a flat 3.0 m/s) is the command, not the curvature schedule. Ground-truth
pose only. The 20 m/s gym-limits vehicle is the nightly's (part 7), not this
gate's.

The lap time pins *this* line's own measurement; it is not a comparison with
the centerline (ADR 0005 leaves ranking peers to tier 5). It is also what
makes the test discriminate: a launch that silently loads the centerline laps
in ~120.4 s, 7.3 s away, over 100x this tolerance.
"""

from pathlib import Path

import pytest
from racing_test_keywords.scenario_runner import run_scenario

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
SCENARIO = str(REPOSITORY_ROOT / "config" / "scenarios" / "spielberg.yaml")

# 5x, as SIM-3090. Measured (part 6 step 1, seed 42, 5x): 61 s / 63 s / 65 s
# of wall time over three runs, lap 113.08 s each (0.00 s spread); seeds 7,
# 1030 and 2026 lapped in 113.13 / 113.09 / 113.16 s with no collision, no
# saturation event and no latch. The centerline lap on the curvature schedule is 120.41 s
# (spielberg_baseline.json).
TIME_SCALE = 5.0
LAP_WALL_SECONDS = 65.0
# 3x the measured wall time, as SIM-3090.
TIMEOUT_SECONDS = 3.0 * LAP_WALL_SECONDS
# Seed 42's lap, 113.08 s in 3 of 3 runs. The window is SIM-3090's lap_time
# tolerance (scripts/compare_metrics.py FLOAT_TOLERANCES), which carries the
# same seed-42 5x precedent.
LAP_TIME = 113.08
LAP_TIME_TOLERANCE = 0.05


def test_sim_3110_optimized_raceline_lap() -> None:
    metrics = run_scenario(
        seed=42,
        scenario=SCENARIO,
        timeout=TIMEOUT_SECONDS,
        time_scale=TIME_SCALE,
        launch_arguments={
            "planner": "optimized_raceline",
            "raceline_optimizer": "tum_mincurv_iqp",
            "speed_source": "trajectory",
        },
    )

    assert metrics["lap_completed"] is True
    assert metrics["collision_count"] == 0
    assert metrics["control_saturation_events"] == 0
    assert metrics["vehicle_parameter_version"] == "f1tenth-default-v1"
    assert metrics["lap_time"] == pytest.approx(
        LAP_TIME, abs=LAP_TIME_TOLERANCE
    )
