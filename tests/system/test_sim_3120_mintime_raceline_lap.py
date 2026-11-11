"""SIM-3120: one seeded Spielberg lap on the minimum-time raceline completes
through the real launch, collision-free, inside the run's timeout. Its lap
time is reported (`log/reports/SIM-3120.json`), not asserted (ADR 0008).

The default vehicle follows `tracks/Spielberg/racelines/tum_mintime/
gym-nominal-std-const-v1/raceline.csv` with
`speed_source: trajectory`, so the line's own speed is the command.
Ground-truth pose only. The 20 m/s gym-limits vehicle is the
planning benchmark's, not this gate's (owner 2026-10-11).

The reported lap time is *this* line's own measurement, read against the
baseline below; it is not a comparison with the minimum-curvature line (ADR
0005 leaves ranking peers to tier 5). A launch that silently loads that line
laps in ~113.08 s and the centerline in ~120.41 s: the report shows it, no
assert does.
"""

from pathlib import Path

from racing_test_keywords.report import write_report
from racing_test_keywords.scenario_runner import run_scenario

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
SCENARIO = str(REPOSITORY_ROOT / "config" / "scenarios" / "spielberg.yaml")

# 5x, as SIM-3090. Measured (part 8 Step 10, 5x, quiet host, CPU idle 91-99 %):
# seed 42 three times, 112.12 s each (0.00 s spread, 44-50 s wall per lap);
# seeds 7, 1030 and 2026 lapped in 112.13 / 112.07 / 112.19 s with no collision
# and no saturation event. The minimum-curvature line laps in 113.08 s
# (SIM-3110), 0.96 s away, ~19x this tolerance; the TUM estimate was 111.88 s.
TIME_SCALE = 5.0
LAP_WALL_SECONDS = 65.0  # SIM-3110's; the measured wall time is lower
# 3x the measured wall time, as SIM-3090 and SIM-3110.
TIMEOUT_SECONDS = 3.0 * LAP_WALL_SECONDS
# Kept as the baseline the report is read against; no assert reads them.
# Seed 42's lap, 112.12 s in 3 of 3 runs; the window is SIM-3090's.
LAP_TIME_BASELINE = 112.12
LAP_TIME_TOLERANCE_BASELINE = 0.05


def test_sim_3120_mintime_raceline_lap() -> None:
    metrics = run_scenario(
        seed=42,
        scenario=SCENARIO,
        timeout=TIMEOUT_SECONDS,
        time_scale=TIME_SCALE,
        launch_arguments={
            "planner": "optimized_raceline",
            "raceline_optimizer": "tum_mintime",
            "speed_source": "trajectory",
        },
    )

    write_report(
        "SIM-3120",
        metrics,
        seed=42,
        scenario="config/scenarios/spielberg.yaml",
        implementation="optimized_raceline/tum_mintime",
        baseline={
            "lap_time": LAP_TIME_BASELINE,
            "lap_time_tolerance": LAP_TIME_TOLERANCE_BASELINE,
        },
    )
    assert metrics["lap_completed"] is True
    assert metrics["collision_count"] == 0
    assert metrics["control_saturation_events"] == 0
    assert metrics["vehicle_parameter_version"] == "f1tenth-default-v1"
