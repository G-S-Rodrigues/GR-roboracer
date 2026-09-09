"""SIM-3020: a seeded run of the full launch completes a lap.

Drives the real ``racing_bringup`` launch end to end (the same one
``ros2 launch`` and ``scripts/check.sh`` use), not a parallel test-only graph.
"""

from racing_test_keywords.scenario_runner import run_scenario


def test_sim_3020_seeded_lap_completes() -> None:
    metrics = run_scenario(seed=42)

    assert metrics["lap_completed"] is True
    assert metrics["lap_time"] > 0.0
    assert metrics["collision_count"] == 0
