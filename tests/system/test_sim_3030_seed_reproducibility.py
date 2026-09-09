"""SIM-3030: the same seed reproduces metrics within tolerance.

Runs the full stack twice with an identical seed — two independent process
launches, since ``racing_metrics`` only ever completes one lap per process
lifetime (see ``racing_metrics/src/node.cpp``'s ``published_`` latch).
"""

from racing_test_keywords.scenario_runner import run_scenario

from scripts.compare_metrics import compare_metrics, format_report


def test_sim_3030_same_seed_reproduces_metrics_within_tolerance() -> None:
    first = run_scenario(seed=1030)
    second = run_scenario(seed=1030)

    differences = compare_metrics(first, second)

    assert differences == [], format_report(first, second, differences)
