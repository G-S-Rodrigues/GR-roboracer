"""SIM-3170: `follow_the_gap` laps the reference stack, seeded.

ADR 0005: every implementation is verified once against ground truth, inside
the reference stack current when it lands. The gym's `contract_test` track
(`analytic_circle`, 5 m wide) is the cheap place to do that: it proves the
controller is wired (scan in, drive out, the supervisor and metrics happy
with its commands) and that it completes a lap on a track it was not tuned
on. It does not prove the controller on a mapped real track: that is the
live SIM-3740 on Porto (local only) and FTG-1070 on Porto's geometry.

The graph is read, not trusted: a Pure Pursuit lap must not satisfy this
test, so it fails when `racing_controller_baseline` ran or
`racing_controller_follow_the_gap` did not.
"""

from racing_test_keywords.probes import NodeNamesProbe
from racing_test_keywords.scenario_runner import run_scenario


def test_sim_3170_follow_the_gap_laps_the_reference_stack() -> None:
    names = NodeNamesProbe()
    metrics = run_scenario(
        seed=42,
        launch_arguments={"controller": "follow_the_gap"},
        probes=(names,),
    )

    assert "racing_controller_follow_the_gap" in names.names, names.names
    assert "racing_controller_baseline" not in names.names, names.names
    assert metrics["lap_completed"] is True
    assert metrics["lap_time"] > 0.0
    assert metrics["collision_count"] == 0
