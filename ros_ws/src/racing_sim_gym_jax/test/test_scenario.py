"""Pure tests for scenario-owned environment identifiers."""

from dataclasses import replace
from pathlib import Path

import pytest
from racing_sim_gym_jax.scenario import load_scenario


def _scenario():
    root = Path(__file__).resolve().parents[4]
    return load_scenario(root / "config/scenarios/contract_test.yaml")


def test_env_id_supports_all_three_upstream_arities():
    scenario = _scenario()
    stem = "analytic_circle_1_scan_collision_progress_velocity+steeringangle"
    assert (
        replace(scenario, timestep_ratio_value=None, max_steps=None).env_id
        == f"{stem}_v0"
    )
    assert (
        replace(scenario, timestep_ratio_value=2, max_steps=None).env_id
        == f"{stem}_2_v0"
    )
    assert (
        replace(scenario, timestep_ratio_value=2, max_steps=500).env_id
        == f"{stem}_2_500_v0"
    )


def test_max_steps_requires_timestep_ratio():
    scenario = replace(_scenario(), timestep_ratio_value=None, max_steps=500)
    with pytest.raises(ValueError, match="timestep_ratio"):
        _ = scenario.env_id


def test_scenario_records_effective_timestep_ratio():
    scenario = _scenario()
    assert scenario.timestep_ratio == 1
    assert scenario.control_period == pytest.approx(0.01)
