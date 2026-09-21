"""Pure tests for scenario-owned environment identifiers."""

from dataclasses import replace
from pathlib import Path

import pytest
import yaml
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


def test_simjax_1050_committed_scenarios_carry_noise():
    """SIMJAX-1050: every committed scenario configures non-zero sensor and
    odometry noise. A scenario without it makes every localization test
    vacuous and still green (SIM-3100 catches it live; this, in ms)."""
    root = Path(__file__).resolve().parents[4] / "config" / "scenarios"
    for name in ("contract_test.yaml", "spielberg.yaml"):
        scenario = load_scenario(root / name)
        assert scenario.scan_noise_sigma_m > 0.0, name
        odometry = scenario.odometry_noise
        assert odometry.distance_scale_sigma > 0.0, name
        assert odometry.yaw_scale_sigma > 0.0, name
        assert odometry.distance_noise_density > 0.0, name
        assert odometry.yaw_noise_density > 0.0, name


def test_simjax_1055_noise_section_is_optional_and_validated(tmp_path):
    """SIMJAX-1055: no `noise` section means exact sensors; a negative
    magnitude is rejected rather than silently clamped."""
    root = Path(__file__).resolve().parents[4] / "config" / "scenarios"
    document = yaml.safe_load(
        (root / "contract_test.yaml").read_text(encoding="utf-8")
    )
    document.pop("noise")
    document["track"] = str(root / document["track"])
    document["map_directory"] = str(root / document["map_directory"])
    quiet = tmp_path / "quiet.yaml"
    quiet.write_text(yaml.safe_dump(document), encoding="utf-8")
    scenario = load_scenario(quiet)
    assert scenario.scan_noise_sigma_m == 0.0
    assert scenario.odometry_noise.distance_scale_sigma == 0.0

    document["noise"] = {"scan": {"sigma_m": -0.1}}
    quiet.write_text(yaml.safe_dump(document), encoding="utf-8")
    with pytest.raises(ValueError, match="noise"):
        load_scenario(quiet)
