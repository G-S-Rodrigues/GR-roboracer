from dataclasses import fields
from pathlib import Path

import pytest
import yaml
from f1tenth_gym_jax.envs.utils import Param

SCENARIOS = Path(__file__).resolve().parents[4] / "config" / "scenarios"


@pytest.mark.parametrize(
    "scenario", sorted(SCENARIOS.glob("*.yaml")), ids=lambda path: path.stem
)
def test_simjax_1060_scenario_physics_match_the_golden_generator(
    scenario: Path,
) -> None:
    """SIMJAX-1060: no scenario configures physics the golden generator
    does not.

    Every committed scenario is compared live against a golden that
    `sim/rollout.py` generates with `f1tenth_gym_jax.make(env_id)` and no
    parameters, i.e. `Param()`'s defaults. A scenario that sets a different
    integrator or scan discretisation makes the live run and its golden
    two different simulators, which reads as a plausible metric mismatch
    and nothing else: `euler`/720 against `rk4`/2000 was invisible on
    analytic_circle and moved Spielberg's minimum wall clearance from the
    golden's 0.8914 m to 0.8893 m, inside every tolerance.
    """
    parameters = yaml.safe_load(scenario.read_text(encoding="utf-8")).get(
        "parameters", {}
    )
    defaults = Param()
    physics = {field.name for field in fields(Param)}
    differing = {
        name: (value, getattr(defaults, name))
        for name, value in parameters.items()
        if name in physics and value != getattr(defaults, name)
    }

    assert differing == {}, (
        f"{scenario.name} sets (scenario, rollout) {differing}; drop them or "
        "make sim/rollout.py pass the same values"
    )
