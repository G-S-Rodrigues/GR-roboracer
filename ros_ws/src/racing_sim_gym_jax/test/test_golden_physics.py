import json
from dataclasses import fields
from pathlib import Path

import pytest
import yaml
from f1tenth_gym_jax.envs.utils import Param

ROOT = Path(__file__).resolve().parents[4]
SCENARIOS = ROOT / "config" / "scenarios"
# A sensor-only variant of a scenario. It is never compared to a golden
# (SIMJAX-1510 proves its metrics id matches none), and the gym's beam count
# and field of view change what the scan sees, not the dynamics: collision is
# vertex-vs-map (f110_env.py:544-579), not scan-based. Keyed by file name,
# never by "id matches no golden": contract_test.yaml has no metrics id and
# racing_metrics defaults it to "baseline", so that rule would exempt it.
SENSOR_VARIANTS = {"spielberg_1080.yaml": frozenset({"num_beams", "fov"})}


def physics_differences(scenario: Path) -> dict:
    """Parameters `scenario` sets off `Param()`'s defaults, minus the fields
    its file name is allowlisted to change in `SENSOR_VARIANTS`."""
    parameters = yaml.safe_load(scenario.read_text(encoding="utf-8")).get(
        "parameters", {}
    )
    defaults = Param()
    physics = {field.name for field in fields(Param)}
    exempt = SENSOR_VARIANTS.get(scenario.name, frozenset())
    return {
        name: (value, getattr(defaults, name))
        for name, value in parameters.items()
        if name in physics
        and name not in exempt
        and value != getattr(defaults, name)
    }


def effective_metrics_id(scenario: Path) -> str:
    document = yaml.safe_load(scenario.read_text(encoding="utf-8"))
    return document.get("metrics", {}).get("scenario_id", "baseline")


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
    differing = physics_differences(scenario)

    assert differing == {}, (
        f"{scenario.name} sets (scenario, rollout) {differing}; drop them or "
        "make sim/rollout.py pass the same values"
    )


def test_simjax_1510_the_sensor_variant_carve_out_cannot_widen(tmp_path):
    """SIMJAX-1510: only the named file, only its sensor fields.

    SIMJAX-1060 exempts `SENSOR_VARIANTS`; this keeps that list honest. An
    exempt file must be compared to no golden, the scenarios the goldens
    come from must never be exempt, and the exemption must not leak to
    other physics fields or other file names."""
    golden_ids = {
        json.loads(path.read_text(encoding="utf-8")).get("scenario_id")
        for path in (ROOT / "tests" / "golden").glob("*.json")
    }
    for name in SENSOR_VARIANTS:
        assert effective_metrics_id(SCENARIOS / name) not in golden_ids, name
    assert "contract_test.yaml" not in SENSOR_VARIANTS
    assert "spielberg.yaml" not in SENSOR_VARIANTS

    def scenario_named(name: str, parameters: dict) -> Path:
        path = tmp_path / name
        path.write_text(
            yaml.safe_dump({"parameters": parameters}), encoding="utf-8"
        )
        return path

    allowed = scenario_named("spielberg_1080.yaml", {"num_beams": 1080})
    assert physics_differences(allowed) == {}
    for field in ("integrator", "theta_dis"):
        bad = scenario_named(
            "spielberg_1080.yaml",
            {field: "euler" if field == "integrator" else 7},
        )
        assert field in physics_differences(bad), field
    other = scenario_named("elsewhere.yaml", {"num_beams": 1080})
    assert "num_beams" in physics_differences(other)
