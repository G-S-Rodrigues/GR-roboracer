"""Facts about the simulators that more than one launch file needs."""

from racing_sim_autodrive.capabilities import CAPABILITIES as AUTODRIVE_CAPS
from racing_sim_gym_jax.capabilities import CAPABILITIES as GYM_CAPS

# The simulator capability tables, by `simulator:=` value; `capability.clock`
# decides whether the rest of the graph reads /clock (ADR 0006).
SIMULATOR_CAPABILITIES = {
    "f1tenth_gym_jax": GYM_CAPS,
    "autodrive": AUTODRIVE_CAPS,
}


def sim_time_for(simulator: str) -> bool:
    """True exactly when `simulator` owns the clock (the gym: yes; AutoDRIVE,
    free-running: no), the value every node but the simulator gets as
    `use_sim_time` (ADR 0006)."""
    if simulator not in SIMULATOR_CAPABILITIES:
        raise ValueError(
            f"unknown simulator {simulator!r}; "
            f"one of {sorted(SIMULATOR_CAPABILITIES)}"
        )
    return SIMULATOR_CAPABILITIES[simulator]["capability.clock"] == "owns_clock"
