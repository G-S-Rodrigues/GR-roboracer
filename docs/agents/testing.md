# Testing

## The one architectural constraint that makes this work

**Every node is a thin ROS shell over a ROS-free core.** `Supervisor`, `PurePursuit`,
`MetricsAccumulator`, `Track`, `GymBackend` contain the behaviour; the `*_node.cpp`/`node.py` files
only translate messages, parameters and time. That is what lets tier 1 cover the real logic in
milliseconds with no graph running. Putting logic in a node is not a style violation here — it moves
that logic from a 10 ms test to a 30 s one, or out of test range entirely.

Corollary: **never add a ROS dependency to `racing_common`.**

## Tiers

| Tier | What it proves | Runner | Cost |
|---|---|---|---|
| 0 | It builds and lints | `colcon build`, `clang-format`, `clang-tidy`, `ruff` | seconds |
| 1 | Pure logic, no graph | `colcon test` (gtest), `pytest sim/tests` | < 10 s total |
| 2 | One node honours its contract | `launch_testing` | ~5 s |
| 3 | The composed system laps | `pytest tests/system` (real `racing_bringup` launch) | ~6 min (SIM-3090 alone ~131 s) |
| 4 | Black-box acceptance | `robot --pythonpath tests/lib tests/acceptance` | ~20 s |
| 5 | The cross-product: tracks × implementations × seeds | `--nightly` *(not built yet)* | minutes-hours |

**Robot Framework appears at tiers 3–4 only** — see ADR 0004.

**Tier 5 is decided but not yet built.** ADR 0005 places the expensive cross-product there so that
`--fast`, `--ci` and `--full` do not grow as implementations accumulate. Neither the tier nor
`--nightly` exists in `scripts/check.sh` today; both arrive with the SLAM/localization phase. Until
then, tier 5 is a place to put a test, not a way to run one.

## Running them

```bash
./scripts/check.sh --fast   # tiers 0-1. The pre-commit gate. Keep under ~45s.
./scripts/check.sh --ci     # + tier 2 and one seeded tier-3 lap. The PR gate.
./scripts/check.sh --full   # tiers 0-4. Required before calling a change done.
```

`--full` was "everything" while tiers 0–4 were everything. ADR 0005 makes that two different
statements: it stays the definition of done, and tier 5 sits outside it deliberately.

All three run inside the `dev` container (`docker exec gr-roboracer-dev bash -lc '...'`).
`./scripts/run_scenario.py --seed N` runs one seeded lap through the real graph and prints its
metrics, for when you want an answer rather than a test result.

## Test IDs

Invented for this repo, because it had no scheme: `<PKG>-<T>NNN`, where `T` is the tier digit.
`COMMON-1010`, `ADAPT-2040`, `SIM-3020`, `ACC-4010`. Every test's docstring or name carries its ID;
the tier tables in the plan map ID to behaviour.

Current counts: **36 tier-1, 14 tier-2, 8 tier-3, 2 tier-4, 0 tier-5.**

## The tests that matter most

- **COMMON-1060** — the gym_jax action mapping. It fails silently; this is the only thing that
  catches it.
- **ADAPT-2040** — QoS profiles on both ends. A mismatch looks like absent data, not an error, so
  the test must assert the *profile*, not that a message arrived.
- **SIM-3040** — the live graph against `tests/golden/baseline.json`. Its value is entirely in its
  tolerances staying tight; see the gotchas.
- **SIM-3060** — `racing_evaluation` scoring ground truth against itself must read exactly zero
  error with full availability. It is the check that the ruler is straight; every localization
  number after it is measured with that ruler.
- **EVAL-1030** — a stalled estimator scores as *unavailable*, never as zero error. An error over no
  samples is zero, so without it a dead estimator looks perfect.
- **SIM-3100** — dead reckoning must drift past a stated floor over a lap. It is the only test that
  notices noise silently disabled, which would make every localization test vacuous and green.

## What stays unverified, deliberately

- **That the `racing_sim_adapter` contract is backend-neutral.** With one backend, tier 2 proves
  gym_jax satisfies the contract, not that the contract is general. ADAPT-2060 checks this
  structurally (no gym_jax symbol in the headers); structure is not semantics. Genuinely resolved
  only when a second backend lands.
- **RViz visual correctness.** A TF sign error passes every numeric test and is obvious to a human
  instantly. Manual — see `racing_vehicle_description/CLAUDE.md`.
- **GPU passthrough.** Not assertable without a GPU runner. Manual.
- **Cross-hardware determinism.** JAX is float32 and not bit-reproducible across GPUs or XLA
  versions. SIM-3030/3040 assert tolerances, never float equality — and each tolerance in
  `scripts/compare_metrics.py` carries the measurement that justifies its size.
