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
| 1 | Pure logic, no graph | `colcon test` (gtest), `pytest sim/tests`, `pytest scripts/tests`, `env -u PYTHONPATH uv run --project raceline --locked pytest raceline/tests` | < 10 s total, of which raceline ~8 s warm (the map extractor added ~2.5 s) (a cold uv sync downloads CPython 3.8 and both envs: minutes) |
| 2 | One node honours its contract | `launch_testing` | ~5 s |
| 3 | The composed system laps | `pytest tests/system` (real `racing_bringup` launch) | ~352 s (`==> system tests` section of `--full`) |
| 4 | Black-box acceptance | `robot --pythonpath tests/lib tests/acceptance` | ~201 s |
| 5 | The cross-product: tracks × implementations × seeds | `pytest tests/nightly` via `--nightly` | minutes-hours |

**Robot Framework appears at tiers 3–4 only** — see ADR 0004.

**Tier 5 exists** (`tests/nightly/`, `check.sh --nightly`), built with the SLAM/localization phase.
ADR 0005 places the expensive cross-product there so that `--fast`, `--ci` and `--full` do not grow
as implementations accumulate: `--nightly` runs everything `--full` does and then the cross-product,
and the scheduled CI job runs it on one seed leg. Nothing in tier 5 is part of "done" — a test
belongs there when its cost is laps rather than seconds and what it catches is a regression only a
combination shows.

## Running them

```bash
./scripts/check.sh --fast   # tiers 0-1, scoped to the branch's changes. The pre-commit gate.
./scripts/check.sh --ci     # + tier 2 and one seeded tier-3 lap. The PR gate.
./scripts/check.sh --ready  # scoped tiers 0-2, SIM-3020, and the tier-3/4 tests the map selects. Done. ~5-7 min.
./scripts/check.sh --full   # tiers 0-4, whole tree. The nightly's gate. ~13 min.
./scripts/check.sh --nightly # + tier 5. Scheduled, not part of done.
```

`--fast` diffs the branch against `main` (plus index, worktree and untracked files) and lints and tests
only what that touches: `clang-format` and `clang-tidy` run only when a C/C++ file changed, and then
only over the changed packages; `ruff` runs over the changed Python files; `colcon test` runs
`--packages-above` the changed packages, so a `racing_common` change re-tests everything that depends
on it. The build stays whole-tree, because `clang-tidy` needs its compilation database (gotcha #7).
It prints `scope: ...` first. Measured in a warm throwaway clone in `dev`:

| Change | Command | Wall time |
|---|---|---|
| whole tree | `CHECK_SCOPE=all ./scripts/check.sh --fast` | 223 s |
| one `.py` (`racing_bringup`) | `./scripts/check.sh --fast` | 10.5 s |
| one `.hpp` (`racing_metrics`) | `./scripts/check.sh --fast` | 53.9 s |
| one `.hpp` (`racing_common`, 6 dependents re-tested) | `./scripts/check.sh --fast` | 47.4 s |

A change to `scripts/check.sh`, `.pre-commit-config.yaml`, `setup.sh`, `.clang-tidy`, any
`package.xml` or `CMakeLists.txt`, or anything under `docker/`, `config/` or `tests/golden/`, an empty
changed set, `CHECK_SCOPE=all`, or a failed `git` call, all fall back to the whole tree. The saving
decays along a branch: once it has touched one of those, every `--fast` on it is whole tree again.

`--ready` selects by the map. `scripts/changed_scope.py --ready` (which `check.sh` evals) takes the
branch's changed set and applies, per path and first match first: a whole-tree trigger or composition
file (the map, `sim_pure_pursuit.launch.py`, `support_node.py`, `scenario_parameters.py`, bringup
`setup.py`); a `neutral_paths` glob (selects nothing); a changed `tests/system/test_sim_NNNN_*.py`
(selects that ID); an implementation's `paths` glob (selects it, and every implementation whose package
depends on the changed package). SIM-3020 always runs. A path nothing places, a test nobody claims, a
claim with no test, or an undiscoverable test file makes the run whole tree: the map fails closed, so a
stale map costs time and never skips a test. `python3 scripts/changed_scope.py --owners SIM-3080` prints
the implementations answerable for a test (the reference lap names every default); an unclaimed ID
exits 1. It prints `ready scope:` first. Measured in a warm throwaway clone in `dev` (tiers 0-2 whole
tree when the branch touches `config/`, scoped otherwise):

| Change | Selected | Wall time |
|---|---|---|
| comment in `config/localization/amcl.yaml` | amcl: SIM-3020, SIM-3080, ACC-4030 (amcl case), ACC-4040 | 426 s |
| comment in `racing_controller_baseline/src/node.cpp` | controller, planner, ground_truth: 6 tier-3 tests, 3 Robot cases | 342 s |
| `racing_common` `.cpp` | whole tree (unmapped), the same as `--full`, ~13 min | not timed |

`--full` was "everything" while tiers 0–4 were everything. ADR 0007 makes `--ready` the definition of
done and leaves `--full` to the nightly; tier 5 still sits outside both deliberately.

All of them run inside the `dev` container (`docker exec gr-roboracer-dev bash -lc '...'`).
`./scripts/run_scenario.py --seed N` runs one seeded lap through the real graph and prints its
metrics, for when you want an answer rather than a test result.

## Running one test

The RED/GREEN loop runs one test, not a gate. Every line below runs in `dev` after
`cd /ws && source setup.sh`. The ID is in the test's name, so filter on it:

| Tier | One test |
|---|---|
| 1, C++ | `./build/<pkg>/<test_binary> --gtest_filter='*Common1060*'` |
| 1, `sim/` | `python3 -m pytest sim/tests -k sim_1010` |
| 1, `scripts/` | `python3 -m pytest scripts/tests -k check_1010` |
| 1, `raceline/` | `env -u PYTHONPATH uv run --project raceline --locked pytest raceline/tests -k raceline_1010` |
| 2 | `launch_test ros_ws/src/<pkg>/test/<file>.py` (the file is the smallest unit: launch_testing has no per-case filter) |
| 3 | `python3 -m pytest tests/system/test_sim_3060_evaluator_identity.py` |
| 4 | `robot --pythonpath tests/lib --outputdir log/robot --test 'Vehicle Completes Baseline Lap Safely' tests/acceptance` |

A C++ edit reaches none of these until
`colcon build --base-paths ros_ws/src --symlink-install --packages-select <pkg>`
has run. Python edits are live through the symlink install. Each runner exits non-zero on failure.

## Test IDs

Invented for this repo, because it had no scheme: `<PKG>-<T>NNN`, where `T` is the tier digit.
`COMMON-1010`, `ADAPT-2040`, `SIM-3020`, `ACC-4010`. Every test's docstring or name carries its ID;
the tier tables in the plan map ID to behaviour.

Current counts: **93 tier-1, 18 tier-2, 10 tier-3, 4 tier-4, 3 tier-5.**

The gate-scope work added tier 1 `CHECK-1010` to `CHECK-1130` (thirteen IDs, `scripts/tests`).

The SLAM/localization phase added: tier 1 `SIMJAX-1070/1080/1090/1100/1110`,
`BRINGUP-1040/1050/1060/1070`, `EVAL-1050`; tier 2 `EVAL-2020`, `BRINGUP-2010/2020`, `ADAPT-2120`;
tier 3 `SIM-3070` (slam_toolbox) and `SIM-3080` (amcl); tier 4 `ACC-4030` (one case per pose source)
and `ACC-4040`; tier 5 `SIM-5010/5020/5030`.

The URDF work added tier 1 `DESC-1010` (the URDF's wheel axles are `wheelbase` apart).
The nightly-flake work added tier 1 `EVAL-1060`.
The ready-gate work added tier 1 `CHECK-1140` to `CHECK-1280` and `BRINGUP-1080`.
The raceline project work added tier 1 `RACELINE-1010/1020/1030/1040/1050`, `CHECK-1290`, `CHECK-1300`.
The map-extractor work added tier 1 `RACELINE-1060` to `RACELINE-1130`.

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
