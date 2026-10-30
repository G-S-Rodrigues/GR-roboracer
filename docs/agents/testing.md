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
| 1 | Pure logic, no graph | `colcon test` (gtest), `pytest sim/tests`, `pytest scripts/tests`, `env -u PYTHONPATH uv run --project raceline --locked pytest raceline/tests` | < 10 s total, of which raceline ~8 s warm (the map extractor added ~2.5 s); measured idle 2026-10-21: raceline suite ~37 s (109 tests, measured 2026-10-25 on a loaded host, load ~20, `pytest raceline/tests`; ~29 s with part 4 at 101 tests; three real-TUM tests ~5 s each, RACELINE-1395 ~5 s; ~17 s on main, ~20 s with part 2.3), already over that target before 2.3 (a cold uv sync downloads CPython 3.8 and both envs: minutes) |
| 2 | One node honours its contract | `launch_testing` | ~5 s |
| 3 | The composed system laps | `pytest tests/system` (real `racing_bringup` launch) | ~352 s (`==> system tests` section of `--full`) |
| 4 | Black-box acceptance | `robot --pythonpath tests/lib tests/acceptance` | ~201 s |
| 5 | The cross-product: tracks × implementations × seeds | `pytest tests/nightly` via `--nightly` | minutes-hours |

**Robot Framework appears at tiers 3–4 only** — see ADR 0004.

**Tier 5 exists** (`tests/nightly/`, `check.sh --nightly`), built with the SLAM/localization phase.
ADR 0005 places the expensive cross-product there so that `--fast`, `--ci` and `--full` do not grow
as implementations accumulate: `--nightly` runs everything `--full` does and then the cross-product,
and the scheduled CI job runs `--nightly-region <region>` per region x seed (`static` on seed 42 only, because tiers 0-2 read no seed; 17 jobs; 60-minute timeouts for `static`, `control` and `shared`, 180 for `slam` and `path_planning`), with `NIGHTLY_TIER5=1` on seed 42, so tier 5 runs in the seed-42 job of each region that owns a tier-5 test. Nothing in tier 5 is part of "done" — a test
belongs there when its cost is laps rather than seconds and what it catches is a regression only a
combination shows.

A red nightly leaves evidence: node logs and Robot output (`logs-<region>-seed-<n>`, failed jobs only), SIM-5030's sample
(`sim-5030`: `log/sim-5030.json`, plus the worst run's recording when an assert fails), the raceline comparison report (`raceline-report`: `log/raceline-report/{report.md,report.json,regeneration.json}`, from `path_planning`/seed 42, uploaded always), the pose-source data report (`pose-source-report`: `log/pose-source-report/{report.md,report.json}`, from `slam`/seed 42, uploaded always), the localization recordings of tier 4 (ACC-4030/4040: `logs-slam-seed-<n>/robot/localization/<source>-<open|closed>-loop.jsonl`, whose stamps locate a lost stretch of availability), and, when a
`run_scenario` lap times out (tier 3 or 4) after a latch, a `verdict=` in the `TimeoutError` text
(`supervisor-lag`, `supervisor-clock-late` (the first status ever is already STALE), `controller-silent` or `unclassified`, scenario_runner's `_HangObserver`); a t=0 wait that never hears the supervisor's `~/clock_received` raises with `verdict=supervisor-clock-unheard`. Tier 4's localization cases (ACC-4030/4040) run seed 42 on every matrix seed (the matrix seed reaches only the seeded lap and tier 5), so a failure there on the "seed 2026" leg is not a seed-2026 draw. A passing run
prints nothing, so SIM-5030's JSON is the only record of its distribution. SIM-5030 runs a fixed seed list (`SEEDS`, 7 seeds x 5 runs) and bounds the pooled median and
the maximum of those runs (`log/sim-5030.json` also carries `by_seed`, evidence only), not their max-min spread (one tail run sets the spread; the samples behind the
bound are in the test).

## Running them

```bash
./scripts/check.sh --fast   # tiers 0-1, scoped to the branch's changes. The pre-commit gate.
./scripts/check.sh --ci     # + tier 2 and one seeded tier-3 lap. The PR gate.
./scripts/check.sh --ready  # scoped tiers 0-2, SIM-3020, and the tier-3/4 tests the map selects. Done. ~5-7 min.
./scripts/check.sh --full   # tiers 0-4, whole tree. The nightly's gate. ~13 min.
./scripts/check.sh --nightly # + tier 5, the whole nightly in one command. CI runs --nightly-region instead; not part of done.
./scripts/check.sh --nightly-region <static|slam|path_planning|control|shared>  # one nightly job (NIGHTLY_TIER5=1 adds its tier 5)
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
| 1, ament_python | `python3 -m pytest ros_ws/src/racing_sim_autodrive/test -k simad_1010` |
| 2 | `launch_test ros_ws/src/<pkg>/test/<file>.py` (the file is the smallest unit: launch_testing has no per-case filter) |
| 2, ament_python | `ROS_DOMAIN_ID=<n> python3 -m pytest ros_ws/src/racing_sim_autodrive/test/test_server_node.py` (pytest collects the `@pytest.mark.launch_test` file; `colcon test` runs the same) |
| 3 | `python3 -m pytest tests/system/test_sim_3060_evaluator_identity.py` |
| manual, host | `docker compose -f docker/docker-compose.yaml run --rm gpu bash /ws/docker/gpu/smoke.sh` (GPU-3010) |
| 4 | `robot --pythonpath tests/lib --outputdir log/robot --test 'Vehicle Completes Baseline Lap Safely' tests/acceptance` |

A C++ edit reaches none of these until
`colcon build --base-paths ros_ws/src --symlink-install --packages-select <pkg>`
has run. Python edits are live through the symlink install. Each runner exits non-zero on failure.

## Test IDs

Invented for this repo, because it had no scheme: `<PKG>-<T>NNN`, where `T` is the tier digit.
`COMMON-1010`, `ADAPT-2040`, `SIM-3020`, `ACC-4010`. Every test's docstring or name carries its ID;
the tier tables in the plan map ID to behaviour.

Current counts: **280 tier-1, 35 tier-2, 12 tier-3, 5 tier-4, 7 tier-5.**

The gate-scope work added tier 1 `CHECK-1010` to `CHECK-1130` (thirteen IDs, `scripts/tests`).

The SLAM/localization phase added: tier 1 `SIMJAX-1070/1080/1090/1100/1110`,
`BRINGUP-1040/1050/1060/1070`, `EVAL-1050`; tier 2 `EVAL-2020`, `BRINGUP-2010/2020`, `ADAPT-2120`;
tier 3 `SIM-3070` (slam_toolbox) and `SIM-3080` (amcl); tier 4 `ACC-4030` (one case per pose source)
and `ACC-4040`; tier 5 `SIM-5010/5020/5030`.

The URDF work added tier 1 `DESC-1010` (the URDF's wheel axles are `wheelbase` apart).
The nightly-flake work added tier 1 `EVAL-1060`.
The ready-gate work added tier 1 `CHECK-1140` to `CHECK-1280` and `BRINGUP-1080`.
The MOLA pose-source work added tier 1 `BRINGUP-1520` to `BRINGUP-1570` and `BRINGUP-2500` (estimate held until the start pose is applied), tier 3 `SIM-3510`, nightly `SIM-5520`, and tier 4 `ACC-4510`.
The raceline project work added tier 1 `RACELINE-1010/1020/1030/1040/1050`, `CHECK-1290`, `CHECK-1300`.
The map-extractor work added tier 1 `RACELINE-1060` to `RACELINE-1130` (nine IDs, 1085 among them).
`raceline/bench/` holds measurements, not tests: pytest does not collect it, the gate does not run it, and it needs the network.
The vehicle-limits work added tier 1 `RACELINE-1210..1250`, `SIMJAX-1120/1130`, `BRINGUP-1090/1100`,
`CHECK-1310/1320`.
The importer-convention work added tier 1 `SIM-1030`.
The image-pin work added tier 1 `CHECK-1330/1340`.
The base-link work (part 3.1b) added tier 1 `SIMJAX-1140`, `DESC-1020`; tier 2 `ADAPT-2130`.
The worktree-scope work added tier 1 `CHECK-1350` to `CHECK-1390`.
The observed-walls work (part 2.3) added tier 1 `RACELINE-1140/1150/1155/1160/1170`.
The hairpin-fold work (part 2.4) added tier 1 `SIM-1040/1050/1060/1070`.
The optimized-raceline runner work (part 4) added tier 1 `RACELINE-1310` to `RACELINE-1395` (23 IDs).

The nightly-split work (part 7.0) added tier 1 `CHECK-1510/1520` (`scripts/tests/test_nightly_regions.py`; `tests/nightly_regions.yaml` assigns each nightly test to a region, `slam`, `path_planning`, `control` or `shared`, plus the implicit `static`).
The trajectory-speed work (part 5) added tier 1 `PP-1040` to `PP-1150`.

The evaluation-harness work (SLAM part 7) added tier 1 `SIMJAX-1500/1510` (and amended `SIMJAX-1060`: `spielberg_1080.yaml` may differ in `num_beams` and `fov`, by file name), `EVAL-1500..1540` (`racing_evaluation.map_accuracy`), `CHECK-1820..1870` and `CHECK-1851` (`scripts/pose_source_report.py`); tier 5 `SIM-5510`, assigned to `slam`. `run_scenario` takes optional `probes` (`tests/lib/racing_test_keywords/probes.py`); the wiring has no tier-1 test.

The availability work (N5) added tier 1 `CHECK-1400/1410` (`scripts/tests/test_localization_keywords.py`).

The planner-selection work (part 6) added tier 1 `BRINGUP-1150/1160/1170`, `CHECK-1420/1430`, `SIM-1080`; tier 2 `PP-2010`; tier 3 `SIM-3110`.

The comparison-report work (part 7) added tier 1 `CHECK-1440` to `CHECK-1500` (`scripts/tests/test_raceline_report.py`); tier 5 `SIM-5040/5050`.

The Porto work (#44 part 3) added tier 1 `RACELINE-1180/1185/1190/1195/1200/1205/1207` (`raceline/tests/test_extract_porto.py`, ~3 s).

The frame-rename work (autodrive R1) added tier 1 `CHECK-2010/2020`.

The gpu-stage work (#47 part 3) added tier 1 `CHECK-1900` to `CHECK-1940` and the manual `GPU-3010`.

The t=0 clock work (F1) added tier 1 `CHECK-1950` to `CHECK-1990` (`scripts/tests/test_scenario_runner_t0.py`) and tier 2 `SAFE-2010`, `SAFE-2020` (`racing_safety_supervisor/test/test_supervisor_clock_received.py`).

The simulator-capabilities work (autodrive-sim part 6) added tier 1 `ADAPT-1010/1020/1030` (1030 also pins the exact guard shape, with a negative case), `SIMJAX-1150`; tier 2 `ADAPT-2140/2145/2150/2160`. Conformance tests select by the backend's declared capabilities (`racing_sim_adapter/test/conformance.py`); a not-applicable test prints `NOT APPLICABLE <id>: ...` and returns (no skip marker), and ADAPT-2150 pins the set.
The nightly-cleanup work (part C1) added tier 1 `CHECK-1530/1540/1550` (`scripts/tests/test_changed_scope.py`, `test_frozen_pose.py`, `test_raceline_report.py`) and `RACELINE-1500`, `METRICS-1040` (`sim/tests/test_compare.py`: a difference equal to, and one beyond, its tolerance), and tier 2 `METRICS-1030` (`~/finalize`).

The AutoDRIVE part-1 work added tier 1 `CHECK-1610` to `CHECK-1660` (`scripts/tests/test_autodrive_compose.py`, `test_autodrive_record.py`).
The AutoDRIVE part-4 work added tier 1 `CHECK-1710` to `CHECK-1790` (thirteen IDs, `scripts/tests/test_autodrive_identify.py`, 1715 in `test_autodrive_record.py`) and `BRINGUP-1055` (BRINGUP-1050 now skips the vehicle files in `NOT_GYM_CAST`; 1055 keeps that list to committed, non-gym files).

The AutoDRIVE part-5 work added tier 1 `SIMAD-1010` to `SIMAD-1140` (`ros_ws/src/racing_sim_autodrive/test/test_core.py`).

The AutoDRIVE part-7 work (our Socket.IO server) added tier 1 `SIMAD-1150` to `SIMAD-1270` (thirteen IDs: the Socket.IO pins, the ROS-free bridge session, the replay schedule, the comma-decimal locale error) `ADAPT-1040` and `ADAPT-1050` (every launch registration carries its own `RACING_SIM_BACKEND`; SIMAD-1150 also checks the running image has the pinned closure, so a stale image fails by name), and tier 2 `SIMAD-2010` to `SIMAD-2050` (the server node against `racing_sim_autodrive.replay`, on an ephemeral port, never the sim's 4567). The four `racing_sim_adapter` launch files are registered a second time (`test_adapter_<name>_autodrive`) with `RACING_SIM_BACKEND=autodrive`; `conformance.BACKENDS` holds each backend's launch, its expected not-applicable set and its per-backend expectations (ADAPT-2030 rate, 2130 h, 2145 clock), and the variable defaults to the gym. The tier-1 and tier-2 AutoDRIVE tests need the Socket.IO closure (`docker/dev-tools-requirements.txt`) on `PYTHONPATH`. The live sim run (port 4567) is a recorded manual step, not a gate.

The AutoDRIVE stall work (part 7.2) added tier 1 `SIMAD-1330/1340` (`StallWatch`) and tier 2 `SIMAD-2080` (300 frames at recorded pace under a `/drive` flood, every one answered: red 9/40 before the executor fix, green 40/40 after) and `SIMAD-2090` (the stall WARNING and stack dump). Select one with `colcon test --packages-select racing_sim_autodrive --pytest-args -k test_server_node`: `-k simad_2080` selects nothing, because pytest names a launch test by its file.

The AutoDRIVE window work (part 2) added tier 1 `CHECK-1670`, `CHECK-1675`, `CHECK-1680` and `CHECK-1685` (`scripts/tests/test_autodrive_gui.py`); the window itself is a manual check recorded in `docs/agents/running-wsl.md`.

The evaluation-isolation work (E1) added tier 2 `EVAL-2030` (one case in each of `test_evaluation_node.py` and `test_evaluation_node_pose.py`: neither the evaluator nor the test node has a topic outside the file's prefix; `test/graph_isolation.py`), and moved EVAL-2010/2020 under `/test_eval2010/`, `/test_eval2020/`.

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
  structurally (no gym_jax symbol in the headers); structure is not semantics. Capability selection now exists
  (ADAPT-1010..1030, 2140..2150), but a second backend (the AutoDRIVE server, replayed) now runs the same files
  (`RACING_SIM_BACKEND`); neutrality against the live sim is the recorded manual step.
- **RViz visual correctness.** A TF sign error passes every numeric test and is obvious to a human
  instantly. Manual — see `racing_vehicle_description/CLAUDE.md`.
- **GPU passthrough.** Not assertable without a GPU runner. Manual: `GPU-3010` in the `gpu` stage
  (`docker/gpu/smoke.sh`).
- **Cross-hardware determinism.** JAX is float32 and not bit-reproducible across GPUs or XLA
  versions. SIM-3030/3040 assert tolerances, never float equality — and each tolerance in
  `scripts/compare_metrics.py` carries the measurement that justifies its size.
