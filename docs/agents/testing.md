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
| 1 | Pure logic, no graph | `colcon test` (gtest), `pytest sim/tests`, `pytest scripts/tests`, `env -u PYTHONPATH uv run --project raceline --locked pytest raceline/tests` | < 10 s for the colcon and `sim/`/`scripts/` suites; the raceline suite ~37 s on a loaded host (2026-10-25), and a cold uv sync downloads CPython 3.8 and both envs (minutes) |
| 2 | One node honours its contract | `launch_testing` | ~5 s |
| 3 | The composed system laps | `pytest tests/system` (real `racing_bringup` launch) | ~352 s (`==> system tests` section of `--full`) |
| 4 | Black-box acceptance | `robot --pythonpath tests/lib tests/acceptance` | ~201 s |
| 5 | The cross-product: tracks × implementations × seeds | `pytest tests/nightly` via `--nightly` | minutes-hours |

**Robot Framework appears at tiers 3–4 only** — see ADR 0004.

**Tier 5 exists** (`tests/nightly/`, `check.sh --nightly`), built with the SLAM/localization phase.
ADR 0005 places the expensive cross-product there so that `--fast`, `--ci` and `--full` do not grow
as implementations accumulate: `--nightly` runs everything `--full` does and then the cross-product,
and the scheduled CI job runs `--nightly-region <region>` per region x seed (`static` on seed 42 only, because tiers 0-2 read no seed; 21 jobs; 60-minute timeouts for `static`, `control` and `shared`, 120 for `path_planning_mintime`, 180 for `slam` and `path_planning`), with `NIGHTLY_TIER5=1` on seed 42, so tier 5 runs in the seed-42 job of each region that owns a tier-5 test. Nothing in tier 5 is part of "done" — a test
belongs there when its cost is laps rather than seconds and what it catches is a regression only a
combination shows.

A red nightly leaves evidence: node logs and Robot output (`logs-<region>-seed-<n>`, failed jobs only), SIM-5030's sample
(`sim-5030`: `log/sim-5030.json`, plus the worst run's recording when a tracking flag is raised), the quality reports of every job (`reports-<region>-seed-<n>`: `log/reports/<ID>.json`), the raceline regeneration record (`raceline-regeneration`: `log/raceline-report/regeneration.json`, from `path_planning`/seed 42, uploaded always), the localization recordings of tier 4 (ACC-4030/4040: `logs-slam-seed-<n>/robot/localization/<source>-<open|closed>-loop[-<planner>].jsonl`, whose stamps locate a lost stretch of availability), and, when a
`run_scenario` lap times out (tier 3 or 4) after a latch, a `verdict=` in the `TimeoutError` text
(`supervisor-lag`, `supervisor-clock-late` (the first status ever is already STALE), `controller-silent` or `unclassified`, scenario_runner's `_HangObserver`); a t=0 wait that never hears the supervisor's `~/clock_received` raises with `verdict=supervisor-clock-unheard`. The other t=0 waits raise `metrics-start-unheard` (racing_metrics' `~/started`), `controller-drive-unheard` (a `/controller/drive`) and `start-unobserved` (the hang observer never saw the held state); after the lap `run_scenario` raises `StartMissed` with `metrics-start-missed` (racing_metrics' first accepted sample after t=0) or `drive-start-missed` (a truth-driven car's first motion later than 3 ticks). An ACC-4020 failure message carries the same record (`safety_record`: the first non-NONE `/safety/status`, and the verdict of the first latching one). A pose-source case whose ground truth freezes (the gym holds its state on a collision) no longer waits out the 400 s timeout: it fails on `lap_completed` about 2 sim s after the crash with `ground truth frozen at <t> s, s <s>, d <d>: ...` and the estimate's lon/lat at the freeze, and the text names a latch (STALE_INPUT, ESTOP_LATCHED) stamped at or before the freeze instead of a collision; a non-latching clamp or TRACK_LIMIT is named and said not to latch. The red run still writes its metrics or localization report (`lap_completed: false`, `lap_time: 0.0`). Processes that `launch` had to SIGKILL or SIGTERM at shutdown are listed as `shutdown escalations` in the failure text and in every report's `diagnostics`; a t=0 timeout carries `t0 stall: ...; nodes: ...`; localization reports carry `first_estimate_stamp` and `travel_before_first_estimate` in `diagnostics`, never in `numbers`. racing_recording's own tests are REC-1010 (gtest) and REC-2010 (`launch_test` on the file; `colcon test --packages-select racing_recording` runs it, but a launch test has no per-case filter). CHECK-2378 replays trimmed real nightly recordings (`scripts/tests/fixtures/nightly/`). Tier 4's localization cases (ACC-4030/4040) run seed 42 on every matrix seed (the matrix seed reaches only the seeded lap and tier 5), so a failure there on the "seed 2026" leg is not a seed-2026 draw. A passing run
prints nothing to the job log, so SIM-5030's JSON and the reports are the record of its distribution. SIM-5030 runs a fixed seed list (`SEEDS`, 7 seeds x 5 runs) and reports the pooled median, the maximum
and `by_seed` in `log/sim-5030.json`. It is regression tracking: it fails on a gate (a lap that does not
complete, a collision, a `StartMissed` verdict) and never on the spread.

## Gates and reports (ADR 0008)

A tier-3, -4 or -5 test fails only on a gate:

- the lap completes within the run's timeout;
- 0 collisions;
- no supervisor latch or intervention;
- the instrument recorded data (`sample_count`, `scored_count`, metrics published, a map with wall
  cells);
- two runs or pipelines of the same code agree (the goldens through `scripts/compare_metrics.py`,
  SIM-3060's identity, SIM-3100's drift floor, the regeneration tests);
- one catastrophic bound per quality metric.

The catastrophic bounds are shared by every implementation:

| Metric | Bound |
|---|---|
| ATE RMSE | <= 5.0 m |
| availability | >= 0.90 |
| lap time | the run's timeout |
| minimum wall clearance | > 0.08 m |
| p95 tracking error | < 1.0 m |
| map median wall error | <= 1.0 m |
| map wall cells | >= 21 800 |

ADR 0008 gives the reason for each value. None is a measurement, so none is re-tuned when a method
improves or the host is loaded.

Every quality number is reported, not asserted. The test that measured it prints `REPORT <ID> {json}`
and writes `log/reports/<ID>.json` (seed, scenario, implementation, revision, host load) through
`tests/lib/racing_test_keywords/report.py`, which also holds the catastrophic values. A passing pytest
prints nothing, so `check.sh --ready`, `--full` and `--nightly` print the reports a run wrote when it
ends, red or green. The nightly uploads `log/reports/` on every job. The measurement comment that once
justified a bound stays beside the report, as its baseline.

- **Do not add a pass/fail quality bound.** "Is it better than X" is a benchmark.
- **Benchmarks** compare implementations of one role. They run on demand, or in CI as a non-required
  job on a PR that adds an implementation. They never run nightly and never block. The command is `python3 scripts/benchmark.py run|render|history` (Running); the Benchmark workflow runs it on a PR whose diff adds an implementation, bumps the data submodule (any change to its maps or racelines runs the planning benchmark) or edits the runner, uploads `benchmark-<role>` artifacts, and is never waited on.
- **Regression tracking** of one method over time (SIM-5030) runs in the nightly, reports, and is
  never red on a number.
- **An experimental implementation** (`status: experimental` in `config/reference_stack.yaml`) is
  gated by tiers 0-2 and a launch test that its argument selects it. It owns no tier-3 test until its
  promotion.

## Running them

```bash
./scripts/check.sh --fast   # tiers 0-1, scoped to the branch's changes. The pre-commit gate.
./scripts/check.sh --ci     # + tier 2 and one seeded tier-3 lap. The PR gate.
./scripts/check.sh --ready  # scoped tiers 0-2, SIM-3020, and the tier-3/4 tests the map selects. Done. ~5-7 min.
./scripts/check.sh --full   # tiers 0-4, whole tree. The nightly's gate. ~13 min.
./scripts/check.sh --nightly # + tier 5, the whole nightly in one command. CI runs --nightly-region instead; not part of done.
./scripts/check.sh --nightly-region <static|slam|path_planning|path_planning_mintime|control|shared>  # one nightly job (NIGHTLY_TIER5=1 adds its tier 5)
```

Heavy runs (a ready gate, a live qualification, a recording) take one lock so they do not overlap and
starve each other: `scripts/heavy_lane.sh -- ./scripts/check.sh --ready` (on the host or in `dev` /
`fast-sim`, which mount `/lane`; `gpu` and the autodrive services do not, so wrap them from the host; it
waits for the lane, then for load under 8, `--max-load`). See gotcha #48.

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
| comment in `config/localization/amcl.yaml` | amcl: SIM-3020, SIM-3080, ACC-4030 (amcl case), ACC-4040 (centerline case) | 426 s (before V3) |
| comment in `racing_controller_baseline/src/node.cpp` | controller, planner, ground_truth: 7 tier-3 tests, 4 Robot cases (ACC-4040 raceline case added by V3) | 342 s (before V3) |
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
| local, devkit container | `ROS_DOMAIN_ID=<S> AUTODRIVE_ROS_DOMAIN_ID=<D> python3 -m pytest tests/autodrive -s` (SIM-3710/3720; needs `autodrive-devkit` up, fails when it is down) |
| 4 | `robot --pythonpath tests/lib --outputdir log/robot --test 'Vehicle Completes Baseline Lap Safely' tests/acceptance` |

A C++ edit reaches none of these until
`colcon build --base-paths ros_ws/src --symlink-install --packages-select <pkg>`
has run. Python edits are live through the symlink install. Each runner exits non-zero on failure.

## Test IDs

Invented for this repo, because it had no scheme: `<PKG>-<T>NNN`, where `T` is the tier digit.
`COMMON-1010`, `ADAPT-2040`, `SIM-3020`, `ACC-4010`. Every test's docstring or name carries its ID;
the tier tables in the plan map ID to behaviour.

Find a test by its ID with `git grep -n -i '<pkg>.<t>nnn'` (names use `sim_3080`, docstrings `SIM-3080`).
`git log -S'<ID>'` says which work added it. No count is kept here: a hand-kept count is stale by the
next part.

Where a test is not where you would look:

- `raceline/bench/` holds measurements, not tests. pytest does not collect it, no gate runs it, and it
  needs the network.
- `tests/nightly_regions.yaml` assigns every nightly test to a region (`slam`, `path_planning`,
  `path_planning_mintime`, `control`, `shared`, and the implicit `static`). CHECK-1510/1520 hold it to
  the suite.
- `scripts/tests/test_docs_check.py` (CHECK-2460..2462) and `scripts/tests/test_observer_map.py`
  (CHECK-2450..2455) test tooling that is not a ROS package: the `gr-roboracer-docs` skill's
  `docs_check.py`, and `scripts/observer_map.py`, which needs the sourced overlay. Both sit under
  `scripts/tests/*`, so no lap is selected for them.
- `scripts/tests/test_heavy_lane.py`: CHECK-2441/2442/2444 drop to uid 65534 with `setpriv` when run as
  root (`dev` and CI are root), and CHECK-2443 asserts root instead of skipping and `fs.protected_regular` >= 1 (`ci.yaml` sets it on the runner).
- SIM-5060 solves only on a night the optimizer code changed. On any other night it records that it
  did not run.
- The conformance tests select by the backend's declared capabilities
  (`racing_sim_adapter/test/conformance.py`). A not-applicable test prints `NOT APPLICABLE <id>: ...`
  and returns, with no skip marker; ADAPT-2150 pins the set. `RACING_SIM_BACKEND` picks the backend
  (default the gym). The servers run under `/test_adapt_gym` and `/test_adapt_autodrive`, and tests
  name topics through `conformance.topic()`.
- A launch test is selected by its file, not its ID: `colcon test --packages-select
  racing_sim_autodrive --pytest-args -k test_server_node` (`-k simad_2080` selects nothing), and the
  same for `racing_sim_gym_jax`'s `test_imu_node` (SIMJAX-1540).
- The tier-1 and tier-2 AutoDRIVE tests need the Socket.IO closure from
  `docker/dev-tools-requirements.txt`. SIMAD-1150 fails by name on an older image.
- `run_scenario` takes optional `probes` (`tests/lib/racing_test_keywords/probes.py`); the wiring has
  no tier-1 test.
- The live sim (port 4567) and `tests/autodrive/` are outside every gate (ADR 0007), run by
  `scripts/autodrive_qualify.sh` or by hand.
- SIM-3750 is local, run by `autodrive_qualify.sh --case fast_lap`.
- SIM-3760 is local, run by `autodrive_qualify.sh --case corner_sweep --sweep-dir <dir>`.
- SIM-3770 (slam_toolbox survey map), SIM-3771 (MOLA map), SIM-3780 (fast lap on amcl) and SIM-3781
  (fast lap on MOLA) are local, run by `autodrive_qualify.sh --case survey|mola_map|fast_lap_amcl|fast_lap_mola
  --sim container|windows`. They fail, never skip, without a sim. CHECK-2480..2484, CHECK-2486..2487 (the MOLA map's T stamp and the held start's reasons), CHECK-2488 (the per-platform stale timeout: 200 ms on AutoDRIVE's files, 100 ms on the gym's), CHECK-2490 (one `stale_input_timeout_s` reader for the live tests; CHECK-2489, the lane wrap, lands with A7-4) and BRINGUP-1980..1984 (1983: the mapping launch hands sim_pure_pursuit no mapper config; 1984: the truth survey config) are their CI-able parts.

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
- **The catastrophic bounds** (ADR 0008) — loose on purpose. A green localization test means the
  estimate did not diverge, not that it is good; how good is in `log/reports/` and the benchmark.

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

The AutoDRIVE part-8b work (launch switch) added tier 1 `BRINGUP-1580` (a track with no raceline file is driven on its centerline; `track_parameters`) and `BRINGUP-1600` to `BRINGUP-1670` and `BRINGUP-1680` (`test_launch_simulator_arg.py`: `speed_source:=trajectory` refused on a map with no raceline; the `simulator` and `autodrive_bridge` arguments, the autodrive Node, `use_sim_time` following `capability.clock` on every node and estimator, the gym's refusal of `autodrive_roboracer`, autodrive's refusals, `server.yaml` pinned to `source.yaml`/`identified.yaml`), tier 2 `BRINGUP-2030` (the support node's centerline `/trajectory`) and tier 3 `SIM-3610` (`simulator:=autodrive` against the recorded stream, no simulator: `ROS_DOMAIN_ID=<n> python3 -m pytest tests/system/test_sim_3610_autodrive_launch_replay.py`). Counts by ID: 298 + 10 = 308 tier-1, 39 + 1 = 40 tier-2, 12 + 1 = 13 tier-3. The shutdown fix then added tier 2 `SIMAD-2110` and `SIMAD-2111` (`test_server_node_signals.py`: the server exits 0 on one SIGINT, on two a gap apart, and with a client replaying; a plain pytest that starts the server itself), so 40 + 2 = 42 tier-2. `test_launch_runs_on_simulated_time` (no ID) was replaced by `BRINGUP-1630`, which launches both values instead of reading a literal `True`, and keeps its `time_scale` declaration assert; `BRINGUP-1056` now launches `autodrive_roboracer` on `simulator:=autodrive`. `reference_stack.yaml` lists autodrive as a simulator alternative (CHECK-1150: 11 implementations).

The sim-`/imu` work (SLAM part 5a) added tier 1 `SIMJAX-1520/1530` (`test_imu.py`: `/imu` linear acceleration is (Δv/dt, v·ω, +g); none on the first step after a reset), `SIMJAX-1550` (`noise.imu` parses, zero default, negative rejected; no committed scenario sets it), `SIMJAX-1560/1570` (`test_noise.py`: IMU noise seeded and restarted by a reseed; scan and odometry streams bit-identical with it on) and tier 2 `SIMJAX-1540` (`test_imu_node.py`, a launch test selected by file: `colcon test --packages-select racing_sim_gym_jax --pytest-args -k test_imu_node`). Counts by ID: 308 + 5 = 313 tier-1, 42 + 1 = 43 tier-2. SIM-3510 is the proof that MOLA is unmoved (it reads `/imu`'s gyro only).

The AutoDRIVE part-9 work (official devkit bridge) added tier 1 `SIMAD-1400` to `SIMAD-1470` (eight IDs;
with `BRINGUP-1690`, which replaces `BRINGUP-1610`'s refusal, nine IDs, counted by ID), tier 2 `SIMAD-2200` to `SIMAD-2260`, tier 3
`SIM-3620`, and the local `tests/autodrive` `SIM-3710/3720`.

The AutoDRIVE part-9.1 fix added tier 2 `SIMAD-2280` (a positive control killed without a goodbye is waited out within one DDS lease, `Listener.wait_until_gone`, or the wait fails naming its topics; two cases, one ID) and tier 1 `SIMAD-1480` (the devkit domain is S+1, or S-1 past 101). Counts by ID: +1 tier-1, +1 tier-2 on the counts above. `SIMAD-2220` now waits for the observer's reset reader to match B's writer first (the `ready=` hook `SIMAD-2200` uses): harness hardening, no new ID and no red (the flake is load-dependent).

The AutoDRIVE part-9.2 fix added tier 2 `SIMAD-2290` (TF the positive control wrote before it stopped, still queued in the listener after the graph emptied, is set aside by `Listener.mark()` on its DDS `source_timestamp` and never counted in SIMAD-2250's window; 9.1's `wait_until_gone` had removed the 1 s spin that used to drain it). Counts by ID: +1 tier-2.

The D1 data-repo migration added tier 1 `CHECK-2320`, `CHECK-2330`, `CHECK-2340`, `CHECK-2350`, `CHECK-2360`, `SIMJAX-1600`, `BRINGUP-1960`, `BRINGUP-1970` and `RACELINE-1530` (nine IDs, counted by ID, on the counts above; the counting topic is still open). `CHECK-1420` keeps its ID and loses its `--ready` scope half to `CHECK-2320` (owner ruling E6). The track data is read from the `third_party/GR-roboracer-data` submodule, so every tier needs it initialised (repo-gotchas #44).

The F-T0 fix added tier 1 `CHECK-2560`..`CHECK-2564` and `CHECK-2570`..`CHECK-2573`, and tier 2 `METRICS-2020` and `METRICS-2030`.

The heavy-lane work added tier 1 `CHECK-2400` to `CHECK-2440` and `CHECK-2441` to `CHECK-2445` (`scripts/tests/test_heavy_lane.py`).

The D2 data layout added tier 1 `CHECK-2950`..`CHECK-2954`, `RACELINE-1600`..`RACELINE-1603`, `RACELINE-1610`..`RACELINE-1612`, `RACELINE-1620` and `BRINGUP-2600`..`BRINGUP-2605` (19 IDs, counted by ID, on the counts above); `BRINGUP-1500` stays, rewritten to read `track_data.mola_files` (`mola_map_files` is gone).
