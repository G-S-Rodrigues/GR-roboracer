# Architecture

This file covers the stack's overall shape and where the owner is taking it. It also covers the simulator
adapter contract, the reference stack map, the track-data repo, gates against benchmarks, the test tiers
and the ready gate, the nightly, the heavy lane, the dev environment and the docs process. Read it before
a grill, spec or plan that touches any of those. It does not cover the simulator backends
(`simulation.md`), SLAM and localization (`localization-slam.md`), raceline generation (`raceline.md`),
the controller (`control.md`) or the league (`competition.md`).

## Current state

### Direction

- **Owner's direction (2026-09-30):** "reach a durable classical stack (GPU-tuned SLAM / localization,
  an optimized raceline, and a controller that follows it), then use it as the baseline and the safety
  fallback for learning reinforcement learning."
- The 2026-10-11 grill ordered the work as raceline first, then a modular simulator bridge with AutoDRIVE
  and the league (target ICRA 2027, qualification about early May 2027, inferred). Its tripwire said: if the
  raceline is not at part 6 by 2026-12-01, run a 1-2 day AutoDRIVE spike. The raceline closed first
  ([#12](https://github.com/G-S-Rodrigues/GR-roboracer/issues/12)), so the tripwire is spent.
- The "modular simulator bridge" landed as a backend-neutral contract package plus one ROS package per
  backend (`racing_sim_gym_jax`, `racing_sim_autodrive`). It did not become a `simulators/` folder: `sim/`
  stays outside `ros_ws/` and [ADR 0003](../adr/0003-sim-outside-ros-workspace.md) was not amended.
- The URDF look work stopped on 2026-10-11 at 0 of 14 look features. The AutoDRIVE car's visual model
  landed later on its own ([PR #62](https://github.com/G-S-Rodrigues/GR-roboracer/pull/62)).
- The 2026-11-08 next-steps run set this order:
  - part 1, the AutoDRIVE fast min-time lap on Porto ([PR #88](https://github.com/G-S-Rodrigues/GR-roboracer/pull/88));
  - part 1L, AutoDRIVE limits identification;
  - part 2, gates against benchmarks ([PR #87](https://github.com/G-S-Rodrigues/GR-roboracer/pull/87), ADR 0008);
  - part 3, the benchmark command ([PR #90](https://github.com/G-S-Rodrigues/GR-roboracer/pull/90));
  - part 4, controller work, which is not specified and will be planned with the owner (see `control.md`).

  Its standing rules were one session at a time and no new pass/fail quality bound.
- Open epics: AutoDRIVE [#44](https://github.com/G-S-Rodrigues/GR-roboracer/issues/44), LiDAR SLAM
  [#47](https://github.com/G-S-Rodrigues/GR-roboracer/issues/47), the investigation toolkit
  [#101](https://github.com/G-S-Rodrigues/GR-roboracer/issues/101).

### Stack shape and the reference stack

- Every node is a thin ROS shell over a ROS-free core ([testing.md](../agents/testing.md), top section).
- Roles and their implementations are named in [`config/reference_stack.yaml`](../../config/reference_stack.yaml).
  [`sim_pure_pursuit.launch.py`](../../ros_ws/src/racing_bringup/launch/sim_pure_pursuit.launch.py) composes
  the stack, and BRINGUP-1030 fails if the two disagree. On main, 2026-11-14:

  | Role | Default | Alternatives | Launch argument |
  |---|---|---|---|
  | simulator | `f1tenth_gym_jax` | `autodrive` | `simulator` (+ `autodrive_bridge`, `autodrive_port`, `autodrive_devkit_domain`) |
  | pose_source | `ground_truth` | `slam_toolbox`, `amcl`, `mola` | `pose_source` (+ `drive_on_estimate`, `estimate_topic`) |
  | planner | `track_centerline` | `optimized_raceline` | `planner` (+ `raceline_optimizer`, `speed_source`) |
  | controller | `pure_pursuit` | none | none |
  | safety | `safety_supervisor` | none | none |
  | evaluation | `localization_error` | none | none |

- No alternative carries `status: experimental` today. The mechanism exists (ADR 0008 §6) and nothing has
  used it.
- `pose_source` became a launch selection in [#4](https://github.com/G-S-Rodrigues/GR-roboracer/issues/4)
  ([PR #9](https://github.com/G-S-Rodrigues/GR-roboracer/pull/9)). Mapping is a separate role in the user
  docs: the survey car drives on `ground_truth` while `slam_toolbox` builds the map
  ([configuration.md](../configuration.md)).
- Selecting between implementations is always a launch argument with a default, never a deletion
  ([ADR 0005](../adr/0005-algorithms-are-added-not-replaced.md)).

### Simulator adapter contract

- [`racing_sim_adapter`](../../ros_ws/src/racing_sim_adapter/CLAUDE.md) holds contract constants
  (`include/racing_sim_adapter/contract.hpp`) and conformance tests. It holds no simulator. The action
  contract is velocity plus steering angle ([ADR 0001](../adr/0001-simulator-action-contract.md)).
- A backend declares five read-only capability parameters
  ([PR #46](https://github.com/G-S-Rodrigues/GR-roboracer/pull/46), which also amended ADR 0006):

  | Capability | gym (`racing_sim_gym_jax/capabilities.py`) | AutoDRIVE (`racing_sim_autodrive/capabilities.py`) |
  |---|---|---|
  | `capability.clock` | `owns_clock` | `free_running` (Unity runs on wall clock, no `/clock`) |
  | `capability.reset` | `seeded_pose` | `initial_pose_only` |
  | `capability.step_mode` | true | false |
  | `capability.time_scale` | true | false |
  | `capability.start_held` | true | false |

- The launch sets `use_sim_time` from `capability.clock` on every node (BRINGUP-1630). Under AutoDRIVE
  every node runs on wall time ([ADR 0006](../adr/0006-simulated-time-is-the-time-base.md)).
- Conformance tests that need a capability are gated in `REQUIREMENTS` in
  `racing_sim_adapter/test/conformance.py`. A test that does not apply prints `NOT APPLICABLE <id>` and
  returns. `RACING_SIM_BACKEND` picks the backend, and the AutoDRIVE row runs from a recorded stream.
  Neutrality against the live sim is a manual step ([testing.md](../agents/testing.md), "What stays
  unverified").

### Track data: GR-roboracer-data

- Maps and racelines live in `G-S-Rodrigues/GR-roboracer-data`, pinned by SHA as `third_party/GR-roboracer-data`
  ([ADR 0009](../adr/0009-track-data-lives-in-gr-roboracer-data.md); D1
  [PR #92](https://github.com/G-S-Rodrigues/GR-roboracer/pull/92); the by-method layout, D2,
  [PR #100](https://github.com/G-S-Rodrigues/GR-roboracer/pull/100)). `analytic_circle` and `config/tracks/`
  stay in-tree, so a clone without the submodule still runs the seeded lap.
- The pin on main is `d25c924`. It holds:

  | Track | Maps (`maps/<method>/[<variant>]`) | Racelines (`racelines/<optimizer>/<limit set>`) |
  |---|---|---|
  | Spielberg | `slam_toolbox/res_0020`, `res_0025`, `res_0050`; `mola` | `tum_mincurv_iqp` and `tum_mintime`, each × `gym-nominal-std-const-v1`, `gym-model-std-const-v1` |
  | Porto | `slam_toolbox/res_0050`; `mola` | `tum_mintime/autodrive-nominal-fast-const-gymfit-v1` |

  FAST-LIO has no maps yet. The owner wants all three methods (slam_toolbox, MOLA, FAST-LIO) for every
  track, each with a PNG you can look at (owner, 2026-11-11).
- Limit-set names follow `<sim>-<source>-<caps>-<grip>[-<qualifier>]-v<N>` (owner ruling #28,
  2026-11-11). The data repo's README owns the grammar. `source` is one of `nominal`, `model`, `identified`
  or `tuned`. `grip` is one of `peak`, `sliding`, `const` or `pNN`. `vN` is bumped whenever any value in
  the set changes and is never reused. The simulator appears only as the `sim` field, because a raceline
  depends on the limits and not on which simulator drives it. The vehicle files in `config/vehicles/` keep
  their names, and each record's yaml maps one to the other. The next sets planned:
  `autodrive-identified-fast-peak-v1`, `autodrive-identified-fast-sliding-v1`, later
  `car-identified-std-peak-v1`.
- New data is two PRs: one into the data repo's `main` first, then a gitlink bump here. A bump is whole
  tree in both `--fast` and `--ready` (CHECK-2320, 2350, 2360). In the Benchmark workflow, a bump runs the
  planning role only. A map change does not run the localization benchmark (`scripts/benchmark.py`,
  `TRIGGERS`).

### Gates, benchmarks, tiers

- What a tier-3/4/5 test may assert is in [ADR 0008](../adr/0008-gates-decide-quality-is-reported.md):
  gates and one catastrophic bound per quality metric. Every quality number is reported to
  `log/reports/<ID>.json`. Tiers, commands, costs and the nightly's evidence trail are in
  [testing.md](../agents/testing.md). They are not repeated here.
- **Ready gate** ([ADR 0007](../adr/0007-ready-is-a-scoped-local-run.md),
  [PR #16](https://github.com/G-S-Rodrigues/GR-roboracer/pull/16)): one local `check.sh --ready` at the end
  of an implementation, run through `scripts/heavy_lane.sh`, plus green CI on the PR head. The selection
  comes from the map's `paths` / `verified_by` / `neutral_paths` and fails closed.
  `scripts/changed_scope.py --owners <ID>` names who answers for a red test.
- **CI** ([`ci.yaml`](../../.github/workflows/ci.yaml)): the PR job runs `--ci` (tiers 0-2 plus SIM-3020).
  The nightly is 21 jobs, one per region and seed, using `--nightly-region`. Regions are listed in
  [`tests/nightly_regions.yaml`](../../tests/nightly_regions.yaml) and seeds are 42, 1030, 7 and 2026. The
  cron says `0 3 * * *`, but scheduled runs actually start around 09:40-10:55 UTC (measured
  2026-11-06..11-13).
- The repo is public, so GitHub-hosted minutes cost nothing. `main` has no branch protection (checked
  2026-11-14). The GR-pr skill is the only thing that enforces "CI green before merge".
- **Benchmarks** ([PR #90](https://github.com/G-S-Rodrigues/GR-roboracer/pull/90)):
  `python3 scripts/benchmark.py run|render|history`, covering the roles `localization` and `planning`.
  `history` tabulates the nightly's quality reports. The `Benchmark (non-blocking)` workflow runs on a
  PR that adds an implementation, bumps the data, or edits the runner. Nothing waits on it.
- A red nightly blocks nothing. Its fix goes into the next plan (CLAUDE.md, "The rule"). That replaces the
  2026-10-11 grill's Q7 and part 3's ruling OQ2 ("halt in-flight parts whose scope names the failing
  implementation").
- The live AutoDRIVE sim and `tests/autodrive/` are outside every gate, so they run by
  `scripts/autodrive_qualify.sh` or by hand.

### Heavy lane

- `scripts/heavy_lane.sh -- <cmd>` takes `flock` on `/lane/heavy.lock`. That lock is the host's
  `/tmp/gr-roboracer-lane`, mounted into `dev` and `fast-sim`
  ([PR #98](https://github.com/G-S-Rodrigues/GR-roboracer/pull/98), gotcha #48). The script then waits for
  load1 to drop under `--max-load` (default 8). `--wait N` bounds only the lock wait.
- The `gpu`, `autodrive-sim` and `autodrive-devkit` services have no `/lane`, so wrap them from the host.

### Dev environment

- Images and why they are split are in [ADR 0002](../adr/0002-development-environment.md):
  `base → underlay → dev (→ gpu) / fast-sim / runtime`. The compose services are `dev`, `fast-sim`,
  `gpu`, `autodrive-sim` and `autodrive-devkit`. `runtime` has no service. ADR 0002 was amended three
  times:
  - a ROS-free uv project may run in `dev` (2026-10-13);
  - the `gpu` stage was added (2026-10-25);
  - ffmpeg and fonts were added to `dev` (2026-10-28).
- The owner's host is WSL2 with WSLg ([running-wsl.md](../agents/running-wsl.md)). The GUI path is
  validated only there. Native Linux RViz is unvalidated
  ([#39](https://github.com/G-S-Rodrigues/GR-roboracer/issues/39), [setup.md](../setup.md)).

### Docs process

- The [`gr-roboracer-docs` skill](../../.claude/skills/gr-roboracer-docs/SKILL.md) separates two audiences.
  User docs are `README.md`, `docs/*.md`, `docs/workflows/` and `docs/assets/`. Agent docs are `CLAUDE.md`,
  the per-package `CLAUDE.md` files, `docs/agents/` and `docs/adr/`. Each fact has one owner page, and
  capability statuses are Available, Experimental, In development, Planned or Exploratory. Video rules are
  in `VIDEO.md` beside the skill.
- A docs-only PR (every file `.md` or under `docs/assets/`) runs `docs_check.py` and merges without
  `--ready` or CI (owner ruling 2026-10-22, [PR #37](https://github.com/G-S-Rodrigues/GR-roboracer/pull/37)).
- [configuration.md](../configuration.md) is a **target contract**. No launcher reads its YAML yet, so
  today's launch arguments are shown beside it ([PR #34](https://github.com/G-S-Rodrigues/GR-roboracer/pull/34)).

### Bootstrap decisions still in force (issue [#1](https://github.com/G-S-Rodrigues/GR-roboracer/issues/1), 2026-09-06)

ADR 0001-0004 hold the bootstrap's big calls. These smaller ones, with their reasons, are not written
elsewhere:

- **One public MIT monorepo.** A message change, its simulator adapter, its consumer and its test must
  land in one commit. Splitting a repo later is cheap; merging is not. `racing_interfaces` stays in-tree
  until an external consumer needs it alone, because early message churn would make every change a
  version bump.
- **C++ is the default for hand-written code** (hot path, latency discipline). Python only where a
  Python library forces it: `racing_sim_gym_jax` (the gym is pure Python/JAX, no C++ at all), `sim/`,
  `tools/`, the Robot keyword library, and later `raceline/` (TUM). Today `racing_sim_autodrive`,
  `racing_evaluation`'s nodes and `racing_bringup` are Python too.
- **Shared math is C++ with a pybind11 module, never a second Python copy**: `racing_common` (track,
  Frenet) and `racing_controller_baseline` (Pure Pursuit, so `sim/rollout.py` drives the same
  controller). A Frenet written twice drifts, and a drift between the controller and the metrics reads
  as a controller bug. The gym ships its own Frenet (`state.frenet_states`, `ey` positive left), a
  third copy that `sim/` must never read.
- **The ROS-free core takes no ROS type**, even for a four-field value: `apply_drive_command` takes a
  neutral `DriveCommand` (`racing_common/include/racing_common/action_map.hpp`), and the node copies
  `AckermannDriveStamped.drive` into it at its boundary.
- **Topic and frame names mirror `f1tenth_system`** (`/scan`, `/odom`, `/drive`, `base_link`, `map`)
  because a car is planned and a later rename is stack-wide. The lidar frame was renamed to `lidar` on 2026-10-28
  (gotcha 26).
- **The PR gate is a `check.sh` mode, not YAML steps** (`--ci`), so CI has no private definition of
  passing and a contributor runs the same gate locally. "Everything on every PR" was rejected because a
  20-minute gate gets bypassed. The local pre-commit hook matters more than CI, because an agent breaks
  code locally and CI reports after the next change is built on it.
- **Build order for a new subsystem: pure core first, ROS last, each contract test before the node it
  guards.** Rejected: a thin ROS spine first ("it moves" hides that none of the math is proven) and one
  vertical slice (a misbehaving car leaves Frenet, the action mapping, the tick rate, QoS and the
  controller all suspect at once).
- **arm64-ready without an arm64 job.** The Dockerfile honours `TARGETARCH` (`docker/Dockerfile:27`);
  CI runs `ubuntu-latest` (x86_64) only. No hardcoded x86 wheel URL in `runtime` (unverified).

## What we measured

### Gate and workflow cost

| Date | What | Number | Source |
|---|---|---|---|
| 2026-09-24 | `--fast` before scoping (whole-tree clang-tidy on every commit) | ~6 min per commit | issue [#8](https://github.com/G-S-Rodrigues/GR-roboracer/issues/8) |
| 2026-09-28 | tier 4 section of `--full` | 201 s (testing.md had said ~20 s) | testing.md |
| 2026-10-11 | one part before #13: `--fast` twice per step, `--full` up to three times (~13 min each), a multi-seed sweep while CI ran | per part, critical path after #13 ≈ max(local `--ready`, CI 8-10 min, Copilot ~4.5 min) | issue [#13](https://github.com/G-S-Rodrigues/GR-roboracer/issues/13) |
| 2026-10-11 | PR CI wall time including the image build | 8-10 min, ~15 min once | issue #13 |
| 2026-11-01 | a whole-tree `--ready` on the shared lane | ~40 min | (scratch topic; number kept here) |

Scoped `--fast` and `--ready` timings are in [testing.md](../agents/testing.md).

### What `--ready` actually selects (part 2 review, 2026-10-12)

- A change to the simulator, `racing_evaluation`, or bringup's support code (`support_node.py`,
  `scenario_parameters.py`, `setup.py`) selects nearly every tier-3/4 test. The saving is real only for
  alternative and newly added implementations.
- 20 `verified_by` entries named a test that another implementation also claimed. SIM-3030, 3040, 3050 and
  3090 were each claimed by 3-5 defaults. Dropping one such claim is invisible to every check. This is
  ADR 0007's "wrong path assignment" risk, counted.

### Nightly health

| Window | Head | Red nights | What |
|---|---|---|---|
| 2026-09-24..10-07 | unchanged main | 4 of 14 | SIM-3060 identity (×2), SIM-3040 lap_time 22.67 vs 22.61, `/scenario/metrics` timeouts (SIM-3020, 3050). Nobody acted on them. |
| 2026-10-28..11-10 | moving | 11 of 14 | Almost all on `slam` legs. Every quality failure there was converted by ADR 0008. Four modes remained: M1 real collision (ACC-4040 raceline), M2 MOLA starts before it initialized, M3 truncated recording, M4 estimator never up. |
| 2026-11-11..11-13 | after N1 ([PR #94](https://github.com/G-S-Rodrigues/GR-roboracer/pull/94)) and F-T0 ([PR #95](https://github.com/G-S-Rodrigues/GR-roboracer/pull/95)) | 1 of 3 | 11-13, slam/1030 at 7c9ff4a: ACC-4040 raceline "ground truth frozen at sim 64.72 s, s 34.5, d -0.88 ... estimate lon +0.38 m". The log named M1 without a replay. |

Facts about the nightly runner (2026-11-10, from job logs):

- Each job is its own GitHub-hosted VM with 4 vCPUs. Legs never share a runner.
- Tier 4 runs seed 42 on every leg, so the four slam legs run the same seeded tier-4 suite. Only host
  timing differs between them.
- load1 on slam legs was 5.8-9.2, the same on passing and failing legs.
- "5x" laps reach only 1.8-2.6x (a ~120 s sim lap takes 46-68 s of wall time).
- M1 ran at about 1 nightly in 4 (2 of 28 slam legs, 11-03..11-10). D3 classifies it as a real stack
  result, so it stays a red gate (see `control.md` and `localization-slam.md`).

### t=0 is released before DDS has matched (2026-11-11, fixed by F-T0)

These numbers were measured on main eb761d0 with a probe that recorded DDS source timestamps, over 72
launches.

- The gym creates its publishers and `/drive` subscription only after the ~1.2 s JAX warm (1.16-1.37 s).
  The runner released t=0 about 8 ms later (7.1-19.4 ms in 44 of 62 launches).
- DDS matching of those new endpoints took 0.15-1.07 s in 21 of 72 launches (29 %). A ~330 ms quantum
  recurred, which suggests an RMW timeout (Fast DDS defaults).
- When only an ungated link was late, its consumer missed t=0. A late racing_metrics gives a shorter
  `lap_time` and drops the t=0 tracking-error peak: the observed 22.46 / 0.15654322 was reproduced to 6
  significant figures by starting accumulation at 0.43 s. A late controller gives a longer lap (23.09).
  The failing signature appeared in 1 of 49 seed-1030 launches.
- Ruled out by measurement:
  - launch position in the pytest session: late launches were 5/13, 4/13, 2/12, 4/12 and 3/12 by position;
  - stale participants within their lease: 25 s of idle before each launch still gave 3 of 10 late;
  - mid-lap stalls: `/drive(t)` carried the command for odometry t−10 ms on 99.81 % of 136,272
    commands, and eight 330-340 ms sim freezes left metrics unchanged.
- Normal runs are not bitwise identical (the first difference appears at 0.02-2.4 s), yet their metrics
  agree to 4 decimals. Pure Pursuit damps a one-tick perturbation.
- The fix, [PR #95](https://github.com/G-S-Rodrigues/GR-roboracer/pull/95), waits for racing_metrics'
  latched `~/started` and a stamp-0 `/controller/drive` before reset. It also names a missed start
  (`metrics-start-missed`, `drive-start-missed`). An earlier fix,
  [PR #63](https://github.com/G-S-Rodrigues/GR-roboracer/pull/63), made the supervisor wait for `/clock`.
  That removed the t=0 stale latch but not the lap-time spread: 1 outlier in 48 runs, 22.63 against the
  usual 22.83.

### The dev host's clock (2026-11-12)

- On the WSL2 VM, the monotonic clock runs about 4 ms/s slow. `systemd-timesyncd` (32 s poll) steps the
  wall clock forward every ~30 s. Steps grew from +77 ms to +122 ms over one evening. `dev` shares the
  kernel, so it sees the same steps, identical to the millisecond. On 11-10, steps went backward by
  15-19 ms instead.
- A 1 ms sleep loop ran for 300 s on the host and in `dev` at the same time. Neither saw a scheduler
  stall over 50 ms.
- Anything stamped with wall time, which is all of AutoDRIVE (see `simulation.md`), sees a ~115 ms jump
  with no real pause. Gotcha #45 covers the backward case.

## Dead ends

- **Running the scoped full suite in CI on the PR.** The owner vetoed it (2026-10-11): the ready run is
  local, and CI keeps `--ci` as the whole-tree tier-0-2 complement.
- **Rebase-merge plus GR-rebase's pre-PR "rebase and re-gate".** Each rebase changed the SHA and forced
  another `--full`. Merge commits made it unnecessary, because `pull_request` CI already tests
  `refs/pull/N/merge`. GR-rebase was folded into GR-pr. Use `git bisect --first-parent`.
- **Scoping `--fast` by letting pre-commit pass filenames.** That scopes to the staged files, which is
  narrower than the branch, and it misses a package an earlier step broke. `check.sh` decides its own
  scope from the branch diff.
- **Hand-kept test counts in testing.md.** They could not be reproduced. In 2026-10-25, "182" tier-1
  matched neither 418 collected items nor ~136 distinct IDs, and tier 4 read 4 where `robot --dryrun`
  found 5. In 2026-10-31, 327 IDs counted statically, or ~315 without lettered sub-IDs. Part 2 dropped
  the counts. Use `git grep` by ID.
- **Three private lock files** (two in `/tmp` on the host, one in `dev`'s `/tmp`). They excluded nobody:
  load reached 12 and SIM-3080/3510 failed on an unrelated branch. One shared `/lane` replaced them.
- **Fixes for SIM-3030 / the t=0 race that do not work:**
  - Lockstep `time_scale` > 1. The sim waits only while `count_publishers("/drive") > 0`, and that graph
    count is true even while its own reader is unmatched. `step_due` waits at most one 1x tick anyway.
  - A fixed sleep after `~/clock_received`. The measured tail reaches 1.07 s, so the sleep would be a
    timeout in disguise.
  - Waiting out discovery leases between launches (refuted above).
- **Reproducing nightly flakes by loading the host.** In 2026-10-12, SIM-3020 failed 0 of 20 runs
  unloaded and 0 of 20 with two busy loops, SIM-3030 failed 0 of 10 loaded, and three dispatched CI runs
  (12 legs) were green. The cause was DDS matching at t=0, which load does not drive.
- **Requiring branch protection** was rejected for a one-owner repo (2026-10-11). A required check would
  also bind the owner's manual merges.
- **Designing against an upstream README.** The bootstrap spec took f1tenth_gym_jax's API from its
  README; reading the source at the pinned SHA contradicted it in six places before any contract froze
  (`step` auto-resets, the observation has no world pose, a map is four files, the env id has two more
  fields, `jax-pf` is unpinned, maps download over HTTP). Spike the source of a young dependency before
  its types reach `racing_interfaces`.
- **CUDA jaxlib in `dev` to speed tier 3** (bootstrap). Gotcha 4 has the 5x step. It also dropped
  `/odom` from 100 to ~94 Hz, and `jax[cuda12]` plus the gym's `[cuda]` extra installed both plugin sets, so
  every process logged `PJRT_Api already exists for device type cuda` at ERROR. Reverted.
- **Widening `stale_input_timeout_ms` 100 → 500 ms against startup hiccups** (bootstrap). The hiccup
  was the JAX recompile on step 2 (gotcha 13); with two warm steps no gap exceeded 50 ms in 3000 steps,
  and the timeout went back to 100.

## Ideas not yet specified

- **A config-file launcher** for [configuration.md](../configuration.md)'s target YAML. The docs spec
  deferred this code (2026-10-13):
  - a versioned schema and parser;
  - one general stack launch that validates combinations;
  - a registry from public identifiers to packages and parameter files;
  - clear errors;
  - parity tests against today's launches.

  `sim_pure_pursuit.launch.py` and `slam_toolbox_mapping.launch.py` stay as entry points. The settled
  docs-level points (2026-10-21) are:
  - the key set is `workflow`, `scenario`, `vehicle`, `stack{simulator, pose_source, mapper, planner, controller}`,
    `runtime{seed, time_scale, drive_on_estimate, recording_path}`, `mapping.output`, `visualization.rviz`;
  - `start_held` and `estimate_topic` have no YAML field;
  - the mapping example writes to `log/maps/<Map>`.
- **Gate laps on a faster car.** Tier-3 wall time scales with lap time: ~120 s at 3 m/s against ~45 s at
  ~8 m/s. The owner may move gate laps to the fast car once its nightlies prove it (raceline grill Q1,
  2026-10-11).
- **Investigation toolkit**: a topic timeline recorder, an A/B runner on top of the heavy lane, and a host
  load sampler ([#101](https://github.com/G-S-Rodrigues/GR-roboracer/issues/101)).
- **Per-file routing of a data bump**: diff the data repo between the two gitlink SHAs, then route
  `--ready` and the benchmark by the files that changed (ADR 0009 E6, the D1 merge review's option c).
  Today a bump is whole tree and planning-only.
- **Pinning the base image.** `ros:jazzy-ros-base` is a tag, and its move on 2026-10-16..19 invalidated
  CI's layer cache. The uv installer (`curl astral.sh/uv/install.sh`) is unpinned in `dev` and `fast-sim`
  (N3 review E1/E2, 2026-10-19). The trade is reproducibility against taking Jazzy security updates by
  hand.
- **Git in `dev` on host-made worktrees.** A compose-level `safe.directory` would fix it, or worktrees
  could be created from inside `dev`. Today only `changed_scope.py` works around it (gotcha #23).
- **Tighter nightly region checks** (part 7.0 review, 2026-10-22). Robot's `--test <name>` is a glob, so a
  future case name containing `*?[.` could select extra cases. CHECK-1510 does not check the seed list or
  extra `include` keys. Revisit when a fourth Robot case or a new matrix key lands.
- **One launch scaffold in `scenario_runner.py`.** `run_scenario` and
  `run_scenario_with_track_limit_violation` each build their own LaunchService. Today CHECK-2250 holds
  them to one t=0 contract.
- **Test lifetime labels** (`guard` / `proof`), so that a proof-only test can be retired. This was a
  non-goal of issue #8 (2026-09-24), because it changes who may delete a test.
- **Slimming the `@`-imported repo-gotchas.md.** In 2026-09-24 it was 2,485 tokens, the largest block in
  a fresh agent's preamble, and 14 of 22 entries were irrelevant to a `scripts/`-only change. It is now
  441 lines. #18 shows the risk of moving entries behind a pointer: it bites people who do not think they
  are doing localization work.
- **Branch protection with `gh pr merge --auto`**, and parallel CI jobs, if teammates join (2026-10-11).
- **An arm64 / Jetson image as a CI job**, once there is hardware to verify against (bootstrap non-goal,
  2026-09-06). The images were kept multi-arch so this is a job, not a rewrite.
- **More tracks from [TUMFTM/racetrack-database](https://github.com/TUMFTM/racetrack-database)**
  (LGPL-3.0; noted 2026-09-29, never placed): 25 full-scale circuits as `x_m, y_m, w_tr_right_m,
  w_tr_left_m`, the same layout as `f1tenth_racetracks`, so the right-then-left width order applies.
  They are full scale, not 1/10, and the raceline has no `kappa` column. Quality varies by circuit (OSM
  and satellite widths). Gotcha 18 applies to any track picked for localization.

## Open questions

- **Why DDS matching of new endpoints sometimes takes 0.3-1.1 s.** It looks like the RMW. The t=0 waits no
  longer depend on it, but any new consumer that emits nothing while held cannot be proven matched by
  delivery.
- **M4: an estimator never comes up while held** (clock 0 for ~394 s). This happened twice, with
  slam_toolbox and amcl. Today a t=0 timeout prints `t0 stall: ...; nodes: ...`, so the next one should
  say whether the process was alive.
- **Why `racing_recording_node` ignored SIGINT and SIGTERM for 15 s** before SIGKILL (11-05). N1 made
  writes durable and reports `shutdown escalations`. The hang itself is unexplained.
- **Who reads a red nightly outside a run.** Issue #13's spec listed "that a red nightly is noticed" as not
  verified. Only a coordinator's part-boundary check reads it.
- **Wrong `paths` assignments** in `reference_stack.yaml` are caught by review and the nightly only.
  Shared claims make a dropped claim invisible (see above).
- **Two raceline sets for one `vehicle_version` are an error** in `racing_bringup.track_data` until a launch
  argument chooses between them. A second limit set for one vehicle file needs that argument first.
- **Should SIM-3030 at 1x assert reproducibility at all?** F-T0 removed the known cause. If it goes red
  again with a named start verdict, decide whether to keep it before touching any tolerance (gotcha #14).
