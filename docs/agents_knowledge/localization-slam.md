# Localization and SLAM

Every pose source (`ground_truth`, `slam_toolbox`, `amcl`, `mola`, the FAR-LIO fork), the mapping
workflows that feed them, map accuracy, the pose-source benchmark, the IMU contract and the `gpu`
image stage. Read it before a grill, spec or plan that touches `pose_source`, a mapper, a map in the
data repo, `/scan`, `/odom` or `/imu` as an estimator input, or relocalization. The raceline
extractor and racelines are in `raceline.md`, controllers in `control.md`, the simulators' sensors,
clocks and the AutoDRIVE bridge in `simulation.md`, gates and test tiers in `architecture.md`,
league rules in `competition.md`.

## Current state

The `pose_source` role is in [`config/reference_stack.yaml`](../../config/reference_stack.yaml),
selected by `pose_source:=` on `sim_pure_pursuit.launch.py`. User-facing commands are in
[racing.md](../workflows/racing.md) and [mapping.md](../workflows/mapping.md), numbers in
[results.md](../results.md#localization-evidence). Issue
https://github.com/G-S-Rodrigues/GR-roboracer/issues/47 is the open parent for the SLAM work.

| Implementation | What it is | Verified by | Landed |
|---|---|---|---|
| `ground_truth` (default) | exact sim pose; the support node owns `map -> odom` | SIM-3040, SIM-3060, SIM-3090, ACC-4010 | |
| `slam_toolbox` | online mapping, no prior map; `drive_on_estimate:=false` only (driving on it latches `TRACK_LIMIT`) | SIM-3070, SIM-3550, ACC-4030 | https://github.com/G-S-Rodrigues/GR-roboracer/pull/9, 1080-beam file https://github.com/G-S-Rodrigues/GR-roboracer/pull/73 |
| `amcl` | `nav2_amcl` on the committed survey grid; the only pose source with a closed-loop lap | SIM-3080, ACC-4030, ACC-4040 | https://github.com/G-S-Rodrigues/GR-roboracer/pull/9 |
| `mola` | MOLA-LO 3.3.0 `lidar2d` pipeline, localizing on the `.mm` MOLA built itself; open loop only | SIM-3510, SIM-3530, ACC-4510 | https://github.com/G-S-Rodrigues/GR-roboracer/pull/54, mapping https://github.com/G-S-Rodrigues/GR-roboracer/pull/72 |
| `far_lio` | not in this repo; `METHODS["far_lio"] = None` in [`scripts/pose_source_report.py`](../../scripts/pose_source_report.py) | none | |

**Estimator contract.** The estimator owns `map -> odom`. The support node composes it with `/odom`
into `/localization/odom` at control rate. Node configurations are built in
[`estimators.py`](../../ros_ws/src/racing_bringup/racing_bringup/estimators.py). Inputs are `/scan`
(best effort), `/odom` and `/imu`, never `/ground_truth/*`. The initial pose is the first ground-truth
pose on `/initialpose` (BRINGUP-2020), published only once a subscriber has matched (BRINGUP-2030),
because MOLA's `/initialpose` subscriber is volatile and a pose published before it joins is lost
silently. For `mola` the support node publishes no estimate until MOLA's pose is within 5 m of the told
pose (BRINGUP-2500,
[`support_node.py`](../../ros_ws/src/racing_bringup/racing_bringup/support_node.py)). That is a
plausibility check with no yaw term, not proof MOLA applied the pose.

**Parameter files** in [`config/localization/`](../../config/localization/): `amcl.yaml`,
`slam_toolbox_online.yaml` (64 beams), `slam_toolbox_online_1080.yaml` (selected by the scenario's beam
count), `slam_toolbox_survey.yaml` and `_survey_truth.yaml`, `mola_lidar2d_pipeline.yaml` (mapping),
`mola_lidar2d_localization_pipeline.yaml`, `mola_lidar2d.yaml`, `mola_lidar2d_mapping.yaml`,
`mola_sm2grid.yaml`, `mola_sm2mm.yaml`. The two MOLA pipeline files carry, in their headers, every
value changed from upstream and the measurement behind it. Read those headers before tuning MOLA.

**Mapping workflows.**
- Survey: [`scripts/map_track.sh`](../../scripts/map_track.sh) runs slam_toolbox with scan matching
  off over one ground-truth lap on the exact-sensor scenario. `slam_toolbox_mapping.launch.py` uses
  `slam_toolbox_survey.yaml` on every scenario unless `slam_toolbox_config:=` names another.
- MOLA: [`scripts/mola_map_track.sh`](../../scripts/mola_map_track.sh) and
  `mola_mapping.launch.py`. One lap at 1x on ground truth, MOLA reads `/scan` and `/odom`. It writes
  `.mm`, a world-frame grid and a provenance yaml carrying `T_map_mola` (truth at MOLA's first
  processed scan, car held at t=0, the script waits for `~/clock_received`). `--from-simplemap`
  builds the map from a lap another simulator drove, joining T to a truth record within 1 ms of the
  first keyframe; `--dry-run` prints the plan.
- AutoDRIVE: both mapping launches run on `simulator:=autodrive` (wall time). SIM-3770 (survey) and
  SIM-3771 (MOLA) build Porto maps from a live lap, with a held start in `drive_porto`
  (https://github.com/G-S-Rodrigues/GR-roboracer/pull/99).
- Maps live in the data submodule under `tracks/<Map>/maps/<method>/`
  ([ADR 0009](../adr/0009-track-data-lives-in-gr-roboracer-data.md),
  https://github.com/G-S-Rodrigues/GR-roboracer/pull/100). Spielberg has `slam_toolbox/res_0050`
  (committed grid amcl uses) plus denser `res_0025` and `res_0020`
  (https://github.com/G-S-Rodrigues/GR-roboracer/pull/43), and `mola/`. Porto has a survey and a MOLA
  map. SIM-5520 (nightly) regenerates the MOLA map from its recorded command.

**Evaluation.**
- [`racing_evaluation`](../../ros_ws/src/racing_evaluation/racing_evaluation/): `localization.py`
  (ATE RMSE, max, availability against `/ground_truth/*`), `map_accuracy.py` (wall distance: raw
  distance from each occupied cell centre to the gym map image's wall faces, so a perfect 5 cm map
  scores 2.5 cm), `frame_fit.py` (rigid fit of a published map frame to the true one). Its `Aligner`
  interpolates the estimate to each truth stamp and counts a truth sample unavailable once truth is
  `window` (0.5 s, a design bound: 1.5 m at 3 m/s) past it with no estimate; so availability cannot
  see an estimate that lags less than 0.5 s, and ATE carries that lag as along-track error.
- [`scripts/compare_localization.py`](../../scripts/compare_localization.py) rescores a recorded run
  offline.
- `python3 scripts/benchmark.py run --role localization` runs every pose source on `spielberg` and
  `spielberg_1080` over seeds 42, 1030, 7, 2026, open loop, about 34 min on a CI runner. It ranks
  nothing and gates nothing except structure (https://github.com/G-S-Rodrigues/GR-roboracer/pull/53,
  https://github.com/G-S-Rodrigues/GR-roboracer/pull/90, [running.md](../running.md#comparing-implementations)).
  A new method adds a `METHODS` entry beside its `POSE_SOURCE_CONFIGS` key or the benchmark fails it by
  design.
- Tier-3/4 tests gate only the catastrophic bounds (ATE RMSE 5 m, availability 0.90,
  [ADR 0008](../adr/0008-gates-decide-quality-is-reported.md),
  https://github.com/G-S-Rodrigues/GR-roboracer/pull/87). Each test keeps its measured baseline
  constants and the measurement string beside them (`test_sim_3080_amcl_localizes.py`,
  `test_sim_3510_mola_localizes.py`, `test_sim_3550_slam_toolbox_1080_localizes.py`). SIM-5030
  (nightly) runs amcl over seeds 1, 3, 7, 42, 99, 123, 1030 with a max of 2.35 m and a median of 0.45 m.

**Sensors as the estimators see them** (gym; AutoDRIVE differences in `simulation.md`).
- `/scan`: 64 beams over 4.7 rad on `spielberg.yaml`; 1080 beams over exactly 4.712389 rad on
  `spielberg_1080.yaml` (SIMJAX-1060 allowlists that file). sigma 3 cm, range 10 m. Cast at the
  declared angles (gotcha 22).
- `/odom`: per-run distance-scale error (`distance_scale_sigma: 0.01`), yaw-scale error
  (`yaw_scale_sigma: 0.014`) and drift, drawn once per seed in
  [`noise.py`](../../ros_ws/src/racing_sim_gym_jax/racing_sim_gym_jax/noise.py). No wheel slip.
- `/imu`: yaw rate, and since https://github.com/G-S-Rodrigues/GR-roboracer/pull/86 linear
  acceleration in the body frame as specific force (REP 145, a_z = +9.81 at rest,
  [`imu.py`](../../ros_ws/src/racing_sim_gym_jax/racing_sim_gym_jax/imu.py)), with optional seeded
  gyro/accelerometer noise and bias in the scenario's `noise:` block (off in every committed
  scenario). Covariances stay 0 with noise on, and a_y ignores slip. **`orientation` is the
  ground-truth orientation** in the gym and on AutoDRIVE (see the triage's gotcha proposal). AutoDRIVE
  adds +g to z unrotated as an interim; the 2026-11-05 ruling rotates g into `base_link` with FAR-LIO
  part 5b, replacing that line, not adding to it
  ([`contract_node.py`](../../ros_ws/src/racing_sim_autodrive/racing_sim_autodrive/contract_node.py)).
  MOLA does not read `/imu` (only `/scan` and `/odom`).

**MOLA in the image.** The apt closure of `ros-jazzy-mola-lidar-odometry` 3.3.0 is pinned by name
(`docker/apt-preferences/mola.pref` and `mola.closure`, CHECK-1880), GPL-3.0, run as separate
processes so the repo stays MIT ([`third_party/NOTICE.md`](../../third_party/NOTICE.md)).
`mola_sm_loop_closure` is not installed: the owner approved adding it for part M3 (2026-11-02), and
only a spike image carried it.

**`gpu` stage.** `dev` plus pinned CUDA 12.8 nvcc and cudart, `FROM dev`, compose profile `gpu`,
own volumes and `ROS_DOMAIN_ID`, built and tested on the owner's machine only (GPU-3010, CHECK-1900 to
1940; [ADR 0002 amendment 2026-10-25](../adr/0002-development-environment.md),
https://github.com/G-S-Rodrigues/GR-roboracer/pull/49). The gym's JAX stays on CPU inside it
(gotcha 4). No node uses it yet; it was built for FAR-LIO's CUDA build.

**The FAR-LIO fork** is `G-S-Rodrigues/GR-roboracer-far-lio` (local `~/gitroot/GR-roboracer-far-lio`),
Apache-2.0, from TUMFTM/FAR-LIO (arXiv 2606.26010). What the fork changed, per its README:
- `pipeline.lidar_2d` (default false; with it off, 3D is bit-exact, FARLIO-1010..1030). With it on:
  covariance extrusion so a wall's normal stays in-plane (`planarize_covariance`), Gauss-Newton and LM
  solve a 3x3 (t_x, t_y, w_z) system and embed it back (a parametrization, not a prior; z, roll and
  pitch cannot move), `node.input_scan:=/scan` takes `LaserScan` through `laser_geometry`, and
  `config/f1tenth.yml` holds F1TENTH values (0.5 m voxels, `tam_gicp_ext` with `ekf_2d_kin`). This is
  fork `main` at 141fa3b (fork PR #1, merged).
- Branch `initial-pose` at 8b18547, 7 commits ahead of fork `main`, **no fork PR**: the
  `node.input_initial_pose` input (RELIABLE, TRANSIENT_LOCAL, frame `far_lio_odom`, one shot,
  FARLIO-1810..1880) and the 2D EKF yaw fix (the node read `mask[0..2]` of a one-entry mask, undefined
  behaviour, so the lidar yaw was never fused and the filter started at yaw 0; now `mask[0]`, and a 2D
  `R_init` that is not one entry is refused, FARLIO-1890..1910).
- CPU build: `colcon build --packages-up-to tam_odometry tam_state_estimation_node --cmake-args
  -DCMAKE_BUILD_TYPE=Release -DBUILD_CUDA=OFF` in the fork's `deps` image (full command in its
  README). In our `dev` it also needs `libpcl-dev` and `ros-jazzy-pcl-conversions`.
- Missing for a pose source: a launch file, map save, `map -> odom`, a vcs pin and the `far_lio`
  entry in this repo (spec part 5b; part 5c is the CUDA variant).
- How the Porto spike c0 wired it by hand (2026-11-12, scratch scripts only): `tam_gicp_ext` and
  `tam_state_estimation_node_ekf_2d_kin` under namespace `/core/state` with `/tf` remapped to
  `/far_lio/tf` (so it never fights the stack's `map -> odom`) and the EKF's odometry to
  `/far_lio/odometry`; config from `f1tenth.yml` with `use_sim_time` false, `cloud_frame: lidar`,
  `input_scan: /scan`, AutoDRIVE wheelbase 0.16868 / 0.15532 m, tyre radius 0.059 m,
  `map.max_distance` 15 m. Mapping sets `update_map` true; localization loads the `.pcd` with
  `update_map` false and `wait_initial_pose` on `/core/state/initial_pose`.

**Not built:** relocalization after a kidnap, teleport detection, the league checkpoint fast path,
a wheel-slip scenario, online SLAM as a backup for a changing track, loop closure in any committed
mapping workflow, moving-barrier handling and map correction for real tracks.

## What we measured

All Spielberg unless named. "Open loop" means the car drives on ground truth and the estimate is only
scored.

**Pose sources** (the public summary is [results.md](../results.md#localization-evidence)):

| Source | Setup | ATE RMSE | Max | Source |
|---|---|---|---|---|
| amcl | 64 beams, seed 42, 5x | 0.14-0.44 m | up to 1.83 m | SIM-3080 baseline string, PR #9 |
| amcl | SIM-5030 nightly, 7 seeds x 5, 2026-10-21..11-01 | pooled median 0.29-0.35 m | worst 20.37 m (10-31, seed 1) | nightly artifacts |
| slam_toolbox | 64 beams, seed 42 | 2.43-2.88 m | 3.67-4.99 m | SIM-3070, PR #9 |
| slam_toolbox | 1080 file on `spielberg_1080`, 5x, seeds 42/7, six laps | 0.81-1.25 m | 1.53-2.24 m | SIM-3550, PR #73 |
| slam_toolbox | 64 file on `spielberg_1080` | 6.9-8.2 m | | PR #73 |
| mola | own map, 1080 beams, 1x, five laps | 0.70-2.13 m (0.74-0.75 at low load) | 1.41-4.15 m | ACC-4510, SIM-3510 |
| mola | stock pipeline on 64 beams | 4.6-5.1 m wrong over the first 5 s | | `mola_lidar2d_pipeline.yaml` header |

- **amcl's tail (SIM-5030, 2026-10-21..11-01, 12 nightlies x 35 runs).** Median steady at 0.29-0.35 m.
  Before 10-31, 11 of 350 runs exceeded 1.0 m and none 1.6 m. Then 10-31 seed 1 hit 20.37 m (along-track
  lag building on the long stretches, then a jump at s 115-123 to -24 m and growing: a global
  mislocalization it never recovered from) and 11-01 seed 3 hit 2.90 m. No change touched amcl's path in
  that range and the CI image was byte-identical. The bad seeds are always 1 and 3.
- **amcl and odometry scale (2026-10-20).** Every negative distance-scale draw is unhealthy, every
  positive one healthy: seed 3 (-1.30 sigma) median 0.536 m, seed 257 (-1.99 sigma) 1.288 m and lost 3/3,
  seed 42 forced -1.3 % 0.572 m, seed 42 natural (+1.25 sigma) 0.259 m. On straights a positive bias is
  pulled back by the scan (+0.11 to +0.63 %/m), a negative one is not (-0.5 to -2.7 %/m) and enters the
  next straight 0.4-1.55 m behind. Widening `alpha3` 0.05 -> 0.081 did nothing for seed 3 and made the
  forced case worse (0.775). The owner accepted it as a known limitation (N2 D5, 2026-10-20). A real
  car's wheel radius and tyre wear produce exactly this bias.
- **N2 controls (2026-10-15..20, 96 open-loop runs, ATE RMSE per run).** The odometry draw, not the
  start pose, decides: seed 3's start with seed 42's noise 0.19-0.26 m, seed 42's start with seed 3's
  noise 0.72-0.91 m. Within seed 3's draw the scale term is the cause: scale zeroed 0.21/0.40, random
  walk zeroed 0.53/0.67, all odometry noise zeroed 0.22/0.43. Seed 42 forced +1.3 %: 0.15/0.16; forced
  -1.3 %: 0.59/1.87. Moving the lidar to its true mount (gotcha 26) changed none of this (seed 3
  0.49-2.16, -1.3 % 0.86/1.07; six-seed pilot 0.17-0.58 m on both). The error is along-track: lateral
  RMS 0.007-0.096 m on all 79 amcl runs under 0.75 m RMSE. 16 of 96 runs put the estimate over 1.1 m
  sideways (outside the corridor), all at RMSE 0.75 m or more. slam_toolbox on seed 3 scored 2.16-2.23 m.
- **Dead reckoning alone (SIM-3100, 2026-09-21, `contract_test.yaml`, one lap at 1x):** ATE RMSE
  0.755 / 1.170 / 0.403 m on seeds 42 / 7 / 1030, and 9.5e-15 m with the noise block removed.
- **Karto's matcher drifts on exact input (PR #9, plan D7, 64 beams, seed 42, 1x).** Yaw error grows a
  constant +3-4e-4 rad per metre on straights (about 1e-4 rad per match, below the matcher's angular
  resolution, +0.08 to +0.19 rad per lap, opposite to the lap's -6.49 rad net turn) whatever the
  odometry noise, scan noise, beam count (541 tried) or `fine_search_angle_offset`. Max pose error:
  12.5 m on defaults, 2.0-3.5 m trusting odometry more, 3.25 m with exact `/odom`, 2.5 m with exact
  scan and odometry, 0.000 m with matching off. Loop closure cannot absorb it: by the lap's end the
  error (2.5-3 m) is outside `loop_search_maximum_distance`. Karto's reading of `/scan`'s metadata was
  checked line by line and is not the cause.
- **amcl closed loop slides on the s 64-108 straight (ACC-4040, 2026-11-11).** Peak along-track error
  over sim 75-97 s ranged -1.17 to +1.03 m in 10 passing runs on main and on F-T0. One gate run reached
  -2.40 m, turned in about 2 m late and hit the wall at 97.03 s with `collision_count` 0 (it counts only
  CoG-cast ranges under 0.01 m). Not caused by F-T0 (https://github.com/G-S-Rodrigues/GR-roboracer/pull/95).
- **amcl on AutoDRIVE at speed (b6, 2026-11-11, PR #99 body).** amcl held 0.04-0.16 m up to 4.43 m/s
  after pulling a launch-wheelspin error of about 2 m (odom 1.8x truth in the first second) under
  0.1 m by t = 2-2.5 s. The sim's collision respawn jumps the car 0.36-1.43 m back and up to 67 deg;
  `/odom` sees no jump and integrates spinning wheels, so each respawn adds 1-2 m of error and the car
  re-crashes every ~0.5 s. Respawn is a kidnap.
- **slam_toolbox at 1080 beams (part 8, 2026-10-30).** `coarse_angle_resolution` 0.0087 -> 0.0035
  alone took ATE from 8.17/6.93 m (seeds 42/7) to 1.23/0.88 m. Ruled out: latency (1x: 7.54/7.21),
  loop closure (off: 7.99/6.94), scan noise (sigma 0: 9.73/9.01, worse). slam_toolbox's multithreading
  is unseeded, so one run is one draw: six laps underestimated seed 42's map-median spread
  (0.439-0.670 m on a load<8 re-measure).
- **MOLA localization depends on the map instance (part 2, 2026-10-30).** On the seed-7 map at MOLA's
  default `icp_prior_weight` 1.0, 2/5 runs pass (2.27, 2.29 m) and 3/5 diverge (37.7 m). At 5.0:
  0.751/0.752/0.742 m. At 50: 0.69-0.70 m. The old map: 1.55 m at 1.0, 1.555-1.562 m at 5.0. Divergence
  starts as along-track lag at the first sharp corner and becomes a wrong lock at the second. The
  localization pipeline also needed `maximum_sigma` 1.0 -> 0.5 and the matcher threshold 2 sigma -> 1
  sigma: on a map from another run the paired ratio sits near 0.78, sigma ramps to its maximum within
  a second, ICP jumps to a wrong optimum (49 deg) and the worker drops 20-90 % of scans. Either change
  alone still fails.
- **MOLA processing rate.** The sim publishes 100 Hz scans; MOLA kept up with about 35 %. A fixed
  `min_time_between_scans` 0.04 (25 Hz) is repeatable and 2-3x better over the first 5 s (0.055-0.061 m
  against 0.12-0.18 m). Halving to 12.5 Hz made maps worse. Scan matching is wall-clock work, so `mola`
  runs at 1x: a faster simulated clock drops scans.
- **MOLA cost on AutoDRIVE start-up.** A 0.127 s frame gap fell during MOLA's launch on Porto
  (2026-11-11), the only gap over 0.056 s in that run; the AutoDRIVE stale-input timeout is now 200 ms
  (PR #99).

**Maps.**

Survey maps (slam_toolbox, exact poses, noise-free 64 beams, matching and loop closure off,
2026-10-23/24). Distance from each ground-truth wall point to the nearest occupied cell, left/right:

| Map | Max | p95 | Median |
|---|---|---|---|
| 0.05 m, 1 lap, scan every 0.2 m / 0.1 rad (committed) | .097/.105 | .067/.066 | .029/.028 |
| 0.025 m, same, 1.5 m/s | .088/.093 | .056/.055 | .026/.027 |
| 0.01 m, same, 1.5 m/s | .249/.314 | .068/.069 | .032/.033 (extractor fails) |
| 0.025 m dense: 2 laps, scan every 0.02 m / 0.02 rad | .074/.081 | .049/.045 | .018/.018 |
| 0.02 m dense, same | .066/.069 | .046/.042 | .017/.018 |

- Scan density matters more than cell size. slam_toolbox adds a scan only every
  `minimum_travel_distance` / `minimum_travel_heading`, so car speed changes nothing until the mapper
  is the bottleneck. At 0.2 m spacing with 64 beams (7-15 cm apart on the wall) 1 cm cells are never
  hit and the wall comes out dotted. The extractor's morphology kernels are sized in cells, a likely
  second cause (untested).
- The dense run overloaded the async mapper ("queue is full" from about 18 s). The synchronous mapper
  and a slower lap were not tried.
- Every survey map sits 1.3-1.7 cm outward of the true wall, whatever the cell size. The hairpin's
  inner tip is rounded by 0.171 m on the 0.05 m map, where min-time lines come closest to the wall.
- The raw-cell-centre metric (`map_accuracy.py`) gives the dense maps 0.067 / 0.049 / 0.046 m p95 at
  0.05 / 0.025 / 0.02 m cells ([results.md](../results.md#mola-map-accuracy)).

MOLA maps (one lap, 1080 beams, 1x, no loop closure):
- Five seed-42 laps, first pipeline (2026-10-28): wall p95 0.52-0.99 m, median 0.16-0.43 m, mean
  offset +0.074 to +0.078 m. The owner accepted MOLA's map as a baseline the same day: **the 5 cm wall
  target applies to the known-pose survey map only**; a SLAM map is reported against it, never gated.
- `icp_prior_weight` 50 in the mapping pipeline: over two laps x seeds 7 and 42, p95 fell from
  0.52-2.49 m to 0.88-1.15 m and the extractor closed all four laps. 5 and 20 also close more laps than
  1.0; 200 closes none.
- Current pipeline, ten laps on seeds 7 and 42 (SIM-3530): p95 0.79-1.57 m, median 0.20-0.41 m, mean
  offset +0.077 to +0.160 m. The committed map scores 0.82 m p95 and 0.21 m median.
- The same command at the same seed gives a different map each run (seed 7: p95 0.51 vs 0.75 m), most
  likely from mola-cli's thread-timed scan dropping.
- Diagnosis (part 2): a steady drift of about 1 % scale, yaw +4 deg per lap, 2-3 m position error, with
  ICP quality about 0.99, so not wrong optima. The extracted track's length ratio ran 1.0019-1.0099.
- **Porto MOLA map (AutoDRIVE, PR #99).** The committed map's T carries a launch slip: the best rigid
  fit is 0.750 m and 1.19 deg off T. Under T: 0.206 / 0.696 / 0.825 m (median / p95 / max). Under the
  best fit: 0.045 / 0.116 / 0.196 m, as good as the Porto survey map's 0.050 / 0.123 / 0.195 m. In the
  A6 re-run the error was a one-time rigid slip of -0.65 to -0.80 m along the start straight (parallel
  walls, x unobservable), with 326 registrations, 0 ICP rejected and no "Sustained ICP failure" line:
  nothing flagged it. An earlier run took T from the first truth record while the car was still driving
  before the reset (an 8.08 m teleport), scoring p95 6.15 m; the held start fixed that procedure.

**FAR-LIO** (fork tests on a synthetic oval: 8 m straights between walls 2.2 m apart, arcs r 3 m; and a
spike on AutoDRIVE Porto).
- Registration-only CPU latency (FARLIO-1710): GICP 0.102 ms mean at 64 beams, 0.394 ms at 1080.
  Fraction of points with a covariance at 64 beams: 0.000 at 0.25 m voxels, 0.605 at 0.5 m; at 1080
  beams 0.92-0.95.
- FARLIO-1610 (one synthetic lap, 2 m/s): worst position 1.6-11.5 m, worst yaw 3.4-14.8 deg; it never
  meets 0.10 m / 2 deg. GICP's own yaw is tens of degrees wrong in every case, before and after the EKF
  fix: lidar yaw error grows from about 1 deg to 22-44 deg over the first straight as along-track
  position is lost. Standing still, the lidar pose slides 1.0 m along the straight in 3 s.
- 98-99.9 % of lap registrations are rejected "converged | too far from guess". The gate
  (`ellipsis_check`, 0.15 m along x 0.075 m across the EKF guess) checks position only. The few it
  accepts carry 2-41 deg of yaw error, and at R_yaw 0.0002 with a 0.3 rad residual clamp (a clamp, not
  a rejection) each moves the EKF yaw 0.5-3 deg in one step. R sweep 0.0002-0.05: no value keeps
  FARLIO-1610, 1890 and 1900 all inside their bounds. A 0.02 rad/s gyro bias integrates to 24.5 deg
  over the lap and fused lidar yaw removes none of it.
- A told initial pose must be within about 0.1 m / 2 deg. 0.3 m / 0.2 m / 5 deg never initializes
  the EKF; the matcher barely corrects an offset (0.051 m left from a 0.054 m offset).
- The lidar-odometry covariance handed to the EKF is constant (`ConstantCovariance`); a
  `cramer_rao_bound()` helper exists with no caller. The EKF only seeds GICP; the GICP solve has no
  prior term, and the map is updated with the GICP pose.
- **Porto spike c0 (2026-11-12, NO-GO).** Fork 8b18547, CPU, `ekf_2d_kin`, AutoDRIVE at about 4 m/s
  with scans 18-55 ms apart. Fixture replay: 87 % of 616 scans gated, along-track error 1.9 m after
  4 m on the first straight (cross-track 0.11 m), 22.5 m one second after a reset pulse with no
  recovery. Live mapping lap 1 RMSE 1.43 m, lap 2 25.5 m (5 % of registrations OK). Localizing on that
  lap-1 map: lap 1 RMSE 12.3 m (0.9 % OK). Divergence starts at the first corner (yaw error 31-40 deg).
  Per scan 1.34 ms mean CPU; RSS 64 MB lidar node plus 43 MB EKF. Cause: the guess fence (sized for
  100 Hz) plus 2D along-track degeneracy, not the wiring.
- The FARLIO-1040 CUDA goldens fail 4/4 in 3/3 runs (520-570 values off by up to 0.009, differing run
  to run) on 141fa3b and 3060e62, though the fork README records 7 bit-identical recordings on the same
  GPU (2026-11-02). Do not regenerate the golden to green it.

## Dead ends

- **Kinematic-ICP, KISS-SLAM + MapClosures, Cartographer, SynPF, Beluga as methods (grill
  2026-10-25).** Kinematic-ICP is odometry only (no map load, loop closure or relocalization) and leans
  on wheel odometry with a no-slip model, the condition that broke Cartographer in the SynPF paper.
  KISS-SLAM is 3D-only offline Python. Cartographer is frozen upstream ("no longer actively maintained",
  last push 2024-01). Beluga stays a candidate component for FAR-LIO's global initial guess because it
  is Apache-licensed.
- **GPL-2.0 and unlicensed code.** FAST-LIO and Point-LIO (GPL-2.0), Lightning-LM (no licence) and
  Scan Context (CC BY-NC-SA) are excluded. HKU FAST-LIO2 also needs a 3D cloud. The 1V-fix-c queue item
  called "FAST-LIO" was the FAR-LIO fork; a FAST-LIO2 would be a separate implementation.
- **Forking MOLA.** MOLA is GPL-3.0, used unmodified from apt as separate nodes (owner Q7 yes,
  2026-10-25). Anything built inside it is GPL and must stay a separate process. The online global
  mapper is MOLA Pro, not open.
- **MOLA loop closure on a one-lap map (part 2, 2026-10-28).** One lap gives one revisit, so one
  candidate (`0<=>11`). Its ICP answer was yaw -68.9 deg against a +6.4 deg prior; GNC rejected it, and
  the log's "accepted loop-closures: 1" counts before GNC. The graph moved 9e-5 m. Submaps are cut every
  10 s of travel (`max_time_between_kfs_to_break_submap` is measured from the submap's first keyframe),
  candidates are bounding-box overlap under the drifted prior with no appearance test, and
  `threshold_sigma_initial: 10.0` on 1.0 m voxels is a 20 m pairing window. The closure tool also needed
  `MOLA_DESKEW_IGNORE_NO_TIMESTAMPS=true`.
- **MOLA three-lap mapping with closure (M3 stage 1, 2026-11-06).** Three laps ran with 0 collisions,
  but the unclosed three-lap map already had p95 34.30 m (seed 7) and 18.16 m (seed 42), and closed grids
  were 13-36 m (best 12.98 m). Both kill conditions hit; the cause is upstream of closure. That run was
  under a lock incident at load 4.5-11.8, so a quiet re-run (stage 1b) was made on 2026-11-06; its score
  is not recorded (unverified).
- **slam_toolbox localizing against a committed pose graph (PR #9, D7).** A graph surveyed with
  matching off has no vertices: `localization_slam_toolbox_node` logs `Ceres was called when there are
  no nodes` and segfaults on the first scan. Matching on with near-zero variance penalties (1e-6 /
  1e-8) gives vertices but a map rotated 8.9 deg (fit median 0.48 m) and localization median 35 m.
  Hence `pose_source:=slam_toolbox` maps online from the true start pose.
- **slam_toolbox tuning that did not help the drift (PR #9):** default mapper parameters, trusting
  odometry more, 541 or 1081 beams at 64-beam settings, finer `fine_search_angle_offset`, zeroed
  odometry noise, `/ground_truth/scan` as mapper input, `use_scan_matching: false` for localization,
  near-zero variance penalties. (At 1080 beams `coarse_angle_resolution` did help, above.)
- **Widening amcl's motion noise for odometry bias.** `alpha3` 0.081 is sign-blind and zero-mean; it
  cannot fix a one-sided scale error (above).
- **A told pose to FAR-LIO from amcl.** amcl-scale uncertainty (0.5 m / 15 deg) never initializes
  FAR-LIO's EKF; only a pose good to about 0.1 m works until the gate and GICP are fixed (part 4c).
- **Fixing FAR-LIO's yaw by tuning R.** See the R sweep above: GICP's yaw is wrong, so no R helps.
- **A slower mapping lap as a fix for drift.** Slower driving adds scans per metre but adds no
  observability on a straight longer than the 10 m lidar range, and does not remove a per-distance
  odometry scale bias (reasoned, 2026-10-28; not measured).

## Ideas not yet specified

- **Kidnap-tolerant pose source (b6, 2026-11-11).** Respawn is a kidnap (league `collision_count`;
  encoders against IMU on the real car). Re-seed amcl wide (about +-1.5 m, +-70 deg) and ignore odom
  for about 0.3 s, as a second implementation, with slip-aware encoder plus IMU odometry. Validate
  offline (first-second odom/truth 1.8 -> below 1.1), on replay (below 0.5 m within 1 s of an injected
  respawn), then live. No `amcl.yaml` retune without a stated cause; a `sigma_hit` change must cite the
  map's measured wall error (0.05 m median, 0.12 m p95).
- **Online odometry-scale estimator upstream of amcl** (2026-10-20), selectable per ADR 0005, for the
  negative-scale failure. Also "recovery on".
- **Phase 3 of the spec, named but not specified:** global relocalization after a kidnap to any pose
  (MOLA `mola_relocalization`; FAR-LIO via a GPL-free global guess such as Beluga), the league fast path
  (checkpoint poses measured beforehand by colliding on purpose in the released compete image; owner:
  "both must work", the fast path never replaces the global one), teleport detection without
  restricted topics (registration residual jump, IMU spike), a seeded wheel-slip scenario with the
  odometry scale broken mid-lap, online SLAM as a backup for a changing track, loop closure for
  FAR-LIO's map. Owner ruling: "only after one is working is that i should start thinking in those
  details".
- **An own 2D racing SLAM (research 2026-10-28, ranked, not measured).** First choice: an own pose
  graph on GTSAM (BSD) with the front end grown from the fork's 2D registration, because it is
  licence-clean in process and has the coupling point the problem needs. Loop closure: a wide
  deterministic correlative or branch-and-bound 2D search (Cartographer's approach, reimplemented)
  gated by a Mahalanobis test on the odometry relative pose, with GTSAM GNC or PCM (Kimera-RPGO) to
  reject closures inconsistent with each other or with odometry, so a moved barrier shows up as a
  rejected closure, not a bent graph. MapClosures (MIT) is the licence-clean descriptor option; its
  2D-input behaviour is unverified (it ground-aligns first). Cartographer 2D is cheap to add as a peer
  for comparison. More than one lap per mapping run is the only way to get more than one closure.
- **Coupling odometry into the scan matcher.** MOLA: try `max_registration_mahalanobis` (default 0,
  off), `deskew_odometry_sensor_label`, `optimize_twist` and the StateEstimationSmoother with
  `KinematicModel::Tricycle` (`use_state_estimator:=True`), all config only. MOLA's prior is weak and
  isotropic (0.5 m on x, y, z alike), not shaped along-track, and MOLA has no degeneracy detection.
  FAR-LIO: add a prior toward the EKF guess inside GICP (Lambda into JTJ and JTr; the prior mean is
  identity because the guess is pre-applied), apply it only on the weak eigenvectors of the planar 3x3,
  and implement the unused `cramer_rao_bound` as a Hessian-based covariance so the EKF trusts wheels
  along-track. All small-medium, in Apache code.
- **FAR-LIO part 4c:** GICP degeneracy on the synthetic lap and a yaw-aware acceptance gate sized for
  AutoDRIVE's 18-145 ms scan periods. Go/no-go proposed for Porto: one-lap ATE RMSE no worse than
  amcl's on the same two-lap recording with lidar.
- **Deterministic MOLA mapping** by lossless offline processing of a recorded bag:
  `mola-lidar-odometry-cli --input-rosbag2 ... --output-simplemap`, then `mola-sm-lc-cli`, then
  `sm2mm` and the grid export. The stack records `log/mola_mapping.jsonl`, not a bag.
- **Does MOLA inherit the odometry scale bias?** Map the same seed with `odom_topic_name` empty or
  `distance_scale_sigma: 0`, and correlate each seed's drawn scale error with the extract length ratio.
- **slam_toolbox survey at 1080 beams.** `slam_toolbox_mapping.launch.py` runs the 64-beam survey
  config on every scenario; select it by beam count, as localization does.
- **slam_toolbox's own online error** (2.4-2.9 m at 64 beams) is likely configuration; diagnose it as
  improvement work (spec non-goal, backlog). Its synchronous mapper for dense surveys is untried.
- **Localization with moving barriers** (backlog, parked by owner "for later"). League barriers get
  knocked and put back slightly off. How does an estimator degrade when the map is a few cm wrong in
  places? Survey section 4 warns against making SLAM solve it (a semi-static layer, not a remap). Needs a
  seeded barrier-shift scenario.
- **Map correction for real tracks** (owner, 2026-10-13). Open sides and possibly an overpass: correct
  the SLAM map by hand or by script before extracting the track. Nothing is built until a workflow for a
  mapped map with no ground truth has been tried on AutoDRIVE. The ICRA 2026 physical course had a
  bridge and deliberate gaps in the boundaries.
- **Validation without ground truth** (spec C10): map-to-map alignment between two mapping runs,
  repeat-lap consistency, loop-closure residuals, registration residual as an on-car health signal, and
  on the car comparing MOLA's first released estimate with `/initialpose` in position and yaw.
- **GPU-tuned SLAM / localization: not for now** (owner, 2026-09-30). GPU access for a league
  submission is unstated; read the league's Technical Guide first.
- **amcl particle cloud in RViz** needs `nav2_rviz_plugins` in the image:
  https://github.com/G-S-Rodrigues/GR-roboracer/issues/38.

## Open questions

- Why does amcl pull a leading estimate back on a straight but not a lagging one (scan likelihood
  asymmetry, resampling)? The lidar mount is ruled out: the true mount reproduced it (N2 controls).
- Are SIM-5030's 20.37 m and 2.90 m tails noise or a code effect? Two events cannot tell; the
  proposed bisect counts runs over 1.6 m on seeds 1 and 3 (15 runs per seed per commit).
- Should amcl take only its first `/map` (`first_map_only: false` today, so any later `/map` on the
  domain replaces it)? It is the reference stack's localizer, so this is a reviewed change.
- What makes mola-cli die? A segfault (exit -11) under heavy foreign load after sustained "Dropping
  LiDAR scan (worker busy)" up to 60 % (2026-11-01), and an abort at start-up (`std::system_error:
  Invalid argument`, 1 of 32 benchmark rows, 2026-11-11). On-car CPU contention is normal.
- Why does nav2's amcl lifecycle manager sometimes stall at start-up? Twice: a SIM-5030 CI leg
  (2026-10-14, "Failed to change state for node: map_server ... async_send_request failed", 1 of 8
  legs) and ACC-4040 in a ready gate (2026-10-24, "Activating map_server" then nothing for 395 s,
  host load 6.3). Neither reproduced. Healthy runs activate in about 10 ms. The runner times out on
  `/localization/odom` without naming the lifecycle state.
- What opened a 3.26 s hole in the *recording* that nightly run 41814536665 scored as amcl
  unavailability (0.977), and why did `racing_recording` ignore SIGINT and SIGTERM until SIGKILL in run
  42010230644? `score_recording` still scores a truth-stream hole as the estimator's.
- Should SIM-5520 also run SIM-3510 on its regenerated map, since MOLA's localization depends on the
  map instance?
- Is the Porto MOLA map's -1.1 deg residual real yaw, and where exactly did it slip? `sm-cli
  export-keyframes` or SIM-3771's `mola_keyframes.tum` would settle both.
- The spike recordings `a_park` and `s1` on Porto show repeated 1.2-1.5 m single-frame jumps to
  (-8.09, 1.57). Unexplained.
- Does the FARLIO-1040 CUDA failure come from the GPU driver (591.74), the deps image rebuilt after
  the golden, or GPU nondeterminism?
- Does the 5-tap FIR on yaw rate in `f1tenth.yml` lag gyro yaw by about 0.76 deg on arcs?
- May a league submission load a map built on the released track, and may it use checkpoint poses
  measured by colliding on purpose? Ask the organizers (owner flag, see `competition.md`).
- What did LamarRacing (ICRA 2026 winners) localize with?
