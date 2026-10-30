# Verification

The measurements on [Results](results.md) and the behaviour of each Available implementation in the
README's capability matrix are backed by tests that run on every pull request or every night. Three
things are not. The Mapping demonstration on Results is a recording, not a test. The failure of the
TUM minimum-time optimization to converge on Spielberg is a single trial that no test gates (see
`raceline/README.md`). GPU runs are not tested in CI: the GPU image's structure is checked on every pull request, but
building and running it is a manual check on a GPU machine. Rows marked Planned or Exploratory are
not claims of working code. This page says what each of five layers proves, then how to run the checks.

## 1. Unit tests

ROS-free logic is tested without a ROS graph, so these tests are the quickest layer: the layer includes the JAX simulator backend on Spielberg and the raceline suite. They cover the track and
Frenet conversions, the pure pursuit law, the metrics accumulator, the simulator backend's scan
angles and the safety supervisor.

- COMMON-1010: Track Frenet round trip.
- PP-1010: Pure Pursuit geometry.
- METRICS-1010: the metrics accumulator.
- SIMJAX-1090: the scan angles of a real `GymBackend`.
- SAFE-1020: stale input stops the car.
- BRINGUP-1700 to BRINGUP-1730: the MOLA mapping launch and script. The launch starts only MOLA with no initial map, the script drives it, and no kill watchdog holds the caller's pipe (BRINGUP-1770).
- BRINGUP-1740: the committed MOLA map's provenance agrees with its grid files (hashes, sizes, pipeline).
- BRINGUP-1750: the committed grid is in the world frame: its wall 95th-percentile distance to the track's true walls is within a measured bound, and a grid rotated 90 degrees is far outside it.
- BRINGUP-1760: the committed grid and the `.mm` agree under the provenance's start-pose transform, with no ground truth. It is the frame check a real car's map can use.
- EVAL-1550 and EVAL-1560: the scenario's true wall grid, and the nearest-neighbour measure between a grid and a `.mm` layer.
- RACELINE-1510: the raceline extractor reads the committed MOLA grid unchanged and returns a closed track whose length is within a measured tolerance of the committed track's.

## 2. Launch and contract tests

These tests start one node (the simulator adapter, or the controller for PP-2010) and check its ROS
contract: what it publishes, at what rate and in which frames, and which inputs it reads. They show
the contract holds without a full lap.

- ADAPT-2010: scan, odometry and IMU use standard ROS message types.
- ADAPT-2030: the backend publishes at its configured 100 Hz.
- ADAPT-2140: the backend declares its capabilities (clock, reset, step mode, time scale, start held) as read-only parameters. The conformance tests that need one select by it, and a test whose capability the backend lacks is named as not applicable in the output.
- PP-2010: the controller node follows the trajectory's speed only under `speed_source: trajectory`, and reads the odometry twist into its lookahead only under `lookahead_gain > 0`.

## 3. System tests

These tests launch the real stack and run a seeded lap, then compare it to a committed record or
bound it against ground truth.

- SIM-3020: a seeded lap through the real launch completes.
- SIM-3040: live metrics match `tests/golden/baseline.json` within tolerance.
- SIM-3090: the Spielberg lap matches `tests/golden/spielberg_baseline.json`.
- SIM-3110: the Spielberg lap on the committed minimum-curvature line (`planner:=optimized_raceline speed_source:=trajectory`, default vehicle, seed 42) completes with 0 collisions and 0 saturation events, at its own measured lap time of 113.08 s (tolerance 0.05 s).
- SIM-3070: with the car driving on ground truth, `slam_toolbox` publishes an estimate whose RMSE against ground truth is within 4.0 m. The test is open loop and does not claim the estimate is good.
- SIM-3510: with the car driving on ground truth (open loop), `mola` localizes on the map MOLA built itself (1080 beams, 1x, seed 42): position RMSE within 2.5 m, maximum within 4.5 m, available in at least 0.99 of samples (five laps measured 0.704-2.131 m, 1.414-4.149 m, 0.9975 or better); the offline score must agree.
- SIM-3530: one MOLA mapping lap at 1x (seed 42) produces a grid whose wall 95th-percentile deviation from the track's true walls is within 1.96 m (median 0.52 m, mean offset 0.21 m, at least 34 900 wall cells; ten laps measured 0.79-1.57 m, 0.20-0.41 m, +0.077..+0.160 m, 43 631 or more cells), and `nav2_map_server` serves the committed grid. The 5 cm target is reported, not gated.
- SIM-3080: with the car driving on ground truth (open loop), `amcl` publishes an estimate on the saved Spielberg map over a full lap with 0 collisions; the estimate is available in at least 0.99 of samples and its position RMSE is within 0.65 m (the seed-42 bound).

## 4. Acceptance tests (Robot Framework)

Black-box cases on the composed stack, one per behaviour a user can name:

- `Vehicle Completes Baseline Lap Safely` (ACC-4010)
- `Vehicle Stops When Track Limits Violated` (ACC-4020)
- `Localization Estimate Tracks Ground Truth: slam_toolbox` and `Localization Estimate Tracks Ground Truth: amcl` (ACC-4030)
- `Localization Estimate Tracks Ground Truth: mola` (ACC-4510), the same case as ACC-4030 on MOLA's own map at 2.5 m RMSE
- `Vehicle Completes A Lap Driving On Its Own Estimate` (ACC-4040)

## 5. Nightly coverage

The nightly runs on a schedule, never in a pull request. It runs one job per region of interest and
seed: five regions (`static`, `slam`, `path_planning`, `control` and `shared`) and the seeds 42, 1030, 7
and 2026. `static` is build and lint with the layer 1 and 2 tests, which read no seed, so it runs once
(seed 42); the other four run every seed, so seventeen jobs. The short regions (`static`, `control`,
`shared`) have a 60-minute timeout, `slam` and `path_planning` 180. The other four split
layers 3 and 4 between them, and the file [`tests/nightly_regions.yaml`](../tests/nightly_regions.yaml)
names which region owns each test. The `control` job also runs one seeded lap on the job's seed. The
tests keep their own pinned seeds, so the seed only changes that lap and layer 5. Layer 5, the
cross-product of tracks, implementations and seeds, runs on the seed-42 job of each region that owns a
layer 5 test. Its track dimension is Spielberg alone today.

- SIM-5010: the committed occupancy grid regenerates from the survey.
- SIM-5020: `slam_toolbox` and `amcl` over seeds 42 and 1030 on Spielberg.
- SIM-5520: the committed MOLA map regenerates from its recorded command. The inputs and the start-pose transform must match exactly and the voxels and the occupancy grid land within a measured nearest-neighbour p95 (1.9 m each); position 0.05 m and yaw 0.2 degrees. A grid rotated 90 degrees is named.
- SIM-5030: the `amcl` run-to-run spread, over seeds 42, 1030, 1, 7, 99, 123 and 3, with 5 runs per seed.
- SIM-5040: every committed raceline regenerates from its recorded command. The inputs (survey map, vehicle, TUM pins, parameters) must match exactly and the line must land within a measured tolerance of the committed one.
- SIM-5050: the raceline comparison report, both vehicles by every committed line by seeds 42, 1030, 7 and 2026. A gym-limits run that does not finish is recorded with the distance it reached; a default-vehicle run that does not finish fails the test. No lap time or ranking is asserted.
- SIM-5510: every pose source the build has, on `spielberg` (64 beams) and `spielberg_1080` (1080 beams, 270 degrees), by seeds 42, 1030, 7 and 2026, open loop. It records the error, availability, changed-pose rate, scan rate, CPU per scan and the produced map's distance to the true walls, and writes `log/pose-source-report/`. A method the build lacks is recorded as absent. Only structure is asserted (a record per row, no latch, a finite error, probes that heard something); no value is bounded and no method is ranked.

## Running the checks

`./scripts/check.sh` takes one mode, and `--nightly-region` also takes a region. Its tiers are the layers above: tier 1 is layer 1, tier 2
is layer 2, and so on up to tier 5 for layer 5. Tier 0 is build and lint.

- `--fast`: build, lint and the unit tests, scoped to what a branch changed. The pre-commit hook runs it.
- `--ci`: `--fast` plus the launch and contract tests and one seeded lap. This is the pull-request job.
- `--ready`: the definition of done for a change. It runs the scoped unit and contract tests, the seeded lap (SIM-3020) and the system and acceptance tests for the implementations the change touched.
- `--full`: tiers 0 to 4, which is layers 1 to 4 plus build and lint, over the whole tree.
- `--nightly`: `--full` plus layer 5, the cross-product. It takes minutes to hours. The schedule runs `--nightly-region` per job, and `--nightly` is the one-command local equivalent of the whole nightly.
- `--nightly-region <region>`: what one nightly job runs, one of `static`, `slam`, `path_planning`, `control` or `shared`. Together the regions cover what `--full` and `--nightly` cover. Set `NIGHTLY_TIER5=1` to add the region's layer 5 tests.

Run the definition of done in the container from the [Quick Start](quick-start.md):

```bash
docker exec gr-roboracer-dev bash -lc 'cd /ws && ./scripts/check.sh --ready'
```
