# Verification

Every claim on [Results](results.md) and in the README's capability matrix is backed by a test that
runs on every pull request or every night. This page says what each of five layers proves, then how
to run the checks.

## 1. Unit tests

ROS-free logic is tested without a ROS graph, in milliseconds. These tests cover the track and
Frenet conversions, the pure pursuit law, the metrics accumulator, the simulator backend's scan
angles and the safety supervisor.

- COMMON-1010: Track Frenet round trip.
- PP-1010: Pure Pursuit geometry.
- METRICS-1010: the metrics accumulator.
- SIMJAX-1090: the scan angles of a real `GymBackend`, its only direct test.
- SAFE-1020: stale input stops the car.

## 2. Launch and contract tests

These tests start the simulator adapter and check what it publishes: types, rate and frames. They
show the ROS contract holds without a full lap.

- ADAPT-2010: scan, odometry and IMU use standard ROS message types.
- ADAPT-2030: the backend publishes at its configured 100 Hz.

## 3. System tests

These tests launch the real stack and run a seeded lap, then compare it to a committed record or
bound it against ground truth.

- SIM-3020: a seeded lap through the real launch completes.
- SIM-3040: live metrics match `tests/golden/baseline.json` within tolerance.
- SIM-3090: the Spielberg lap matches `tests/golden/spielberg_baseline.json`.
- SIM-3070: `slam_toolbox` localizes while the car drives on ground truth.
- SIM-3080: `amcl` localizes on the saved map.

## 4. Acceptance tests (Robot Framework)

Black-box cases on the composed stack, one per behaviour a user can name:

- `Vehicle Completes Baseline Lap Safely` (ACC-4010)
- `Vehicle Stops When Track Limits Violated` (ACC-4020)
- `Localization Estimate Tracks Ground Truth: slam_toolbox` and `Localization Estimate Tracks Ground Truth: amcl` (ACC-4030)
- `Vehicle Completes A Lap Driving On Its Own Estimate` (ACC-4040)

## 5. Nightly coverage

The nightly runs on a schedule, never in a pull request. It has four legs, one per seed 42, 1030, 7 and 2026.
Each leg runs tiers 0-4, whose tests keep their own pinned seeds, then one seeded lap on the leg's seed.
The seed-42 leg adds tier 5, the cross-product of tracks, implementations and seeds. Its track
dimension is Spielberg alone today.

- SIM-5010: the committed occupancy grid regenerates from the survey.
- SIM-5020: `slam_toolbox` and `amcl` over seeds 42 and 1030 on Spielberg.
- SIM-5030: the `amcl` run-to-run spread, over seeds 42, 1030, 1, 7, 99, 123 and 3, with 5 runs per seed.

## Running the checks

`./scripts/check.sh` takes one mode:

- `--fast`: build, lint and the unit tests, scoped to what a branch changed. The pre-commit hook runs it.
- `--ci`: `--fast` plus the launch and contract tests and one seeded lap. This is the pull-request job.
- `--ready`: the definition of done for a change. It runs the scoped unit and contract tests, the seeded lap (SIM-3020) and the system and acceptance tests for the implementations the change touched.
- `--nightly`: tiers 0-4 plus tier 5, the cross-product. It takes minutes to hours and runs on a schedule.

Run the definition of done in the container from the [Quick Start](quick-start.md):

```bash
docker exec gr-roboracer-dev bash -lc 'cd /ws && ./scripts/check.sh --ready'
```
