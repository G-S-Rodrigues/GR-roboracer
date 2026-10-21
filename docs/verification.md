# Verification

The measurements on [Results](results.md) and the behaviour of each Available implementation in the
README's capability matrix are backed by tests that run on every pull request or every night. Three
things are not. The Mapping demonstration on Results is a recording, not a test. The failure of the
TUM minimum-time optimization to converge on Spielberg is a single trial that no test gates (see
`raceline/README.md`). GPU runs are not tested in CI. Rows marked Planned or Exploratory are not
claims of working code. This page says what each of five layers proves, then how to run the checks.

## 1. Unit tests

ROS-free logic is tested without a ROS graph, so these tests are the quickest layer: the layer includes the JAX simulator backend on Spielberg and the raceline suite. They cover the track and
Frenet conversions, the pure pursuit law, the metrics accumulator, the simulator backend's scan
angles and the safety supervisor.

- COMMON-1010: Track Frenet round trip.
- PP-1010: Pure Pursuit geometry.
- METRICS-1010: the metrics accumulator.
- SIMJAX-1090: the scan angles of a real `GymBackend`.
- SAFE-1020: stale input stops the car.

## 2. Launch and contract tests

These tests start the simulator adapter and check what it publishes: types, rate and frames. They
show the ROS contract holds without a full lap.

- ADAPT-2010: scan, odometry and IMU use standard ROS message types.
- ADAPT-2030: the backend publishes at its configured 100 Hz.
- PP-2010: the controller node follows the trajectory's speed only under `speed_source: trajectory`, and reads the odometry twist into its lookahead only under `lookahead_gain > 0`.

## 3. System tests

These tests launch the real stack and run a seeded lap, then compare it to a committed record or
bound it against ground truth.

- SIM-3020: a seeded lap through the real launch completes.
- SIM-3040: live metrics match `tests/golden/baseline.json` within tolerance.
- SIM-3090: the Spielberg lap matches `tests/golden/spielberg_baseline.json`.
- SIM-3110: the Spielberg lap on the committed minimum-curvature line (`planner:=optimized_raceline speed_source:=trajectory`, default vehicle, seed 42) completes with 0 collisions and 0 saturation events, at its own measured lap time of 113.08 s (tolerance 0.05 s).
- SIM-3070: with the car driving on ground truth, `slam_toolbox` publishes an estimate whose RMSE against ground truth is within 4.0 m. The test is open loop and does not claim the estimate is good.
- SIM-3080: with the car driving on ground truth (open loop), `amcl` publishes an estimate on the saved Spielberg map over a full lap with 0 collisions; the estimate is available in at least 0.99 of samples and its position RMSE is within 0.65 m (the seed-42 bound).

## 4. Acceptance tests (Robot Framework)

Black-box cases on the composed stack, one per behaviour a user can name:

- `Vehicle Completes Baseline Lap Safely` (ACC-4010)
- `Vehicle Stops When Track Limits Violated` (ACC-4020)
- `Localization Estimate Tracks Ground Truth: slam_toolbox` and `Localization Estimate Tracks Ground Truth: amcl` (ACC-4030)
- `Vehicle Completes A Lap Driving On Its Own Estimate` (ACC-4040)

## 5. Nightly coverage

The nightly runs on a schedule, never in a pull request. It has four legs, one per seed 42, 1030, 7 and 2026.
Each leg runs layers 1 to 4, whose tests keep their own pinned seeds, then one seeded lap on the leg's seed.
The seed-42 leg adds this layer, the cross-product of tracks, implementations and seeds. Its track
dimension is Spielberg alone today.

- SIM-5010: the committed occupancy grid regenerates from the survey.
- SIM-5020: `slam_toolbox` and `amcl` over seeds 42 and 1030 on Spielberg.
- SIM-5030: the `amcl` run-to-run spread, over seeds 42, 1030, 1, 7, 99, 123 and 3, with 5 runs per seed.

## Running the checks

`./scripts/check.sh` takes exactly one mode. Its tiers are the layers above: tier 1 is layer 1, tier 2
is layer 2, and so on up to tier 5 for layer 5. Tier 0 is build and lint.

- `--fast`: build, lint and the unit tests, scoped to what a branch changed. The pre-commit hook runs it.
- `--ci`: `--fast` plus the launch and contract tests and one seeded lap. This is the pull-request job.
- `--ready`: the definition of done for a change. It runs the scoped unit and contract tests, the seeded lap (SIM-3020) and the system and acceptance tests for the implementations the change touched.
- `--full`: tiers 0 to 4, which is layers 1 to 4 plus build and lint. The three nightly legs that are not seed 42 run it.
- `--nightly`: `--full` plus layer 5, the cross-product. It takes minutes to hours and runs on a schedule.

Run the definition of done in the container from the [Quick Start](quick-start.md):

```bash
docker exec gr-roboracer-dev bash -lc 'cd /ws && ./scripts/check.sh --ready'
```
