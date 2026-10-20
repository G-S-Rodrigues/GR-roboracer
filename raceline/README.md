# raceline

Offline raceline generation for GR-roboracer: a ROS-free uv project. Nothing in
`ros_ws/` imports it; its output is files (committed under `config/racelines/`
by a later part). It runs with `uv` in the `dev` container (ADR 0002 as
amended 2026-10-13, ADR 0003): `colcon build --base-paths ros_ws/src` never
scans it, and it needs no CUDA.

## How to run

```bash
docker exec gr-roboracer-dev bash -lc 'cd /ws && env -u PYTHONPATH uv run --project raceline --locked pytest raceline/tests'
```

`env -u PYTHONPATH` is required: `check.sh` and `setup.sh` source ROS, and
`PYTHONPATH` sits ahead of a venv's site-packages, so without it `rclpy` imports
silently (RACELINE-1020, repo-gotchas #24).

`raceline/bench/upstream_baseline.py` measures unmodified upstream map
extraction beside ours (below). The gate never runs it, pytest does not collect
it, and it needs the network.

Two projects live here:

- `raceline/` (Python 3.12, numpy 2): our code. `raceline.tum` is the seam.
- `raceline/tum_env/` (Python 3.8, numpy 1.18.1): ForzaETH's
  `global_racetrajectory_optimization` @ b1b38ec, unmodified, run by
  `tum_env/run_tum.py` as a subprocess; the two exchange CSV files.

## The install decision (spike, 2026-10-13)

The owner's order: current versions, then an isolated env on the original pins,
then a fork. Measured on Spielberg (`mincurv_iqp`, `config/tracks/spielberg.yaml`):

1. **Current versions failed, unmodified.** Python 3.12, numpy 2.5.4, scipy
   1.18.1, casadi 3.8.1, tph 0.79 (quadprog 0.1.13 override, since tph pins the
   sdist-only 0.1.7). `tph.spline_approximation` dies with `ValueError: The
   user-provided objective function must return a scalar value`. scipy 1.13.1
   fails too (`Input vector should be 1-D`). Cause: numpy 2 / scipy strictness
   inside tph. No rule-1 line exists, so no comparison between the two
   environments could be made.
2. **The original pins run with two exceptions** (coordinator ruling
   2026-10-13, open topic part1-tum-install-pins). TUM's code is unmodified in
   both:
   - `trajectory-planning-helpers` **0.78**, not 0.76: TUM's code at b1b38ec
     passes `spline_len` to `tph.iqp_handler`, which 0.76 and 0.77 reject
     (`TypeError ... unexpected keyword argument 'spline_len'`); its
     requirements.txt is out of step with its code.
   - `quadprog` **>=0.1.8** (locks to 0.1.12), not 0.1.7: 0.1.7 builds from
     sdist but fails to import (`undefined symbol _Z7qpgen2_...`).
   - Everything else is requirements.txt: numpy 1.18.1, scipy 1.3.3, casadi
     3.5.1, matplotlib 3.3.1, scikit-learn 0.23.1. Python 3.8 is the oldest
     interpreter uv offers and the newest those wheels exist for.
3. **Spielberg `mincurv_iqp`** in that env: lap time 30.15 s, 3392 points, path
   length 339.04 m, max |kappa| 0.653 rad/m, speed 4.29..15.0 m/s (v_max 15 from
   the ForzaETH ini), final IQP curvature error 0.0257 rad/m, wall time 447.8 s
   (old scipy is slow; hence the tests use a small oval).
4. **`mintime`** on Spielberg in that env, tried once, not gated (spec): IPOPT
   did not converge in 25 min (still in its restoration phase, no result),
   killed by the 1500 s timeout. Part 8 owns making it work.
5. **RACELINE-1010** (60 m asymmetric oval, `mincurv_iqp`) takes about 5 s.
   Worst overshoot of the corridor bound between QP nodes: 0.0009 m.

`trajectory_optimizer` creates its own `outputs/` directory, so no workaround
was needed. It writes into the venv's site-packages, so do not run two at once.

## Gym vehicle parameters

Read 2026-10-13 from `f1tenth_gym_jax/envs/utils.py:61-78` (class `Param`)
@1b4eb3f in the `dev` install.

- Part 3 (limits): mu 1.0489, a_max 9.51 m/s^2, v_max 20.0 m/s, s_max 0.4189 rad,
  sv_max 3.2 rad/s.
- Part 8 (minimum time): m 3.74 kg, I 0.04712, C_Sf 4.718, C_Sr 5.4562, h 0.074,
  lf 0.15875, lr 0.17145, width 0.31, length 0.58.
- lf + lr = 0.3302 against the default vehicle file's 0.33. Part 3 sets the
  gym-limits file to 0.3302; its header records the reference point (the gym's
  state is the centre of gravity). The loader is `raceline.vehicle`
  (`load_vehicle`), which reads the supervisor's and controller's keys and the
  GGV tables beside the file.

## Does the gym read `<map>_raceline.csv`?

Yes (`envs/track/track.py:194-208`): `;`-delimited, 7 columns
`[s, x, y, psi, kappa, vx, ax]`; without the file it falls back to the
centerline. **The first two raceline points also set the reset pose and racing
direction** (`f110_env.py:218-225`). Part 6 must know this.

## Map -> track (part 2)

`raceline.extract.extract_track(grid, resolution, origin, start_pose, direction)`
turns an occupancy grid (`load_map` reads a map_server YAML + PNG) into a
`raceline.track.Track`: centerline at 0.1 m, widths to each wall, curvature
computed from the path (never copied). It is a port of ForzaETH's map to
centerline code (`NOTICE.md`), plus a clean-up pre-pass the committed maps
needed: stray wall islands and corridor patches are filled, free space is
closed 5x5 before upstream's class-default 3x3 opening (its shipped config uses 9x9; `NOTICE.md`), skeleton spurs are pruned, and the
loop is the closed skeleton contour near `start_pose` that encloses the largest
area. `start_pose.yaw` is the gym/ROS yaw (0 = +x), not TUM's psi. Per the
spec, part 4's runner takes its input from `extract_track` on the map; the
committed `config/tracks/` file stays the gym's world.

```python
from pathlib import Path
from raceline.extract import Pose2D, extract_track, load_map

grid = load_map(Path("config/scenarios/maps/Spielberg/Spielberg.yaml"))
track = extract_track(
    grid, grid.resolution, grid.origin, Pose2D(0.0, 0.0, -2.879), "cw"
)
```

**What it is judged against.** The committed `spielberg.yaml` centerline is
not ground truth for the extracted centerline: a medial axis cannot follow a
centerline through a hairpin tighter than half the corridor width (the
committed track has a 0.165 m radius at the top-left corner), so a first
measurement against it failed (lateral max 0.89 m, length 1.3 % short, owner
ruling 2026-10-13). RACELINE-1110 judges the extracted left and right bounds
against the gym PNG's own free-cell wall edges, and RACELINE-1120 judges the
survey's against the same edges. Measured (2026-10-13, 0.05 m cells; the gym column
is before part 2.1 redrew the PNG, see the note under the table):

| | gym PNG | survey |
|---|---|---|
| bound to own edge, max | 0.034 (left), 0.000 (right) | 0.129 (left), 0.114 (right) |
| edge to bound, max | 0.049 (left), 0.000 (right) | 0.114 (right); left: 98.0 % of edge points within 0.15 m |
| centerline / committed length (medial axis alone) | 0.9863 | 0.9882 |
| centerline / committed length (shipped, re-centred) | 0.9986 | 1.0017 |
| tolerance | 0.05 m | 0.15 m, 97 % coverage, <= 10 % of the bound on the frame |

After part 2.1 (2026-10-14, gym PNG redrawn in the ROS convention): gym bound to
own edge 0.036 (left) / 0.000 (right), edge to bound 0.049 / 0.000, length
0.9981; survey bound to edge 0.090 / 0.092 against the new edges, left
coverage 97.85 % within 0.15 m, length 1.0017.

Until part 2.3 the survey's free space leaked past the wall at the top-left
hairpin out to the map frame (up to 0.96 m beyond the true wall); see "Observed
walls" below, which fixes it and keeps the leaking behaviour as
`walls="non_free"`. The medial axis cuts hairpins whatever the
cell size, so the shipped centerline is re-centred after the bounds are known:
at a corner it is the outer bound moved inward by the half-width (the inner
wall is clipped, the outer one is intact), on straights the mean of both
bounds' offsets (RACELINE-1130; assumes a constant half-width). Lateral
distance to the committed centerline, medial axis alone -> shipped:

| | gym lap p95 | gym corner p95 | gym max | survey lap p95 | survey corner p95 | survey max |
|---|---|---|---|---|---|---|
| m | 0.272 -> 0.081 | 0.649 -> 0.082 | 1.025 -> 0.622 | 0.273 -> 0.053 | 0.649 -> 0.413 | 0.817 -> 0.705 |

"Corner" is the committed track's |kappa| >= 0.1 (about 500 of 3400 points).
The remaining max is the 0.165 m hairpin near (-75.4, 52.3); the survey's corner
p95 is its top-left leak. The length ceiling went 1.5 % -> 0.3 %.

**Observed walls (part 2.3).** `extract_track` and `track_bounds` take
`walls="observed"` (the default) or `"non_free"` (part 2's behaviour, byte for
byte, kept as the baseline: ADR 0005).

- *Why the survey leaked.* The survey did record the top-left outer wall (2
  occupied cells), but the map frame cuts a strip of 4-13 unknown cells off
  from the unknown outside. `_fill_patches` saw a small patch and filled it as
  corridor, wall included, so the image edge became the bound. `observed`
  pads the grid with 10 unknown cells before binarizing and crops after. Survey
  numbers, `non_free` -> `observed`: largest total width 3.158 -> 2.502 m (gym
  2.525); left edge to bound 0.987 -> 0.091; coverage 97.85 % -> 100 %; left
  bound on the frame 6.03 % -> 3.28 %; corner lateral p95 0.413 -> 0.063;
  bound points with no occupied cell within 0.15 m: 74 -> 0. The gym is unchanged.
- *Open walls.* A gap the lidar saw through leaves bound stretches with no
  occupied cell near them. `observed` finds them (`OBSERVED_TOL` 0.15 m),
  replaces each with a cubic Hermite curve between the observed walls at its
  ends (grown 0.3 m onto solid wall, tangents from the next 0.5 m) and extracts
  again. Bridged sides come out narrower than truth (down to 0.77 of it for a
  gap on both walls), never wider by more than 0.033. A Hermite curve lies inside
  the arc it replaces, which on an inner wall is a wider corridor: R_in 4 with a
  90 deg gap reads +0.074 and 120 deg +0.183. So a bridge is capped at an
  end-tangent turn of 70 deg and a chord of 6.0 m (`MAX_BRIDGE_TURN_DEG`,
  `MAX_BRIDGE_M`); past either the extractor raises `ValueError` naming
  "unobserved". The chord is a primary limit: past ~7 m the end-tangent error
  (a 0.5 m secant of a pixelised contour) gave +0.042..+0.070 inside the turn
  cap. The measured turn includes the 0.3 m extensions (a nominal 20 deg gap
  reads 36 deg at R 4). A gap on a non-circular wall (straight to arc) is not
  swept; real maps are its check.
- *No ground truth.* `observed_mask(grid, pts)` says which points of a bound
  have an occupied cell near them. The False ones are bridged or on the frame;
  a large share, or one long run, means the map does not support a raceline
  (RACELINE-1170 pins the survey at 0). Repeating the extraction on two SLAM
  runs of one track, and counting contacts in simulation, wait for the
  AutoDRIVE map workflow.
- *The frame.* If the true wall lies beyond the map frame, the frame stays the
  bound under both selectors and the track errs narrow (left 0.649 against 1.0
  on a ring cropped at x = 5.6), never wide; `observed_mask` flags those
  points. Padding can also narrow a bound next to the frame, never widen one (a
  frame-touching 0.3 m unknown patch: right width min 0.937 -> 0.621).

**Against unmodified upstream (part 2.2).** The same metrics as RACELINE-1110
to 1130, run on ForzaETH's `extract_centerline` / `smooth_centerline` /
`extract_track_bounds` at `202450f` by `raceline/bench/upstream_baseline.py`
(2026-10-13; numpy 2.5, OpenCV 4.14, scikit-image 0.26 from `uv.lock`). Lateral
and length columns are the centerline vs the committed one; bound columns are
the bounds vs the gym PNG's own free-cell edges (survey: coverage within 0.15 m
instead of edge to bound). Re-run it, and refresh this table, whenever the
extractor changes. The "part 2" rows are `walls="non_free"` (the bench pins it).

| map | extractor | lap p95 | corner p95 | lateral max | length ratio | bound->edge max (L / R) | edge->bound max (gym) or coverage within 0.15 m (survey) (L / R) | bound on frame (L / R) | side share left (L / R) | s |
|---|---|---|---|---|---|---|---|---|---|---|
| gym | part 2 | 0.081 | 0.082 | 0.622 | 0.9986 | 0.034 / 0.000 | 0.049 / 0.000 | 0.000 / 0.000 | 1.000 / 0.000 | 0.63 |
| gym | upstream @202450f K=9 | 0.303 | 0.676 | 0.881 | 0.9873 | 0.100 / 0.494 | 0.100 / 0.126 | 0.000 / 0.000 | 1.000 / 0.000 | 0.32 |
| gym | upstream @202450f K=3 | 0.303 | 0.676 | 0.881 | 0.9873 | failed [1] | failed [1] | failed [1] | failed [1] | 0.33 |
| survey | part 2 | 0.053 | 0.413 | 0.705 | 1.0017 | 0.129 / 0.113 | 0.980 / 1.000 | 0.060 / 0.000 | 1.000 / 0.000 | 0.64 |
| survey | upstream @202450f K=9 | failed [2] | failed [2] | failed [2] | failed [2] | failed [2] | failed [2] | failed [2] | failed [2] | 0.20 |
| survey | upstream @202450f K=3 | failed [2] | failed [2] | failed [2] | failed [2] | failed [2] | failed [2] | failed [2] | failed [2] | 0.30 |

[1] upstream raised OSError: More than two track bounds detected! Check input

[2] upstream raised ValueError: Only invalid closed contour line lengths

Reading it:

- `cent_length` is the committed Spielberg centerline length, which **favours
  upstream**: it gets the true lap length, part 2 gets only `start_pose`.
- Upstream is run twice: `K=9`, the opening kernel its shipped stack config sets
  (`global_planner_params.yaml:8`), and `K=3`, the class default that part 2
  ported. Part 2 opens 3x3.
- Upstream's bounds are in its cell-corner convention (`px * res + origin`), so
  up to half a cell (0.025 m per axis) of its bound error is that convention.
- Upstream raised where a cell reads `failed`. On the gym map with `K=3` it
  still produced a centerline (those columns are filled) but its bounds step
  raised `OSError` ("More than two track bounds detected"). On the survey it produced nothing:
  upstream does not produce a track from the survey, `K=9` or `K=3`.
- Not compared: width medians (upstream's `dist_to_bounds` needs TUM's LGPL
  `interp_track`), the start-pose and heading checks (upstream orients its
  centerline later), curvature (upstream computes none in these functions).
- Part 2's rows reproduce the tables above to 3 decimals, except the survey
  right bound's 0.1135 m (printed 0.113; the table above rounds it 0.114).
- Stand-ins: `ament_index_python`, `matplotlib`, `trajectory_planning_helpers`,
  `global_racetrajectory_optimization`, `geometry_msgs`, `f110_msgs`,
  `visualization_msgs`; each raises if upstream calls it. The only argument
  preparation is the skeleton cast from bool to uint8 0/255. Upstream's source
  is fetched at the pinned SHA and its sha256 checked; nothing is edited.

**Finer cells (scratch study, owner ask).** The gym PNG regenerated at finer
resolution with `sim/track_importer.py` (not committed):

Measured before part 2.1 (2026-10-14) fixed the importer's convention; the shift column is historical.

| cell | bound to own edge, max (left) | time | length ratio | shift of the PNG vs the committed walls |
|---|---|---|---|---|
| 0.05 m | 0.050 | 0.36 s | 0.9863 | (-0.02, +0.05) m |
| 0.025 m | 0.021 | 1.8 s | 0.9859 | (-0.01, +0.03) m |
| 0.0125 m | 0.010 | 11 s | 0.9858 | (+0.00, +0.015) m |

Errors scale with the cell; the length shortfall does not. The PNG's offset from
the committed walls was `sim/track_importer.py`'s pixel convention, fixed by
part 2.1 (SIM-1030): the importer now fills cells by their ROS-convention
centres, so the gym collides with the committed walls to within a cell. Part 4's
verifier still adds the survey's 0.15 m to its margin.

Cost: `extract_track` takes ~0.4 s per Spielberg map at 0.05 m. The three new
dependencies add ~116 MB of wheels to a cold `uv sync` (331 MB installed).

## Licences

TUM `global_racetrajectory_optimization` and `trajectory_planning_helpers` are
LGPL-3.0, installed and not vendored. The fixture inputs are ForzaETH MIT; see
`NOTICE.md`.
