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

## Licences

TUM `global_racetrajectory_optimization` and `trajectory_planning_helpers` are
LGPL-3.0, installed and not vendored. The fixture inputs are ForzaETH MIT; see
`NOTICE.md`.
