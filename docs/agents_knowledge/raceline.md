# Raceline

Path planning offline: map to track extraction, the TUM minimum-curvature and minimum-time
optimizers on the owner's fork, vehicle limits, the geometric verifier, regeneration and the
planning comparison. Read it before a grill, spec or plan that touches `raceline/`, a vehicle file's
limits or GGV, a committed raceline, or the track importer. The controller that follows the line is in
[control.md](control.md). SLAM map quality is in [localization-slam.md](localization-slam.md). The
AutoDRIVE car and its limit identification are in [simulation.md](simulation.md). League tracks and
dates are in [competition.md](competition.md).

The user page is [docs/path-planning.md](../path-planning.md). The operator's reference, with every
flag, constant and measured tolerance, is [raceline/README.md](../../raceline/README.md). This file
holds what neither says: why, what failed, and what is still open.

## Current state

Issue [#12](https://github.com/G-S-Rodrigues/GR-roboracer/issues/12) (closed 2026-11-01) built the
pipeline in parts 0-8. The spec's diagram still describes it:

```
SLAM map -> (1) extract track -> (2) min-curvature path -> (3) speed profile -> Pure Pursuit follows v
                              \-> (4) min-time (path + speed together) ------/
```

- **Project layout.** `raceline/` is a ROS-free uv project (Python 3.12, numpy 2) at the repo root,
  added in [#17](https://github.com/G-S-Rodrigues/GR-roboracer/pull/17). TUM runs in a separate
  Python 3.8 env, `raceline/tum_env/`, as a subprocess exchanging CSV files (`raceline/raceline/tum.py`,
  `tum_env/run_tum.py`, `tum_env/run_tum_mintime.py`). Never import TUM into the 3.12 env (gotcha 24).
- **TUM pin.** `tum_env/pyproject.toml` pins the owner's fork
  [GR-roboracer-global_racetrajectory_optimization](https://github.com/G-S-Rodrigues/GR-roboracer-global_racetrajectory_optimization)
  at `8bedf4ca` (fork PRs #1 and #2, merged 2026-10-25). The fork starts at ForzaETH's `pip_package`
  @`b1b38ec`, itself TUM upstream @`a9995e2`. Its min-curvature path is byte-identical to upstream
  (TUMFORK-4010). Everything it adds is min-time only: `MintimeOptions`, `ipopt_opts`, `StopAndSave`
  (wall-time self-stop, resumable state, `solver_status.json`), `warm_start_dir`, `seed_line`,
  `export_dir`, exact arc length and a periodic exported speed. The patch table is in the fork's
  `Readme.md`, section "GR-roboracer fork". Upstream TUM `ace99a4` ("fix velocity profile at s=0 when
  braking zone") is deliberately not taken: it monkeypatches tph's closed velocity solver globally.
- **Extractor (1).** `raceline/raceline/extract.py` is a ROS-free rewrite of ForzaETH race_stack
  @`202450f` `global_planner_utils.py` (MIT), landed in
  [#20](https://github.com/G-S-Rodrigues/GR-roboracer/pull/20). `raceline/NOTICE.md` lists every
  change from upstream with its reason, and the hand-run `gh api compare` command that shows upstream
  drift. `walls="observed"` ([#28](https://github.com/G-S-Rodrigues/GR-roboracer/pull/28)) treats
  only lidar-seen walls as walls and bridges an unobserved gap only up to `MAX_BRIDGE_M` 4.0 m chord
  and `MAX_BRIDGE_TURN_DEG` 70 deg; past that it exits 3 ("unobserved"). `walls="non_free"` is kept
  beside it (ADR 0005). `raceline/bench/upstream_baseline.py` measures unmodified upstream extraction
  (outside the gate, needs network).
- **Track from a map.** `python -m raceline.track_from_map` writes a committed `config/tracks/<name>.yaml`
  from an official map. `config/tracks/porto.yaml` came from it (#44 part 10). The Spielberg track is
  imported from f1tenth_racetracks by `tools/import_f1tenth_racetrack.py`; `sim/track_importer.py`
  draws its gym PNG and cuts offset-wall folds (`_remove_folds`,
  [#30](https://github.com/G-S-Rodrigues/GR-roboracer/pull/30); gotchas 25 and 28).
- **Optimizers (2+3, 4).** `raceline/raceline/optimize.py`: `tum_mincurv_iqp` (two passes, ForzaETH's
  recipe: the pass-1 line becomes pass 2's centerline) and `tum_mintime` (one solve seeded from a
  min-curvature reference at the same margin, written to disk and read back first, gotcha 43). Default
  margins 0.30 m (min-curv) and 0.40 m (min-time) per side. The min-time ini constants each carry
  their measurement in a comment. `--reprofile-speed` (opt-in, `reprofile_speed`) re-profiles a
  min-time line's speed on its exported path to 0.995 x the GGV; the Porto line uses it, the Spielberg
  lines do not.
- **Vehicles.** `raceline/raceline/vehicle.py` reads limits from the same file the supervisor reads
  (no second copy). Vehicles: `f1tenth_default` (3.0 m/s, gate laps), `f1tenth_gym_limits` (the gym's
  own `Param` caps, 20 m/s, owner 2026-10-14), `autodrive_roboracer` and `autodrive_roboracer_fast`.
  Each has `<stem>/ggv.csv`, `ax_max_machines.csv`, `tum_mintime.yaml` and tyre sets `forzaeth_f110`
  and/or `gym_fit` (default `gym_fit`, owner Q3 2026-10-22). Keys are documented values-free in
  `config/vehicles/tum_mintime/reference.yaml`. RACELINE-1220 loads every committed vehicle.
- **Verifier.** `raceline/raceline/verify.py`, landed with the runner in
  [#31](https://github.com/G-S-Rodrigues/GR-roboracer/pull/31). It is ADR 0005's ground truth for a
  raceline: closure, direction, psi, kappa against `kappa_max`, a_y and a_x against GGV and machines,
  and signed clearance to two wall sets (input bounds, and the ground-truth track's clean polygon with
  the sharp-tip rule). Exit 2 means rejected, nothing written. TUM's own `check_traj` only prints.
- **CLI and records.** `python -m raceline` writes
  `third_party/GR-roboracer-data/tracks/<Map>/racelines/<optimizer>/<limit set>/{raceline.csv,raceline.yaml,limits.md}`
  (+ `reference.csv` for min-time), layout per ADR 0009 and its D2 amendment
  ([#92](https://github.com/G-S-Rodrigues/GR-roboracer/pull/92),
  [#100](https://github.com/G-S-Rodrigues/GR-roboracer/pull/100)). The yaml pins sha256 of every input
  and the git blob of every `CODE_FILES` member (`raceline/raceline/__main__.py`); RACELINE-1395 goes red
  on any byte change. `python -m raceline.diagnose` plots a committed line. `--track` still writes
  `tracks/<lower-case stem>/`, which the launch lookup (map name, case kept) never reads; generate a
  line for the launch with `--map` (raceline/README says so; found in the D1 review, 2026-11-02).
  `--map` needs `--start-pose` and `--direction`: a map carries neither s = 0 nor a driving direction.
- **Verifier limits by design.** Clearance at bridged (unobserved) wall stretches is reported
  (`min_clearance["<set>_unobserved"]`, `unobserved_share`), never failed. The sharp-tip relief applies
  only where a wall cell lies within the cut loop's reach (Spielberg: one tip, r_t 0.326 m); a line
  outside the clean polygon fails whatever the cells say. A track whose centerline crosses itself raises
  "self-intersect", so no ground truth exists yet for a track that passes over itself. The verifier trusts
  whatever `--ground-truth-map` it is given: RACELINE-1395 pins it for committed lines, a hand run with
  the wrong map is unguarded.
- **Committed lines** (data pin `d25c924`): Spielberg `tum_mincurv_iqp` and `tum_mintime`, each for
  `gym-nominal-std-const-v1` (default vehicle) and `gym-model-std-const-v1` (gym-limits vehicle), and
  Porto `tum_mintime/autodrive-nominal-fast-const-gymfit-v1`
  ([#88](https://github.com/G-S-Rodrigues/GR-roboracer/pull/88)). All are generated from a map
  (Spielberg: the 0.05 m slam_toolbox survey), never from the committed centerline.
- **Racing a line.** `planner:=optimized_raceline raceline_optimizer:=<opt> speed_source:=trajectory`
  ([#35](https://github.com/G-S-Rodrigues/GR-roboracer/pull/35)); `config/reference_stack.yaml` lists
  `optimized_raceline` as an alternative to `track_centerline`, verified by SIM-3110 (min-curv lap),
  SIM-3120 (min-time lap) and ACC-4040 (min-curv on amcl). Lookup (`racing_bringup/track_data.py`)
  fails closed, never falls back to the centerline. `speed_source:=trajectory` on a map with no
  raceline raises in the launch (the centerline carries v = 0). On AutoDRIVE, SIM-3750 drives the
  Porto fast line (local only).
- **Regeneration and comparison.** Nightly SIM-5040 reruns every min-curv record's `command:`; SIM-5060
  reruns min-time records only on a night the fork pin or `MINTIME_CODE_FILES` changed. Both use 0.01 m
  / 0.01 m/s tolerances. The SIM-5050 nightly comparison is retired; the planning role of
  `scripts/benchmark.py` ([#90](https://github.com/G-S-Rodrigues/GR-roboracer/pull/90)) drives every
  committed Spielberg line on both gym vehicles, and any data-submodule bump triggers it.

## What we measured

All on Spielberg in the gym unless stated. The extraction accuracy, the committed lines' clearances,
lap estimates and iteration counts are in [raceline/README.md](../../raceline/README.md) and
[docs/path-planning.md](../path-planning.md); they are not repeated here.

**Comparison report, nightly SIM-5050, 4 seeds, 2026-11-05** (run 42010230644, last before retirement):

| vehicle | line | laps done | lap s (min/med/max) | DNF distance m | min clearance m (metrics / verifier) | path m | max kappa | TUM est s |
|---|---|---|---|---|---|---|---|---|
| gym-limits | centerline | 0/4 | DNF | 6.6-32.1 | - | 343.3 | - | - |
| gym-limits | mincurv | 0/4 | DNF | 9.1-73.3 | - / 0.323 | 338.7 | 0.500 | 29.60 |
| gym-limits | mintime | 0/4 | DNF | 51.9-104.9 | - / 0.455 | 338.7 | 0.626 | 32.37 |
| default | centerline | 4/4 | 120.42 / 120.44 / 120.54 | - | 0.779 / - | 343.3 | - | - |
| default | mincurv | 4/4 | 113.08 / 113.11 / 113.16 | - | 0.327 / 0.320 | 338.7 | 0.502 | 112.89 |
| default | mintime | 4/4 | 112.07 / 112.12 / 112.19 | - | 0.453 / 0.422 | 335.7 | 0.664 | 111.88 |

Zero collisions on every completed lap. Min-curv solve (extraction + two TUM passes) took 996-1003 s.
The same night's regeneration of both min-curv records matched to 2e-9 m and 2e-8 m/s. The gym-limits
DNFs are the controller at speed, not the line: every line, the centerline included, leaves the track
(owner 2026-11-05: fast-car video deferred until a high-speed controller exists; see
[control.md](control.md)).

**Upstream reference line** (f1tenth_racetracks@866fe65 `Spielberg_raceline.csv`, its own unknown
vehicle model, never driven): 1692 points, 338.1 m, max kappa 0.448, v 4.5-8.0 m/s, profile lap
45.05 s. At 3 m/s no Spielberg corner is grip-limited (0.448 1/m with mu about 1.05 allows about
4.8 m/s), which is why the gym-limits vehicle exists.

**Min-time exploration, part 8b, 2026-10-23/24** (exploratory, never committed). Default vehicle,
margin 0.40, regular h 0.5: converged in 1108 s, lap 111.8 s. With the 3 m/s cap binding at all 680
nodes it is in effect a shortest-path line; it cut 3 cm between nodes (input clearance 0.521 against
0.525) and failed closure by 2.27e-3 m. The gym-limits margin chain, each step warm-started from the
last (reference fixed at the 0.40 min-curv line, tyre set `gym_fit`):

| margin m | result | TUM lap est s | solve |
|---|---|---|---|
| 0.40 | Optimal, 722 it (warm from the default car's line) | 32.33 | 151 s |
| 0.30 | Optimal | 31.98 | 147 s |
| 0.20 | Optimal; fails input clearance 0.329 < 0.330 | 31.64 | 385 s |
| 0.10 | hit max_iter 2000 at 430 s (objective 31.595, still falling); converged on resume | about 31.6 | - |
| 0.20 -> 0.15 | restoration failed at iter 416 | - | - |

Top speed plateaued at 19.54-19.58 m/s at every margin (drag/power, not the 20 m/s bound). Closure
failed at s about 108 on every run (2.6-2.9e-3 m), which the fork's exact arc length later fixed.
Lap time gains from 0.40 to 0.20 m of margin: 0.7 s of 32.

Which limit binds (share of the 680 nodes within 1 % of it): on the default car the 3 m/s cap at every
node and the track bound at 25 %, tyres and power never (22 W peak of 150). On the gym-limits car at
margins 0.10-0.30, nothing at 65-68 % of nodes, the GGV at 15-17 %, power at 17-18 % (255 W peak of 260)
and the track bound at 8-10 %. Below 0.30 m the survey map's error decides, not the margin: at 0.20 the
CoG passes 0.058 m from the ground-truth wall, and the resumed 0.10 line (which also fails a_y) puts
its CoG 0.036 m beyond the true wall while 0.074 m inside the survey's. The 0.40 and 0.30 lines also
brake twice at s 82-86 (19.54 -> 17.94, hold 17.88, brake again) at 6.8 m/s^2 against a lap maximum of
7.7, so not grip-limited; the 0.20 and 0.10 lines brake once. Likely a local optimum inherited through
the warm-start chain (unproven).

**Min-time closure, root cause, 2026-10-24.** TUM's `create_raceline` calls
`calc_splines(use_dist_scaling=False)` and `interp_splines` places the 0.1 m output points linearly in
each spline's parameter, so s drifts from arc length wherever neighbouring node spacings differ: error
about 0.028 x the spacing jump (correlation 0.95-0.97), zero at every knot, at most 4.8 mm per point,
1.4e-4 m per lap. The geometry was fine (a distance-scaled spline through the same nodes lies within
1.3 mm). Exact arc-length placement on the same splines closes to 1.6-1.9e-5 m, the route the fork's
exact arc length took. Iterative min-curv passes the check because each pass re-references, keeping
offsets small and spacing even (6.8e-5 m, jump 0.3 % at h 0.2); single-pass min-curv on the centerline
fails it too (5.7e-3 m at h 0.2, 1.9e-2 m at h 0.5).

**Min-curv pass 1 against pass 2** (part 4 spike, 2026-10-20, gym-limits, Spielberg survey): pass 1 alone
had max |kappa| 0.75, minimum clearance to the input walls 0.227 m at the hairpin and |ds - |dxy|| 3.6e-4;
pass 2 had 0.48, 0.425 m and 1.4e-5. The second pass is what makes the line pass the verifier.

**How TUM's min-time behaves** (read from `opt_mintime.py` in the pinned env, 2026-10-23, plus the 8b
logs). CasADi with Ipopt 3.12.3 and MUMPS. No HSL solver is installed (it needs an STFC licence and an
image change) and this Ipopt has no `max_wall_time`, hence the fork's callback stop. Gauss-Legendre
collocation, degree 3, 24 variables per interval. The state bounds (v >= 1 m/s, beta, xi and omega_z each
+-pi/2) and the track-width bound hold at the nodes only; the collocation states between them are
unbounded, which is where the between-node overshoot comes from (`MINTIME_LIMIT_SCALE` 0.98 and
`MINTIME_WIDTH_EXTRA` absorb it). `width_opt` is one scalar for the lap and TUM's `safety_width`
argument is ignored for min-time; a per-node margin could only go in through the widths in `track.csv`.
Curvature is interpolated by node index, and the preview/review windows become node counts through the
mean element length, so on a two-level grid they shrink to 0.8-1.4 m in corners and grow to 3-11 m on
straights. The scale constants are full-size (7500 N drive, 20000 N brake), so our 7-36 N forces sit
near 1e-3 scaled. The friction circle divides by the wheel load and nothing keeps it positive. Seconds
per iteration: default car, regular h 0.5, 1.06 (1044 iterations, 1110 s); gym-limits car, same grid,
about 14; two-level grids 60-75. Every slow run reallocated MUMPS memory three times.

**Min-time GGV overshoot, Porto cw, 2026-11-08** (fast AutoDRIVE car 4.5 m/s / 4.0 m/s^2, margin
0.40). IPOPT reports `Solve_Succeeded` (143 iterations, lap 7.30 s) but the verifier rejects the
exported line: v^2|kappa| reaches 1.052 x the GGV a_y at a corner exit (v 3.975, kappa -0.653). The
base car at 3.0/2.0 is rejected on braking (1.044 x). Peak v^2|kappa| stays near 10.3-10.4 m/s^2
whatever the a_y table says, because the table reaches the solver only as `mue` from GGV row 0
(`optimize.py`, gotcha 46) and per-tyre Kamm circles. The solver's own accelerations peak at 6.43
m/s^2 total, so the exported path is more curved than the model's states: kappa -0.663 against a
model-implied -0.357 at s 26.5 m. Throwaway variants:

| variant | verifier | worst ratio | lap est s |
|---|---|---|---|
| margin 0.30 | rejected | a_y 1.0054 | 7.13 |
| GGV a_y 9.3 | rejected | a_y 1.116 | 7.32 |
| GGV a_y 10.5 | passes | 0.977 | 7.30 |
| v cap 4.0 | passes | 0.978 | 7.93 |
| `MINTIME_STEP_REG` 0.3 / 0.2 / 0.1 | rejected | 1.043 / 1.025 / 1.020 (+ a_x 1.067 at 0.1, 52 s) | - |
| re-profile on the exported path (landed as `--reprofile-speed`) | passes | 0.995 | 7.303 -> 7.306 |

In every Porto corner the solution sits at the fork's yaw-rate box bound |omega_z| = pi/2 rad/s, with
beta up to +-0.3. Curvature estimation was ruled out (geometric and column kappa agree within 0.3 %).

**AutoDRIVE car on Spielberg min-time, 2026-10-31** (derived AutoDRIVE dynamics, stand-in `gym_fit`
tyres, uncommitted): stopped at the wall-time cap after 36 iterations (1500 s) and 42 iterations
(3000 s, quiet host), about 50 s per iteration. The same vehicle file on Porto converged in 664
iterations / 8.7 s; the F1TENTH Spielberg record takes 1518. Not diagnosed.
Sensitivity of that Porto line (max xy / max v change against the derived baseline; PR #79's part-8
addendum, step A3): the other mass candidate 3.906 kg 5.5 mm / 0.0103 m/s, yaw inertia x0.5 25 mm /
0.043, x2 33 mm / 0.053; drive and brake rate 0.01 s and the sim's own steering slew ~0; the F1TENTH
tyre stand-in failed the verifier (exit 2). Yaw inertia, an assumption with no source number, is the
parameter that binds; all moves exceed the 0.01 m regeneration floor.

**Install spike, 2026-10-13**: see raceline/README "The install decision". **Bridge cap sweep,
2026-10-20** (35,091 synthetic runs, R_in 4.0-6.4 m): first bridged corridor wider than truth + 0.04 m
at chord 4.38 m, worst +0.066 m per side at 6.0 m; at the chosen 4.0 m none exceeds +0.034 m.

**SLAM map resolution vs extraction, 2026-10-23/24**: on a 0.01 m slam_toolbox map (scan every 0.2 m)
the walls come out dotted and the extractor fails. The extractor's morphology kernels are sized in
cells (5x5 close, 3x3 open), a likely second cause (untested). The 0.05 m survey rounds the hairpin's
inner tip 0.171 m off, the spot where min-time lines come closest to the wall. Full table in
[localization-slam.md](localization-slam.md).

**Porto curvature column, 2026-10-31**: `config/tracks/porto.yaml`'s curvature column integrates to
-381.06 deg per lap over the raw polyline but to -360.04 deg over the Gaussian-smoothed path
(sigma 0.5 m) it was computed on (raw 32.710 m, smoothed 32.056 m). The column is right for the
smoothed curve; nothing in the stack integrates it.

## Dead ends

- **TUM on current libraries.** Python 3.12, numpy 2.5, scipy 1.18, tph 0.79 fails in
  `tph.spline_approximation` (objective must return a scalar); scipy 1.13.1 also fails. TUM's own pins
  fail too: tph 0.76/0.77 lack `iqp_handler(spline_len=...)`, quadprog 0.1.7 builds but does not import.
  Only tph 0.78 + quadprog 0.1.12 on the original pins works (raceline/README).
- **Cold (unseeded) min-time.** TUM's `opt_mintime.py` guesses v = 20 m/s with zero steer as its
  initial point. On the default car the 3 m/s cap clips that guess and it converges; on the gym-limits
  car IPOPT sits in its feasibility phase (14 s/iteration, inf_pr flat at 0.16) for hours. The spike's
  upstream run did not converge in 25 min. Swapping tyre set A, `mu_init` 0.1, or an 8 m/s initial
  speed alone did not help. Swapping only v_max confirmed the cause: the default car at v_max 20 stalled
  like the gym car (359 iterations in 1534 s), the gym car at v_max 3 converged cold (1120 iterations,
  990 s). The fix was seeding from a converged line (`seed_line`); a v_max-3 gym line as seed also works
  (1657 iterations, 390 s, the same optimum as the default-car seed).
- **The centerline as the min-time reference** (part 8 diagnostics, Spielberg). TUM's 2.0 m curvature
  window reads the corner and the hairpin as 0.33 and 0.46 1/m where the centerline has 0.99 and 1.31, so
  the solver plans corner speeds the geometry cannot carry: exported a_y 7-11x the GGV (gym-limits),
  1.5x (default), kappa 1.3-2.5. The frame does not fold (worst 1 - n kappa over the usable corridor
  0.21). A tighter window (0.5 m at h 0.5, 1.0 m at h 1.0) fails restoration or hits max_iter. On the
  min-curv line as reference the window error drops to about 1.3x and a_y stays at most 0.92x, which is
  why the chain references min-curv. Even then 5 of 11 step/window/margin cases timed out, hit local
  infeasibility or failed restoration, and at h 1.0 every line cut the hairpin tip between nodes (0.21-0.23
  m against 0.28). The same runs showed a 5-10x a_x spike at the lap seam, from a clamped speed
  interpolation; the fork's periodic exported speed fixed it.
- **Two-level (corner-fine) grid.** TUM's `step_non_reg` / `eps_kappa`: h 0.2 with step_non_reg 2
  ran 4 h without converging (238+ iterations, inf_du stuck near 7.8); h 0.25 corners / 0.5 straights
  stalled the same way. Regular h 0.2 stalled cold. Hence `MINTIME_STEP_REG` 0.5 and
  `MINTIME_STEP_NON_REG` 0.
- **Resume settings.** Resuming with `mu_init` 1e-2 threw away the near-optimal point; 1e-6 kept it.
  A large margin step (0.20 -> 0.15) can fail where a larger one (0.20 -> 0.10) converges.
- **Fixing the GGV overshoot inside the solve.** A finer collocation step, a smaller margin or a
  higher a_y table does not bring v^2|kappa| under the GGV (table above). Only re-profiling the speed
  on the exported path did.
- **Fork of race_stack for the extractor.** Ruled out by the owner on 2026-10-13: upstream barely
  moves (one commit to the file on `ros2-jazzy` since 2024-05), the code is a rewrite rather than a
  patch, and a fork would not merge upstream changes into it anyway. Forks are only for patches to
  code we install (TUM, `pip_package`, tph).
- **Rebuilding Spielberg's centerline from the yaml kappa** to remove the hairpin fold: the column is
  the upstream raceline's (gotcha 28), the spline tightens to r 0.496 and adds a new fold near idx 89.
  The importer cuts the fold instead.
- **Driving the gym-limits lines with Pure Pursuit.** Every line DNFs (table above). Not a raceline
  fix; it waits on a high-speed controller.

## Ideas not yet specified

- **Own raceline solvers** (owner: later, after the AutoDRIVE bridge, per the ICRA 2027 target). Each
  lands as an added optimizer and is verified against analytic ground truth, never against TUM.
  L1 forward-backward speed profile (circle: v = sqrt(a_y r); straight from rest: v^2 = 2 a_x s).
  L2 minimum-curvature QP from scratch with OSQP or quadprog (on `analytic_circle`, the largest
  concentric circle that fits the margin). L3 own min-time OCP in CasADi, point mass then kinematic
  then dynamic bicycle, checked on the circle and a straight-plus-hairpin, then added to the planning
  benchmark. Reading order for learning: tph `calc_splines` -> `opt_shortest_path` -> `opt_min_curv` ->
  `iqp_handler` -> `calc_head_curv_an` -> `calc_vel_profile`, then TUM `main_globaltraj.py` ->
  `prep_track.py` -> `opt_mintime.py`.
- **Upgrade TUM to current libraries** (optional, later). Triggers: the old env stops installing
  (wheels yanked, Python 3.8 dropped by uv), a league track needs a newer TUM feature, or L3 wants to
  share code. SIM-5040/5060 show any change to a line as a reviewed diff.
- **More league tracks** imported with `tools/import_f1tenth_racetrack.py`: ADR 0005's "across every
  track" needs more than one real gym track. A Porto/AutoDRIVE planning-benchmark row waits on a
  benchmark role that can run AutoDRIVE.
- **Graded curvature-dependent min-time grid** (owner, 2026-10-22). TUM's grid has two levels only; a
  grid that densifies with curvature means a fork change. Revisit only if a regular grid cannot hold
  the owner's 0.02-0.05 m apex target.
- **Continuation schedule for min-time** (owner, 2026-10-24). Start at a large margin and conservative
  limits, step toward the target, warm-starting each step, so a usable line exists early and no solve
  sits 2 h with nothing. Needs a fallback (retry from the last converged step, or halve the step).
  Car parameters in order: gym/ForzaETH sets, then AutoDRIVE-identified, then the real car's.
  Caveat from 8b: a warm-start chain can carry a local optimum along (the s 82-86 double brake above).
- **Untried min-time solver levers** (2026-10-23 read, none measured), for the per-iteration cost.
  Without a fork: IPOPT options through the wrapper's `nlpsol` (`mu_strategy adaptive`,
  `mumps_mem_percent` 8000-20000 to skip the reallocation retries, `mumps_pivtol`, `mumps_scaling`,
  `nlp_scaling_method`, `print_timing_statistics` to see the linear-solver share); mass similarity
  through the vehicle values (mass, I_z, force and power limits and `f_z0` times k of about 270,
  `penalty_F` over k^2) so the scaled controls are O(1). With a fork: scale constants, the 20 m/s
  `v_guess`, bounds on collocation states, an Fz >= epsilon constraint, curvature windows in metres.
  With an image change (ADR 0002): a licensed HSL `libhsl.so`.
- **Tyre and vehicle parameters from the real car** (owner, 2026-10-22): a few RC laps at a venue,
  fit the Pacejka and vehicle values, parameters only. For AutoDRIVE, a wall-free test scene to fit
  the tyres (owner, 2026-11-02; owner will spec it). Today `autodrive_roboracer/tum_mintime_tyres_gym_fit.yaml`
  is a copy of the gym set ([#79](https://github.com/G-S-Rodrigues/GR-roboracer/pull/79)).
- **Clearance budget** (owner, 2026-10-22; target confirmed Q15): an achieved 2-5 cm from the car's
  full outline, wheels included, to the true wall. Today the verifier checks the CoG and subtracts
  `EXTRACTION_ALLOWANCE` 0.15 m, so below a margin of about 0.17 m the ground-truth rule protects
  nothing. The idea: check the outline, one owner-set margin (ceiling about 0.10 m) covering
  between-node overshoot (0.3-7 cm measured) and half a map cell, and measure the real map-vs-truth
  offset at every apex. If the map error is too big, the fix is a better map, not a bigger margin.
- **Reject a min-time solve whose curvature disagrees with its reference** (P8-R1; owner, 2026-11-05:
  for future assessment). e_rel separated bad solves (5.7-6.4) from a good one (1.37); a threshold
  near 3 rests on one good sample. Likely lands in the fork, as a hard error.
- **Take upstream `ace99a4`** (owner, 2026-11-05: a part after G1, the gym bundled-map fix), a fork PR
  plus a reviewed regeneration, which the edits listed below can ride along with.
- **Map correction for real tracks** (owner, 2026-10-13): open sides and overpasses corrected by hand
  or by script before extraction. Nothing is built until a mapped-map workflow without ground truth
  has been tried on AutoDRIVE.
- **Extraction aids that need no ground truth** (parked in the 2026-10-13 grill): use the mapping
  lap's driven path as the extractor's topological guide (picks the corridor, orders passes through an
  overpass, gives upstream's `cent_length`); self-consistency checks (bounds on the map's own wall
  edges, loop closure, width sanity, length vs the driven lap). Bridging gaps over 4 m needs a better
  tangent estimate (longer secant or fitted arc), not a looser cap.
- **Gate lap on the fast vehicle** (owner, 2026-10-11): move SIM-3110/3120 to the gym-limits car once
  its laps are stable across seeds, as a reviewed change with regenerated goldens. Blocked by the DNFs.
- **Promote `optimized_raceline` to the reference-stack default**: a spec non-goal, a separate
  reviewed decision that re-points every test and golden.
- **Shortest-path line** (ForzaETH uses one for overtaking sectors): no method is implemented.
- **Edits waiting for the next records regeneration.** Any byte change to a `CODE_FILES` member or a
  pinned vehicle file regenerates every record (about 40 min-1 h of TUM under the lane, a data-repo PR
  and a re-pin), so these wait to ride along: `optimize.py`'s `tum_ini` comment still says dragcoeff 0.0
  because "f1tenth_gym_jax has no drag" (stale for AutoDRIVE's linear drag 0.273); the `--track` folder
  fix above; `f1tenth_gym_limits.yaml` still says the vehicle "runs nightly only", though SIM-5050 is
  retired. Proposed instead (2026-11-10): make the provenance hash ignore comments, so a comment edit no
  longer invalidates a line.
- **`sim/track_importer._first_crossing` is O(n^2) in memory**: about 60 MB for Spielberg's 864 points,
  about 2 GB at 5000. A dense imported track would run out of memory rather than raise. Chunk by rows
  when it matters.
- **RACELINE-1380 needs network on a cold uv cache.** After an image rebuild a whole-tree `--ready`
  resolves `raceline/tum_env` from GitHub (failed once on DNS, 2026-10-28). Pre-populating the TUM env in
  the image (ADR 0002) or accepting it is undecided.

## Open questions

- **Why is the exported min-time path more curved than the solver's states?** Candidates: the 0.5 m
  node grid with spline interpolation, and the |omega_z| <= pi/2 box bound every Porto corner sits
  on. At 4.5 m/s or more that bound may be what limits the fast car. Raise it, derive it, or leave
  it: owner's call. `--reprofile-speed` hides the symptom, not the cause.
- **Regenerate the Spielberg min-time records with `--reprofile-speed`?** They pass the verifier
  without it; the change is a reviewed diff of committed lines and RACELINE-1395 hashes (owner).
- **Why does one min-time optimum take 817 iterations in one container and 1715 in another?** Gotcha
  43. Iteration bounds come from `gr-roboracer-dev` until it is explained. Inside `gr-roboracer-dev`
  (2026-10-25) the BLAS setup did not move the count: default 3 threads against a run labelled OpenBLAS
  Nehalem kernel on 1 thread gave 817/817 (in-memory reference), 1518/1518 (+0.01 m width) and 170/170
  (default car), with identical lines. Thread scheduling is the weaker lead; x0 rounding remains.
- **Why does the AutoDRIVE car's Spielberg min-time take about 50 s per iteration?** Diagnose before
  any AutoDRIVE Spielberg record is attempted (owner, 2026-11-05: future assessment). Comparable to
  the 8b slow runs (14-75 s per iteration, each with three MUMPS reallocations); the untried solver
  levers above target that cost.
- **Does the extractor fail loudly or return a plausible wrong track on an open side, and how does an
  overpass appear on a 2D lidar SLAM map?** Unmeasured until the AutoDRIVE mapped-map workflow
  exists. Mapped tracks without ground truth are not yet driven (README capability row). Gaps on
  non-circular walls (straight-to-arc joints) were never swept.
- **Is the survey map stale?** The committed Spielberg survey (`tracks/Spielberg/maps/slam_toolbox/res_0050/`,
  2026-10-14) was mapped on the walls before part 2.1 moved them by up to 0.078 m. Every Spielberg
  line is generated from it. Regenerate it, and so every line and amcl's map, or never: owner.
- **Is one a_y per vehicle the right limit set?** On AutoDRIVE the lateral limit varies about 44 %
  with radius (gotcha 46), and the Porto s 8-11 right-hander lets go intermittently at a_y about 6
  (3-4 % of laps). A line planned to one a_y is right at one radius only. Whether limit sets carry
  per-corner or per-radius limits shapes any future AutoDRIVE line.
- **The gym-limits GGV is a modelling bound.** a_y = mu g, but the gym's `st` tyres are linear and
  never saturate, so a gym-limits line is slower than the gym allows. Whether that matters waits on a
  controller that can drive the line at all.
