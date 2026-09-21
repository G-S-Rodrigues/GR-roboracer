# Repository gotchas

Traps any task in this repository can hit. Each one cost real time at least once. They share a shape:
**the wrong thing does not raise an error** — it warns, or silently produces a plausible number.

## Environment

1. **Never put the tree on `/mnt/c/`.** The 9p bridge makes `colcon build` roughly an order of
   magnitude slower and does not propagate `inotify`, so file watchers stop firing with no error.
   Symptom: "my rebuild didn't pick up the change." The tree lives on ext4 inside the distro.

2. **`git clean -xdf` destroys `.scratch/`** — every grill, spec and plan, with no git history to
   recover from. `.gitignore` gives no protection; `-x` exists precisely to ignore it. Use
   `git clean -e .scratch`.

3. **A missing CUDA jaxlib is a `WARNING`, not an error.** JAX falls back to CPU and returns correct
   results far more slowly. Assert the backend (`jax.default_backend() == "gpu"`), never just that
   the import worked. `sim/rollout.py` does this and refuses to run otherwise.

4. **CUDA belongs in `fast-sim`, not in `dev`.** This was tried and measured: on the committed
   one-agent/64-beam scenario the GPU is **5x slower per step** (8.36 ms vs 1.62 ms) — close enough
   to the 10 ms period to drop `/odom` below its rate. That measurement is the entire reason, and
   under ADR 0006's `time_scale > 1` it binds harder, because a slower step is what bounds an
   accelerated run. (Until 2026-09-10 this gotcha also argued that a wall-clock-timed sim node means
   a GPU cannot speed up a tier-3 run. `time_scale` ended that; the measurement above is what
   carries the decision now.) See ADR 0002, ADR 0006, and the plan's D29.

5. **Clone with the SSH URL.** `gh auth`'s `Git operations protocol: ssh` does not rewrite an
   existing HTTPS remote; an HTTPS clone prompts for a username and password forever. Fix with
   `git remote set-url origin git@github.com:<owner>/<repo>.git`.

20. **A colcon build in a `.scratch/` worktree reads the wrong `config/`.** `racing_sim_gym_jax`'s
    `setup.py` installs `config/` from `parents[3]` of itself, which a symlink install inside
    `/ws/.scratch/<worktree>/` resolves through the build directory to `/ws/.scratch/config` — a
    shared path that may point at some other, possibly deleted, worktree. Every tier-2 adapter test
    there then dies with `FileNotFoundError` on a scenario file. Symptom: tests that pass in `/ws`
    fail in the worktree on a file that plainly exists. Point the worktree's
    `install/racing_sim_gym_jax/share/racing_sim_gym_jax/config` at its own `config/`.

## ROS

6. **QoS mismatch looks like absent data, not an error.** A reliable subscriber never hears a
   best-effort publisher, and nothing anywhere says so. When a topic "isn't publishing", compare the
   two ends' QoS before you look at the code.

7. **clang-tidy needs `-p=build`** — run `colcon build` first, or it has no compilation database.

8. **`sim/`, `tools/` and `scripts/` are excluded from `ament_lint`** and linted by `ruff` from
   `scripts/check.sh` instead. Do not remove the exclusion to "fix" a lint gap.

9. **Never add a ROS dependency to `racing_common`.** Tier 1 depends on it having none: that is what
   lets the whole track/Frenet/action layer be tested in milliseconds with no graph.

19. **A late-starting estimator latches an emergency stop before it ever publishes.**
    `stale_input_timeout_ms` is 100 ms, and its comment in `config/vehicles/f1tenth_default.yaml`
    says — correctly — that widening it is the wrong response to a latch. But `nav2_amcl` is
    lifecycle-managed, and `slam_toolbox` needs several scans before it emits anything. Start the
    drive stream before the active pose source is up and publishing and the supervisor latches an
    unrecoverable stop during startup, on a graph that is otherwise perfectly healthy. Symptom: an
    emergency stop already latched at t=0, with no fault anywhere in the logs. The fix is a
    deterministic t=0 — wait for every participant, then call the sim node's `~/reset` — never a
    longer timeout.

10. **A stale ROS graph outlives the launch that started it.** `pkill -f 'ros2 launch'` kills the
    launcher and leaves every node running; they keep publishing on the same `ROS_DOMAIN_ID` and
    contaminate the next run with a *second* supervisor and controller. Symptom: impossible message
    rates, or an emergency stop latched before your run began. Kill `install/racing_` too, and note
    that `ros2 node list` shows stale discovery entries for a few seconds after.

## Simulation

11. **The gym_jax action mapping fails silently.** Never change it without re-running COMMON-1060.

12. **Call `step_env`, not `step`.** `step` auto-resets on termination without telling you.

18. **A rotationally symmetric map makes localization error unmeasurable, and it looks like
    success.** `analytic_circle` is a perfect annulus: every point on the centerline sees an
    identical scan, so scan matching has zero observability of position along the track. A SLAM or
    localization node run on it builds a plausible ring, converges, reports no fault, and lets its
    pose slide freely along `s`. Nothing errors and no metric objects. **Never measure localization
    on `analytic_circle`** — it is kept only because it is ~6.5x cheaper per lap than a league track
    (22.85 s vs ~147 s) and is the right track for everything that is not localization. The same trap
    in a subtler form is a long straight between parallel walls, which is why Monza was rejected as
    the league track and Spielberg chosen.

13. **Warm every JAX signature before the run, not just the first.** Stepping a freshly *reset* state
    and stepping an *already stepped* state are two signatures; warming only the first leaves a
    ~0.5s (CPU) / ~1.5s (GPU) compile stall on step 2 of a live run — long enough to trip the safety
    supervisor's stale-input timeout and latch an unrecoverable emergency stop.
    `GymBackend._warm` takes two chained steps for this reason.

## Visualization

16. **A missing mesh is invisible, not an error.** `robot_state_publisher` never loads geometry, so a
    URDF referencing meshes that do not exist starts cleanly and passes every structural test; only
    RViz shows the hole. Meshes other than `hokuyo.stl` and the two wheels are generated by
    `racing_vehicle_description/scripts/generate_f1tenth_meshes.py` — run it after changing the URDF's
    dimensions, and let `test_every_referenced_mesh_exists` confirm the set is complete.

## Metrics

14. **Golden baselines are regenerated as a reviewed diff, never to make a run pass.** The same goes
    for tolerances in `scripts/compare_metrics.py`: widening one to green a run silently removes the
    guard. Every tolerance there carries the measurement that justifies it.

17. **Noise on `/scan` silently rewrites two metrics.** `racing_metrics` subscribes `/scan` and
    uses it for `minimum_wall_clearance` *and* for `collision_count`
    (`racing_metrics/src/node.cpp:105,132,160`). The moment `/scan` carries sensor noise, both
    measure the noise floor rather than the vehicle, and both then vary with the noise seed while
    passing every structural check. `racing_metrics` reads **`/ground_truth/scan`** for this reason.
    It is deliberately not switched to a geometry-derived clearance, which would be simpler: a
    scan-based clearance also detects obstacles that are not in the track geometry, which is what
    Stage 4-5 opponents will be.

15. **Lap distance comes from the track, never from a configured number.** A `track_length` that
    disagrees with the track is invisible — the run completes, publishes metrics and passes every
    structural check, while calling a fraction of a lap a lap and reporting a proportionally wrong
    `lap_time`. This actually happened (a 31.4159 default against a 61.23 m centerline) and cost a
    "still-unexplained" 2x discrepancy that sat in a test docstring for a while.
    The same trap sat in the golden generator: `sim/rollout.py` ended a lap on the gym's winding
    number (angle swept around a point beside s=0), which on a non-circular track fires early —
    after 331.6 m of Spielberg's 343.3 m — while `racing_metrics` ends it on centerline distance.
    The rollout now reuses `racing_metrics`' rule (`CenterlineLap`, SIM-1010). A generator and a
    live consumer that each define "a lap" will disagree, invisibly, on the first track where it
    matters.
