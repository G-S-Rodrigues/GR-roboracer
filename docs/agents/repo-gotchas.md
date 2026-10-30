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

4. **`dev` stays CPU; CUDA goes in `gpu` or `fast-sim`; the gym's JAX stays on CPU in both `dev` and
   `gpu`.** This was tried and measured: on the committed one-agent/64-beam scenario the GPU is **5x
   slower per step** (8.36 ms vs 1.62 ms) — close enough to the 10 ms period to drop `/odom` below
   its rate. That measurement is the entire reason, and under ADR 0006's `time_scale > 1` it binds
   harder, because a slower step is what bounds an accelerated run. It is why `dev` stays CPU, and
   also why the gym node's JAX stays on CPU inside `gpu`, which is `FROM dev` by owner ruling. Its
   JAX is pinned to CPU by `docker/gym-cpu-overrides.txt`, and `gpu` installs no Python package
   (CHECK-1910; GPU-3010 asserts `cpu`). `gpu` exists for ROS nodes whose kernels are large enough to
   win, such as FAR-LIO's GICP. Installing a CUDA jaxlib into `gpu` would bring back the 5x step. (Until
   2026-09-10 this gotcha also argued that a wall-clock-timed sim node means a GPU cannot speed up a
   tier-3 run. `time_scale` ended that; the measurement above carries the decision now.) See ADR 0002
   and its 2026-10-25 amendment, ADR 0006, and the plan's D29.

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

21. **Rebuilding the image can strand the colcon build volume.** The apt layer resolves current
    Jazzy binaries, so a rebuild may bump a library the overlay linked against (seen: `libfastcdr`
    2.2.7 -> 2.2.8). `build/` and `install/` are named volumes that survive the rebuild, and the next
    `colcon build` fails with `No rule to make target '/opt/ros/jazzy/lib/libfastcdr.so.2.2.7'`.
    After any image rebuild, `rm -rf build/* install/*` inside `dev` before building. The `gpu` service
    has its own volumes (`ws-gpu-build`, `ws-gpu-install`), so the same remedy applies inside it.

27. **An unpinned install in the image is latent until the layer cache misses.** CI's gha cache
    replays a resolve from whenever the layer last ran (here 2026-10-02), so drift surfaces only when
    the upstream base tag moves, and then on every leg and every PR at once. Seen: the unpinned
    `pip3 install pre-commit` resolved `virtualenv 21.14.11`, which needs `packaging>=26.3`, and pip
    could not replace Debian's `packaging 24.0` (no RECORD file); three nightlies failed at "Build the
    dev image" before any test ran. Symptom: a Dockerfile untouched for weeks fails to build. Remedy:
    the dev tools are a full closure in `docker/dev-tools-requirements.txt`, installed `--no-deps`,
    then `pip3 check`; CHECK-1330/1340 guard it. Bumps are reviewed diffs. The `gpu` stage's apt closure
    follows the same rule (`docker/gpu-apt-pins.txt`, CHECK-1910/1920), and the trap was measured:
    unversioned `cuda-toolkit-config-common` resolved to 13.4.92 on 2026-10-25.

## ROS

6. **QoS mismatch looks like absent data, not an error.** A reliable subscriber never hears a
   best-effort publisher, and nothing anywhere says so. When a topic "isn't publishing", compare the
   two ends' QoS before you look at the code.

7. **clang-tidy needs `-p=build`** — run `colcon build` first, or it has no compilation database.

8. **`sim/`, `tools/`, `scripts/` and `raceline/` are excluded from `ament_lint`** and linted by `ruff` from
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
    rates, or an emergency stop latched before your run began. Kill `install/racing_` too, `lib/mola_launcher`, `mola-cli` (measured outliving `ros2 launch` by 15 minutes, `test_mola_map.py`) and `racing_mola_map_alignment` (`pose_source:=mola`'s `map -> mola_map` static_transform_publisher lives under `/opt/ros`, so the first pattern never matched it), and note
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

22. **The gym's scan beams are not where a `LaserScan` of it says they are.** f1tenth_gym_jax passes
    jax_pf an increment of `fov/(N-1)`, and jax_pf's `get_scan` spreads N beams with
    `linspace(start, start + inc * N, num=N, endpoint=True)` — `inc * N/(N-1)` apart. Its sweep is
    4.775 rad for 64 beams, not 4.7; the last beam is 4.3 deg off. It also reads each angle from a
    `linspace(0, 2pi, theta_dis)` table (endpoint included) at a truncated index. Nothing errors:
    slam_toolbox built a warped, unclosed Spielberg map from it that looked like a tuning problem.
    `/scan` is therefore cast by `racing_sim_gym_jax.laser.cast_scan` at the declared angles
    (SIMJAX-1090); `/ground_truth/scan` keeps the gym's ranges (metrics and goldens read them) and
    declares the gym's spacing (SIMJAX-1100). Never compare two scans beam-by-index: read both at
    their declared angles. Plan D6 has the measurements.

25. **A map PNG's pixel is a cell, and PIL's polygon fill is not.** Row 0 is the bottom and cell (i, j) is
    centred at `origin + (j+0.5, i+0.5)*res`. PIL's `ImageDraw.polygon` centres pixels on integer
    coordinates and paints the outline. Until part 2.1 the importer drew the gym's walls up to 0.078 m
    off the committed track with no error (SIM-1030).

28. **An offset wall folds where the centerline's radius is under its width, and nothing errors.**
    Spielberg's hairpin (idx 275-282) turns at r 0.64 m against a 1.1 m half-width, so `xy +- w*n`
    crosses itself and the even-odd fill drew the swallowtail as a detached 11-cell occupied spur in
    the gym PNG. Symptom: a speck of wall beside the track, a wall polygon that crosses itself, a
    raceline that "touches" it. The importer's `_walls` now cuts each fold at its crossing
    (`_remove_folds`; SIM-1040/1050/1060/1070 guard it) and raises "self-intersect" past half the
    corridor. The yaml's curvature column is the *upstream raceline's* (it integrates to -394.4 deg
    per lap), so never rebuild a centerline from it.

## Visualization

16. **A missing mesh is invisible, not an error.** `robot_state_publisher` never loads geometry, so a
    URDF referencing meshes that do not exist starts cleanly and passes every structural test; only
    RViz shows the hole. Meshes other than `hokuyo.stl` and the two wheels are generated by
    `racing_vehicle_description/scripts/generate_f1tenth_meshes.py` — run it after changing the URDF's
    dimensions, and let `test_every_referenced_mesh_exists` confirm the set is complete. The AutoDRIVE meshes
    come from `import_autodrive_meshes.py` instead (DESC-1030 runs the same check over every URDF in
    `urdf/descriptions.yaml`).

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

23. **`check.sh --fast` is scoped, and can miss what only the whole tree catches.** It lints and
    tests the packages the branch touched. `clang-tidy` there sees one package's compilation
    database, so a header change that breaks a *dependent* package's tidy result passes `--fast` and
    fails `--ci`. The whole-tree trigger list in `scripts/changed_scope.py` is a hand-kept copy and
    goes stale: `ruff.toml` and `.clang-format` were missing until a review caught them, and any
    config file added later is missing the same way, so a change to only it lints nothing under
    `--fast`. Symptom: `--fast` green, `--ci` red. Remedy: `CHECK_SCOPE=all
    ./scripts/check.sh --fast`. The measurement procedure (a throwaway clone whose `main` is the
    branch head, since a branch that already touches `check.sh` or `config/` is always whole tree)
    is in the part-1 PR; inside `dev`, cloning `/ws` needs `git config --global --add safe.directory
    /ws/.git`.

    `config/reference_stack.yaml`'s `paths` / `verified_by` (ADR 0007) is the *second* hand-kept list.
    Unlike the first, it fails closed: an unplaced path or an unclaimed test widens `--ready` to the whole
    tree, so a stale map shows up as a slow `--ready`, never a skipped test. The exception is a path
    assigned to the *wrong* implementation, which nothing catches but review and the nightly.

    The scope's base is `refs/remotes/origin/main` when it exists (local `main` only when it is absent,
    and the line says `main (no origin/main)`); it is never fetched, and `; base <ref> <sha>` is printed
    on the scope and ready lines so a stale one shows. In `dev`, git fails twice over on a worktree made
    on the host: its `.git` file names a host path that does not exist (`not a git repository`), or, with
    a hand-rewritten relative path, root-vs-UID-1000 ownership that `safe.directory=/ws` does not cover
    (`dubious ownership`). `changed_scope.py` therefore, only when plain git fails, locates the git dir itself behind a trust gate
    (CHECK-1350..1370) and runs git with `GIT_DIR`/`GIT_WORK_TREE`; it logs `changed_scope: git dir ...`
    on stderr. Any other git in `dev` on a host worktree still needs the hand rewrite and a
    `safe.directory`. Never `git worktree prune/repair` in `dev`: it rewrites the host's back-pointers.

24. **The sourced ROS `PYTHONPATH` leaks into every uv venv run after it.** `check.sh` sources ROS and
    the overlay, and `PYTHONPATH` is honoured inside a uv venv, ahead of its site-packages. A
    `raceline/` run without `env -u PYTHONPATH` can import `rclpy` and `racing_common` (ADR 0003's
    no-ROS rule broken, no error) and pytest itself may die loading ROS's launch plugin. Run raceline
    code as `env -u PYTHONPATH uv run --project raceline --locked ...`. Guard: RACELINE-1020. The
    TUM optimizer runs in `raceline/tum_env/` (Python 3.8, original pins with two bumped: tph 0.78, quadprog 0.1.12; see the README), reached as a subprocess
    by `raceline.tum` (README there); never import it into the 3.12 env.

26. **`base_link` is the centre of gravity, not the rear axle.** The gym state, every published pose,
    z = h (0.074 m), and the supervisor/metrics point are the CoG; the URDF drawing and `laser_x` are
    authored in `rear_axle`, its TF child lr = 0.17145 m behind, and the sim converts the mount by lr.
    Reading 0.275 as CoG-relative put the beams 0.171 m ahead of the real lidar and nothing errored.
    Pure Pursuit's law is exact only at the rear axle and runs on the CoG by owner ruling (a small
    steering-angle bias). **A pose measured at the rear axle (AutoDRIVE's `roboracer_1`, a real car's
    wheel odometry) must be moved lr along heading and given z = h before it is published as
    `base_link`.** Under an estimator, `map -> base_link` z may be 0 and `map -> odom` carry -h (measured
    under amcl in part 3.1b); the car is then drawn 0.074 m low in RViz (numbers unaffected).
    `rear_axle` is the **ground point** under the rear-axle midpoint (z = -h from `base_link`), not
    the axle centre (a wheel radius higher), because every mesh is authored there. The AutoDRIVE
    description uses lr 0.15532 / h 0.07354, in its own URDF.
    The lidar's TF frame is lidar, renamed from laser on 2026-10-28 to match the AutoDRIVE devkit;
    device identifiers (laser_x, LaserMount, the racing_sim_gym_jax.laser module) keep the old word,
    and a bag or RViz config recorded before the rename still names laser.

## AutoDRIVE

29. **The official devkit stalls on a headless sim, silently.** Its `Bridge` handler reads every key
    unguarded (`autodrive_bridge.py:309-375`). The first `-batchmode -nographics` message has no
    `V1 LIDAR Range Array` (every later one does; `V1 Front Camera Image` is present on all), the
    handler raises `KeyError`, sends no reply, and the lock-step exchange stops: topics exist, nothing
    publishes. Seen live in `autodrive-devkit`'s log. Anything that stands in for the devkit must
    reply to the first message too (the recorded stream, `ros_ws/src/racing_sim_autodrive/test/fixtures/`).

30. **The sim connects once and never reconnects, and speaks an older Socket.IO than its URL says.** A
    bridge started late or restarted gets nothing; restart the sim (`autodrive-sim` waits for the
    bridge port for this). Its URL asks for `EIO=4`, but it sends Engine.IO v3 pings and no namespace
    CONNECT packet. python-socketio 5.17 / python-engineio 4.14 accepts the socket and drops every
    event ("None is not connected to namespace /"): no error, no data. python-socketio 4.2.0 /
    python-engineio 3.13.0 (the devkit's pair; gevent 26.9.0 and gevent-websocket 0.10.1 on Python
    3.12) held 200 s, 2782 messages, no disconnect. `tools/autodrive_record.py` pins that set.

31. **The sim's message rate follows host load, and every lidar frame after the first has `inf`.** A
    recording on a busy host (load 20-30 on 16 cores) had a median period of 18.2 ms (~55 Hz), a
    10-90 % range of 11-55 ms (~18-89 Hz) and a longest gap of 145 ms, not a fixed 20 Hz: do not
    assert a rate band on a recorded stream. The first frame has no lidar at all. In every later
    frame `inf` (no return) is ~1.9 % of the ranges; they are not faults, `NaN` would be.

32. **Jazzy-Humble DDS discovery is slow, and a loaded host drops 100+ ms of messages anyway.** With
    the devkit's QoS (reliable, volatile, depth 1) a Jazzy publisher and a Humble subscriber needed 0
    to 2.8 s after both were up before the first delivery (up to 55 messages lost). Some of the 20
    probe runs took over 1 s; the per-run logs were not kept, so no fraction is claimed. The
    `sequence size exceeds remaining buffer` warning appears once or more. After the first delivery
    the probe's own 20 Hz publish timer held (that is the probe's rate, not the sim's) but gaps over
    100 ms occurred (max 1.4 s, ~3.6 per minute on a host at load ~30); a Jazzy-to-Jazzy control
    under the same load had ~3 per minute (max 0.59 s), so they are load, not the distro pair. The
    fix is waiting for the first delivery, never a longer timeout (gotcha 19).

33. **`/ws/setup.sh` sources `/ws/install/setup.bash`, not the worktree's.** In a `.scratch/<worktree>`
    a build or test after `source setup.sh` can run on the main checkout's overlay, another branch's
    code, with no error. `source /ws/.scratch/<worktree>/install/setup.bash` after it and check
    `ros2 pkg prefix <pkg>` points under the worktree. (`check.sh` sources its own `repo_root`.)

34. **Compose projects are shared across worktrees.** `docker/docker-compose.yaml` sets
    `name: gr-roboracer`, so a worktree's compose file addresses the same containers as the main
    checkout's. `docker compose ... up -d dev` or `--force-recreate dev` from a worktree remounts `dev`
    onto that worktree, because `..` in the compose file resolves there. `up` with no service name or a
    changed `dev` definition can recreate it too; `--remove-orphans` removes containers, it does not
    recreate them. `dev` is the shared container, mounted on the worktree as `/ws`. Always name the
    autodrive service and pass `--no-deps`; never `down`.

35. **Receive time is not the AutoDRIVE sim's time.** Encoder speed on the recorded 3.5-4.9 s straight
    (16 frame pairs, wheel radius 0.059) differs from `V1 Linear Velocity` x by a median 0.006 m/s (max
    0.007) when dt is the `V1 Lap Time` delta, and by 0.106 (max 0.305) when dt is the receive time of
    the frame, 0.069 (max 0.214) over windows of at least 0.5 s. Sim time over wall time, over sliding 3 s
    windows within each stretch where the lap time rises (296 windows), ran 0.90-1.04, and the frame period was ~80 ms there against an 18 ms median overall. Nothing
    errors: a wall-clock dt gives a plausible speed. `racing_sim_autodrive.sim_dt` is the lap-time delta;
    it is `None` whenever the lap clock does not rise (40 pairs in the fixture, all while the reset
    level was held; a drop at the lap line is presumed, not recorded). Lap time is not a race-time
    input. Which clock a front end feeds the throttle loop is open topic autodrive-clock-for-dt.

36. **AutoDRIVE's pose, velocity and steering are three different things.** `V1 Position` is the rear
    axle (the frame `roboracer_1`), `V1 Linear Velocity` is the body's (about the CoG, 0.155 m ahead)
    velocity in the car frame, and `V1 Steering` is the slewed actual angle, not the command. Measured
    on the identification recording: pose lateral speed 0.0003 m/s, vy/omega 0.160 m. A converter or
    speed loop that treats them as one point is wrong by lr and nothing errors. The same file also
    cannot measure a top speed on Porto (about 8 m of straight; highest steady 2.977 m/s), and its
    braking depends on speed (8.4 / 5.8 / 6.0 m/s^2 from 0.5 / 3 / 4.9 m/s): never read one number
    from `identified.yaml` as the car's.

38. **The Windows AutoDRIVE sim writes and parses numbers in the Windows culture.** Under pt-PT it
    sent `'V1 Throttle': '0,0000'` and stalled on our reply of `0.0`: the car never starts, nothing
    errors. `Set-Culture en-US` fixed it (measured 2026-10-30, `docs/agents/running-wsl.md`). Set it
    for the sitting and restore the saved culture after; `Set-Culture` is a persistent per-user
    setting. A stand-in server or client for the sim should fail loudly on a comma decimal in the
    first message.
