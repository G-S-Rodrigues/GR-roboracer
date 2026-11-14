# Control

Pure Pursuit, the safety supervisor below it, the speed loops below that, and the controllers planned
to sit beside Pure Pursuit. Read before a grill, spec or plan that touches `racing_controller_baseline`,
`racing_safety_supervisor`, a new controller, AutoDRIVE's throttle loop, or how a run starts at t=0.
The line the controller follows and its speed profile are in [raceline.md](raceline.md). The pose it
drives on is in [localization-slam.md](localization-slam.md). The simulators' action contracts and
bridges are in [simulation.md](simulation.md), the league's rules in [competition.md](competition.md),
and learned control in [reinforcement-learning.md](reinforcement-learning.md).

## Current state

Verified against `origin/main` 7c9ff4a on 2026-11-14.

### Pure Pursuit is the only controller

- Package [`racing_controller_baseline`](../../ros_ws/src/racing_controller_baseline/). The ROS-free core
  is [`pure_pursuit.cpp`](../../ros_ws/src/racing_controller_baseline/src/pure_pursuit.cpp), the ROS shell
  is [`node.cpp`](../../ros_ws/src/racing_controller_baseline/src/node.cpp), and
  [`python/bindings.cpp`](../../ros_ws/src/racing_controller_baseline/python/bindings.cpp) exposes the
  same core to [`sim/rollout.py`](../../sim/rollout.py), which generates the goldens. The package's
  CLAUDE.md keeps the core authoritative for both.
- In [`config/reference_stack.yaml`](../../config/reference_stack.yaml) the `controller` role is
  `pure_pursuit`, verified by SIM-3030/3040/3050/3090 and ACC-4010/4020/4040. It is the only role with
  no `alternatives` block.
- There is **no `controller:=` launch argument**. `sim_pure_pursuit.launch.py` hard-codes the node
  ([racing.md, Control](../workflows/racing.md#control)). The README lists tracking/dynamic MPC and MPCC
  as Exploratory.
- The node is **event-driven**. Every `/odom` message (reliable, depth 10) computes and publishes one
  command on `/controller/drive`, stamped with the `/odom` stamp. There is no timer.
- It reads x, y, yaw and `twist.linear.x` (the CoG speed). The launch picks the pose by remap only:
  `/ground_truth/odom` by default, `/localization/odom` under `drive_on_estimate`. The composed
  estimate carries the dead-reckoning twist (`racing_bringup/support_node.py`, "composed.twist"), and
  the AutoDRIVE bridge publishes a body twist.
- `/trajectory` (reliable, transient-local) is a list of (x, y, v). `planner:=track_centerline`
  publishes the centerline with a constant v of 1.0. `planner:=optimized_raceline` publishes a
  committed line.

**The law**, in `PurePursuit::compute`:

- The closest point is a **global search over every segment**, with no progress window.
- The target sits at progress + lookahead, where lookahead = max(`lookahead_distance`,
  `lookahead_gain` x measured speed). Measured speed is `/odom`'s twist, not the line's v. The owner
  ruled this in raceline part 5 (2026-10-21).
- curvature = 2y/d^2 and steering = atan(wheelbase x curvature). The law is exact at the rear axle and
  runs on the CoG by owner ruling (gotcha #26).
- Speed comes from `speed_source`. `curvature_schedule` (the default) gives
  max_speed / (1 + gain x |curvature|), clamped. `trajectory` gives the v of the **nearest vertex at the
  CoG's own closest point**, clamped to [minimum_speed, maximum_speed]. There is no preview and no
  feed-forward of a coming deceleration.
- `speed_source` is a launch argument forwarded only when given (argument > vehicle file > node
  default). The launch refuses `speed_source:=trajectory` on a map with no raceline.
- `lookahead_gain` is a node parameter only (vehicle file). It is 0 in every vehicle file, so the
  speed-scaled lookahead is built and off.

| Vehicle file | Wheelbase | Lookahead | PP / supervisor max speed | Max accel | Max steer | Steer rate | Half-width | Stale timeout |
|---|---|---|---|---|---|---|---|---|
| `f1tenth_default.yaml` | 0.33 | 1.0 | 3.0 | 2.0 | 0.4 | 1.5 | 0.15 | 100 ms |
| `f1tenth_gym_limits.yaml` | 0.3302 | 1.0 | **20.0** | 9.51 | 0.4189 | 3.2 | 0.155 | 100 ms |
| `autodrive_roboracer.yaml` | 0.324 | 1.0 | 3.0 | 2.0 | 0.5236 | 1.5 | 0.135 | 200 ms |
| `autodrive_roboracer_fast.yaml` | 0.324 | 1.0 | 4.5 | 4.0 | 0.5236 | 1.5 | 0.135 | 200 ms |

The fast file came with [PR #88](https://github.com/G-S-Rodrigues/GR-roboracer/pull/88). The 200 ms
AutoDRIVE timeout came with [PR #99](https://github.com/G-S-Rodrigues/GR-roboracer/pull/99) (owner
call #27).

### The safety supervisor

[`racing_safety_supervisor`](../../ros_ws/src/racing_safety_supervisor/) takes `/controller/drive` and
`/ground_truth/odom` and publishes `/drive` and `/safety/status`. The core is
[`supervisor.cpp`](../../ros_ws/src/racing_safety_supervisor/src/supervisor.cpp). In order:

1. **STALE_INPUT** latches when the last command is older than `stale_input_timeout_ms`. It is measured
   on the node's ROS clock, so it follows `/clock` under `use_sim_time` (ADR 0006). Gotcha #19 explains
   why the fix for a latch is never a longer timeout.
2. **ESTOP_LATCHED** holds after `~/emergency_stop` until `~/clear_emergency_stop` (`std_srvs/Trigger`).
3. **TRACK_LIMIT** stops the car for that tick only, and **does not latch**. It tests the CoG point
   against the track with `vehicle_half_width`, not the car's footprint.
4. **COMMAND_LIMIT** clamps speed, steering angle, `acceleration` and `steering_angle_velocity` to the
   vehicle file's caps and reports which ones it clamped. It does not latch. Pure Pursuit never sets
   `acceleration` or `steering_angle_velocity`, so those two clamps never fire for it.

The supervisor reads **ground truth** for the track-limit check. A race-time stack has none (league rules
in [competition.md](competition.md)). Under `use_sim_time` it also latches its first `/clock` on
`~/clock_received` for the t=0 wait.

### Below the controller: who tracks the speed command

- **Gym.** The speed and steering commands go through deadbeat laws with gain 1/timestep = 100
  ([ADR 0001](../adr/0001-simulator-action-contract.md)). The gym's speed follows the command almost
  exactly, so **a gym run cannot show a difference in longitudinal control**.
- **AutoDRIVE.** `racing_sim_autodrive/control.py`'s `ThrottleLoop` is feed-forward v/gain plus PI on
  encoder speed (allowed at race time). `throttle_gain` is 24.86 m/s per unit. The gains `kp 0.02,
  ki 0.05` carry the comment "UNTUNED: no gains exist yet"
  ([`server.yaml`](../../config/vehicles/autodrive_roboracer/server.yaml)). Full throttle spins the
  wheels and throttle 0 locks them; nothing modulates slip.
- **Real car.** The VESC owns the speed loop, so `ThrottleLoop` does not transfer.

### Held start and t=0, as the controller sees them

- **Gym** has `start_held` (`racing_sim_gym_jax/capabilities.py`). Since
  [PR #95](https://github.com/G-S-Rodrigues/GR-roboracer/pull/95) (F-T0) the runner's deterministic t=0
  waits for the supervisor's `~/clock_received`, racing_metrics' `~/started` and **one
  `/controller/drive` sample** before it calls the sim's `~/reset`. A truth-driven run whose first
  motion comes later than 3 ticks raises `StartMissed` (`drive-start-missed`, provisional owner call
  #23). Gotcha #19 holds the details.
- **AutoDRIVE** has no `start_held` (`capability.start_held: False`). `tests/autodrive/_porto_drive.py`'s
  `drive_porto` builds one from the supervisor
  ([PR #99](https://github.com/G-S-Rodrigues/GR-roboracer/pull/99)): `~/emergency_stop`, reset to spawn,
  start the mapper on the parked car, then `~/clear_emergency_stop`. `hold_error` checks the hold saw
  only `REASON_ESTOP_LATCHED` and no STALE_INPUT. This is the real car's own order (park, latch, start
  the mapper, release), and it changes no production code.
- ACC-4020's runner starts held and resets at t=0 since
  [PR #75](https://github.com/G-S-Rodrigues/GR-roboracer/pull/75).

### How a controller is measured today

- `racing_metrics` records lap times, collisions, minimum wall clearance, maximum and p95 tracking
  error, and `control_saturation_events`.
- **`tracking_error` is |d| in the track centerline's Frenet frame** (from
  `/ground_truth/track_relative_state`). It is not the distance to the line being driven. On an
  optimized line it mixes the line's offset from the centerline with tracking error. **Nothing
  measures path or speed tracking against `/trajectory`.**
- `racing_recording` keeps `/ground_truth/odom`, `/drive`, the track-relative state and `/safety/status`,
  and the line is a committed CSV. A line-relative error can therefore be computed offline from a
  recording, with no ROS change.
- `scripts/benchmark.py` ([PR #90](https://github.com/G-S-Rodrigues/GR-roboracer/pull/90)) benchmarks
  `pose_source` and `planner` only (`BENCHMARKED_ROLES`). There is **no controller role**. A new
  controller implementation gets a green detect plus the warning "no benchmark for role controller".
- The goldens (`tests/golden/baseline.json`, `spielberg_baseline.json`) pin Pure Pursuit's own
  behaviour. A change to its **default** behaviour moves them and needs a reviewed regeneration
  (gotcha #14). A new controller added as an alternative touches no golden.
- Gates and bounds come from [ADR 0008](../adr/0008-gates-decide-quality-is-reported.md): tracking p95
  < 1.0 m and clearance > 0.08 m are catastrophic bounds. Lap time and tracking are reported only. A new
  controller may land `status: experimental` (§6).

## What we measured

### Command lag against lap time (seed-diagnostic replay, 2026-11-06)

Offline replay of a recorded gym run reproduces it faithfully. Each tick of *persistent* command lag adds
about 0.01 s of lap time; a one-off delay changes nothing. The lap-time gap between seeds 42 and 1030
comes from the seed, not from lag. The probe was superseded by PR #95's deterministic t=0 (gotcha 19).

### The fast gym car DNFs everywhere (nightly SIM-5050, run 42010230644; owner note 2026-11-05)

Spielberg, gym, 4 seeds per row, Pure Pursuit with `speed_source:=trajectory`, fixed 1.0 m lookahead,
gain 0:

| Vehicle | Line | Completed | Lap s (min / median / max) | DNF distance m | Min clearance m | Path m | TUM lap estimate s |
|---|---|---|---|---|---|---|---|
| gym_limits | centerline | 0/4 | DNF | 6.6 - 32.1 | - | 343.3 | - |
| gym_limits | tum_mincurv_iqp | 0/4 | DNF | 9.1 - 73.3 | 0.323 (verifier) | 338.7 | 29.60 |
| gym_limits | tum_mintime | 0/4 | DNF | 51.9 - 104.9 | 0.455 (verifier) | 338.7 | 32.37 |
| default | centerline | 4/4 | 120.42 / 120.44 / 120.54 | - | 0.779 | 343.3 | - |
| default | tum_mincurv_iqp | 4/4 | 113.08 / 113.11 / 113.16 | - | 0.327 | 338.7 | 112.89 |
| default | tum_mintime | 4/4 | 112.07 / 112.12 / 112.19 | - | 0.453 | 335.7 | 111.88 |

- At 3 m/s Pure Pursuit follows the profile. The default car's min-time lap is within 0.2-0.3 % of
  TUM's estimate.
- The lines verify, so the owner's reading is "the controller is the suspect, not confirmed" (owner,
  2026-11-05). No run has separated "the lookahead parameter" from "the law".

### AutoDRIVE fast min-time lap on Porto (2026-11-09, [PR #88](https://github.com/G-S-Rodrigues/GR-roboracer/pull/88))

Full table in [results.md](../results.md#autodrive-fast-minimum-time-lap).

- Lap 2: 7.40 s (stack) and 7.38 s (simulator), against TUM's estimate of 7.303 s. 0 collisions, no
  latch.
- Peak 4.44 m/s against the 4.5 cap; lap 2's mean was 3.97 m/s.
- Speed tracking costs about 1.4 % of the lap at today's caps (part 4 draft, 2026-11-10). **The caps
  bound the AutoDRIVE lap now, not the controller.**

### AutoDRIVE car limits that a controller would meet ([PR #93](https://github.com/G-S-Rodrigues/GR-roboracer/pull/93))

Measured in [results.md](../results.md#autodrive-car-identification). The parts that matter for control:

- Drive limit with today's full-throttle-spins controller: a_x = 5.38 - 0.273 v
  ([`limits_sliding.yaml`](../../config/vehicles/autodrive_roboracer/limits_sliding.yaml)). With slip
  held near 0.15 it could reach 8.15 - 0.273 v
  ([`limits_peak.yaml`](../../config/vehicles/autodrive_roboracer/limits_peak.yaml)).
- Peak braking is `null`: it needs partial-slip control, which `ThrottleLoop` lacks.
- Lateral grip depends on the corner radius (gotcha #46).
- The caps 4.5 m/s, 4.0 m/s^2 and 1.5 rad/s stay until the owner rules on raising them.

### The car's wheel odometry over-reads at launch (2026-11-11)

AutoDRIVE's encoder speed jumps to the command while the body accelerates at about 5 m/s^2. At launch
it over-runs truth by about 0.7 m at 2.9 m/s, and by about 3.4 m at 5.3 m/s. It shifted MOLA's Porto map
by about 0.67 m ([localization-slam.md](localization-slam.md)). A real car's wheel odometry does the
same during wheelspin. A launch ramp in the controller or the supervisor is one of the fixes on the
table (see Ideas).

### A wall hit with no TRACK_LIMIT and no collision (ACC-4040 amcl, 2026-11-11)

From the F-T0 ready gate's failure ([PR #95](https://github.com/G-S-Rodrigues/GR-roboracer/pull/95),
"Ready gate history"):

- amcl's estimate slid 2.40 m behind along Spielberg's s 64-108 straight
  ([localization-slam.md](localization-slam.md)). Pure Pursuit, driving on that estimate, turned about
  2 m late. Its steering went from -0.015 to -0.40 rad and the speed from 2.87 to 1.32 m/s.
  `REASON_COMMAND_LIMIT`/`CLAMP_STEERING` fired at 96.770 s.
- The body hit the wall at 97.03 s (s 111.27, d 0.81). The supervisor raised no TRACK_LIMIT, because
  it tests the CoG with half-width 0.15 and at a heading error the box's corner reaches the wall
  first.
- `racing_metrics` reported `collision_count` 0 and `minimum_wall_clearance` 0.208 m. Its collision
  threshold is 0.01 m on CoG-cast ranges, so it never sees a body collision. The runner's frozen-pose
  stop caught the crash.
- The drive stream never paused: every drive stamp came 10 ms apart, and the last status was
  `REASON_NONE`.
- The runner first labelled this crash "a latch", because it treated any non-NONE status as a latch.
  [PR #96](https://github.com/G-S-Rodrigues/GR-roboracer/pull/96) fixed that.
- An A/B gave F-T0 1 failure in 6 and main 0 in 5, with lap times of 121.49-121.63 s on the passes.

### t=0 first motion ([PR #95](https://github.com/G-S-Rodrigues/GR-roboracer/pull/95))

- First motion came at 1 tick (0.01 s) in 55 of 55 launches. The code predicts 2 ticks, and the
  mechanism is not established.
- The `/controller/drive` wait never bound in the session check: it took at most 12 ms.

### AutoDRIVE amcl fast lap on the Windows sim (b6, failed 3 of 3, [PR #99](https://github.com/G-S-Rodrigues/GR-roboracer/pull/99))

- The car ran 0.36-0.67 m wide of the headless lap on two corner exits while localisation was good
  (0.04-0.16 m). Why is open: estimate lag or the Windows build's dynamics (inferred).
- After a collision the sim's respawn kidnaps amcl, and the car re-crashes about every 0.5 s.
- Both STALE_INPUT latches (246 ms and 204 ms gaps) came from the dev container's wall clock stepping
  back (gotcha #45), so they were most likely false.

## Dead ends

- **No alternative controller has been tried.** Nothing below Pure Pursuit's law has been changed
  except the speed-scaled lookahead (built, off).
- **A sim-side matched `/drive` count to prove the start (P2, F-T0) was rejected.** On Fast DDS the
  sim's `count_publishers("/drive")` reads the same participant database as its other count, so the
  check proves nothing. A command reaching the sim before release is detected, not proven
  ([PR #95](https://github.com/G-S-Rodrigues/GR-roboracer/pull/95)).
- **Reading b6 as "amcl slides at speed" was wrong.** The legwork showed amcl held 0.04-0.16 m. The
  wide corner exits and the respawn kidnap were the cause
  ([PR #99](https://github.com/G-S-Rodrigues/GR-roboracer/pull/99)).
- **Raceline part 5 at first rejected `lookahead_gain > 0` under `curvature_schedule`, and dropped
  that rejection** (2026-10-21). The "circular" reason applies only to the line's v, not to measured
  speed. Bringing it back is a one-check change.

## Ideas not yet specified

### Controller progression (backlog; survey [§6](../research/autonomous-racing-open-source-2026.md#6-mpc-ultimately-target-mpcc))

Pure Pursuit (speed from the trajectory) → kinematic tracking MPC → dynamic MPC → MPCC. "Each one
added, never replacing the last" ([ADR 0005](../adr/0005-algorithms-are-added-not-replaced.md)): a
`controller:=` launch argument with Pure Pursuit as the default, and each new one as an `alternatives`
entry, first `status: experimental`.

### High-speed controller for the fast car (owner, 2026-11-05; owner to spec)

> `f1tenth_gym_limits` (20 m/s) DNFs on every line and seed in the nightly SIM-5050 report ... The lines
> verify (min-time clearance 0.455 m, TUM estimate 32.4 s), so the controller is the suspect, not
> confirmed.

Per ADR 0005 it is added beside Pure Pursuit. Follow-on once a lap completes: the min-time
demonstration video on the fast car (deferred V2b).

### Part 4 draft (2026-11-10, read-only draft, nothing decided)

"Part 4: Controller work" of the 2026-11-08 next-steps run came after parts 1-3 (all merged: PR #88,
#87, #90). The draft set out four scopes. None adds a quality gate.

| Scope | What | Size | Needs |
|---|---|---|---|
| **A** (recommended first, PR 4a) | Offline ROS-free module scoring a recording against the line it drove: cross-track RMS/p95/max, heading error, speed error (line v vs measured, `/drive` vs measured), clamp counts. A `controller` role in `benchmark.py` reusing `planning_run` on Spielberg for `default` and `gym_limits`. Rows are Pure Pursuit configurations (for example `lookahead_gain` 0 against a speed-scaled value) until a second controller exists. Optional `controller:=pure_pursuit` argument and an AutoDRIVE fast row. | 7-9 TDD steps | part 3 merged |
| **B** (fallback if no solver dependency) | Stanley, experimental, at the front axle (needs lr in the vehicle file). | 9-11 | A |
| **C** (recommended second, 4b) | Kinematic tracking MPC, experimental. Steering and steering-rate constraints matched to the supervisor caps, explicit 1-frame input delay (gotcha #42), speed preview from the line's v. Either a pinned solver dependency that exists on Humble and Jazzy, or a small condensed box QP on Eigen. | 14-18 | A's tables |
| **D** (later, own part) | A traction-aware speed loop beside `ThrottleLoop`: slip limiting from encoder vs IMU speed, modulated braking, tuned gains. | 8-11 + live sessions | 1L increments 1-2 and a cap ruling |

Points the draft recorded:

- Scope A verifies the metric module on synthetic recordings (a line shifted 0.2 m reads 0.2 m). Its one
  live check is an equivalence: on the centerline, line-relative p95 must equal `racing_metrics`'
  centerline p95.
- Without ground truth on the real car, the same module runs on `/localization/odom` labelled "relative
  to the estimate". Lap-to-lap repeatability and encoder vs commanded speed are the other checks. For
  an MPC, add the one-step prediction residual and solver status / iteration counts, counted and never
  gated. For Stanley, add steering jitter (std of Δsteer).
- The trap in A is turning the `gym_limits` DNFs into a hunt. A configuration row answers "parameter or
  law?" by measurement, and the scope stops at "the table shows it".
- An MPC's tuning must stay track-independent, since the final gives about 3 days on an unseen track.
- A kinematic model is wrong past the grip limit, which is where `gym_limits` drives. A dynamic MPC is
  the step after C.
- The competition-critical path (the estimator lap on a SLAM map, the mapping driver, a Humble
  submission image) may outrank part 4. Qualification rewards 10 clean laps, not a faster controller.

### Smaller ideas

- **A progress window on the closest-point search.** The global search can jump to the other branch on
  a track that passes over itself, one of CLAUDE.md's real-car cases. This comes from reading the code;
  it is not measured.
- **Input-delay compensation** for AutoDRIVE's one-frame command lag (gotcha #42). The devkit's frame
  median is about 18 ms.
- **A launch ramp**: rate-limit acceleration in the controller or the supervisor, to cut launch
  wheelspin and the odometry over-read (2026-11-11 wheelspin open topic, option 3; it also spares a real
  car's tyres).
- **Raise the AutoDRIVE caps** (4.5 / 4.0) once 1L's numbers are in. This is an owner ruling and Scope
  D's precondition.
- **A first-class held start** on the AutoDRIVE server or the supervisor, as the gym has. Planner and
  reviewer recommended "not now; revisit at b7" (2026-11-11). Adding it changes the capability contract.
- **Input age on a steady clock** for wall-clock simulators, so a dev clock step cannot latch
  STALE_INPUT (b6 follow-up, [PR #99](https://github.com/G-S-Rodrigues/GR-roboracer/pull/99)).
- **A mapping driver**: a follow-the-gap or wall-follow controller that needs no centerline, added
  beside Pure Pursuit to drive the mapping lap on an unseen track. This is the provisional answer to
  the AutoDRIVE part 11a question Q2, which the owner has not confirmed; the draft called it "the
  costliest assumption". No follow-the-gap code exists.
- **Real car: convert wheel speed from the rear axle to the CoG** before publishing `base_link` odometry.
  The sim's twist is the speed's magnitude, not v·cos β (accepted, raceline part 5 E16).

## Open questions

- **Part 4 is unanswered.** The draft's 11 questions to the owner are open: scope and order (A then C,
  A then B, A alone, D first); priority against the race-time path; whether benchmark rows may be
  parameter sets, which stretches ADR 0008 §5, and how the gain is passed; whether line-relative
  tracking is offline or a `racing_metrics` field, and whether the centerline field is renamed
  "centerline offset"; whether a solver dependency is allowed and whether a tier-1 "solve time < 10 ms"
  check is a structural guard or reported only; whether the speed policy is shared with Pure Pursuit;
  whether the reference point is the CoG or the natural axle; the package name; which cars and tracks
  the benchmark covers; whether raising the caps is in scope; whether the mapping driver belongs here.
- **The 10 ms budget binds any MPC.** At time_scale > 1 the gym waits at most one 1x tick (10 ms) for
  each step's answer, and at 1x it never waits ([ADR 0006](../adr/0006-simulated-time-is-the-time-base.md)).
  A controller whose compute nears 10 ms changes the run with stale commands.
- **No QP/MPC solver is in any ROS image.** OSQP, acados and qpOASES appear nowhere in `docker/` or any
  `package.xml`. CasADi exists only in the offline raceline environment, which must never be imported
  (gotcha #24). The submission image is `FROM ros:humble`, so a solver must exist there too.
- **Editing `f1tenth_gym_limits.yaml`** (for example to set `lookahead_gain`) trips RACELINE-1395's
  hash of the file. A benchmark row needs a launch-time seam or a separate vehicle file.
- **The supervisor reads ground truth.** Its track-limit check needs a pose source that is legal at race
  time before it can run on the league's car or our own.
- **Why b6's car ran 0.36-0.67 m wide on corner exits** on the Windows sim is not known.
- **Which mechanism gives first motion at 1 tick, not 2.** A probe stamping `/drive` around `~/reset`
  would tell the two candidates apart (gotcha #19).
