# Simulation

The two simulator backends behind `racing_sim_adapter`: f1tenth gym_jax (stepped, owns `/clock`,
seeded noise) and the AutoDRIVE league simulator (free-running Unity build, wall clock). Also the
league car's vehicle file and its identified limits, the RViz vehicle model, and simulator ideas
(Foxglove, Isaac, Gazebo, sim-to-sim). Read before a grill, spec or plan that touches a simulator,
the bridge, the AutoDRIVE car's numbers or the URDF. How to launch, connect and record the AutoDRIVE
sim is in [`docs/agents/autodrive.md`](../agents/autodrive.md) and the WSL GUI route in
[`docs/agents/running-wsl.md`](../agents/running-wsl.md) ("AutoDRIVE GUI"); this file does not repeat
them. League rules and standings are in `competition.md`; the raceline optimizer that consumes the
limits is in `raceline.md`; SLAM and estimators on AutoDRIVE are in `localization-slam.md`; the
supervisor and controllers are in `control.md`.

## Current state

### The seam

- `racing_sim_adapter` is the contract: names in
  [`contract.hpp`](../../ros_ws/src/racing_sim_adapter/include/racing_sim_adapter/contract.hpp), the
  ADAPT-* conformance suite in [`conformance.py`](../../ros_ws/src/racing_sim_adapter/test/conformance.py).
  Each backend declares capabilities in its own `capabilities.py` (clock `owns_clock` /
  `free_running`, reset `seeded_pose` / `initial_pose_only`, `step_mode`, `time_scale`, `start_held`),
  and the suite selects by them ([PR #46](https://github.com/G-S-Rodrigues/GR-roboracer/pull/46)).
  A test whose capability is absent prints `NOT APPLICABLE <reason>` and returns; ADAPT-2150 pins the
  expected not-applicable set per backend (empty for the gym).
- [ADR 0006](../adr/0006-simulated-time-is-the-time-base.md) was amended: the simulator owns the
  clock when it has one. A `free_running` source (AutoDRIVE, a real car) runs the stack on
  `use_sim_time:=false` and its own stamps.
- [ADR 0001](../adr/0001-simulator-action-contract.md) holds upstream for both: the controller emits
  speed and steering angle. AutoDRIVE's throttle comes from a speed loop inside its backend.
- `config/reference_stack.yaml`: `simulator` is `f1tenth_gym_jax`; `autodrive` is a listed
  alternative, verified by the recorded-stream tests SIM-3610 / SIM-3620 because no gate has the sim.
  Which simulator becomes the default is open (owner, 2026-10-25: decided "after AutoDRIVE lands, on
  capabilities and on which one better improves the stack").
- The conformance rows run under their own namespaces (`/test_adapt_gym` with `/clock` remapped,
  `/test_adapt_autodrive`; [PR #69](https://github.com/G-S-Rodrigues/GR-roboracer/pull/69), gotcha 39).

### f1tenth gym_jax

- [`racing_sim_gym_jax`](../../ros_ws/src/racing_sim_gym_jax/CLAUDE.md): a ROS-free `backend.py`
  under a ROS shell `node.py`. It owns `/clock`, takes `time_scale`, `~/reset` (seeded) and
  `~/step_mode`, and declares every capability. Above 1x it lock-steps on `/drive` (SIMJAX-1070).
  Simulated time base: [PR #6](https://github.com/G-S-Rodrigues/GR-roboracer/pull/6).
- Ground-truth split and seeded noise ([PR #7](https://github.com/G-S-Rodrigues/GR-roboracer/pull/7)):
  `/scan` and `/odom` are noisy, `/ground_truth/{scan,odom}` exact. `noise.py` draws from one
  generator seeded by the run seed. Scan: gaussian `sigma_m` 0.03 (Hokuyo UST-10LX repeatability),
  `dropout_probability` 0.0 (no datasheet value; a dropout reads `+inf`). Odometry is an integrated
  random walk plus a per-run calibration error (`yaw_scale_sigma` 0.014, `distance_noise_density`
  0.0144 m/sqrt(m), `yaw_noise_density` 0.0248 rad/sqrt(rad), from F1TENTH's VESC calibration
  procedure; `config/scenarios/spielberg.yaml`). A per-sample jitter would give bounded error and
  zero estimator corrections (SIMJAX-1020).
- `/imu` linear acceleration is (dv/dt, v·omega, +g) with scenario `noise.imu` off by default
  ([PR #86](https://github.com/G-S-Rodrigues/GR-roboracer/pull/86)).
- `base_link` is the CoG at z = h = 0.074; `/scan` is cast at the lidar mount, `/ground_truth/scan`
  from `base_link` ([PR #24](https://github.com/G-S-Rodrigues/GR-roboracer/pull/24), gotchas 26, 37).
- `sim/` is the separate headless JAX uv project ([ADR 0003](../adr/0003-sim-outside-ros-workspace.md)).
- **The gym still loads its bundled Spielberg, not ours.** In `dev` it searches
  `/usr/local/lib/python3.12/dist-packages/maps` before `F1TENTH_GYM_JAX_MAP_DIR` (which `backend.py`
  only sets), so `Spielberg` is upstream's 2000x2000 PNG at 0.05796 m/px, never the committed
  1344x2085 at 0.05 m. No Spielberg lap has read the committed gym PNG. For Porto and
  `analytic_circle` the gym tries a plain-HTTP download first. [ADR 0009](../adr/0009-track-data-lives-in-gr-roboracer-data.md)
  defers the fix to a "part G1"; it changes every Spielberg lap number, so it is a reviewed golden diff.

### AutoDRIVE

Issue [#44](https://github.com/G-S-Rodrigues/GR-roboracer/issues/44) (open). Launch, routes, front
end B and the first drive: [`docs/agents/autodrive.md`](../agents/autodrive.md). What it is built from:

- **Images by digest** ([PR #48](https://github.com/G-S-Rodrigues/GR-roboracer/pull/48)):
  [`config/autodrive/simulator.yaml`](../../config/autodrive/simulator.yaml) is a compose fragment
  naming `autodrive_roboracer_sim` and `autodrive_roboracer_api` at `2026-iros-practice`; the compose
  services `autodrive-sim` (headless `-batchmode -nographics`) and `autodrive-devkit` extend it.
  Its header lists everything an edition bump must redo (fixture, `identified.yaml`, lateral and
  corner-sweep recordings, `tests/autodrive`). The sim image is Ubuntu 18.04 + CUDA 10.1 (not 22.04
  as the docs repo says); the Unity build is dated 2026-03-06.
- **One ROS-free core, two front ends** (owner, 2026-10-25, option 3): `racing_sim_autodrive`
  ([CLAUDE.md](../../ros_ws/src/racing_sim_autodrive/CLAUDE.md)). `core.py` decodes a frame
  (tolerating one without lidar), `lidar.py` the gzip+base64 ranges (beam count from the array),
  `control.py` the throttle loop (feedforward v / gain plus trim, on encoder speed), `bridge.py`
  `BridgeSession` (reset handshake, CoG ground truth, dead-reckoned odometry, `StampClamp`,
  `StallWatch`, `dt_source` = `lap_time` (default) or `receipt`). Front end A is `server_node.py`
  (our Socket.IO server, python-socketio 4.2.0 / engineio 3.13.0 / gevent,
  [PR #61](https://github.com/G-S-Rodrigues/GR-roboracer/pull/61)); front end B is `devkit_node.py`
  over the unmodified official bridge ([PR #71](https://github.com/G-S-Rodrigues/GR-roboracer/pull/71)),
  with `devkit_standin.py` as its CI stand-in. Only B is the race path (`competition.md`).
- **`/odom` under AutoDRIVE** is encoder wheel speed integrated with the sim's exact yaw rate, and it
  is re-anchored at the true pose on every reset (`bridge.py` `integrate_odometry`, ~411-443). So it
  has a perfect gyro and no reset jump; the throttle loop closes on the same encoder speed and cannot
  see wheelspin.
- **Vehicle files** ([PR #55](https://github.com/G-S-Rodrigues/GR-roboracer/pull/55)):
  [`autodrive_roboracer.yaml`](../../config/vehicles/autodrive_roboracer.yaml) with
  `autodrive_roboracer/source.yaml` (values read from the Unity prefab, scene and guide, each with its
  anchor), `identified.yaml` (written by [`tools/autodrive_identify.py`](../../tools/autodrive_identify.py)
  from `identification/2026-iros-practice.jsonl.gz`; CHECK-1780 re-derives it), `server.yaml` (the
  server's own values; BRINGUP-1100 pins vehicle files to the default's keys). Wheelbase 0.324 for
  dynamics ([PR #78](https://github.com/G-S-Rodrigues/GR-roboracer/pull/78)). The fast car
  `autodrive_roboracer_fast.yaml` caps speed 4.5 m/s and acceleration 4.0 m/s^2, steering rate 1.5
  unchanged ([PR #88](https://github.com/G-S-Rodrigues/GR-roboracer/pull/88)). Both AutoDRIVE files set
  `stale_input_timeout_ms` 200, the gym's stay 100 ([PR #99](https://github.com/G-S-Rodrigues/GR-roboracer/pull/99),
  owner call #27).
- **Limits by radius** ([PR #93](https://github.com/G-S-Rodrigues/GR-roboracer/pull/93)):
  `autodrive_roboracer/limits_peak.yaml` and `limits_sliding.yaml` from the lateral schedule and the
  corner-sweep recordings (CHECK-2670). "Nothing loads this file until the owner rules"; the fast
  tables still carry a_y 9.81 x 0.98 = 9.61 (gotcha 46).
- **Tools and scripts:** `tools/autodrive_record.py` (stand-in recorder, Engine.IO v3 pins),
  `tools/autodrive_frame_measure.py` (refits `config/autodrive/porto_frame.yaml`),
  `scripts/autodrive_qualify.sh` (live cases, `--sim container|windows`), `autodrive_route.sh` +
  `autodrive_relay.py` (Windows sim to our server), `autodrive_gui.sh` (devkit for the GUI),
  `scripts/windows/autodrive.ps1` (click helper).
- **Tests:** tier 1 `SIMAD-1xxx`, tier 2 `SIMAD-2xxx` against the recorded fixture
  `ros_ws/src/racing_sim_autodrive/test/fixtures/porto_stream.jsonl.gz`; SIM-3610 / SIM-3620 replay it
  through the whole launch; `tests/autodrive/` holds the local live and devkit cases SIM-3710..3781
  (bridge equivalence, stand-in fidelity, qualification, fast lap, corner sweep, survey and MOLA
  maps, fast lap on amcl and MOLA). No gate runs a live sim.
- **Porto** ([PR #45](https://github.com/G-S-Rodrigues/GR-roboracer/pull/45), moved to the data repo
  by [PR #92](https://github.com/G-S-Rodrigues/GR-roboracer/pull/92)): the official `Porto.pgm`
  (BSD-2) and a track extracted from it, regenerated clockwise in
  [PR #74](https://github.com/G-S-Rodrigues/GR-roboracer/pull/74). A survey map and a MOLA map from a
  live lap landed in PR #99.
- **Held start:** `tests/autodrive/_porto_drive.py` starts the car under the supervisor's emergency
  stop, resets, starts the mapper on the parked car, then releases (PR #99, A6). It exists because of
  the launch wheelspin below.

### Vehicle visual model

- [`racing_vehicle_description`](../../ros_ws/src/racing_vehicle_description/CLAUDE.md) holds two
  RViz-only URDFs, picked by `urdf/descriptions.yaml` (DESC-1090): `f1tenth.urdf` for the gym cars,
  `autodrive_roboracer.urdf` for both AutoDRIVE files. No Gazebo, plugin, sensor or transmission tags
  (DESC-1030).
- The AutoDRIVE model ([PR #62](https://github.com/G-S-Rodrigues/GR-roboracer/pull/62)) is AutoDRIVE's
  BSD-2 `F1TENTH.fbx` imported at full resolution (owner, 2026-10-30: "use the full 47 MB") by
  `scripts/import_autodrive_meshes.py`: 22 binary STLs, 937,506 triangles, ~46.9 MB. Attribution in
  `meshes/NOTICE.md`. DESC-1040..1080 check CoG, lidar, tyre centres, overhangs and length against
  the vehicle file.
- Drawn and TF wheel positions stay 0.33 m apart (the image's own TF, 0.09 + 0.33 + 0.08 = 0.50 m
  body) while the dynamics wheelbase is 0.324 (prefab Ackermann field). With lr 0.15532 the TUM
  model's lf is 0.16868, 6 mm short of the PhysX front-wheel position 0.17468.
- The manual left-turn TF-sign check in the package's CLAUDE.md is owed after any change there; an
  agent cannot do it.

## What we measured

### The AutoDRIVE build (2026-10-25 spike, image `2026-iros-practice`)

From the grill of #44 (numbers live there only), confirmed where noted by later PRs.

| Fact | Value |
|---|---|
| Protocol | one Socket.IO event `Bridge` each way; sim is the client, bridge the server on :4567; no timestamp, no step counter, no lockstep control; commands `V1 Throttle`, `V1 Steering`, `V1 Reset` only |
| Throttle | open-loop motor torque; 0 brakes (brake torque = MotorTorque); steady speed 24.86 m/s per unit (gain falls 25.08 to 24.81 over 0.02-0.12) |
| Steering | [-1, 1] maps to -steer x 0.5236 rad, slew 3.2 rad/s, sign +1, Ackermann |
| Encoders | accumulated wheel angle in rad (third-party "echo the throttle" refuted) |
| Lidar | 1081 ranges (+-2.35619 at 0.004363323; the guide's 1080 is the interval count); first headless message has none |
| Pose | `V1 Position` z at rest 0.059 = wheel radius, so the rear-axle centre; CoG height 0.07354 (gym's 0.074 holds) |
| Noise | none in any sensor; covariances constant 0.0025; telemetry rounded to 1e-4 |
| Physics | fixed step 0.001 s; no seed; rigidbody 3.47 kg + 4 x 0.109 wheels = 3.906; inertia PhysX-computed; drag 0.273 |
| Unresolved | prefab MotorTorque 85.6 vs scene 428; guide top speed 22.88 with no clamp in the source |
| Headless cost | 2 CPU cores, 330 MB; runs with no GPU (so a 4-core GitHub runner could host it) |
| Tracks | the practice build carries Porto (active), Berlin, SRL 2024 CDC, SRL 2024 IROS, SRL 2025 CDC-TF, SRL 2025 ICRA; no CLI switch; GUI switch unverified |
| Teleport | Unity has a `V1 CoSim` pose path the shipped bridge never sends (untested) |

The Windows practice zip is not byte-identical to the Linux image (level0, assets, boot.config
differ; build stamps about 3 h apart). The Windows build renders with Direct3D 12 on the RTX 4060
(`Player.log`), the Linux build ships only Vulkan.

### Car identification (part 4, 2026-10-26, PR #55; 3 repeats, 20839 frames, load 5.6)

- Porto has at least 7.9 m of clear straight from spawn. The highest steady speed was 2.977 m/s at
  throttle 0.12; 4.92 m/s at 0.2 was not steady. Top speed is not measurable on Porto.
- Braking depends on speed: 8.4 m/s^2 from 0.5 m/s, 5.8 from 3, 6.0 from 4.9 (mean 6.34 over 21
  events). The 8.4 is likely the 90-10 % method catching a one-frame snap to rest (unproven). The IMU
  trace fits about 5.40 + 0.273·v braking and 5.37 - 0.273·v accelerating (0.273 is the prefab's
  linear drag), with a 7.0 transient at 4.1-4.3 m/s (inferred: wheelspin).
- Steering limit 0.523633 rad (n 12), rate 3.2 rad/s (n 295), effective wheelbase 0.3388.
- `V1 Linear Velocity` is the CoG's: lateral -0.0743 at yaw rate -0.4646 puts it ~0.16 m ahead of
  the rear axle (gotcha 36).

### Lateral grip by radius (part 1L, 2026-11-11, PR #93)

| Radius (m) | Open-loop peak | Post-cliff | Closed-loop reached | Fast line's v²·kappa there | Proposed a_y (not adopted) |
|---|---|---|---|---|---|
| 0.56, 0.78 | not reached | not reached | not reached | no point of the line | none |
| 1.21 | 5.10 | 4.89 | 4.55 (n 10) | 8.54 at 3.08 m/s | 4.3 |
| 2.05 | 6.46 | 5.35 | none | 9.00 at 4.15 m/s | 5.1 |
| 3.08 | 7.33 | 5.50 | 6.40 (n 3) | 7.73 at 4.50 m/s | 5.2 |

The sweep stopped at its fourth rung on the sim's collision count. Every committed table (9.61) and
the fast line's own demand exceed every measured value at the radii reached. That the line is not
lateral-feasible is an inference: the open- and closed-loop values come from other drives. The
corner-sweep line generator and the rung-combining script were throwaway, so the recording cannot be
re-produced from the repo alone (`simulator.yaml` header).

### Frame rate and gaps (all AutoDRIVE, all measured)

| Date, setup | Median period | Tail | Source |
|---|---|---|---|
| 2026-10-25, headless, probe replying at once, no GPU | 20.0 Hz steady | | #44 grill |
| 2026-10-25, headless recorder, host load 20-30 / 16 cores | 18.2 ms (~55 Hz) | 10-90 % 11-55 ms, max 145 ms | gotcha 31 |
| 2026-10-30, GUI build via devkit (owner connected) | lidar 48 Hz, imu 34 Hz | imu max gap 70 ms | `running-wsl.md` |
| 2026-10-30, part 7 live, Windows GUI to our server through socat without `nodelay` | 68 ms flowing (11.5 Hz average) | max 1.106 s | fixed in [PR #68](https://github.com/G-S-Rodrigues/GR-roboracer/pull/68) |
| 2026-10-30, 8b, headless, bare recorder, 300 s, load 12.7-15.2 | 6.98 ms | p99 40.1 ms, max 104.8 ms, 1 gap >= 100 ms | #44 slot log |
| 2026-11-01, headless, bare Socket.IO counter (no ROS), 15-20 s, healthy | 4.7-6.6 ms (126-192 Hz) | max 52-76 ms (one 107 ms) | part 11a / W1 rate checks |
| 2026-11-01, same check, degraded for ~1 h (below) | 94-155 ms (6.6-11.1 Hz) | max 179-234 ms | part 11a rate checks |
| 2026-11-02, bare probe at rest, 30 s, 2 launches, load 0.7-2.3 | Windows GUI 13.1 ms (~69 Hz); headless 6.8-7.8 ms | Windows max 56-74 ms, 0 gaps >= 100 ms; headless max 149-168 ms, 7 and 21 gaps >= 100 ms | GUI-vs-headless comparison |
| 2026-11-11, headless, 395 s, load 0.9-2.8 | 5.4 ms parked, 5.6 driving (~180 Hz) | max 73.7 / 59.5 ms, 0 gaps >= 100 ms | stall-type measurement 1 |
| 2026-11-11, same with 3 busy loops, load 3.8-6.4 | 15.1 parked, 16.6 driving (~62 Hz) | max 87.6 / 97.6 ms, 0 gaps >= 100 ms | stall-type measurement 2 |
| 2026-11-11, Windows route, parked, ~4 min | 13.8 ms | p99 28.4, one 106.1 ms gap | 1V-fix-b spike |
| 2026-11-11, full stack + MOLA | | 127 and 132 ms (both latched at 100 ms) | PR #99 |

- **A gap is a transport delay, not a Unity stall.** Under load the lap clock kept running at 0.98 sim
  s per wall s and the car moved v x gap during a gap (gap 84.4 ms, lap delta 89.6 ms, travel
  0.087 m). The car drives blind on the last command through it, which is what the STALE latch is
  for. This decided owner call #27 (200 ms, AutoDRIVE files only; the numbers are in
  [`docs/agents/autodrive.md`](../agents/autodrive.md)).
- **The supervisor ages inputs by arrival time on its node clock** (`node.cpp:91-92`), not by the
  command's `header.stamp`. Only `~/clear_emergency_stop` releases a latch, and nothing calls it.
- **Headless lap-clock steps stop at exactly 100 ms** (146 frames in one recording; Windows max 76 ms),
  and headless lap time ran 0.967-0.995 of wall time. Inferred, not read from the build: a Unity
  maximum delta time.
- **Stall watch:** our server publishes about 9 messages per frame (two 1081-range scans) and replies
  synchronously; the handler itself takes p50 1.1 ms, p99 2.0-2.5 ms, max 4-13 ms.
- **The headless sim once fell to 7-11 Hz on an unloaded host** (2026-11-01, after a Docker/WSL
  restart; load 3.4 still gave 9 Hz). The Unity main thread used ~70 % CPU for ~7 frames/s, about
  100 ms of CPU per frame against ~6 ms healthy. Ruled out: GPU (`GfxDevice: Null`, CPU lidar),
  compose or cgroup limits, memory pressure, network (0.15 ms round trip), Unity worker count (cpuset 4
  or 8 CPUs) and ROS (measured with none). It correlated with the laptop on battery (`PowerOnline False`,
  Balanced plan; observed, not proven): on AC it came back to 126-134 Hz, then dropped to 8.3 Hz once
  more within 10 minutes. The cause was never found. At that period every live run latched at start (the timeout was then
  100 ms), whatever the transport, so part 11a gated each run on a 15 s bare-rate check (>= 120 Hz,
  median <= 7 ms) before spending it.

### GUI (Windows) against headless (Linux) physics (2026-11-02, main ef3a24f)

72 valid runs, 3 launches per build x 3 repeats x 4 manoeuvres, indexed on `V1 Lap Time`. In the
regime Porto allows (up to 4.9 m/s, yaw rate up to 0.46 rad/s) the builds are the same physics
within their own run-to-run spread:

| Quantity | Windows | Headless |
|---|---|---|
| steady speed at throttle 0.12 | 2.9771 m/s (gain 24.809) | 2.9771 m/s |
| braking 90-10 % from 0.12 / 0.20 | 5.78 / 6.04 m/s^2 | 5.78 / 6.03 |
| stopping distance from 0.12 / 0.20 | 0.753 / 1.943 m | 0.757 / 1.949 m |
| steady yaw rate at steering +-0.3 (reported angle +-0.1571 rad) | +-0.4646 rad/s | the same |
| yaw-rate rise to 63 % | 35.1 ms | 36.4 ms |
| time 0 to 1 m/s, throttle 0.12 | 0.1940 s | 0.1963 s (ranges do not overlap) |
| command lag to reported throttle | 1 frame | 1 frame |

The one difference is a 2.0 ms later throttle response headless; build against rig (the recorders
ran in different places) is not separated. Whole-run pose is not comparable: a command lands on a
frame, so a long gap moves the brake frame (final position spreads 0.32-0.66 m within a build).
Grip limits, speeds above 4.9 m/s, lidar and camera were not compared.

### Launch wheelspin and encoder odometry (2026-11-11)

At launch the wheels reach the commanded speed at once while the body accelerates at about 5 m/s^2.
On recorded full-throttle launches the encoders ran 0.46 m ahead of truth at +0.1 s and 3.3-3.5 m
ahead at +1.3 s, then stopped growing; at steady speed encoder distance over truth is 1.007, so it is
slip, not wheel radius. Released at 2.74-2.99 m/s the model predicts +0.40 m at +0.19 s and ~0.74 m
flat from +0.55 s; MOLA's keyframe error was 0.35 m, 0.61 m and a best rigid fit of 0.67 m. MOLA
keeps it because it fuses `/odom` (`sigma_wheel_odom_linear_vel` 0.1) with `icp_prior_weight` 50 and
the start straight constrains x weakly (gotcha 18). PR #99's MOLA map carries it: best fit to truth a
0.750 m shift and 1.19 deg. After a reset with a full-throttle launch, 0.75-0.9 m of phantom travel
accumulates in the first ~0.7 s. Unproven: that the wheels also reach the command at once at 2.75
m/s (measured at 5.1-5.5 m/s).

### The host's wall clock (2026-11-12)

SIM-3750 (ground-truth fast lap) failed 16 of 16 runs on this host with STALE_INPUT in the first
seconds, on main and on the last commit that had passed (2026-11-10), gaps 108-288 ms at load 1.8-6.1,
also in a throwaway container from dev's image. The WSL VM clock runs slow and systemd-timesyncd steps
it forward ~92-122 ms every 30 s (offset +203 ms, growing). Under AutoDRIVE the server stamps and the
supervisor's staleness check both use wall time, so a forward step reads as an input gap with no real
pause. The stamp clamp (gotcha 45) covers backward steps only. The proposed host remedy was
`wsl --shutdown`; the code remedy is a steady clock for staleness when there is no `/clock`.

The clamp itself was never exercised live: three valid 40-lap runs on 2026-11-11 saw 0 clamped
frames and 0 step-backs (an unclamped probe had seen 9 in 300 s). SIMAD-1490 and SIMAD-2310 verify it
by test only; SIM-3760 reports `stamp_step_backs` per rung.

### Runs on the AutoDRIVE car

Qualification rehearsal 2026-10-31 (13.73 s laps, score 137.3 s, 0 collisions) and the fast min-time
lap 2026-11-09 (7.38 s, ground truth): `competition.md` and [`docs/results.md`](../results.md). The
amcl fast lap on the Windows sim failed 3 of 3 (PR #99): latches on apparent 204 and 246 ms gaps and
collision respawns that kidnap the estimator (`localization-slam.md`).

Part 11a's mapping driver (2026-10-31 to 11-01, branch `44-autodrive-sim-11a` at bd20fcc, unmerged,
parked 2026-11-08): a Follow the Gap controller (`controller:=follow_the_gap`, held by
`controller_hold:=true` and released by setting its `hold` parameter) drove five live Porto runs.
With `max_range` 4.0 it steered for the deepest beam of the widest gap, chased a side alcove (2.77 m
deep against 2.3 m ahead) and stuck 0.3 m from the right wall in TRACK_LIMIT. With `max_range` and
clearance 2.0 it lapped twice (sim laps 19.43 and 18.23 s, 0 collisions). In 2 of the 5 runs the
emergency stop latched at launch. `/odom` paused about 3 s at every reset (longest period 2.97 s). Unproven explanation: Follow the Gap
publishes only when a scan arrives, so no command reaches the supervisor during the pause. The
proposed fix was a zero-command heartbeat on a timer while held.

### Gym determinism

SIM-3030's same-seed spread (lap 22.46 vs 22.67 s, tracking error 0.157 vs 0.334 m) came from the
deterministic t=0 releasing before every consumer matched the sim's endpoints. Fixed by
[PR #95](https://github.com/G-S-Rodrigues/GR-roboracer/pull/95) (F-T0), not by tolerance. ADAPT-2090
(time_scale 5 must give `/clock` above 1.5x wall) fails on a loaded host because the 2 ms step budget
collapses below 1x (measured 0.91x at load 18-21); the shared heavy lane
([PR #98](https://github.com/G-S-Rodrigues/GR-roboracer/pull/98)) is the mitigation, the assertion is
unchanged.

Before PR #95, SIM-3040 (analytic_circle, seed 42) ran 120 times on 2026-10-30, each started only on
an idle host. 116 laps were 22.82-22.84 s, and four (22.63, 22.74, 22.90, 23.03) failed the golden's
0.05 s lap tolerance. Maximum tracking error was 0.3530 m in nearly every run. A 12-run main-vs-branch
A/B on 2026-10-31 gave 22.83 s and 0.3530 m every time, with p95 tracking error at 0.1363-0.1369 m.
So "same seed, same lap" held to 0.01 s apart from rare outliers. Whether F-T0 removed those
outliers was not re-measured at this scale.

### Measured at bootstrap (gym, 2026-09-06..09, issue [#1](https://github.com/G-S-Rodrigues/GR-roboracer/issues/1))

Upstream f1tenth_gym_jax `1b4eb3f` with jax_pf `1b1417d`, read at source and run on CPU (jax 0.7.2):

- Action bounds: steering angle ±0.4189 rad, steering rate ±3.2 rad/s, speed -5 to 20 m/s,
  acceleration ±9.51 m/s². The order is `[steering, longitudinal]` in all four modes.
- The observation is one flat float32 vector per agent, 71 long (7 state plus 64 beams), starting
  `[s, ey, epsi, ...]`. It has no world x, y or psi, so `/odom` is built from `State.cartesian_states`.
- The seed picks the start pose: `backend.reset` passes `PRNGKey(seed)` to upstream's `reset`
  unchanged (`backend.py:147-153`), which draws s uniformly over the whole lap, ey within ±0.45 m
  (±0.3 times a 1.5 multiplier) and epsi 0. There is no start line.
- An env id without `max_steps` gets upstream's derived 90 s of simulated time. Spielberg, Porto and
  the survey scenarios set 30000; `contract_test.yaml` sets none.
- `timestep_ratio` is a Python loop unrolled into the jaxpr (four dynamics calls per RK4 substep), so
  compile time grows linearly with it and any change recompiles.
- Compile cost: a ~1.14 s stall in the first steps of an unwarmed CPU run, a warmed CPU compile of
  ~0.64 s in `dev`, and `gym_jax warmed in 0.782s` at the first bringup launch.

## Dead ends

- **The AutoDRIVE GUI in a container on WSL.** The Linux build ships only a Vulkan renderer; WSL gives
  containers no NVIDIA Vulkan; Mesa's D3D12 Vulkan (dzn) is in neither the 18.04 image nor host Mesa
  25.2 (only lavapipe, far too slow). `xvfb-run` makes Unity exit on start (unexplained). The Windows
  build (W1) is the route; W2 (dzn) was never tried.
- **Third-party claims (Barq Racing repo):** an 18 Hz tick, encoders echoing throttle and 1081 as an
  error were hypotheses; the spike measured otherwise (above).
- **The sim's lap time as a clock for the stack.** `V1 Lap Time` gives an accurate dt (gotcha 35), but
  the rules restrict `lap_time` at race time and the official bridge exposes no other sim clock. So
  feeding the supervisor a sim-time clock (option A, call #27) is blocked, and the `lap_time` dt
  source stays a front-end-A development default.
- **Curing a STALE latch by widening the timeout.** Ruled out except as a measured per-vehicle value
  (gotcha 19); the 200 ms is about 2x the measured sim-alone worst gap. Other options weighed and not
  taken: a recoverable STALE (on a real car it would resume driving after a stall with no operator)
  and judging by frame age (still wall-clock).
- **Front end A's executor in a thread beside the gevent hub.** `executor.spin()` under a `/drive`
  flood never blocks and starves the hub of the GIL, so replies drained at ~20/s and CI stalled.
  Fixed by `spin_once` plus a sleep (e13f86d, SIMAD-2080, [PR #66](https://github.com/G-S-Rodrigues/GR-roboracer/pull/66)).
- **Comparing builds by whole-run pose** (see the GUI-vs-headless section).
- **Identifying top speed on Porto:** not reachable (about 8 m of straight at ~25 m/s per unit).
- **The URDF look loop** (stopped 2026-10-11 at look features 0/14): looks in league runs come from
  AutoDRIVE's own car. `score_urdf.py` was not committed (85 % look-loop only; 3 of its 21 tier-A gates
  already red after PR #24); its useful gates became DESC-1040..1080. Two cosmetic leftovers in the
  gym model stay: motor meshes inside the rear tyres (`generate_f1tenth_meshes.py:660` still uses
  `HALF_TRACK - 0.004`) and `generate_chassis_stl.py` writing a mesh different from the committed one.
- **A Windows-side Claude session to record the GUI** (route B): it removes neither the clock offset
  nor the click helper and adds a second session, config and permission surface. `ddagrab` (a desktop
  rectangle) breaks the sim-window-only rule; `gdigrab` window mode returns black frames for a
  flip-model D3D app (inferred). Details in [`docs/agents/autodrive.md`](../agents/autodrive.md).
- **Our own car or Spielberg in AutoDRIVE** needs Unity 2022.3.52f1, the BSD-2 source (which uses the
  commercial Vehicle Physics Pro) and our own build; whether that source matches the binaries is
  unproven. Racing it in the league is a disqualification.

## Ideas not yet specified

- **Finish part 11** (owner, 2026-10-30): "use the map stack to map the actual track used in the
  AutoDRIVE simulator so that it runs. Create a centerline, then min curvature and speeds, then a min
  time, and run pure pursuit for the 3 laps. Start with one lap until it proves it works, then expand
  to 2 and then 3." Maps landed in PR #99; the centerline from the AutoDRIVE map, its lines, the 1/2/3
  lap gates on our own estimate and the GUI video remain. Coordinator's provisional answers (2026-10-30,
  owner away): a centerline-free mapping driver (Follow the Gap) because a centerline-fed mapping lap
  is circular; amcl first, MOLA later; headless commands on user pages, the GUI route on the agent
  page. A runner in `scenario_runner.py` must pass CHECK-2250 (start held, deterministic t=0) or
  review a widening of it. The Follow the Gap driver, its held start and the 11a tests are on the
  unmerged branch `44-autodrive-sim-11a` (Runs on the AutoDRIVE car); 11b-11e were planned, never started.
- **Modular simulators folder** (owner, 2026-09-30): "refactor it to be a virtual class and create a
  chield for each simulator ... a simulators folder with a readme", and "headless jax is one of the
  simulator include it in the same folder". The shape found then: a `SimulatorBackend` ABC whose axis
  is time ownership (stepped against free-running), one generic ROS shell, children under
  `ros_ws/src/simulators/`. Moving `sim/` there contradicts ADR 0003's letter and needs an amendment.
- **Foxglove** as the viewer for every simulator. League stewards used Foxglove layouts;
  `foxglove_bridge` is a Jazzy apt package.
- **Sim-to-sim validation:** the same autonomy launch against gym_jax and AutoDRIVE, compared (the
  survey's Stage 6 exit criterion, with AutoDRIVE in place of Gazebo). The same `autodrive_roboracer`
  vehicle in both simulators would make the default-simulator decision compare like with like.
- **Isaac Sim via RoboRacer Arena** (arXiv 2608.23040): tracks from occupancy maps, 256 parallel
  vehicles; a possible third backend and RL backend. No code found.
- **Gazebo:** only if AutoDRIVE does not cover the integration-simulator role.
- **System identification of the gym vehicle**, so velocity profiles use measured limits (the
  AutoDRIVE car has `identified.yaml`; the gym car has none).
- **Past final tracks** from each edition's `compete` image (2025-icra, 2025-cdc-tf, 2026-icra,
  2026-iros) need no Unity work. Berlin or an SRL track in the practice build's GUI could give the
  straight that top-speed identification needs (GUI track switching unverified).
- **Identification above 5 m/s:** an accelerate-then-brake manoeuvre on Porto's straight fits up to
  ~6.7 m/s; it needs a new `SCHEDULE_ID` in `autodrive_identify.py record` and a regenerated
  `identified.yaml`. Combined slip and peak non-sliding braking have no recording at all.
- **A dev-only teleport** through Unity's `V1 CoSim` fields, for kidnap tests on AutoDRIVE (the gym's
  teleport scenario covers the kidnap test until then).
- **A nightly AutoDRIVE leg:** headless runs on CPU on 2 cores; needs a scripted run that fits CI.
- **Mesh decimation** of the AutoDRIVE model if RViz or CI cost demands it (RViz draws it at 6-10 fps
  against 31 for the gym car). Decimation moves DESC-1070's rear edge (a 0.05 % quantile of chassis x)
  and CHECK-2030's triangle sum; the four wheel files are identical up to reflection.

## Open questions

- **How should `racing_metrics` see a contact under AutoDRIVE?** Its collision threshold (0.01 m) is
  below the lidar's 0.06 m `range_min`, so only the sim's counter is truth there.
- **Is `porto_frame.yaml` biased?** `tools/autodrive_frame_measure.py:27` passes `laser_x` 0.2733
  (rear-axle-relative) into a fit whose poses are the CoG; the CoG-relative mount is 0.1180. It
  affects only development ground-truth numbers.
- **Does the native-Linux GUI (W3) work?** Never run; `docs/running.md` has no AutoDRIVE window
  section until it does (`running-wsl.md`, "Linux window, unverified").
- **Why does headless have more long gaps than the Windows GUI**, and is the 2 ms headless throttle
  delay the build or the rig?
- **Which grip does the planner use?** The limits files are written but not loaded; an owner ruling
  decides between one conservative number (4.3 at R 1.21), a tyre refit to the by-radius data, or a
  fork change taking mue(curvature) (`raceline.md`). Radii under 1.21 m were never reached.
- **Do the 2026-iros `compete` physics match the practice build's?** And why was IROS 2026's
  qualification about 1.5 s faster (`competition.md`)?
- **May the AutoDRIVE meshes' vendor-derived parts be redistributed?** The FBX contains parts named
  for NVIDIA's Jetson Xavier NX and Hokuyo's UST-10LX with no provenance (owner ruling V1 D1 imported
  them under BSD-2). Fallback: the repo's Apache `hokuyo.stl` and drop the Jetson parts.
- **Should the AutoDRIVE body keep the sim's near-black colours?** `_BaseColor` 0.106 grey from the
  prefab; the real car has orange parts (owner, unanswered since 2026-10-30).
- **At what priority do the server, controller and supervisor run?** `dev` has no `CAP_SYS_NICE`, so a
  negative `nice` is a silent no-op; the evaluation host's load is unknown.
- **Where does SIM-3620's "stand-in stall" come from?** The replay test sometimes reports that the
  stand-in, not the stack, went silent past the timeout. One case: 167 ms with no `/ground_truth/odom` at frame 59,
  load1 ~14, 2026-11-05. The X1 hunt instrumented every hop (stand-in publish, devkit node, stack),
  but only its first control ran. That control named a planted 150 ms delay at hop 0, the stand-in's
  own publish, after one fix to the analyser. The owner parked X1 on 2026-11-08: "12/12 laps without
  latch suffice; reopen only if a stall shows while driving".
