# Open-Source Autonomous Racing: Technical Research and Recommended ROS 2 Architecture

**Focus:** F1TENTH / RoboRacer, with a path toward full-scale autonomous racing  
**Project objective:** a simulation-first ROS 2 racing stack for learning, experimentation, and eventual F1TENTH deployment  
**Report date:** 2026

## Technical conclusion

The proposed direction is strong, but four architectural decisions should change before implementation begins:

1. **Use ROS 2 Jazzy initially, not Lyrical.** Lyrical Luth is the newest ROS 2 LTS and is supported through May 2031, but the strongest racing stacks currently target Jazzy. Jazzy remains supported through May 2029, so it gives a substantially lower integration and porting burden while remaining modern. Treat Lyrical as a CI and migration target rather than making ecosystem porting the first project.
2. **Use F1TENTH Gym JAX as simulator number one, not Gazebo.** Add Gazebo when URDF, TF, sensor, collision, and ROS integration fidelity become important. Do not add MuJoCo early unless vehicle-dynamics research itself becomes a near-term objective.
3. **Use one monorepo containing many ROS packages**, rather than a separate Git repository for localization, planning, control, perception, simulation, and every other subsystem.
4. Replace `racing line -> MPC -> behavior` with a clearer hierarchy:

   **global trajectory -> tactical behavior -> local trajectory generation -> controller -> independent safety supervisor**.

The target architecture should be closer to a combination of **ForzaETH + UNICORN + TUM Autonomous Motorsport** than to a conventional Nav2-style autonomous-vehicle stack.

---

# 1. Strongest projects to study

| Project | Why it matters | What to take from it |
|---|---|---|
| [ForzaETH Race Stack](https://github.com/ForzaETH/race_stack) | Strongest complete open F1TENTH head-to-head reference found. Peer-reviewed, multiple competition successes, and demonstrated high-speed operation. | Overall architecture, state machine, opponent pipeline, overtaking planners, system identification, and operational tooling. |
| [UNICORN Racing Stack](https://github.com/HMCL-UNIST/unicorn-racing-stack) | One of the cleanest current full ROS 2 racing-stack references; Jazzy, x86, and arm64/Jetson are documented. | ROS 2 package decomposition, modern integration, planner/state-machine structure, and deployment portability. |
| [F1TENTH Gym JAX](https://github.com/f1tenth/f1tenth_gym_jax) | Current official fast simulator; deterministic, vectorizable, JIT-compatible, and multi-agent. | Primary algorithm/SIL backend and later reinforcement-learning backend. |
| [TUM global trajectory optimization](https://github.com/TUMFTM/global_racetrajectory_optimization) | Mature racing-line and minimum-time tooling. | Offline qualifying-line benchmark and track/trajectory conventions. |
| [Liniger MPCC](https://github.com/alexliniger/MPCC) | Canonical open MPCC reference descended from autonomous Formula Student work. | Controller formulation and progression beyond tracking MPC. |
| [TUM Graph-Based Local Trajectory Planner](https://github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner) | Full-scale racing local planner tested in Roborace-class operation. | Action-set architecture: keep, pass-left, pass-right, and feasible trajectory generation. |
| [BDEvan F1TENTH Benchmarks](https://github.com/BDEvan5/f1tenth_benchmarks) | Useful common experimental framework for classical, local-map, MPCC, mapless, and RL approaches. | Benchmark methodology rather than production architecture. |
| [TUM Autonomous Motorsport software](https://github.com/TUMFTM) | The most useful open architecture family for eventual full-scale and A2RL ambitions. | Full-scale decomposition, trajectory supervision, latency discipline, SiL/HiL, state estimation, and vehicle dynamics. |

## ForzaETH

This is the project to read first. It contains distinct state-estimation, perception, planning, state-machine, control, system-identification, and system-management components rather than concentrating all racing intelligence in one optimizer. The published system has real F1TENTH competition results and high-speed operation.

However, **do not simply fork it and begin modifying it**. The ROS 2 Jazzy branch warns that the published results came from the ROS 1 implementation and that the ROS 2 stack remains less tested and lacks some features, including SynPF integration, scan alignment, car-to-car synchronization, Bayesian optimization, and system identification. Use it first as an architecture and algorithm reference.

Important references:

- [ForzaETH race stack](https://github.com/ForzaETH/race_stack)
- [ForzaETH Race Stack paper](https://arxiv.org/abs/2403.11784)
- [ForzaETH technical report PDF](https://f1tenth.org/publications/ForzaETH.pdf)
- [ForzaETH publications](https://www.forzaeth.ch/categories/papers/)

## UNICORN

UNICORN is especially relevant because its ROS 2 Jazzy architecture is explicitly:

**perception -> tracking -> prediction -> planning -> state machine -> control**

Its planner is decomposed into global optimization, lane-changing, recovery, spline, and SQP-related packages. It has been verified on x86_64 and arm64/Jetson. Its preferred installation path has leaned on RoboStack/conda, although container support is also present; take its **software architecture** without assuming that its environment-management decision must be copied.

## A smaller competition implementation

The open [f1tenth-icra-race](https://github.com/vaithak/f1tenth-icra-race) stack is worth studying because it reports fifth place at the ICRA 2025 RoboRacer competition and is much easier to comprehend than ForzaETH. It uses LiDAR opponent estimation, adaptive-breakpoint clustering, filtering, spline planning, a racing state machine, and Pure Pursuit. It is an excellent implementation reference for the first head-to-head milestone.

## How to use these projects

No single repository should become the project wholesale:

- use **ForzaETH** as the main competition architecture and operational reference;
- use **UNICORN** as the modern ROS 2 organization and portability reference;
- use **TUM** for global optimization, local-planning structure, control, dynamics, and full-scale engineering discipline;
- use **Gym JAX** for rapid, deterministic experiments;
- use **BDEvan's benchmarks** to compare algorithms under common conditions;
- use **f1tenth-icra-race** as a small, readable first overtaking implementation.

---

# 2. Simulator choice: change the proposed order

## Start with F1TENTH Gym JAX

The official `f1tenth_gym_jax` is the most attractive starting point for this use case. It provides deterministic execution, explicit random-state handling, vectorization, JIT compilation, and simultaneous multi-agent simulation. These properties are valuable for regression testing, parameter sweeps, Monte Carlo evaluation, opponent scenarios, and later RL.

Write a **thin ROS 2 simulator adapter around Gym JAX** so the rest of the stack sees stable interfaces that can later be supplied by Gazebo or the real car:

```text
                 racing autonomy stack
                         |
              stable ROS 2 interfaces
                         |
                racing_sim_adapter
                 +-------+---------+
                 |       |         |
              Gym JAX  Gazebo    real car
```

The interface boundary is more valuable than selecting one simulator and allowing that simulator's message and model assumptions to spread through the entire stack.

## Gazebo

Gazebo remains useful, but add it when the project needs:

- URDF/xacro validation;
- TF and sensor-frame validation;
- realistic LiDAR placement and field of view;
- IMU and sensor-noise modeling;
- collisions and geometry;
- ROS-native sensor pipelines;
- hardware-like launch and configuration;
- testing of discovery, timing, lifecycle, and integration behavior.

Use Gazebo primarily as an **integration simulator**, not as the inner loop for every racing-algorithm experiment.

The older official [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) remains useful for interface conventions, maps, and examples, but its historical baseline and container design should not be inherited wholesale.

## MuJoCo

There is no comparably mature, community-standard F1TENTH ROS 2 MuJoCo stack that justifies making MuJoCo a day-one dependency.

Change:

> Gazebo first + MuJoCo early

to:

> **Gym JAX first -> Gazebo integration later -> MuJoCo only for a concrete dynamics experiment.**

For eventual full-scale work, studying [TUM Open-Car-Dynamics](https://github.com/TUMFTM/Open-Car-Dynamics) may teach more about racing vehicle modeling than maintaining a second generic physics-engine integration. It is a modern C++ framework with ROS 2 and Python integration, validated against AV-21 data.

## Isaac and other simulation environments

Isaac Sim is valuable when GPU sensor simulation, cameras, domain randomization, or perception-heavy work becomes central. It is not the best first dependency for a LiDAR-led classical racing baseline. Preserve the simulator-adapter contract so Isaac can be added without changing planning and control packages.

Reuse existing tracks, maps, and models where licensing permits. Keep a canonical track format and write importers for simulator-specific formats; do not let Gazebo world files or Gym map conventions become the domain model.

---

# 3. Localization and SLAM

The important distinction is:

**SLAM is mainly a track-mapping problem. Racing localization is a different problem.**

For initial map generation, [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) is the obvious modern ROS 2 baseline. It supports mapping, serialization, localization, continued mapping, and lifelong-mapping workflows.

Once the track is known, switch to a racing-oriented pipeline:

```text
LiDAR
   |
known-map localization ----------> global track pose
                                     s, d, heading error
IMU ------+
          +-- EKF/state estimator -> vx, vy, yaw-rate, etc.
wheel/VESC+
```

Keep **global localization** and **vehicle-state estimation** logically distinct. The localization system answers where the vehicle is on the track; the state estimator fuses fast measurements to estimate the dynamic state needed by the controller.

ForzaETH's **SynPF** is particularly interesting. Its evaluation reports robustness under wheel-slip conditions and lower onboard computation than its Cartographer-style alternative. For learning value, initially reuse or port an existing particle-filter or scan-matching implementation and later write a race-specific localizer once the rest of the stack provides a meaningful benchmark.

Recommended progression:

1. simulator ground truth for controller bring-up;
2. existing particle filter on a known map;
3. IMU/wheel/VESC state estimation with `robot_localization` or an equivalent EKF;
4. measure latency, jitter, relocalization behavior, and slip sensitivity;
5. add a SynPF-inspired race-specific localizer or scan-matching alternative;
6. implement confidence-aware degradation and recovery.

---

# 4. Small barrier changes: do not make SLAM solve everything

Use three distinct environmental representations:

```text
REFERENCE TRACK
persistent, stable
centerline + boundaries + localization map
            |
            +--------> global localization
            +--------> racing-line optimization

SEMI-STATIC DELTA LAYER
moved barriers / changed track pieces
confidence + persistence + timestamp
            |
            +--------> local planning

DYNAMIC OBJECT LAYER
opponents / temporary obstacles
            |
            +--------> prediction + local planning
```

A moved barrier should normally **not rewrite the map against which the particle filter localizes**. Temporary geometry belongs in a local or semi-static delta layer with timestamps, confidence, and expiry. If a change persists across sessions and is confirmed, it can be promoted through an explicit map-update workflow.

This separation matches successful racing systems: known track geometry helps classify and filter opponents, while local-map racing work shows that LiDAR-visible boundaries can be reconstructed online and used for high-performance local optimization without rebuilding a complete global map.

If a section genuinely loses usable global boundaries, the degraded mode can become:

**LiDAR -> local boundaries -> local centerline -> local MPCC/spline planner -> rejoin the global track later.**

This is stronger and easier to reason about than asking SLAM to continuously absorb temporary track changes.

---

# 5. Racing-line and minimum-time optimization

The classical global-planner direction is correct.

Start with [TUMFTM/global_racetrajectory_optimization](https://github.com/TUMFTM/global_racetrajectory_optimization). It implements shortest-path, minimum-curvature, minimum-time, powertrain-aware optimization, and friction-map support. Its high-fidelity minimum-time formulation uses optimal-control and nonlinear-programming machinery rather than only geometric curvature smoothing.

Progress as follows:

```text
centerline
   |
minimum-curvature line
   |
velocity profile
   |
system identification
   |
minimum-time trajectory
   |
MPCC / local planner
```

Do not spend the first month recreating TUM's optimizer. Use it as the reference implementation and benchmark.

Implement the infrastructure around it:

- canonical track representation;
- Frenet transformation and inverse transformation;
- track importer/exporter;
- map, boundary, and raceline consistency checking;
- trajectory ROS messages;
- track-boundary and vehicle-envelope constraints;
- visualization, lap-time, and constraint-violation metrics;
- versioned track and vehicle parameters.

Later, implement a smaller minimum-time optimal-control problem in CasADi to understand the formulation and compare its result with TUM's implementation.

---

# 6. MPC: ultimately target MPCC

A normal tracking MPC is a good milestone:

```text
minimize sum_k (lateral_error^2
                + heading_error^2
                + velocity_error^2
                + control_cost)
```

But it still asks:

> How accurately can the vehicle track this already-selected trajectory?

For racing, **Model Predictive Contouring Control (MPCC)** asks something closer to:

> How can the vehicle maximize progress around the track while remaining inside dynamic, actuator, track, and collision constraints?

MPCC explicitly incorporates contouring error, lag error, and track progress. [Liniger's implementation](https://github.com/alexliniger/MPCC) remains a key open reference from autonomous Formula Student racing. TUM's full-scale work also demonstrates the importance of model-based and robust predictive control close to vehicle limits.

Recommended progression:

**Pure Pursuit -> kinematic tracking MPC -> dynamic tracking MPC -> MPCC.**

For tooling, favor **CasADi + acados/HPIPM** for the eventual online controller and keep IPOPT primarily for offline optimization and prototyping. Measure end-to-end deadline performance; mean solve time alone is not enough. Record worst-case and high-percentile solve time, infeasibility rate, warm-start behavior, and fallback activation.

---

# 7. Behavior, obstacle avoidance, and overtaking

This is the largest recommended change to the proposed architecture.

Do **not** make MPC responsible for:

- whether to overtake;
- which side to pass;
- whether to follow;
- recovery;
- aborting an overtake;
- race-state rules.

Use this hierarchy:

```text
                       Global racing trajectory
                                 |
Opponent perception -> Prediction|
            |                    |
            +---------+----------+
                      v
               Tactical behavior
        KEEP / FOLLOW / PASS_LEFT /
        PASS_RIGHT / YIELD / RECOVER
                      |
               intent + constraints
                      v
             Local trajectory planner
       +---------+---------+---------+
       |candidate|candidate|candidate|
       +---------+---------+---------+
                      |
               trajectory selector
                      |
                  MPC / MPCC
                      |
             independent safety
                 supervisor
                      |
             Ackermann command
```

This resembles ForzaETH's decomposition and the pattern used by TUM's graph-based planner, which generates feasible alternatives such as **keep**, **pass left**, and **pass right** for higher-level selection.

The **independent safety supervisor** should validate trajectories and commands against:

- track limits;
- collision envelopes;
- time to collision;
- acceleration, steering, speed, and actuator-rate constraints;
- stale opponent or perception data;
- localization confidence;
- control-deadline failures;
- node health and heartbeat state;
- emergency-stop state.

It must be able to replace or clamp unsafe output without depending on the planner that produced it. This architectural idea transfers especially well to full-scale racing.

---

# 8. Opponent perception and prediction

For the first dynamic-racing version, avoid sophisticated AI.

Implement:

```text
2D LiDAR
   |
clustering
   |
remove / classify known track boundaries
   |
data association
   |
KF/EKF opponent tracker
   |
(s, d, vs, vd, covariance)
   |
short-horizon prediction
```

ForzaETH reports practical results from a classical LiDAR pipeline using adaptive-breakpoint clustering, track-relative filtering, and opponent-state estimation. The smaller ICRA 2025 stack provides an accessible implementation of the same family of ideas.

Then progress to GP-based prediction. [Predictive Spliner](https://github.com/ForzaETH/predictive-spliner) predicts opponent behavior and generates overtaking trajectories. [M-Predictive Spliner](https://arxiv.org/abs/2506.16301) extends the approach to multiple opponents and spatiotemporal planning.

Game-theoretic approaches such as [SGTP-Racer](https://github.com/zhouhengli/SGTP-Racer) and alpha-RACER are interesting after that, but they belong several milestones later because failures are substantially harder to diagnose once prediction and planning are jointly interactive.

---

# 9. Reinforcement learning

Leaving RL until later is the correct decision.

Gym JAX supports that strategy by providing a vectorized environment suitable for large experiment batches without changing the real ROS architecture. The classical stack should remain the safety fallback and the benchmark against which learned approaches are evaluated.

Try RL in this order:

1. parameter and weight tuning;
2. residual vehicle-dynamics compensation;
3. tactical overtake decisions;
4. trajectory-conditioned policies;
5. learned local planning;
6. only much later, end-to-end control.

Every learned component should have:

- a classical baseline;
- an offline evaluation dataset;
- deterministic scenario seeds;
- explicit observation and action contracts;
- out-of-distribution and uncertainty monitoring;
- a runtime fallback;
- sim-to-real validation gates.

Use [BDEvan's F1TENTH benchmarks](https://github.com/BDEvan5/f1tenth_benchmarks) as a reference for comparing classical, mapless, local-map, MPCC, and end-to-end approaches. The [TC-Driver paper](https://f1tenth.org/publications/TC_Driver.pdf) is also relevant to trajectory-conditioned reinforcement learning.

---

# 10. Repository and container architecture

Do **not** create this for a personal research stack:

```text
slam.git
planning.git
control.git
perception.git
simulation.git
...
```

Use a monorepo with replaceable ROS packages:

```text
autonomous_racing/
|-- .devcontainer/
|-- docker/
|   |-- Dockerfile.dev
|   |-- Dockerfile.runtime
|   `-- compose.yaml
|
|-- vcs/
|   `-- third_party.repos
|
|-- config/
|   |-- vehicles/
|   |-- tracks/
|   `-- scenarios/
|
|-- ros_ws/src/
|   |-- racing_interfaces/
|   |-- racing_common/
|   |-- racing_bringup/
|   |-- racing_sim_adapter/
|   |-- racing_sim_f110_jax/
|   |-- racing_vehicle_description/
|   |
|   |-- racing_state_estimation/
|   |-- racing_localization/
|   |-- racing_map_manager/
|   |
|   |-- racing_obstacle_detection/
|   |-- racing_opponent_tracking/
|   |-- racing_opponent_prediction/
|   |
|   |-- racing_global_planner/
|   |-- racing_behavior/
|   |-- racing_local_planner/
|   |-- racing_trajectory_selector/
|   |
|   |-- racing_controller_baseline/
|   |-- racing_mpc/
|   |-- racing_mpcc/
|   |
|   |-- racing_safety_supervisor/
|   |-- racing_metrics/
|   `-- racing_recording/
|
|-- tools/
|   |-- track_processing/
|   |-- raceline_generation/
|   `-- system_identification/
|
`-- tests/
    |-- scenarios/
    `-- acceptance/
```

The packages remain replaceable and independently testable, while changes to a message contract, simulator adapter, planner, and test can still happen atomically in one pull request.

Split repositories only when there is a real independent lifecycle: a reusable library with external consumers, a separate release cadence, access-control requirements, or a genuinely independent hardware product. Package boundaries—not repository boundaries—should provide most modularity early.

## Containers

Use containers for **environment and deployment boundaries**, not one container per ROS node:

```text
dev                    ROS + compilers + tests + tooling
fast-sim               JAX/CUDA dependencies
sim-3d (later)         Gazebo dependencies
runtime-f1tenth        minimal hardware runtime
```

The Dev Container should use the `dev` image. Keep production images smaller and pin dependencies. Generate an SBOM and record the exact image digest used for experiments.

Do not spend early milestones on Jetson cross-compilation. First make x86 simulation deterministic. Add native arm64 builds and CI after the software graph is stable. UNICORN's arm64/Jetson support is a useful reference at that stage.

---

# 11. What to reuse, fork, or build

| Component | Recommendation |
|---|---|
| VESC, LiDAR, and F1TENTH hardware | **Reuse** [f1tenth_system](https://github.com/f1tenth/f1tenth_system). |
| Fast simulation | **Reuse** Gym JAX; write a thin ROS adapter. |
| Gazebo model and worlds | **Reuse selectively** from established F1TENTH simulators; modernize only what is needed. |
| Tracks and maps | **Reuse** official F1TENTH tracks and TUM track data where licensing allows. |
| Map generation | **Reuse** `slam_toolbox`. |
| First racing localization | **Reuse or port** an existing PF; study SynPF. |
| State estimation | **Reuse initially** `robot_localization`; replace only where racing needs justify it. |
| Track/Frenet domain model | **Build yourself.** |
| Pure Pursuit or Stanley baseline | **Build yourself.** |
| Global minimum-curvature/minimum-time optimizer | **Reuse TUM**, wrap and benchmark it. |
| Opponent LiDAR detector/tracker | **Build yourself**, benchmark against ForzaETH and ICRA implementations. |
| Behavior/tactical planner | **Build yourself.** |
| Local Frenet/spline planner | **Build yourself**, compare with ForzaETH and TUM. |
| Tracking MPC | **Build yourself using solver libraries.** |
| MPCC | **Build yourself from Liniger and TUM references.** |
| Safety supervisor | **Build yourself.** |
| Metrics and scenario runner | **Build yourself.** |
| RL environment core | **Reuse** Gym JAX. |
| RL policies and experiments | **Build yourself later.** |

The physical interface is worth preserving because `f1tenth_system` exposes the established VESC, LiDAR, TF, and Ackermann conventions that the simulator adapter should mimic.

The best learning return comes from implementing the **domain glue and decision-making**—track representation, interfaces, metrics, behavior, local planning, safety, and controllers—while reusing commodity drivers, SLAM, solvers, and mature offline optimization tools.

---

# 12. Testing architecture

Keep Robot Framework, but use it only at the top of a layered test strategy:

```text
gTest / pytest
      |
ROS launch_testing
      |
deterministic racing scenario runner
      |
Robot Framework acceptance/specification tests
```

The **scenario runner** is the important piece. A test should not merely say `vehicle reached finish`; it should generate quantitative metrics:

- lap completion and lap time;
- collision count and collision severity;
- minimum wall clearance;
- maximum and 95th-percentile tracking error;
- minimum time to collision;
- localization error in simulation;
- solver latency, deadline misses, and infeasibility rate;
- control saturation and actuator-rate violations;
- opponent-detection precision/recall and track continuity;
- overtaking success, contact rate, and rule violations;
- recovery success and time to resume racing.

Robot Framework can then express behavioral requirements in readable form while the scenario runner owns simulation control and measurement. Example:

```robotframework
*** Test Cases ***
Vehicle Completes Baseline Lap Safely
    Run Racing Scenario    baseline_map    seed=42
    Lap Completion Should Be    true
    Collision Count Should Be    0
    Minimum Wall Clearance Should Exceed    0.08
    P95 Tracking Error Should Be Below    0.12
```

## Test determinism and evidence

Every simulation regression should record:

- code revision;
- container image digest;
- scenario and random seed;
- track and vehicle parameter versions;
- ROS bag or compact event log;
- metric summary;
- planner/controller configuration;
- CPU/GPU platform and timing statistics.

Use a small fast scenario set on every pull request and a larger seeded matrix nightly. Add hardware-in-the-loop and replay tests later. Treat rosbag replay as a first-class method for reproducing perception, localization, and timing failures.

---

# 13. Proposed improved runtime architecture

```text
SENSORS / SIMULATOR
 LiDAR   IMU   wheel/VESC   ground truth (test only)
    |      |       |
    |      +-------+----------------+
    |                               v
    |                       State estimation
    v                               |
Localization -----------------------+----> Vehicle state
    |                                      + confidence
    v
Track-relative pose (s, d, heading error)
    |
    +------------------------+
                             |
LiDAR -> obstacles -> tracking -> prediction
                             |
Global trajectory ----------+----------> Tactical behavior
                                            |
                                 intent + constraints
                                            |
                                  Local trajectory set
                                            |
                                   trajectory selector
                                            |
                                        MPC / MPCC
                                            |
                                  Safety supervisor
                                            |
                                   Ackermann command
```

## Cross-cutting services

The following should be independent of individual algorithms:

- time synchronization and timestamp validation;
- diagnostics and node-health monitoring;
- structured event logging;
- metrics and lap segmentation;
- configuration/version reporting;
- data recording and replay;
- emergency stop and command arbitration;
- experiment manifest generation.

## Interface principles

1. Use standard ROS messages for sensors and Ackermann commands where possible.
2. Create custom messages for racing-domain objects—track-relative state, boundaries, opponents, predictions, trajectories, behavior intent, safety status—not for generic geometry already represented by ROS.
3. Include timestamp, frame, source, validity horizon, and confidence/covariance where applicable.
4. Do not expose simulator-specific state to production planning and control interfaces.
5. Make latency and stale-data behavior explicit.

---

# 14. Staged roadmap

## Stage 0 — Foundations and repeatability

**Goal:** one-command deterministic development environment.

- ROS 2 Jazzy workspace in a monorepo;
- Dev Container and development/runtime container split;
- CI for x86 builds, linting, unit tests, and small simulations;
- canonical vehicle, track, trajectory, and scenario configuration;
- Gym JAX adapter with stable ROS interfaces;
- metrics, seed control, artifact recording, and emergency stop.

**Exit criteria:** a seeded simulator run can be reproduced locally and in CI with identical scenario configuration and comparable metrics.

## Stage 1 — First simulated lap

**Goal:** complete a collision-free lap with simple, explainable components.

- use simulator ground-truth pose initially;
- import or generate a centerline and boundaries;
- implement Frenet transforms;
- implement Pure Pursuit;
- add speed scheduling from curvature;
- record lap time, tracking error, clearance, and control saturation.

**Exit criteria:** repeatable collision-free laps on several tracks, with tests for geometry, controller behavior, and simulator contracts.

## Stage 2 — Autonomous mapping and localization

**Goal:** remove dependence on ground-truth pose.

- map a track with `slam_toolbox`;
- localize with an existing particle filter or scan matcher;
- fuse IMU and wheel/VESC data for dynamic state estimation;
- add localization confidence and relocalization;
- test injected noise, slip, dropped scans, and initialization errors.

**Exit criteria:** lap completion and tracking metrics degrade gracefully relative to ground truth; localization failures trigger a safe fallback.

## Stage 3 — Optimized qualifying lap

**Goal:** maximize single-car lap performance.

- wrap TUM minimum-curvature and velocity-profile tools;
- identify a simple vehicle model;
- implement kinematic tracking MPC;
- progress to a dynamic model and MPCC;
- build parameter-sweep and regression tooling;
- add solver watchdogs and controller fallbacks.

**Exit criteria:** a measured lap-time improvement over Pure Pursuit with no increase in collisions or constraint violations, across multiple seeds and tracks.

## Stage 4 — Changed boundaries and static/dynamic obstacles

**Goal:** remain safe and useful when the reference map is imperfect.

- build local boundary extraction;
- add semi-static delta and dynamic object layers;
- implement a local Frenet or spline planner;
- generate multiple feasible candidate trajectories;
- add TTC and collision-envelope checks;
- implement stop, avoid, and recover behaviors.

**Exit criteria:** successful completion of seeded barrier-shift and obstacle scenarios without corrupting the localization reference map.

## Stage 5 — Multi-car racing

**Goal:** controlled following and overtaking.

- implement LiDAR clustering and track-aware filtering;
- add KF/EKF tracking in Frenet coordinates;
- start with constant-velocity/constant-progress prediction;
- implement KEEP, FOLLOW, PASS_LEFT, PASS_RIGHT, ABORT, and RECOVER;
- measure overtake success, contact rate, time loss, and prediction calibration;
- progress to GP prediction or Predictive Spliner-style planning.

**Exit criteria:** statistically meaningful overtaking success against multiple scripted opponents with bounded contact and rule-violation rates.

## Stage 6 — Gazebo integration and hardware parity

**Goal:** validate ROS, model, sensor, and launch integration.

- add vehicle URDF/xacro, TF, LiDAR, IMU, noise, and collisions;
- make Gazebo and Gym JAX satisfy the same simulator-adapter contract;
- validate QoS, timestamps, lifecycle, and degraded communications;
- add rosbag replay and hardware-like launch files.

**Exit criteria:** the same autonomy launch and scenario intent run against both simulators with only backend configuration changes.

## Stage 7 — F1TENTH hardware deployment

**Goal:** safe, incremental sim-to-real transfer.

- reuse `f1tenth_system` and validate actuator limits;
- deploy on Jetson/arm64 with native builds and pinned images;
- perform wheels-up I/O tests and low-speed closed-course tests;
- identify steering, drivetrain, delay, and tire parameters;
- tune localization and controller latency;
- increase speed only behind explicit safety gates;
- preserve manual and hardware emergency stops.

**Exit criteria:** repeatable physical laps with safety supervision, complete logs, and a documented rollback/fallback path.

## Stage 8 — Research directions and full-scale transfer

**Goal:** explore algorithms whose concepts transfer toward IAC/A2RL-class systems.

- robust/tube MPC and friction-aware control;
- multi-opponent prediction and game-theoretic planning;
- learned residual dynamics and tactical policies;
- high-fidelity vehicle dynamics with Open-Car-Dynamics or a justified MuJoCo model;
- SiL/HiL timing analysis, fault injection, and health management;
- stricter trajectory validation and safety cases;
- 3D track geometry, banking, high-speed localization, and redundant sensing.

The transfer value is in architecture, interfaces, optimization, testing, timing, and safety discipline—not in assuming that a 1:10 vehicle model scales directly to a full-size race car.

---

# 15. Recommended first implementation slice

The first vertical slice should be deliberately small:

```text
Gym JAX
  -> ROS simulator adapter
  -> ground-truth track-relative state
  -> canonical trajectory message
  -> Pure Pursuit
  -> safety clamp
  -> Ackermann command
  -> metrics + deterministic scenario test
```

This slice establishes the contracts that every later simulator, localizer, planner, and controller will use. Only after it is deterministic and measured should the project add SLAM, localization, optimized trajectories, MPC, and opponents.

The first controller should be simple because it validates the infrastructure. The first serious research controller can then be compared against a trustworthy baseline rather than debugged simultaneously with the simulator, message design, track processing, and test harness.

---

# Primary repositories and technical sources

## Core F1TENTH / RoboRacer

1. [RoboRacer / F1TENTH documentation](https://f1tenth.org/)
2. [F1TENTH documentation repository](https://github.com/f1tenth/f1tenth_doc)
3. [F1TENTH system and hardware drivers](https://github.com/f1tenth/f1tenth_system)
4. [F1TENTH Gym JAX](https://github.com/f1tenth/f1tenth_gym_jax)
5. [F1TENTH Gym ROS bridge](https://github.com/f1tenth/f1tenth_gym_ros)
6. [Original F1TENTH simulator](https://github.com/f1tenth/f1tenth_simulator)
7. [F1TENTH evaluation environment paper](https://proceedings.mlr.press/v123/o-kelly20a.html)
8. [ROS 2 Lyrical Luth release information](https://docs.ros.org/en/kilted/Releases/Release-Lyrical-Luth.html)
9. [ROS 2 Jazzy documentation](https://docs.ros.org/en/jazzy/)

## Complete and competition racing stacks

10. [ForzaETH race stack](https://github.com/ForzaETH/race_stack)
11. [ForzaETH Race Stack paper](https://arxiv.org/abs/2403.11784)
12. [ForzaETH technical report](https://f1tenth.org/publications/ForzaETH.pdf)
13. [UNICORN Racing Stack](https://github.com/HMCL-UNIST/unicorn-racing-stack)
14. [ICRA 2025 F1TENTH head-to-head stack](https://github.com/vaithak/f1tenth-icra-race)
15. [TUM Phoenix ROS 2 F1TENTH stack](https://github.com/tum-phoenix/f1tenth_ros)

## Mapping, localization, and estimation

16. [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)
17. [robot_localization](https://github.com/cra-ros-pkg/robot_localization)
18. [ForzaETH papers, including SynPF](https://www.forzaeth.ch/categories/papers/)

## Global planning, local planning, and control

19. [TUM global race-trajectory optimization](https://github.com/TUMFTM/global_racetrajectory_optimization)
20. [TUM trajectory planning helpers](https://github.com/TUMFTM/trajectory_planning_helpers)
21. [Liniger MPCC](https://github.com/alexliniger/MPCC)
22. [TUM graph-based local trajectory planner](https://github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner)
23. [TUM vehicle dynamics and control stack](https://github.com/TUMFTM/mod_vehicle_dynamics_control)
24. [acados](https://github.com/acados/acados)
25. [CasADi](https://github.com/casadi/casadi)
26. [HPIPM](https://github.com/giaf/hpipm)

## Opponent prediction and competitive planning

27. [Predictive Spliner code](https://github.com/ForzaETH/predictive-spliner)
28. [Predictive Spliner paper](https://arxiv.org/abs/2410.04868)
29. [M-Predictive Spliner paper](https://arxiv.org/abs/2506.16301)
30. [SGTP-Racer](https://github.com/zhouhengli/SGTP-Racer)
31. [EVO-MPCC](https://github.com/zhouhengli/EVO-MPCC)
32. [Game-Theoretic Motion Planner](https://github.com/WeiqiLyu/Game-Theoretic-Motion-Planner)

## Benchmarking, RL, dynamics, and full-scale transfer

33. [F1TENTH benchmark implementations](https://github.com/BDEvan5/f1tenth_benchmarks)
34. [Unifying F1TENTH: Survey, Methods and Benchmarks](https://arxiv.org/abs/2402.18558)
35. [TC-Driver paper](https://f1tenth.org/publications/TC_Driver.pdf)
36. [TUM Open-Car-Dynamics](https://github.com/TUMFTM/Open-Car-Dynamics)
37. [TUM Autonomous Motorsport repositories](https://github.com/TUMFTM)

---

# Final recommendation

Build a **ROS 2 Jazzy monorepo** around stable racing-domain interfaces. Use **Gym JAX first**, **Gazebo later for integration**, and defer **MuJoCo** until a specific dynamics question justifies it. Reuse mature drivers, SLAM, simulators, solvers, and offline trajectory optimization. Implement the track/Frenet model, scenario harness, metrics, behavior layer, local planner, controller progression, and safety supervisor yourself.

The highest-value path is:

**deterministic first lap -> measured classical baseline -> optimized qualifying lap -> local-map obstacle handling -> opponent tracking and tactical overtaking -> multi-car prediction -> hardware deployment -> robust and learning-based research.**

This sequence maximizes learning while preserving a working system at every stage and builds architectural habits that transfer to full-scale autonomous racing.
