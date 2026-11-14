# Reinforcement learning

Learned components for the stack: the order to try them in, the rule that every learned piece keeps a
classical baseline and fallback, sim-to-sim transfer, batched rollouts, and the later survey stages
(obstacles, multi-car, hardware, full scale). Read before a grill, spec or plan that adds a learned
component, a training loop, or batched gym rollouts. Nothing here is built yet. The classical
controller a learned piece would sit beside is in [control.md](control.md), the simulators in
[simulation.md](simulation.md), and the league in [competition.md](competition.md).

## Current state

Verified against `origin/main` 7c9ff4a on 2026-11-14.

- **No learned implementation and no training pipeline exist.** The README lists three learning
  directions as Exploratory: trajectory-conditioned or residual control, learned localization inside a
  structured estimator, and end-to-end sensor-to-command driving. The README says the classical
  implementations stay available and the safety supervisor stays in the driving stack.
- **The rollout seam an RL loop would use exists.** [`sim/rollout.py`](../../sim/rollout.py) is a
  seeded, ROS-free `f1tenth_gym_jax` rollout harness. It takes an `ActionProvider`
  (`(observation, state) -> (steering, speed)`). Today the only provider is
  `pure_pursuit_action_provider`, which calls the C++ Pure Pursuit through its pybind module. It
  generates the goldens and runs in the `fast-sim` container on the GPU
  ([running.md](../running.md#running-a-scenario), ADR 0002, ADR 0003).
- **Rollouts are single-environment.** `jax.vmap` appears only in scan casting
  (`racing_sim_gym_jax/backend.py` vmaps the cast over agents' poses, `laser.py` over beams). No code
  steps a batch of environments. Batched `vmap` rollouts were a non-goal of the bootstrap, and RL is
  what would bring them in.
- **Traps an RL loop would hit are already gotchas.** Call `step_env`, not `step`, which auto-resets on
  termination without saying so (#12). Warm every JAX signature (#13). The gym's JAX stays on CPU in
  `dev` and `gpu`, and a GPU belongs in `fast-sim` (#3, #4).
- **The fallback a learned piece sits in front of** is the safety supervisor
  ([control.md](control.md#the-safety-supervisor)). It clamps whatever command reaches it and stops on a
  stale input or a track limit, so it does not care whether a person or a policy wrote the command.

## What we measured

Nothing yet. No RL experiment has run.

## Dead ends

None yet.

## Ideas not yet specified

### Direction (owner, 2026-09-30)

> Reach a durable classical stack (GPU-tuned SLAM / localization, an optimized raceline, and a
> controller that follows it), then use it as the baseline and the safety fallback for learning
> reinforcement learning.

### The order, easiest first

From the survey's [§9](../research/autonomous-racing-open-source-2026.md#9-reinforcement-learning). Each
learned piece keeps a classical baseline and a classical fallback:

1. **Parameter and weight tuning** over vectorized gym_jax rollouts: Pure Pursuit's lookahead, later an
   MPC's weights. This is the natural first step: `sim/rollout.py` already drives the authoritative
   controller core, and `lookahead_gain` is built and off ([control.md](control.md)).
2. **Residual dynamics compensation.**
3. **Tactical decisions**, such as whether to overtake.
4. **Trajectory-conditioned policies** (TC-Driver). They need the optimized raceline, which exists
   ([raceline.md](raceline.md)).
5. **Learned local planning.**
6. **End-to-end control**, much later.

The survey lists what every learned component needs: a classical baseline, an offline evaluation
dataset, deterministic seeds, explicit observation and action contracts, out-of-distribution
monitoring, a runtime fallback and sim-to-real gates. Read it there rather than here.

How this meets the repo's rules:

- A learned controller is one more implementation of the `controller` role, added beside Pure Pursuit
  and selected by a launch argument ([ADR 0005](../adr/0005-algorithms-are-added-not-replaced.md)). It
  can land `status: experimental` first
  ([ADR 0008 §6](../adr/0008-gates-decide-quality-is-reported.md)).
- A learned result is compared in a benchmark, never gated on a quality number (ADR 0008).
- `sim/` stays ROS-free ([ADR 0003](../adr/0003-sim-outside-ros-workspace.md)). A training loop
  belongs there or in its own uv project, not in `ros_ws/`.

### Sim-to-sim transfer as the RL lesson

Train in gym_jax and evaluate in AutoDRIVE, whose physics differ. This is a small, safe version of the
sim-to-real gap. The differences are known: the gym's speed command is deadbeat (ADR 0001), while
AutoDRIVE's car spins its wheels at full throttle, locks them at zero, and has grip that depends on the
corner radius (gotcha #46, [control.md](control.md#what-we-measured)). The backlog also wants the
non-learned version: run the same autonomy launch against gym_jax and AutoDRIVE and compare (the
survey's Stage 6 exit criterion, with AutoDRIVE in place of Gazebo).

### Isaac Sim through RoboRacer Arena

arXiv 2608.23040 builds tracks from occupancy maps and runs 256 parallel vehicles. It is a possible
third simulator bridge and an RL backend. No code was found as of the backlog entry.

### Later stages from the survey roadmap

None is specified. The survey's [§14](../research/autonomous-racing-open-source-2026.md#14-staged-roadmap)
holds each stage's scope and exit criteria.

- **Stage 4: changed boundaries, static and dynamic obstacles.** Its exit criterion includes a seeded
  barrier-shift scenario. The owner parked localization with moving barriers "for later"
  ([localization-slam.md](localization-slam.md)). `racing_metrics` keeps a scan-based clearance so it
  also sees obstacles that are not in the track geometry (gotcha #17).
- **Stage 5: multi-car racing, prediction, overtaking.** The tactical-decision step of the RL order
  lands here.
- **Stage 7: F1TENTH hardware.** Our own car. CLAUDE.md's "Built for the real car" holds the design
  constraint.
- **Stage 8: full-scale transfer** (A2RL / IAC concepts).

## Open questions

- **When does RL start?** The direction says "after the classical base". No date or exit criterion
  for "the classical base is durable" has been set. Controller part 4 is still a draft
  ([control.md](control.md#part-4-draft-2026-11-10-read-only-draft-nothing-decided)).
- **What the league's evaluation host gives a submission.** The rules ask for `--gpus all` on an
  RTX 4090 host, but whether the team container gets a GPU at race time is not stated
  ([competition.md](competition.md)). That decides whether a learned policy may need a GPU to run.
- **Where batched rollouts live.** It is open whether they extend `sim/rollout.py` or go in a new
  `sim/` module. Either way, a batched loop must not change the goldens `rollout.py` produces
  (gotcha #14).
