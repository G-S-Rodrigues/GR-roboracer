# ADR 0006 — Simulated time is the stack's time base

**Status:** accepted
**Date:** 2026-09-10
**Amends:** ADR 0003 — its Context claimed `racing_sim_gym_jax` is "bound to real time by
construction". That sentence has been corrected in ADR 0003 itself; ADR 0003's decision is unaffected.
**Does not change:** ADR 0002's CUDA-stays-in-`fast-sim` decision — see below. Its *reason* changed,
and `docs/agents/repo-gotchas.md` #4 has been rewritten accordingly.

## Context

`racing_sim_gym_jax` stamps every message from a simulated-time counter that starts at 0, while every
node in the graph reads its own clock from system time. Nothing publishes `/clock`, and no node
declares `use_sim_time` — zero occurrences repo-wide before this ADR.

That worked for the bootstrap slice because nothing compared the two. Three things break it at once:

1. **Any TF-consuming component fails silently.** `slam_toolbox`, `nav2_amcl` and anything else
   off-the-shelf look up transforms *by stamp*. Handed scans stamped near t=0 against a wall-clock TF
   buffer, every lookup fails — and a failed lookup is a warning, not an error. This is
   `docs/agents/repo-gotchas.md` #6's shape with a different cause.
2. **Test cost is pinned to simulated distance.** Because the sim node steps on a wall-clock timer at
   the control period, simulated seconds *are* wall seconds. A 62.8 m lap costs 22.85 s; a 405 m
   league track costs ~147 s. Multiply by the implementations ADR 0005 accumulates and the mandatory
   gate stops being runnable.
3. **t=0 is nondeterministic.** `scripts/compare_metrics.py` documents `minimum_wall_clearance`
   carrying ~2.4x its measured spread solely because the clearance nadir sits in the opening ticks,
   and whether it lands inside the accumulation window depends on the DDS discovery race. The comment
   there already names the fix: a deterministic t=0 via the sim node's `~/reset` service.

The obvious objection is ADR 0003's, and it is a real one. That ADR states the ROS sim node **"is
bound to real time by construction"**, and treats it as a feature: *"so the rest of the graph
experiences it the way it will experience real hardware."* Wall-clock binding is what makes a tier-3
run a realistic rehearsal — timer jitter, callback latency, a controller that genuinely has 10 ms to
answer. Simulated time can hide a node that is too slow to keep up on a car.

## Decision

**The simulator owns the clock.** `racing_sim_gym_jax` publishes `rosgraph_msgs/msg/Clock` on
`/clock` from the same counter it already stamps messages with, and every node in the bringup launch
runs with `use_sim_time: true`. A `time_scale` parameter (default **1.0**) sets how fast simulated
time advances relative to wall clock. A run's t=0 is defined by the `~/reset` service, called only
after every expected participant is discovered.

**`time_scale` defaults to 1.0, and that default is the point.** Interactive runs, RViz sessions and
the acceptance suite keep the real-time rehearsal ADR 0003 wanted — the graph still experiences
10 ms budgets and real callback latency. Raising `time_scale` is an explicit, per-run opt-in used by
long-track and nightly sweeps, where the question being asked is "does the estimator converge over
405 m", not "does the controller meet its deadline".

So the amendment to ADR 0003 is narrow: the sim node is no longer bound to real time *by
construction*; it is bound to real time *by default*, and the binding is now a parameter rather than
a property. What ADR 0003 wanted from that binding is preserved wherever it is the thing being
tested, and what ADR 0003 actually *decided* — `sim/` outside `ros_ws/`, two dependency managers, no
CUDA in `dev` — rests on the ROS node being timer-driven and graph-bound, which is untouched.

## Why this does not reopen ADR 0002 (CUDA stays out of `dev`)

`docs/agents/repo-gotchas.md` #4 justifies keeping CUDA out of the `dev` image partly on the grounds
that "the ROS sim node steps on a wall-clock timer at the control period, so a GPU cannot make a
tier-3 run faster." `time_scale > 1` removes that specific argument, and the decision must not be
left standing on a reason that is no longer true.

**The decision is unchanged, on the other measurement in the same gotcha:** on the committed
one-agent / 64-beam scenario the GPU is **5x slower per step** (8.36 ms vs 1.62 ms). Under
`time_scale > 1` that is worse, not better — a slower step is exactly what bounds an accelerated run.
CUDA stays in `fast-sim`. Gotcha #4 has been rewritten to lead with that measurement, so the reason
on record is the one that is still true.

## Consequences

- Every node must take time from its ROS clock. A node reading wall time directly will behave
  inconsistently and **will not error** — the supervisor's staleness logic is the most exposed, and
  `stale_input_timeout_ms: 100` is deliberately unforgiving.
- `tests/golden/baseline.json` does **not** move. It is generated headless by `sim/rollout.py`, whose
  `lap_time` is already simulated time (`sim/rollout.py:179`). What changes is that the live graph
  agrees with it more closely — SIM-3040's tolerances should tighten after re-measurement, never
  loosen (repo-gotchas #14).
- Wall-clock overrun becomes invisible at `time_scale > 1`: a node that misses its deadline just
  makes the run take longer. Deadline behaviour is therefore only meaningful at `time_scale: 1.0`,
  which is where tier 4 and every interactive run stay.
- Long tracks become affordable, which is what makes ADR 0005's matrix payable.
- Anything replaying a recording must honour the recorded stamps rather than assuming wall clock.
