# ADR 0004 — Robot Framework at tiers 3–4 only, not per package

**Status:** accepted
**Date:** 2026-09-09
**Supersedes:** the original brief's per-package `.robot` suites

## Context

The original brief asked for a Robot Framework suite in every package. Robot Framework is genuinely
good at one thing this repository needs: stating acceptance criteria in language a non-author can
read and check against intent —

```robotframework
Vehicle Completes Baseline Lap Safely
    Run Racing Scenario    seed=42
    Lap Completion Should Be    true
    Collision Count Should Be    0
    Minimum Wall Clearance Should Exceed    0.08
```

It is a poor fit for everything else here. A `.robot` suite needs a live ROS graph, so each one costs
tens of seconds where the gtest covering the same logic costs milliseconds — and the logic is
already covered, because every node is a thin shell over a ROS-free core (see
`docs/agents/testing.md`).

The decisive argument is not cost, it is direction of failure. Per-package Robot suites make
timing-dependent assertions, and a timing-dependent assertion fails intermittently. The cheapest way
to make an intermittent test pass is to widen its tolerance — which silently deletes the guard while
leaving a green test in place. That is a failure mode an agent, or a hurried human, will take.

## Decision

**Robot Framework appears at tiers 3 and 4 only**: `tests/acceptance/racing.robot`, two cases,
ACC-4010 and ACC-4020. Tier 3 is plain `pytest` over the same `racing_test_keywords` library.

`tests/lib/racing_test_keywords/` is a **build item, not glue** — no mature ROS 2 keyword library for
Robot Framework exists, so the `rclpy` wrapper had to be written. It is shared by tiers 3 and 4, and
new capability goes into `scenario_runner`/`ros_helpers` first, because tier 3 exercises it directly
and more cheaply than a `.robot` suite can.

## Consequences

- Acceptance criteria stay readable and few. Two cases is the point; a reader checks both.
- Per-package behaviour is covered by gtest and `launch_testing` instead, at 1/1000 the cost.
- One shared keyword library to maintain, rather than one per package.
- If a `.robot` suite is ever proposed for a package, the question to answer first is what it proves
  that a tier-1 test on that package's ROS-free core cannot.
