# ADR 0001 — Simulator action contract: velocity and steering angle

**Status:** accepted
**Date:** 2026-09-07
**Supersedes:** none

## Context

`f1tenth_gym_jax` accepts a two-element action whose meaning is selected by the environment id. The
first element is always steering and the second is always longitudinal, but each element has two
legal meanings. Steering is either a target angle in radians or a steering velocity in radians per
second; longitudinal input is either a target velocity in metres per second or acceleration in
metres per second squared.

This choice is hazardous because every pairing can move the car plausibly. A swapped or mismatched
field looks like controller tuning error rather than an interface failure.

## Decision

The system uses `velocity+steeringangle`. Its action vector is
`[drive.steering_angle, drive.speed]`, with steering first. This is the pair naturally produced by
Pure Pursuit and matches the real `f1tenth_system` VESC boundary: target speed plus servo angle.
Keeping the simulated and physical command contracts aligned avoids a controller translation when
moving between them.

The shared `apply_drive_command` function still supports and tests all four environment variants:

| Environment variant | Action vector |
|---|---|
| `acceleration+steeringangle` | `[steering_angle, acceleration]` |
| `acceleration+steeringvelocity` | `[steering_angle_velocity, acceleration]` |
| `velocity+steeringangle` | `[steering_angle, speed]` |
| `velocity+steeringvelocity` | `[steering_angle_velocity, speed]` |

The gym dynamics always consume steering velocity and acceleration internally. For
`steeringangle`, they convert the angle error with gain `1 / timestep`; for `velocity`, they convert
the speed error with the same gain. At the fixed 0.01 s physics timestep, both are deadbeat laws with
gain 100, not the classic F1TENTH gym PID. The divisor is the physics timestep, not the control
period, and the resulting steering velocity and acceleration are then clamped to their configured
bounds.

## Consequences

Controllers and scenario defaults speak in target speed and steering angle. A scenario that chooses
another legal gym variant remains explicit and receives the correct fields through the same tested
mapping.

The deadbeat conversion can make setpoint changes stiff: the target is attempted in one 10 ms
substep, subject to the simulator clamps. Tuning or analysis must not assume classic PID dynamics.

## Alternatives rejected

**`acceleration+steeringvelocity`.** These are the dynamics model's native inputs, but they force
Pure Pursuit output and the physical VESC command through extra control laws at the adapter boundary.

**`acceleration+steeringangle` or `velocity+steeringvelocity`.** Each preserves only half of the
controller-to-car contract and adds a conversion for the other half without an offsetting benefit.

**A fixed mapping inside the gym node.** Rejected because a future environment-id change could make
the car silently consume the wrong fields. The mapping belongs in the shared core and all variants
remain under tier-1 coverage.
