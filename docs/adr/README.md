# Architecture decision records

## The gate

A decision earns an ADR only when **all three** hold:

1. **Hard to reverse.** Undoing it means touching many files, or regenerating artifacts other work
   already depends on.
2. **Surprising without context.** A competent reader would reasonably assume the opposite.
3. **A real trade-off.** Something genuine was given up. If one option is simply better, that is not
   a decision, it is an implementation.

Everything else belongs in a code comment, a `CLAUDE.md`, or `docs/agents/repo-gotchas.md`. An ADR
that records an obvious choice trains readers to skip the directory.

## Records

| ADR | Subject | Status |
|---|---|---|
| [0001](0001-simulator-action-contract.md) | gym_jax env-id variant → `AckermannDriveStamped` mapping | accepted |
| [0002](0002-development-environment.md) | Repo on ext4, underlay in the image, artifacts in named volumes | accepted |
| [0003](0003-sim-outside-ros-workspace.md) | `sim/` outside `ros_ws/` — two products, one physics | accepted |
| [0004](0004-robot-framework-at-tiers-3-4.md) | Robot Framework at tiers 3–4 only, not per package | accepted |
