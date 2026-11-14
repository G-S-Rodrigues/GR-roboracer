# Research

Background research behind the design decisions in this stack. These documents are the *source* the
spec and ADRs cite — when an ADR says "research §7", it means the survey below.

## Documents

- [Autonomous racing open-source landscape (2026)](autonomous-racing-open-source-2026.md) — survey of
  the open-source autonomous racing ecosystem: simulators, stacks, planners, controllers and the
  trade-offs between them. Cited throughout the spec.
- [Reliable sources](reliable-sources.md): the codebases and papers this stack trusts, tiered, each
  with what to trust it for, its licence and the date it was last checked.

## Reference codebases

### GitHub

- [TUMFTM](https://github.com/TUMFTM) — TU München Institute of Automotive Technology. Source of
  `trajectory_planning_helpers` and the minimum-curvature / minimum-time raceline optimization work
  this stack plans to reuse rather than reimplement.

## Not committed

**ForzaETH Race Stack** (paper, PDF, ~9.2 MB) is deliberately **not** vendored into this repository.
Binary assets in a monorepo are a permanent cost — they stay in git history forever and are paid on
every clone — and its redistribution licence is unclear. Link to it instead; the survey document
above summarises the parts this stack draws on.
