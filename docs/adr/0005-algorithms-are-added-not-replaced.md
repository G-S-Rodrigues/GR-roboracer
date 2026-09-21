# ADR 0005 — Algorithms are added, never replaced; the reference stack bounds the matrix

**Status:** accepted
**Date:** 2026-09-10
**Supersedes:** none
**Constrains:** ADR 0004 (what may be added to `tests/acceptance/`)

## Context

This repository is a research stack, not a product. Its point is to hold several implementations of
the same job — localization, planning, control — and to compare them: on one track, and eventually
across every track, so that an improvement is shown to be general rather than a fit to one circuit.

That means the ordinary instinct is wrong here. When a better method lands, deleting the one it beat
is not cleanup — it destroys the baseline the next comparison needs, and it deletes the evidence for
why the new one is better. Pure pursuit stays after MPC lands. Ground-truth pose stays after SLAM
lands.

The cost is a test matrix, and the cost is not small. Composition is multiplicative: two tracks and
three pose sources is six configurations, and it is nine the moment a fourth pose source appears. The
stack's tier-3 and tier-4 tests run a **real launch in real time**, so each configuration is paid in
wall-clock minutes. On the bootstrap track a lap costs 22.85 s; on Spielberg, ~147 s. A naive matrix
turns `./scripts/check.sh --full` — the repository's one mandatory gate — into something nobody runs,
and a gate nobody runs is worse than no gate, because it still reads as a gate.

## Decision

**Two rules, and the second is what makes the first affordable.**

### 1. Additive, not substitutive

A new implementation of an existing role is **added alongside** the ones already there. The earlier
implementations keep working, keep their configuration, and keep the tests they landed with,
**unchanged**. Selecting between them is a launch-time argument with a default, never a code
deletion. Removing an implementation is its own decision, argued on its own, and never a side effect
of adding another.

### 2. Every implementation is verified once, in the reference stack — never against every peer

> **The reference stack** is one named composition, defined in exactly one place, naming the
> current default choice for each replaceable role.

The file that holds it is `config/reference_stack.yaml`, created by the SLAM/localization phase, the
first work this rule governs (`.scratch/02-slam-localization/spec/2026-09-10-first-slam-and-evaluation-harness.md`).
It names each role's default implementation; `racing_bringup/launch/sim_pure_pursuit.launch.py`
composes that stack, and BRINGUP-1030 fails if the launch and the file disagree — so the file is the
one definition and the launch its one realization, never two descriptions free to drift.

A new implementation is verified **once**: against ground truth, inside the reference stack current
at the time it lands. It is *not* verified against every peer, and *not* against its predecessors. A
localization method landing in five months is tested against whatever planner and controller the
reference stack names then. Nobody runs it against pure pursuit. Pure pursuit's own tests keep
running against ground-truth pose, untouched, and that is precisely what "the old one still works"
means here.

**The composition axis is pinned, not swept.** Test count therefore grows **linearly** with the
number of implementations, not quadratically.

### 3. What this permits in `tests/acceptance/`

ADR 0004 keeps the `.robot` suite to a handful of cases on the grounds that a reader checks all of
them. The additive rule is a standing invitation to violate that, so it is bounded explicitly:

**One acceptance case per claim, never per combination.** "The estimate tracks ground truth" is one
case, parameterized over pose source — a fourth method adds a data row, not a test case. A
cross-product belongs in tier 3 or tier 5 `pytest`, which is parameterizable and cheap to filter.

### 4. Where the expensive coverage lives

`--fast` and `--ci` never grow with the matrix; the development loop is not allowed to get slower.
`--full` stays the definition of done and runs the whole matrix on the cheap track plus a single lap
of the expensive one. The full cross-product — tracks × implementations × seeds — runs at **tier 5**
under `--nightly`.

**Tier 5 and `--nightly` do not exist yet.** `docs/agents/testing.md` documents tiers 0–4 and
`scripts/check.sh` accepts `--fast|--ci|--full`; both are extended by the SLAM/localization phase.
This section is the decision about where that coverage belongs, not a description of what is built.
Do not write a test expecting `--nightly` to run it until that lands.

## Consequences

- The repository accumulates implementations permanently. Some will be superseded and never used
  again; they stay anyway, because "what did the previous method score on this track" must remain
  answerable years later.
- A comparison harness with **one** implementation under it proves nothing about being
  implementation-neutral. `docs/agents/testing.md` already records this failure for a different
  contract: *"with one backend, tier 2 proves gym_jax satisfies the contract, not that the contract
  is general."* So the first method to fill a role should land with a second one beside it, cheap and
  deliberately different, whose only job is to exercise the seam.
- Combination bugs are **not** covered, by construction. A pairing of controller and localization
  that fails only together will not be caught until someone runs that pairing. This is the trade
  being made, knowingly: linear coverage of implementations in exchange for no coverage of
  compositions.
- `config/reference_stack.yaml` is a load-bearing file. Changing what it names
  re-points every future test, so a change to it is a reviewed decision, not a convenience.
- A nightly failure is found the next morning rather than at commit time.
