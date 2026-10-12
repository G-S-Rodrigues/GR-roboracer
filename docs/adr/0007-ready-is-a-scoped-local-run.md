# ADR 0007 — Ready is one scoped local run

**Status:** accepted
**Date:** 2026-10-12
**Amends:** ADR 0005 §4's "`--full` stays the definition of done", and §3's row wording ("a fourth
method adds a data row, not a test case"). ADR 0005 §1 (algorithms are added, never replaced) and §2
(verified once, in the reference stack) are untouched.

## Context

Under `GR-manage`, one part of a plan ran `--full` (~13 min) three or four times: at the end of
implementation, again before the PR because the rebase changed the SHA, a third time when `GR-pr` found
no record of the gate in its own session, and again on each predecessor merge. Two `--fast` runs per
step sat on top (the skill's Verify, then the pre-commit hook), and a nightly-style multi-seed sweep ran
while CI waited. CI then re-ran tiers 0-2 and a seeded lap on the PR head, so most of the whole-matrix
cost bought no new information.

ADR 0005 already says why `--full` is not every test that exists. What it did not say is that `--full`
grows with every implementation added, while a change to one implementation cannot break the tests of
an unrelated one: each implementation is verified once, in the reference stack (§2).

## Decision

1. **Ready means one local `check.sh --ready`, run once at the end of an implementation (and again after
   a failure), plus CI green on the PR head.** `--ready` runs `--fast`'s scoped tiers 0-2, the seeded
   lap (SIM-3020, the reference lap) and the tier-3/4 tests the branch's implementations answer for.
2. **The selection is `config/reference_stack.yaml`, extended additively.** Each implementation lists
   `paths` (globs, `fnmatch`, so `*` also matches `/`) and `verified_by` (the tier-3/4 IDs it answers
   for). The Robot tag `impl:<role>=<implementation>` is derived, not repeated. A `neutral_paths` list
   holds paths that select no implementation, only the reference lap. Packages that depend on a changed,
   owned package select their implementations too (the dependency graph is parsed from `package.xml`
   and pinned to `colcon --packages-above` by CHECK-1140).
3. **It fails closed.** A changed path no glob places, a whole-tree trigger, the map itself, the
   composition code every lap runs (`sim_pure_pursuit.launch.py`, `support_node.py`,
   `scenario_parameters.py`, bringup `setup.py`), a test no implementation claims, a claim with no test,
   a test file discovery cannot read, an empty changed set, a failed `git` call, an unreadable map and
   `CHECK_SCOPE=all` are all whole tree. The symptom of a stale map is a slow `--ready`, never a skipped
   test. BRINGUP-1080 checks that tests, claims and Robot tags agree.
4. **Measuring-instrument couplings are `paths`, not claims.** The simulator feeds every estimator and
   `racing_evaluation` scores them, so their paths select the estimators' tests, but only the estimator
   claims them. `scripts/changed_scope.py --owners SIM-3080` therefore prints `pose_source=amcl` and
   nothing else.
5. **Earlier implementations are covered by the nightly `--full`,** which stays tiers 0-4 and runs on a
   schedule. The full run is local and nightly, not a CI job on the PR (owner ruling in the grill).
6. **Tiers 0-2 keep every `config/` change whole tree.** CHECK-1040 is unchanged. Only the tier-3/4
   selection refines `config/` through the map. The spec expected CHECK-1040 to change; it does not,
   because tier-1/2 consumers of `config/` (`racing_bringup` tests, `racing_sim_gym_jax` installing it,
   repo-gotchas #20) cannot be placed on one implementation.
7. **Row-as-test-case (amends ADR 0005 §3's wording).** Robot cannot tag a row of a templated test, so
   in `racing.robot` each implementation of ACC-4030 is a one-row test case sharing the template, with
   its own `[Tags] impl:...`. It is still one claim and one ID; a new method adds a four-line case.
8. **Merge commits are the merge method** (`gh pr merge --merge`), so a ready record keyed to a branch
   SHA stays true after the merge.

## Consequences

- A path assigned to the wrong implementation skips a test silently. Fail-closed catches *missing*
  entries, not wrong ones; review and the nightly are the backstop.
- The map is the second hand-kept list after `changed_scope.py`'s whole-tree triggers
  (repo-gotchas #23), and unlike the first it fails closed.
- A change to the simulator, `racing_evaluation` or bringup's composition code selects nearly every
  tier-3/4 test. The saving is on alternative and newly added implementations.
- `--ready` on a branch that touches `check.sh` or `config/` still pays a whole-tree tier 0-2.

## What it gives up

The guarantee that every branch ran every tier-3/4 test before merging. A regression only a combination
shows, or only an unselected test sees, reaches `main` and is found by the next nightly instead.
