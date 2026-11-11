# ADR 0008 — Tests decide whether it works; how well it works is reported

**Status:** accepted
**Date:** 2026-11-09
**Amends:** ADR 0005 §2 (what verifying an implementation asserts), §3 (an acceptance row carries no
per-method bound) and §4 (what the nightly asserts); ADR 0007 §2 (an implementation may be
experimental, with paths and no `verified_by`) and §5 (the nightly reports, it does not rank). ADR 0005
§1 (added, never replaced) and ADR 0007's selection and fail-closed rules are untouched.

## Context

Quality numbers were wired as pass/fail gates, each set at the worst measured run times 1.1 to 1.4.
SIM-3080 bounds amcl's ATE RMSE at 0.65 m with 1.14x headroom over its worst seed-42 run. SIM-3510's
bounds sit at 1.17x and 1.08x of MOLA's worst lap, and its own comment says the margin is "thin under
load". SIM-3110 and SIM-3120 pin a lap time to 0.05 s. A bound that close to a method's spread
measures the host as much as the method: an unseeded matcher, a particle filter or a loaded host moves
the number, the gate goes red, and the work that follows is a fix part and a root-cause hunt, not
algorithm work. Measured on 2026-11-08: X1 got 0 of 40 runs green in 4 days, and three runs queued on
one heavy lane (gotcha #29) to re-measure bounds.

A tight bound also cannot tell a regression from noise, and it penalises improvement work: a method
that changes for the better has to re-measure and re-set its bound in the same PR. And the stack's
purpose (ADR 0005) is comparison. A pass/fail at one seed compares nothing.

## Decision

### 1. A test gates on "it works"

A tier-3, -4 or -5 test may fail only on:

- it builds, launches and publishes (tiers 0-2 unchanged);
- the lap completes within the run's timeout;
- 0 collisions;
- no supervisor latch or intervention (no `REASON_ESTOP_LATCHED`, no stale-input stop, and
  `control_saturation_events == 0` where a test asserts it today);
- the instrument recorded data (`sample_count`, `scored_count`, metrics published, a map with wall
  cells, probes that heard something);
- an equivalence check: two runs, or two pipelines, of the same code agree. These are the goldens
  (SIM-3030/3040/3050/3090 through `scripts/compare_metrics.py`), the ruler checks (SIM-3060 identity,
  SIM-3100 drift floor), the AutoDRIVE equivalence and fidelity tests (SIM-3710/3720), and the
  regeneration tests (SIM-5010/5040/5060/5520, BRINGUP-1750);
- **one catastrophic bound per quality metric.**

### 2. A catastrophic bound says "broken", never "worse"

There is one value per metric, shared by every implementation, track and seed. It is justified by
geometry, never by measured spread: past it the method is lost, silent or off the track. It is never
tightened toward a measured number. A catastrophic bound that fires on a working method is wrong, and
it is loosened by argument in a reviewed PR, not re-measured.

| Metric | Catastrophic bound | Why this value |
|---|---|---|
| Localization error (ATE RMSE, live and offline) | <= 5.0 m | About 4.5 Spielberg half-widths (1.1 m): an estimate that far off is lost. Every working run measured so far is <= 2.94 m (slam_toolbox 64-beam, SIM-5020). Measured failures are far past it: 6.93-8.17 m (the 64-beam file on the 1080 scenario) and 37.7 m (MOLA at `icp_prior_weight` 1.0). |
| Localization availability | >= 0.90 | An estimate missing for more than a tenth of the lap is a stalled estimator. Every working run measured >= 0.994. It is asserted first, because a dead estimator scores zero error (EVAL-1030). |
| Lap time | the run's timeout (already 3x the measured wall time in every lap test) | No number. A lap that does not finish in 3x its wall time is hung. No lap-time bound is added (spec, standing rules). |
| Minimum wall clearance | > 0.08 m (ACC-4010 and ACC-4040 today, unchanged) | It is under the vehicle half-width (0.15 m) measured from the CoG, so the body is already in the wall. Measured 1.72-1.81 m on the baseline. |
| Tracking error, p95 | < 1.0 m (ACC-4010 today, unchanged) | About one track half-width. Measured 0.139 m (baseline golden). |
| Map wall accuracy, median | <= 1.0 m | Under a half-width. Measured 0.20-0.56 m (SIM-3530, SIM-3550). |
| Map wall cells | >= 21 800 (half the fewest measured, 43 631) | A map with half its walls missing is not a map. |

### 3. Every quality number is reported

ATE RMSE and maximum, availability, lap time, clearance, tracking error and map accuracy are printed
and written by the test that measured them, to `log/reports/<ID>.json`. Each record carries the seed,
scenario, implementation, code revision and host load. No test fails on a reported number. The
measurement comments that justified the old bounds stay in the tests, as the baseline a report is read
against. The helper is `tests/lib/racing_test_keywords/report.py` (`write_report`, and the catastrophic
values in `CATASTROPHIC_BOUNDS`); `scripts/check.sh --ready` prints the reports a run wrote.

### 4. Regression tracking is nightly only, and never blocks

A nightly test that tracks one method over time (SIM-5030, "did amcl get worse") writes its report and
never fails on a quality number. A nightly job is red only on a gate. The trend is read from the
nightly's uploaded reports. It is never a reason to stop a branch, and it never turns into a fix part
by itself.

### 5. Benchmarks never block

A benchmark compares implementations of one role with each other (localization: amcl, slam_toolbox,
mola; planning: centerline, min-curvature, min-time) and prints implementation x seed tables. It runs
in two places only: on demand, from one command; or in CI, as a non-required job on a PR that adds an
implementation. It never runs on the nightly schedule, and nothing waits on it.

### 6. Experimental, then verified

An implementation may land **experimental**: merged, selectable by its launch argument, and
benchmarked before its tier-3 verification.

- **How it is marked.** In `config/reference_stack.yaml` it is an `alternatives` entry with
  `status: experimental`, its `paths` and an empty `verified_by`.
- **Its gate.** Tiers 0-2: it builds and lints, its unit and contract tests pass, and a launch test
  shows the argument selects it.
- **What it never is.** A role's default, or part of the reference lap.
- **No expiry.**
- **Promotion to verified** is one reviewed PR. It adds the tier-3/4 test that verifies the
  implementation once, in the reference stack, against ground truth (ADR 0005 §2), with §1's gates and
  §2's catastrophic bounds. It lists that test in `verified_by`, removes `status`, and moves the
  implementation's README row.

### 7. Lighter fix parts

A fix for a broken or flaky gate (test, harness, CI) may run as a light part, and the workflow still
applies: plan, review, fix test-first, review, PR. The plan may be a paragraph: the symptom, the cause
as far as it is known, the fix, and the test that shows the fix works. A fix part needs no grill and
no spec.

## Consequences

- **A moderate quality regression no longer fails a PR.** For example, amcl's RMSE going from 0.3 m
  to 1.5 m. It shows in the report the branch's own `--ready` prints, in the nightly trend, and in the
  next benchmark, all after merge. This is the trade, made knowingly.
- **A green test no longer says a method is good.** A quality claim in `README.md`,
  `docs/results.md` or `docs/verification.md` cites a report or a benchmark, not a test.
- **Gotcha #14's guard narrows to what is still a gate:** the goldens, the `compare_metrics.py`
  tolerances and the catastrophic bounds. Moving a bound to reported is done once, here, from one
  inventory. It is never done one test at a time to green a run.
- **Fewer red runs from host load.** A catastrophic bound does not move with load, so `--ready` and
  the nightly stop failing on headroom.
- **The numbers need a reader.** Until the benchmark command exists, they sit in logs and CI
  artifacts only.
- **Combination coverage is unchanged.** It is still none, by construction (ADR 0005).
