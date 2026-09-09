#!/usr/bin/env python3
"""Run one seeded scenario through the real ROS graph and write its metrics.

The tier-3 tests pin their seeds so they can assert against a golden. This is
the same run for any seed: what the nightly multi-seed sweep drives, and what a
human runs to ask "does it lap on seed 7?" without editing a test.

It fails on the invariants a lap must hold for any seed -- the lap completes,
nothing is hit -- and reports the rest. Per-field agreement with
`tests/golden/baseline.json` is seed-42-specific and belongs to SIM-3040, not
here; use `scripts/compare_metrics.py` for that comparison.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "tests" / "lib"))

from racing_test_keywords.scenario_runner import (  # noqa: E402
    DEFAULT_TIMEOUT_SECONDS,
    run_scenario,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--scenario", default=None)
    parser.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT_SECONDS
    )
    parser.add_argument("--output", type=Path, default=None)
    arguments = parser.parse_args()

    metrics = run_scenario(
        seed=arguments.seed,
        scenario=arguments.scenario,
        timeout=arguments.timeout,
    )
    record = json.dumps(metrics, indent=2, sort_keys=True, default=str)
    if arguments.output is not None:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(record + "\n", encoding="utf-8")
    print(record)

    failures = []
    if not metrics["lap_completed"]:
        failures.append("lap did not complete")
    if metrics["collision_count"] != 0:
        failures.append(f"{metrics['collision_count']} collisions")
    if failures:
        print(f"seed {arguments.seed}: " + "; ".join(failures), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
