#!/usr/bin/env python3
"""Which nightly tests each region of interest runs.

`tests/nightly_regions.yaml` assigns every nightly test ID to one region.
`select` turns a region into the system test files, Robot case names and
tier-5 files `check.sh --nightly-region` runs; `--shell` renders them as bash
to `eval`. The map is validated against the tests that exist, so a new test
nobody assigned fails the nightly loudly instead of being dropped. See
CHECK-1510/1520.
"""

from __future__ import annotations

import argparse
import re
import shlex
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    from scripts.changed_scope import (
        discover_acceptance_tags,
        discover_system_tests,
    )
except ImportError:  # run as `python3 scripts/nightly_regions.py`
    from changed_scope import (  # type: ignore[no-redef]
        discover_acceptance_tags,
        discover_system_tests,
    )

REGIONS = ("slam", "path_planning", "control", "shared")
MAP_PATH = Path("tests") / "nightly_regions.yaml"
NIGHTLY_TEST_NAME = re.compile(r"test_sim_(\d{4})_[^/]*\.py")
ACCEPTANCE_ID = re.compile(r"ACC-4\d{3}")


@dataclass(frozen=True)
class Selection:
    system_tests: tuple[str, ...]
    robot_tests: tuple[str, ...]
    nightly_tests: tuple[str, ...]


def discover_nightly_tests(root: Path) -> dict[str, str]:
    """Tier-5 ID -> repo-relative file, from `test_sim_NNNN_*.py` names.

    Fails closed on any other `test_*.py`, as `discover_system_tests` does."""
    found: dict[str, str] = {}
    nightly = root / "tests" / "nightly"
    for path in sorted(nightly.glob("**/test_*.py")):
        relative = path.relative_to(root).as_posix()
        match = NIGHTLY_TEST_NAME.fullmatch(path.name)
        if match is None or path.parent != nightly:
            raise ValueError(
                f"tier-5 test not discoverable as SIM-NNNN: {relative}"
            )
        test_id = f"SIM-{match.group(1)}"
        if test_id in found:
            raise ValueError(
                f"duplicate {test_id}: {found[test_id]}, {relative}"
            )
        found[test_id] = relative
    return found


def discover_robot_cases(root: Path) -> dict[str, tuple[str, ...]]:
    """Tier-4 ID -> its Robot case names, via Robot's parser."""
    directory = root / "tests" / "acceptance"
    if not directory.is_dir():
        return {}
    from robot.api import TestSuiteBuilder  # lazy: only the nightly needs it

    suite = TestSuiteBuilder().build(str(directory))
    cases: dict[str, list[str]] = {}
    for case in suite.all_tests:
        match = ACCEPTANCE_ID.search(case.doc or "")
        if match is None:
            raise ValueError(f"tier-4 case without an ACC-4NNN ID: {case.name}")
        cases.setdefault(match.group(0), []).append(str(case.name))
    return {k: tuple(v) for k, v in cases.items()}


def nightly_set(root: Path) -> set[str]:
    """Every ID the nightly runs today: tiers 3, 4 and 5."""
    return (
        set(discover_system_tests(root))
        | set(discover_acceptance_tags(root))
        | set(discover_nightly_tests(root))
    )


def load_regions(path: Path, known: set[str]) -> dict[str, tuple[str, ...]]:
    """The region -> IDs map, refused unless it partitions `known` exactly."""
    import yaml  # lazy: stdlib-only callers need not have it

    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    regions: dict[str, tuple[str, ...]] = {}
    owner: dict[str, str] = {}
    for name, ids in data.items():
        if name not in REGIONS:
            raise ValueError(f"unknown region '{name}' (allowed: {REGIONS})")
        if not ids:
            raise ValueError(f"empty region '{name}'")
        for test_id in ids:
            if test_id in owner:
                raise ValueError(
                    f"duplicate {test_id}: {owner[test_id]}, {name}"
                )
            if test_id not in known:
                raise ValueError(f"unknown {test_id} in region '{name}'")
            owner[test_id] = name
        regions[name] = tuple(ids)
    for name in REGIONS:
        if name not in regions:
            raise ValueError(f"empty region '{name}'")
    missing = sorted(known - set(owner))
    if missing:
        raise ValueError(f"unassigned: {', '.join(missing)}")
    return regions


def select(root: Path, region: str, tier5: bool) -> Selection:
    """What `region` runs. `static` holds no IDs, so selects nothing."""
    if region == "static":
        return Selection((), (), ())
    if region not in REGIONS:
        raise ValueError(f"unknown region '{region}'")
    system = discover_system_tests(root)
    robot = discover_robot_cases(root)
    nightly = discover_nightly_tests(root)
    known = set(system) | set(robot) | set(nightly)
    ids = load_regions(root / MAP_PATH, known)[region]
    return Selection(
        system_tests=tuple(system[i] for i in ids if i in system),
        robot_tests=tuple(n for i in ids if i in robot for n in robot[i]),
        nightly_tests=(
            tuple(nightly[i] for i in ids if i in nightly) if tier5 else ()
        ),
    )


def render_shell(selection: Selection) -> str:
    """`selection` as bash arrays for `check.sh` to `eval`."""

    def array(name: str, items: tuple[str, ...]) -> str:
        return f"{name}=(" + " ".join(shlex.quote(i) for i in items) + ")"

    return "\n".join(
        [
            array("region_system_tests", selection.system_tests),
            array("region_robot_tests", selection.robot_tests),
            array("region_nightly_tests", selection.nightly_tests),
        ]
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--region", required=True)
    parser.add_argument("--tier5", action="store_true")
    parser.add_argument("--shell", action="store_true")
    parser.add_argument("--root", type=Path, default=None)
    args = parser.parse_args(argv)
    root = args.root or Path(__file__).resolve().parents[1]
    try:
        selection = select(root, args.region, args.tier5)
    except ValueError as error:
        print(f"nightly_regions: {error}", file=sys.stderr)
        return 1
    print(render_shell(selection))
    return 0


if __name__ == "__main__":
    sys.exit(main())
