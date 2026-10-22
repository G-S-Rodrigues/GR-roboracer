"""Tests for scripts.nightly_regions: which nightly tests each region runs."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

from scripts.nightly_regions import (
    REGIONS,
    discover_nightly_tests,
    discover_robot_cases,
    load_regions,
    nightly_set,
    select,
)

REPO_ROOT = Path(__file__).resolve().parents[2]

ROBOT = """\
*** Test Cases ***
Lap Case
    [Documentation]    ACC-4010
    [Tags]    impl:controller=pure_pursuit
    Log    ok

First Source: one
    [Documentation]    ACC-4030. One.
    [Tags]    impl:pose_source=slam_toolbox
    Log    ok

Second Source: two
    [Documentation]    ACC-4030. Two.
    [Tags]    impl:pose_source=amcl
    Log    ok
"""

GOOD_MAP = """\
slam:
  - ACC-4030
  - SIM-5010
path_planning:
  - SIM-3110
control:
  - SIM-3010
  - ACC-4010
shared:
  - SIM-3030
"""


def _tree(tmp_path: Path, region_map: str = GOOD_MAP) -> Path:
    for relative in (
        "tests/system/test_sim_3010_a.py",
        "tests/system/test_sim_3030_b.py",
        "tests/system/test_sim_3110_c.py",
        "tests/nightly/test_sim_5010_d.py",
    ):
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("")
    (tmp_path / "tests/acceptance").mkdir(parents=True)
    (tmp_path / "tests/acceptance/x.robot").write_text(ROBOT)
    (tmp_path / "tests/nightly_regions.yaml").write_text(region_map)
    return tmp_path


def _select_error(tmp_path: Path, region_map: str) -> str:
    root = _tree(tmp_path, region_map)
    with pytest.raises(ValueError) as error:
        select(root, "slam", tier5=True)
    return str(error.value)


def test_check_1520_selects_a_regions_files_and_cases(
    tmp_path: Path,
) -> None:
    """CHECK-1520: a region gets its system files, every Robot case of its
    ACC ID, and tier-5 files only when asked."""
    root = _tree(tmp_path)
    chosen = select(root, "slam", tier5=True)
    assert chosen.system_tests == ()
    assert chosen.robot_tests == ("First Source: one", "Second Source: two")
    assert chosen.nightly_tests == ("tests/nightly/test_sim_5010_d.py",)
    assert select(root, "slam", tier5=False).nightly_tests == ()
    control = select(root, "control", tier5=True)
    assert control.system_tests == ("tests/system/test_sim_3010_a.py",)
    assert control.robot_tests == ("Lap Case",)


def test_check_1520_static_selects_nothing(tmp_path: Path) -> None:
    """CHECK-1520: `static` holds no IDs."""
    root = _tree(tmp_path)
    chosen = select(root, "static", tier5=True)
    assert (chosen.system_tests, chosen.robot_tests, chosen.nightly_tests) == (
        (),
        (),
        (),
    )


def test_check_1520_duplicate_id_is_refused(tmp_path: Path) -> None:
    """CHECK-1520: an ID in two regions raises, naming it."""
    message = _select_error(
        tmp_path, GOOD_MAP.replace("shared:\n", "shared:\n  - SIM-3010\n")
    )
    assert "duplicate SIM-3010" in message


def test_check_1520_unknown_id_is_refused(tmp_path: Path) -> None:
    """CHECK-1520: an ID with no test raises, naming it."""
    message = _select_error(
        tmp_path, GOOD_MAP.replace("shared:\n", "shared:\n  - SIM-3999\n")
    )
    assert "unknown SIM-3999" in message


def test_check_1520_unassigned_test_is_refused(tmp_path: Path) -> None:
    """CHECK-1520: a test no region owns raises, naming it."""
    message = _select_error(tmp_path, GOOD_MAP.replace("  - SIM-5010\n", ""))
    assert "unassigned: SIM-5010" in message


def test_check_1520_unknown_region_is_refused(tmp_path: Path) -> None:
    """CHECK-1520: a region name outside the four raises."""
    message = _select_error(tmp_path, GOOD_MAP + "planning:\n  - SIM-3110\n")
    assert "unknown region 'planning'" in message


def test_check_1520_empty_region_is_refused(tmp_path: Path) -> None:
    """CHECK-1520: a region with no IDs raises."""
    message = _select_error(tmp_path, GOOD_MAP + "shared: []\n")
    assert "empty region 'shared'" in message


def test_check_1520_stray_nightly_file_is_refused(tmp_path: Path) -> None:
    """CHECK-1520: a tests/nightly file the SIM-NNNN pattern cannot read
    raises rather than being skipped."""
    root = _tree(tmp_path)
    (root / "tests/nightly/test_foo.py").write_text("")
    with pytest.raises(ValueError, match="test_foo.py"):
        discover_nightly_tests(root)
    with pytest.raises(ValueError, match="test_foo.py"):
        select(root, "slam", tier5=True)


def test_check_1520_shell_output_evals_to_the_arrays(tmp_path: Path) -> None:
    """CHECK-1520: --shell prints the three arrays, ACC-4030 as both case
    names, and nothing for tier 5 without --tier5."""
    root = _tree(tmp_path)

    def run(*extra: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [
                sys.executable,
                str(REPO_ROOT / "scripts/nightly_regions.py"),
                "--root",
                str(root),
                "--shell",
                *extra,
            ],
            capture_output=True,
            text=True,
            check=False,
        )

    slam = run("--region", "slam")
    assert slam.returncode == 0, slam.stderr
    assert "region_robot_tests=('First Source: one' 'Second Source: two')" in (
        slam.stdout
    )
    assert "region_nightly_tests=()" in slam.stdout
    with_tier5 = run("--region", "slam", "--tier5")
    assert "region_nightly_tests=(tests/nightly/test_sim_5010_d.py)" in (
        with_tier5.stdout
    )
    static = run("--region", "static")
    assert static.stdout.count("=()") == 3
    bogus = run("--region", "bogus")
    assert bogus.returncode == 1
    assert "unknown region 'bogus'" in bogus.stderr


def test_check_1510_union_equals_discovered_nightly_set() -> None:
    """CHECK-1510: the regions partition the nightly set found by discovery
    over tests/system, tests/acceptance and tests/nightly: nothing dropped,
    nothing twice, nothing invented."""
    known = nightly_set(REPO_ROOT)
    regions = load_regions(REPO_ROOT / "tests/nightly_regions.yaml", known)
    assigned = [i for ids in regions.values() for i in ids]
    assert len(assigned) == len(set(assigned))
    assert set(assigned) == known
    assert set(regions) == set(REGIONS)


def test_check_1510_union_equals_collected_files_and_cases() -> None:
    """CHECK-1510: pytest's own collection of tests/system and tests/nightly,
    and Robot's case list, equal what the regions select."""
    collected = subprocess.run(
        [
            sys.executable,
            "-m",
            "pytest",
            "--collect-only",
            "-q",
            "tests/system",
            "tests/nightly",
        ],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    assert collected.returncode == 0, (
        "pytest could not collect tests/system and tests/nightly. Their "
        "conftests import ROS, so source /opt/ros/jazzy/setup.bash and the "
        "overlay (install/setup.bash) first, as check.sh does.\n"
        + collected.stdout
        + collected.stderr
    )
    files = {
        line.split("::")[0]
        for line in collected.stdout.splitlines()
        if line.startswith("tests/") and "::" in line
    }
    selected = [select(REPO_ROOT, r, tier5=True) for r in REGIONS]
    selected_files = [f for s in selected for f in s.system_tests]
    selected_files += [f for s in selected for f in s.nightly_tests]
    assert len(selected_files) == len(set(selected_files))
    assert set(selected_files) == files
    cases = sorted(
        n for names in discover_robot_cases(REPO_ROOT).values() for n in names
    )
    selected_cases = sorted(n for s in selected for n in s.robot_tests)
    assert selected_cases == cases


def test_check_1510_ci_matrix() -> None:
    """CHECK-1510: the nightly job's matrix runs every region (the map's
    plus `static`), and each region carries an integer timeout. Only
    `matrix.region` and `matrix.include` are read; `${{ }}` strings
    elsewhere are never parsed."""
    import yaml

    workflow = yaml.safe_load(
        (REPO_ROOT / ".github/workflows/ci.yaml").read_text(encoding="utf-8")
    )
    matrix = workflow["jobs"]["nightly"]["strategy"]["matrix"]
    assert "region" in matrix, "ci.yaml nightly matrix has no `region`"
    mapped = yaml.safe_load(
        (REPO_ROOT / "tests/nightly_regions.yaml").read_text(encoding="utf-8")
    )
    assert set(matrix["region"]) == set(mapped) | {"static"}
    assert len(matrix["region"]) == len(set(matrix["region"]))
    timeouts = {
        entry["region"]: entry.get("timeout") for entry in matrix["include"]
    }
    for region in matrix["region"]:
        assert isinstance(timeouts.get(region), int), (
            f"no timeout for region {region}"
        )
