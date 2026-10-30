"""CHECK-1820..1870: the pose-source report's logic (tier 1, ROS-free).

Every fixture is synthetic. The report records numbers and fails only on
structure: nothing here (or in the module) ranks methods or bounds an error.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

from scripts.pose_source_report import (
    METHODS,
    RECORD_FIELDS,
    SCENARIOS,
    absent_record,
    cpu_ms_per_scan,
    parse_proc_stat,
    pose_rate,
    present_sources,
    record,
    render,
    single_match,
    structural_failures,
)

LIB_ROOT = Path(__file__).resolve().parents[2] / "tests" / "lib"
if str(LIB_ROOT) not in sys.path:
    sys.path.insert(0, str(LIB_ROOT))

BOTH = ["slam_toolbox", "amcl"]
SEEDS = (42, 7)


def _expected(launch_keys=BOTH, seeds=SEEDS) -> dict:
    return {
        "launch_keys": list(launch_keys),
        "scenarios": list(SCENARIOS),
        "seeds": list(seeds),
    }


def _done(source, scenario, seed, **overrides) -> dict:
    """A completed, fully probed record."""
    made = record(
        source,
        scenario,
        seed,
        run={
            "outcome": "completed",
            "time_scale": 5.0,
            "drive_on_estimate": False,
            "host_load_delta": 0.5,
            "error": None,
            "metrics": {
                "lap_completed": True,
                "lap_time": 120.4,
                "collision_count": 0,
            },
        },
        score={
            "ate_rmse": 0.3,
            "ate_maximum": 1.0,
            "availability": 1.0,
            "maximum_gap": 0.1,
        },
        rates={"pose_rate_hz": 40.0, "tf_rate_hz": 100.0, "scan_rate_hz": 40.0},
        cpu=2.0,
        map_accuracy={
            "wall_cell_count": 100,
            "maximum": 0.2,
            "p95": 0.1,
            "median": 0.04,
            "signed_median": 0.03,
            "signed_mean": 0.03,
        },
    )
    made.update(overrides)
    return made


def _all_records(**overrides) -> list[dict]:
    out = []
    for source in METHODS:
        for scenario in SCENARIOS:
            for seed in SEEDS:
                if source in BOTH:
                    out.append(_done(source, scenario, seed, **overrides))
                else:
                    out.append(absent_record(source, scenario, seed))
    return out


def test_check_1820_presence_is_launch_keys_intersect_methods() -> None:
    assert present_sources({"slam_toolbox", "amcl"}) == (BOTH, [])
    # METHODS order, never the launch's own.
    assert present_sources(["amcl", "slam_toolbox"]) == (BOTH, [])
    present, unknown = present_sources({"amcl", "foo", "slam_toolbox"})
    assert (present, unknown) == (BOTH, ["foo"])
    # mola and far_lio are in METHODS but not in this launch set: absent.
    assert present_sources({"amcl"}) == (["amcl"], [])
    failures = structural_failures(_all_records(), _expected(BOTH + ["foo"]))
    assert "foo: in the build but not in METHODS" in failures


def test_check_1830_one_key_set_for_every_record() -> None:
    absent = absent_record("mola", "spielberg", 42)
    done = _done("amcl", "spielberg", 42)
    assert tuple(absent) == RECORD_FIELDS
    assert tuple(done) == RECORD_FIELDS
    assert "map_wall_signed_mean" in RECORD_FIELDS
    assert absent["outcome"] == "absent"
    metrics = [f for f in RECORD_FIELDS if f.startswith(("ate", "map_wall"))]
    assert all(absent[f] is None for f in metrics)
    assert done["map_kind"] == "prior"
    assert done["map_wall_signed_mean"] == pytest.approx(0.03)
    assert done["laps_completed"] == 1
    assert done["latched"] is False
    assert done["cpu_ms_per_scan"] == 2.0
    # A record built from a run that failed leaves its metrics None.
    failed = record(
        "amcl",
        "spielberg",
        42,
        run={"outcome": "infrastructure", "error": "boom"},
        score=None,
        rates=None,
        cpu=None,
        map_accuracy=None,
    )
    assert tuple(failed) == RECORD_FIELDS
    assert failed["error"] == "boom" and failed["ate_rmse"] is None


def test_check_1840_pose_rate_counts_changed_transforms_only() -> None:
    samples = [
        (index * 0.01, (float(index // 10), 0.0, 0.0)) for index in range(100)
    ]
    rates = pose_rate(samples, 1.0)
    assert rates["pose_rate_hz"] == pytest.approx(10.0)
    assert rates["tf_rate_hz"] == pytest.approx(100.0)
    assert pose_rate([], 1.0) == {"pose_rate_hz": 0.0, "tf_rate_hz": 0.0}
    assert pose_rate(samples, 0.0) == {
        "pose_rate_hz": None,
        "tf_rate_hz": None,
    }


def test_check_1850_proc_stat_cpu_and_the_single_pid_rule() -> None:
    # Fields 3.. after the last ")": utime is field 14, stime field 15.
    tail = "S 1 2 3 4 5 6 7 8 9 10 {u} {s} 0 0"
    text = "1234 (a b) c) " + tail.format(u=30, s=20)
    assert parse_proc_stat(text) == 50
    assert parse_proc_stat("7 (plain) " + tail.format(u=1, s=2)) == 3
    assert cpu_ms_per_scan(100, 150, 100, 250) == pytest.approx(2.0)
    assert cpu_ms_per_scan(100, 150, 100, 0) is None
    cmdlines = {
        10: "/opt/ros/jazzy/lib/nav2_amcl/amcl --ros-args",
        11: "python3 something_else",
    }
    assert single_match(cmdlines, "lib/nav2_amcl/amcl") == 10
    with pytest.raises(ValueError, match="no process"):
        single_match(cmdlines, "lib/slam_toolbox/")
    cmdlines[12] = "/opt/ros/jazzy/lib/nav2_amcl/amcl"
    with pytest.raises(ValueError, match="2 processes"):
        single_match(cmdlines, "lib/nav2_amcl/amcl")
    # /proc/<pid>/cmdline separates arguments with NUL.
    assert single_match({5: "a\0lib/x/y\0b"}, "lib/x/y") == 5


def test_check_1860_only_structure_fails_never_a_value() -> None:
    assert structural_failures(_all_records(), _expected()) == []
    # No thresholds exist: an absurd error or map never fails.
    wild = _all_records(ate_rmse=1e6, map_wall_maximum=50.0)
    assert structural_failures(wild, _expected()) == []
    # absent and dnf never fail.
    dnf = _all_records(outcome="dnf")
    assert structural_failures(dnf, _expected()) == []
    for outcome in ("latched", "budget", "infrastructure"):
        bad = _all_records(outcome=outcome)
        failures = structural_failures(bad, _expected())
        assert any(f"amcl/spielberg/seed 42: {outcome}" in f for f in failures)
    missing = [r for r in _all_records() if r["seed"] != 7]
    assert any(
        "missing record: mola/spielberg/seed 7" in f
        for f in structural_failures(missing, _expected())
    )
    nan = structural_failures(_all_records(ate_rmse=math.nan), _expected())
    assert any("ate_rmse is not finite" in f for f in nan)
    silent = structural_failures(_all_records(pose_rate_hz=None), _expected())
    assert any("probe heard nothing" in f for f in silent)


def test_check_1870_render_order_is_fixed_not_ranked() -> None:
    records = _all_records()
    for index, row in enumerate(records):
        if row["outcome"] != "absent":
            row["ate_rmse"] = 100.0 - index  # would sort the other way
    shuffled = list(reversed(records))
    text = render(shuffled)
    assert text == render(records)
    lines = [line for line in text.splitlines() if line.startswith("|")]
    keys = []
    for line in lines[2:]:
        cells = [c.strip() for c in line.strip("|").split("|")]
        keys.append((cells[0], cells[1], int(cells[2])))
    expected = [
        (source, scenario, seed)
        for source in METHODS
        for scenario in SCENARIOS
        for seed in sorted(SEEDS)
    ]
    assert keys == expected
    mola = [line for line in lines if line.split("|")[1].strip() == "mola"]
    assert mola and all("absent" in line for line in mola)


def test_check_1860_a_cpu_probe_that_errored_is_a_structural_failure() -> None:
    """C1: a doubled or vanished estimator process must not pass silently."""
    row = _done("amcl", "spielberg", 42, cpu_error="2 processes match 'x'")
    assert row["cpu_error"] is not None
    assert row["cpu_error"] == "2 processes match 'x'"
    clean = _done("amcl", "spielberg", 42)
    assert clean["cpu_error"] is None
    records = _all_records()
    records[records.index(clean_of(records))]["cpu_error"] = "boom"
    failures = structural_failures(records, _expected())
    assert any("cpu probe: boom" in f for f in failures)


def clean_of(records) -> dict:
    return next(r for r in records if r["source"] == "amcl")


def test_check_1851_the_cpu_probe_records_a_doubled_process(tmp_path) -> None:
    from racing_test_keywords.probes import ProcessCpuProbe

    for pid in (11, 12):
        directory = tmp_path / str(pid)
        directory.mkdir()
        (directory / "cmdline").write_bytes(b"/x/lib/slam_toolbox/node\0--a")
        (directory / "stat").write_text("1 (n) S " + "0 " * 20)
    probe = ProcessCpuProbe("lib/slam_toolbox/", proc=tmp_path)
    probe.started()
    probe.close()
    assert probe.error is not None and "2 processes" in probe.error
    assert probe.cpu_ms_per_scan(100) is None
