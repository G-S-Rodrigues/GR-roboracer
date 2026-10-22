"""Tests for scripts.raceline_report: the raceline comparison report's logic.

Tier 1 and ROS-free. Every fixture is synthetic and built in ``tmp_path``;
nothing here reads the real submodule, a recording or the committed lines.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from scripts.raceline_report import (
    compare_lines,
    normalise_command,
    provenance_diff,
    raceline_geometry,
    read_manifest,
    read_recording_tolerant,
    regeneration_argv,
    run_outcome,
    structural_failures,
    summarise,
    upstream_reference,
)

STATE = "/ground_truth/track_relative_state"
STATUS = "/safety/status"
LENGTH = 100.0
WINDOW = 5.0


def circle_csv(
    path: Path,
    points: int = 100,
    radius: float = 5.0,
    speed: float = 2.0,
    header_lines: int = 0,
    shift: tuple[float, float] = (0.0, 0.0),
    start: int = 0,
    speed_of=None,
) -> Path:
    rows = []
    arc = 0.0
    step = 2.0 * math.pi * radius / points
    for index in range(points):
        angle = 2.0 * math.pi * ((index + start) % points) / points
        v = speed if speed_of is None else speed_of(index)
        rows.append(
            ";".join(
                f"{value:.9f}"
                for value in (
                    arc,
                    radius * math.cos(angle) + shift[0],
                    radius * math.sin(angle) + shift[1],
                    angle + math.pi / 2.0,
                    1.0 / radius,
                    v,
                    0.0,
                )
            )
        )
        arc += step
    head = ["# header line"] * header_lines
    path.write_text("\n".join(head + rows) + "\n", encoding="utf-8")
    return path


def state(t: float, s: float, d: float = 0.0) -> dict:
    return {
        "record_type": "event",
        "stamp": {"sec": int(t), "nanosec": int(round((t % 1) * 1e9))},
        "topic": STATE,
        "payload": {"s": s % LENGTH, "d": d},
    }


def status(t: float, reason: int, clamps: int = 0) -> dict:
    return {
        "record_type": "event",
        "stamp": {"sec": int(t), "nanosec": int(round((t % 1) * 1e9))},
        "topic": STATUS,
        "payload": {"reason": reason, "active_clamps": clamps},
    }


def driving_then_frozen(freeze_at: float = 20.0, end: float = 40.0) -> list:
    events = []
    t = 0.5
    while t <= end:
        events.append(state(t, 3.0 * min(t, freeze_at)))
        t += 0.1
    return events


# CHECK-1440 -------------------------------------------------------------


def test_check_1440_circle_geometry(tmp_path: Path) -> None:
    geometry = raceline_geometry(circle_csv(tmp_path / "c.csv"))
    expected = 2.0 * math.pi * 5.0
    assert geometry["length_m"] == pytest.approx(expected, rel=5e-4)
    assert geometry["max_abs_kappa"] == pytest.approx(0.2)
    assert geometry["lap_estimate_s"] == pytest.approx(expected / 2.0, rel=5e-4)


# CHECK-1450 -------------------------------------------------------------


def outcome_of(events, text="", window=WINDOW) -> dict:
    return run_outcome(events, text, LENGTH, window)


def test_check_1450_frozen_tail_without_latch_is_dnf() -> None:
    result = outcome_of(driving_then_frozen())
    assert result["outcome"] == "dnf"
    assert result["distance_m"] == pytest.approx(3.0 * (20.0 - 0.5), abs=1e-6)


def test_check_1450_command_limit_with_frozen_tail_is_dnf() -> None:
    events = driving_then_frozen() + [status(10.0, 2, 4), status(30.0, 2, 4)]
    assert outcome_of(events)["outcome"] == "dnf"


def test_check_1450_track_limit_with_frozen_tail_is_dnf() -> None:
    events = driving_then_frozen() + [status(30.0, 1)]
    assert outcome_of(events)["outcome"] == "dnf"


@pytest.mark.parametrize(
    ("reason", "clamps"), [(3, 0), (4, 0), (2, 64), (0, 64)]
)
def test_check_1450_supervisor_latch_is_latched(reason, clamps) -> None:
    events = driving_then_frozen() + [status(12.0, reason, clamps)]
    assert outcome_of(events)["outcome"] == "latched"


def test_check_1450_observer_text_is_never_read() -> None:
    text = "first non-NONE status=REASON_COMMAND_LIMIT latched"
    result = outcome_of(driving_then_frozen(), text)
    assert result["outcome"] == "dnf"
    assert result["observer_text"] == text


def test_check_1450_still_moving_is_budget() -> None:
    result = outcome_of(driving_then_frozen(freeze_at=1e9, end=30.0))
    assert result["outcome"] == "budget"


def test_check_1450_distance_wraps_across_s_zero() -> None:
    events = []
    t = 0.5
    s = LENGTH - 2.0
    while t <= 30.0:
        events.append(state(t, s))
        s = min(s + 0.5, LENGTH + 3.0)
        t += 0.1
    result = outcome_of(events)
    assert result["outcome"] == "dnf"
    assert result["distance_m"] == pytest.approx(5.0, abs=1e-6)


def test_check_1450_held_at_zero_samples_are_ignored() -> None:
    """The sim publishes at /clock 0 before and at ``~/reset`` (K2).

    Only the last stamp-0 sample is the reset pose; distance counts from it.
    """
    events = [state(0.0, 10.0), state(0.0, 50.0)]
    events += [state(0.01 * i, 50.0 + 0.3 * i) for i in range(1, 400)]
    events += [state(0.01 * i, 50.0 + 0.3 * 399) for i in range(400, 700)]
    result = outcome_of(events, window=1.0)
    assert result["outcome"] == "dnf"
    assert result["distance_m"] == pytest.approx(0.3 * 399)


def test_check_1450_truncated_final_line_parses(tmp_path: Path) -> None:
    good = json.dumps(state(1.0, 1.0))
    path = tmp_path / "r.jsonl"
    path.write_text(
        json.dumps({"record_type": "manifest", "seed": 1})
        + "\n"
        + good
        + "\n"
        + good[: len(good) // 2],
        encoding="utf-8",
    )
    assert len(read_recording_tolerant(path)) == 1
    assert read_manifest(path)["seed"] == 1


def test_check_1450_truncated_middle_line_raises(tmp_path: Path) -> None:
    good = json.dumps(state(1.0, 1.0))
    path = tmp_path / "r.jsonl"
    path.write_text(good[:10] + "\n" + good + "\n", encoding="utf-8")
    with pytest.raises(ValueError):
        read_recording_tolerant(path)


def completed(vehicle, line, seed, lap) -> dict:
    return {
        "vehicle": vehicle,
        "line": line,
        "seed": seed,
        "outcome": "completed",
        "metrics": {"lap_time": lap, "collision_count": 0},
    }


def dnf(vehicle, line, seed, distance) -> dict:
    return {
        "vehicle": vehicle,
        "line": line,
        "seed": seed,
        "outcome": "dnf",
        "distance_m": distance,
    }


def test_check_1450_aggregation_never_shows_a_dnf_as_a_number() -> None:
    runs = [
        completed("v", "centerline", 1, 100.0),
        completed("v", "centerline", 2, 110.0),
        dnf("v", "centerline", 3, 40.0),
        dnf("v", "centerline", 4, 55.0),
    ]
    (row,) = summarise(runs)
    assert (row["n_completed"], row["n_total"]) == (2, 4)
    assert (row["lap_min"], row["lap_max"]) == (100.0, 110.0)
    assert row["lap_median"] == 105.0
    assert (row["dnf_min_m"], row["dnf_max_m"]) == (40.0, 55.0)
    (all_dnf,) = summarise([dnf("v", "mincurv", s, 10.0) for s in (1, 2)])
    assert all_dnf["n_completed"] == 0
    assert all_dnf["lap_min"] is None
    assert all_dnf["lap_median"] is None
    assert all_dnf["lap_max"] is None


# CHECK-1460 -------------------------------------------------------------

VERSIONS = {"gym": "gym-v1", "default": "default-v1"}
EXPECTED = {
    "vehicles": VERSIONS,
    "default_vehicle": "default",
    "seeds": [1, 2],
    "rows": {"gym": ["centerline"], "default": ["centerline"]},
}
GOOD_METRICS = {
    "lap_time": 100.0,
    "minimum_wall_clearance": 0.5,
    "maximum_tracking_error": 0.4,
    "p95_tracking_error": 0.3,
    "control_saturation_events": 0,
    "collision_count": 0,
}


def good_run(vehicle="default", seed=1, **override) -> dict:
    run = {
        "vehicle": vehicle,
        "line": "centerline",
        "seed": seed,
        "outcome": "completed",
        "metrics": {
            **GOOD_METRICS,
            "vehicle_parameter_version": VERSIONS[vehicle],
        },
        "manifest_version": VERSIONS[vehicle],
    }
    run.update(override)
    return run


def all_good() -> list:
    return [good_run(v, s) for v in VERSIONS for s in (1, 2)]


def gym_dnf(seed: int) -> dict:
    return {
        "vehicle": "gym",
        "line": "centerline",
        "seed": seed,
        "outcome": "dnf",
        "distance_m": 12.0,
        "manifest_version": "gym-v1",
    }


def test_check_1460_clean_set_has_no_failures() -> None:
    assert structural_failures(all_good(), EXPECTED) == []


def test_check_1460_gym_limits_dnf_alone_is_not_a_failure() -> None:
    runs = [good_run("default", 1), good_run("default", 2)]
    runs += [gym_dnf(1), gym_dnf(2)]
    assert structural_failures(runs, EXPECTED) == []


def failures_with(replacement: dict, drop=None) -> list:
    runs = [r for r in all_good() if (r["vehicle"], r["seed"]) != drop]
    if drop is None:
        runs = [r for r in runs if (r["vehicle"], r["seed"]) != ("default", 1)]
    return structural_failures(runs + [replacement], EXPECTED)


def test_check_1460_non_finite_metric() -> None:
    metrics = {**good_run()["metrics"], "lap_time": float("nan")}
    found = failures_with(good_run(metrics=metrics))
    assert any("lap_time" in line for line in found)


def test_check_1460_missing_row_and_seed() -> None:
    runs = [r for r in all_good() if (r["vehicle"], r["seed"]) != ("gym", 2)]
    found = structural_failures(runs, EXPECTED)
    assert len(found) == 1
    assert "missing" in found[0]


def test_check_1460_wrong_metrics_version() -> None:
    metrics = {**good_run()["metrics"], "vehicle_parameter_version": "gym-v1"}
    found = failures_with(good_run(metrics=metrics))
    assert any("default-v1" in line for line in found)


def test_check_1460_wrong_manifest_version_on_a_dnf() -> None:
    wrong = {**gym_dnf(2), "manifest_version": "default-v1"}
    runs = [r for r in all_good() if (r["vehicle"], r["seed"]) != ("gym", 2)]
    found = structural_failures(runs + [wrong], EXPECTED)
    assert any("gym-v1" in line for line in found)


def test_check_1460_infrastructure_timeout() -> None:
    bad = good_run(outcome="infrastructure", error="~/reset timed out")
    assert any("infrastructure" in x for x in failures_with(bad))


@pytest.mark.parametrize("outcome", ["latched", "budget"])
def test_check_1460_latched_or_budget_on_a_gym_vehicle(outcome) -> None:
    run = {**gym_dnf(2), "outcome": outcome}
    runs = [r for r in all_good() if (r["vehicle"], r["seed"]) != ("gym", 2)]
    found = structural_failures(runs + [run], EXPECTED)
    assert any(outcome in line for line in found)


def test_check_1460_default_vehicle_dnf_fails() -> None:
    run = {
        **gym_dnf(1),
        "vehicle": "default",
        "manifest_version": "default-v1",
    }
    found = failures_with(run)
    assert any("default" in line and "dnf" in line for line in found)


# CHECK-1470 -------------------------------------------------------------


def test_check_1470_shifted_line(tmp_path: Path) -> None:
    a = circle_csv(tmp_path / "a.csv")
    b = circle_csv(tmp_path / "b.csv", shift=(0.03, 0.0))
    result = compare_lines(a, b)
    assert result["max_xy_m"] == pytest.approx(0.03, abs=1e-6)
    assert result["max_dv_mps"] == pytest.approx(0.0, abs=1e-9)


def test_check_1470_reindexed_line_is_identical(tmp_path: Path) -> None:
    a = circle_csv(tmp_path / "a.csv")
    b = circle_csv(tmp_path / "b.csv", start=7)
    result = compare_lines(a, b)
    assert result["max_xy_m"] == pytest.approx(0.0, abs=1e-6)


def test_check_1470_speed_difference(tmp_path: Path) -> None:
    a = circle_csv(tmp_path / "a.csv")
    b = circle_csv(
        tmp_path / "b.csv", speed_of=lambda i: 2.0 + (0.25 if i == 40 else 0.0)
    )
    assert compare_lines(a, b)["max_dv_mps"] == pytest.approx(0.25)


def test_check_1470_row_count_mismatch_raises(tmp_path: Path) -> None:
    a = circle_csv(tmp_path / "a.csv", points=99)
    b = circle_csv(tmp_path / "b.csv", points=100)
    with pytest.raises(ValueError, match="99"):
        compare_lines(a, b)


# CHECK-1480 -------------------------------------------------------------


def provenance_doc() -> dict:
    return {
        "format_version": 1,
        "track": "spielberg",
        "source": {"path": "m.yaml", "sha256": "aa"},
        "ground_truth": {"path": "t.yaml", "sha256": "bb"},
        "ground_truth_map": {"path": "g.yaml", "sha256": "cc"},
        "vehicle": {"path": "v.yaml", "sha256": "dd"},
        "code": {"raceline/verify.py": "old", "raceline/tum.py": "same"},
        "tum": {"tph": "0.78"},
        "parameters": {"margin": 0.3, "start_pose": [0.0, 0.0, -2.879]},
        "verdict": {"ok": True, "max_abs_kappa": 0.4997, "tips": []},
        "command": "python -m raceline --map m.yaml --timeout 1800",
    }


def test_check_1480_parameter_off_by_1e12_is_a_difference() -> None:
    fresh = provenance_doc()
    fresh["parameters"]["margin"] = 0.3 + 1e-12
    result = provenance_diff(provenance_doc(), fresh)
    assert result["differences"]


def test_check_1480_verdict_is_reported_not_compared() -> None:
    fresh = provenance_doc()
    fresh["verdict"]["max_abs_kappa"] += 1e-4
    result = provenance_diff(provenance_doc(), fresh)
    assert result["differences"] == []
    assert result["verdict_deltas"]["max_abs_kappa"] == pytest.approx(1e-4)


def test_check_1480_code_drift_is_separate() -> None:
    fresh = provenance_doc()
    fresh["code"]["raceline/verify.py"] = "new"
    result = provenance_diff(provenance_doc(), fresh)
    assert result["differences"] == []
    assert list(result["code_drift"]) == ["raceline/verify.py"]


def test_check_1480_out_dir_in_the_command_is_normalised() -> None:
    fresh = provenance_doc()
    fresh["command"] += " --out-dir /tmp/x"
    assert provenance_diff(provenance_doc(), fresh)["differences"] == []


def test_check_1480_a_changed_input_hash_is_a_difference() -> None:
    fresh = provenance_doc()
    fresh["source"]["sha256"] = "zz"
    assert provenance_diff(provenance_doc(), fresh)["differences"]


# CHECK-1490 -------------------------------------------------------------


def test_check_1490_argv_from_command(tmp_path: Path) -> None:
    doc = {"command": "python -m raceline --map a.yaml --vehicle v.yaml"}
    assert regeneration_argv(doc, tmp_path) == [
        "python",
        "-m",
        "raceline",
        "--map",
        "a.yaml",
        "--vehicle",
        "v.yaml",
        "--out-dir",
        str(tmp_path),
    ]


def test_check_1490_foreign_prefix_raises(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="python -m raceline"):
        regeneration_argv({"command": "rm -rf / --map a"}, tmp_path)


def test_check_1490_existing_out_dir_raises(tmp_path: Path) -> None:
    doc = {"command": "python -m raceline --map a --out-dir /x"}
    with pytest.raises(ValueError, match="out-dir"):
        regeneration_argv(doc, tmp_path)


def test_check_1490_normalise_round_trip() -> None:
    command = "python -m raceline --map a.yaml --timeout 1800"
    assert normalise_command(command) == normalise_command(
        command + " --out-dir /tmp/x"
    )
    assert normalise_command(command) == command


# CHECK-1500 -------------------------------------------------------------


def test_check_1500_absent_directory_is_none(tmp_path: Path) -> None:
    assert upstream_reference(tmp_path / "nope", "Spielberg") is None


def test_check_1500_empty_directory_is_none(tmp_path: Path) -> None:
    assert upstream_reference(tmp_path, "Spielberg") is None


def test_check_1500_directory_without_the_track_is_none(tmp_path: Path) -> None:
    (tmp_path / "Monza").mkdir()
    assert upstream_reference(tmp_path, "Spielberg") is None


def test_check_1500_three_header_lines_parse(tmp_path: Path) -> None:
    folder = tmp_path / "Spielberg"
    folder.mkdir()
    circle_csv(folder / "Spielberg_raceline.csv", header_lines=3)
    reference = upstream_reference(tmp_path, "Spielberg")
    assert reference["length_m"] == pytest.approx(2.0 * math.pi * 5.0, rel=5e-4)
    assert reference["max_abs_kappa"] == pytest.approx(0.2)
    assert "upstream" in reference["label"]


def test_check_1500_headerless_file_parses_the_same(tmp_path: Path) -> None:
    folder = tmp_path / "Spielberg"
    folder.mkdir()
    circle_csv(folder / "Spielberg_raceline.csv", header_lines=0)
    reference = upstream_reference(tmp_path, "Spielberg")
    assert reference["lap_estimate_s"] == pytest.approx(math.pi * 5.0, rel=5e-4)
