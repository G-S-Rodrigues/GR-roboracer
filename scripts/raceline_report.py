"""The tier-5 raceline comparison report and regeneration helpers.

ROS-free: standard library, ``yaml`` and ``numpy``. Only SIM-5050 adds
``racing_common`` (the track length a lap is counted on, repo-gotchas #15).
Everything here is a pure function of its arguments: no network, no git, and
nothing is read from disk that the caller did not name.
"""

from __future__ import annotations

import json
import math
import shlex
import statistics
from pathlib import Path

import numpy as np

from scripts.frozen_pose import FrozenPose

STATE_TOPIC = "/ground_truth/track_relative_state"
STATUS_TOPIC = "/safety/status"
COMMAND_PREFIX = ["python", "-m", "raceline"]
# SafetyStatus.msg: REASON_STALE_INPUT 3, REASON_ESTOP_LATCHED 4; the
# supervisor's CLAMP_ESTOP bit. COMMAND_LIMIT (2) and TRACK_LIMIT (1) are
# *not* latches: any clamp sets COMMAND_LIMIT (supervisor.cpp:45-81).
LATCH_REASONS = frozenset({3, 4})
CLAMP_ESTOP = 64
METRIC_FIELDS = (
    "lap_time",
    "minimum_wall_clearance",
    "maximum_tracking_error",
    "p95_tracking_error",
    "control_saturation_events",
)
# Inputs a regenerated line must reproduce exactly (R5, O4).
INPUT_KEYS = (
    "format_version",
    "track",
    "vehicle_version",
    "optimizer",
    "source",
    "ground_truth",
    "ground_truth_map",
    "vehicle",
    "tum",
    "parameters",
)


# -- geometry ---------------------------------------------------------------


def _load_csv(csv_path: Path) -> np.ndarray:
    """A TUM raceline CSV: 7 ';' columns, any leading '#' lines skipped."""
    rows = []
    for line in Path(csv_path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        rows.append([float(field) for field in line.split(";")])
    if not rows or any(len(row) != 7 for row in rows):
        raise ValueError(f"{csv_path}: expected 7 ';'-separated columns")
    return np.asarray(rows)


def _closed_segments(table: np.ndarray) -> np.ndarray:
    xy = table[:, 1:3]
    return np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)


def raceline_geometry(csv_path: Path) -> dict:
    """Closed length, max |kappa| and the sum of ds/v of a raceline CSV."""
    table = _load_csv(csv_path)
    ds = _closed_segments(table)
    speed = table[:, 5]
    mean_speed = 0.5 * (speed + np.roll(speed, -1))
    return {
        "length_m": float(ds.sum()),
        "max_abs_kappa": float(np.abs(table[:, 4]).max()),
        "lap_estimate_s": float((ds / mean_speed).sum()),
    }


def compare_lines(committed: Path, fresh: Path) -> dict:
    """Index-free nearest-point distance and speed difference of two lines.

    Both directions are taken and the worse reported, so a line that drops a
    stretch of the other does not pass. A row-count mismatch raises (C2).
    """
    a, b = _load_csv(committed), _load_csv(fresh)
    if len(a) != len(b):
        raise ValueError(
            f"row count differs: committed {len(a)} vs fresh {len(b)}"
        )
    worst_xy = worst_dv = 0.0
    for left, right in ((a, b), (b, a)):
        for start in range(0, len(left), 512):
            chunk = left[start : start + 512]
            delta = chunk[:, None, 1:3] - right[None, :, 1:3]
            distance = np.linalg.norm(delta, axis=2)
            nearest = distance.argmin(axis=1)
            worst_xy = max(worst_xy, float(distance.min(axis=1).max()))
            dv = np.abs(chunk[:, 5] - right[nearest, 5])
            worst_dv = max(worst_dv, float(dv.max()))
    return {"max_xy_m": worst_xy, "max_dv_mps": worst_dv}


def upstream_reference(root: Path, track: str) -> dict | None:
    """Geometry of the upstream raceline of ``track``, or None.

    None when the directory is absent, empty (an uninitialised submodule) or
    has no file for the track. Reads files only; never the network or git.
    """
    root = Path(root)
    if not root.is_dir() or not any(root.iterdir()):
        return None
    csv_path = root / track / f"{track}_raceline.csv"
    if not csv_path.is_file():
        return None
    return {
        **raceline_geometry(csv_path),
        "path": str(csv_path),
        "label": "upstream, its own vehicle model; not driven",
    }


# -- recordings -------------------------------------------------------------


def _records(path: Path) -> list[dict]:
    lines = [
        line
        for line in Path(path).read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    records = []
    for index, line in enumerate(lines):
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError as error:
            if index == len(lines) - 1:
                break  # a killed run may end on a truncated line (R3)
            raise ValueError(
                f"{path}: line {index + 1} is not JSON: {error}"
            ) from error
    return records


def read_recording_tolerant(path: Path) -> list[dict]:
    """Every event of a recording; skips only a truncated final line."""
    return [r for r in _records(path) if r.get("record_type") == "event"]


def read_manifest(path: Path) -> dict:
    for record in _records(path):
        if record.get("record_type") == "manifest":
            return record
    raise ValueError(f"{path}: no manifest record")


def _stamp(event: dict) -> float:
    return event["stamp"]["sec"] + event["stamp"]["nanosec"] * 1e-9


def _post_reset(states: list[dict]) -> list[dict]:
    """The state events at or after ``~/reset`` (K2).

    The sim is held at /clock 0 until the reset, and the samples it publishes
    meanwhile carry stamp 0, the last of them the reset pose itself (measured
    on gym-limits, seed 42: two stamp-0 samples, then the first moving one at
    0.01 s). The run starts at the last stamp-0 sample.
    """
    held = [i for i, e in enumerate(states) if _stamp(e) == 0.0]
    return states[held[-1] :] if held else states


def _wrap(delta: float, length: float) -> float:
    return (delta + length / 2.0) % length - length / 2.0


def run_outcome(events, observer_text, track_length, frozen_window_s) -> dict:
    """Classify a run that produced no metrics, from its recording alone.

    ``latched``: a /safety/status event with reason STALE_INPUT or
    ESTOP_LATCHED, or CLAMP_ESTOP in ``active_clamps`` (A1). ``dnf``: the
    pose (s and d) is constant over the last ``frozen_window_s`` of sim time.
    ``budget``: still moving. ``observer_text`` is carried for the report and
    never read to classify (its "latch" is any non-NONE reason). Only statuses
    stamped after 0 count: the sim is held at /clock 0 until ``~/reset`` and
    the supervisor's status during the hold is not the run's (N1; the same
    rule as ``_post_reset``).

    ``distance_m`` is forward progress from the first post-reset ``s``,
    unwrapped modulo ``track_length`` (R2).
    """
    states = _post_reset(
        sorted((e for e in events if e["topic"] == STATE_TOPIC), key=_stamp)
    )
    latched = any(
        e["topic"] == STATUS_TOPIC
        and _stamp(e) > 0.0
        and (
            e["payload"]["reason"] in LATCH_REASONS
            or e["payload"]["active_clamps"] & CLAMP_ESTOP
        )
        for e in events
    )
    distance = reached = 0.0
    for before, after in zip(states, states[1:], strict=False):
        distance += _wrap(
            after["payload"]["s"] - before["payload"]["s"], track_length
        )
        reached = max(reached, distance)
    pose = FrozenPose(frozen_window_s, track_length)
    for e in states:
        pose.update(_stamp(e), e["payload"]["s"], e["payload"]["d"])
    frozen = pose.frozen
    outcome = "latched" if latched else "dnf" if frozen else "budget"
    return {
        "outcome": outcome,
        "distance_m": reached,
        "observer_text": observer_text,
    }


# -- aggregation and structure ---------------------------------------------


def summarise(runs: list[dict], lines: dict | None = None) -> list[dict]:
    """One row per (vehicle, line). A DNF is never a lap time or a zero.

    ``lines`` maps ``(vehicle, line)`` to static fields (geometry, verifier
    clearance, solve time) merged into the row.
    """
    order: list[tuple[str, str]] = []
    for run in runs:
        key = (run["vehicle"], run["line"])
        if key not in order:
            order.append(key)
    rows = []
    for vehicle, line in order:
        group = [
            r for r in runs if (r["vehicle"], r["line"]) == (vehicle, line)
        ]
        done = [r for r in group if r["outcome"] == "completed"]
        laps = sorted(r["metrics"]["lap_time"] for r in done)
        stopped = [r["distance_m"] for r in group if r["outcome"] == "dnf"]
        clearances = [
            r["metrics"]["minimum_wall_clearance"]
            for r in done
            if r["metrics"].get("minimum_wall_clearance") is not None
        ]
        row = {
            "vehicle": vehicle,
            "line": line,
            "n_completed": len(done),
            "n_total": len(group),
            "outcomes": sorted({r["outcome"] for r in group}),
            "lap_min": laps[0] if laps else None,
            "lap_median": statistics.median(laps) if laps else None,
            "lap_max": laps[-1] if laps else None,
            "dnf_min_m": min(stopped) if stopped else None,
            "dnf_max_m": max(stopped) if stopped else None,
            "collisions": (
                sum(r["metrics"].get("collision_count", 0) for r in done)
                if done
                else None
            ),
            "min_wall_clearance": min(clearances) if clearances else None,
        }
        row.update((lines or {}).get((vehicle, line), {}))
        rows.append(row)
    return rows


def structural_failures(runs: list[dict], expected: dict) -> list[str]:
    """Why the comparison is structurally broken; empty when it is not.

    ``expected``: ``vehicles`` (name -> parameter version), ``default_vehicle``,
    ``seeds`` and ``rows`` (vehicle -> lines). A gym-limits ``dnf`` alone is
    not a failure (O1a); any non-completed default-vehicle run is (Q2).
    """
    failures = []
    by_key = {(r["vehicle"], r["line"], r["seed"]): r for r in runs}
    for vehicle, lines in expected["rows"].items():
        version = expected["vehicles"][vehicle]
        for line in lines:
            for seed in expected["seeds"]:
                label = f"{vehicle}/{line}/seed {seed}"
                run = by_key.get((vehicle, line, seed))
                if run is None:
                    failures.append(f"missing run: {label}")
                    continue
                outcome = run["outcome"]
                if outcome == "infrastructure":
                    failures.append(
                        f"{label}: infrastructure failure: "
                        f"{run.get('error', '')}"
                    )
                    continue
                if run.get("manifest_version") != version:
                    failures.append(
                        f"{label}: recording manifest vehicle_parameter_"
                        f"version {run.get('manifest_version')!r}, "
                        f"expected {version!r}"
                    )
                if outcome == "completed":
                    metrics = run["metrics"]
                    for field in METRIC_FIELDS:
                        value = metrics.get(field)
                        if not isinstance(value, (int, float)) or not (
                            math.isfinite(value)
                        ):
                            failures.append(
                                f"{label}: {field} is not finite: {value!r}"
                            )
                    if metrics.get("vehicle_parameter_version") != version:
                        failures.append(
                            f"{label}: metrics vehicle_parameter_version "
                            f"{metrics.get('vehicle_parameter_version')!r}, "
                            f"expected {version!r}"
                        )
                elif outcome in ("latched", "budget"):
                    failures.append(f"{label}: {outcome}")
                elif vehicle == expected["default_vehicle"]:
                    failures.append(
                        f"{label}: default-vehicle run ended {outcome}, "
                        "not a lap (Q2)"
                    )
    return failures


# -- regeneration -----------------------------------------------------------


def normalise_command(command: str) -> str:
    """``command`` without ``--out-dir <x>`` and ``--reference <x>``.

    The out dir differs between original and rerun by construction, and a
    min-time rerun is handed the committed reference line by path while the
    original wrote its own (CHECK-1560); what the reference *is* is compared
    through ``parameters.reference.sha256``.
    """
    tokens = shlex.split(command)
    kept = []
    skip = False
    for token in tokens:
        if skip:
            skip = False
        elif token in ("--out-dir", "--reference"):
            skip = True
        else:
            kept.append(token)
    return shlex.join(kept)


def mintime_code_changed(doc: dict, current: dict) -> list[str]:
    """What differs between the code a committed min-time line was proven with
    and the code now (CHECK-1570): ``tum:<name>`` for a dependency pin and
    ``code:<path>`` for a blob. ``current`` has the yaml's own ``tum`` and
    ``code`` shape, ``code`` holding the files that count. Empty = unchanged.
    """
    changed = [
        f"tum:{name}"
        for name in sorted(set(doc.get("tum", {})) | set(current["tum"]))
        if doc.get("tum", {}).get(name) != current["tum"].get(name)
    ]
    old = doc.get("code", {})
    changed += [
        f"code:{path}"
        for path in sorted(current["code"])
        if old.get(path) != current["code"][path]
    ]
    return changed


def read_regeneration(path: Path, not_before: float) -> dict:
    """The regeneration record at ``path``, or ``{}`` when it is missing or
    older than ``not_before`` (epoch seconds): a stale file from an earlier
    session must render "not measured", never as this run's result (N2)."""
    path = Path(path)
    if not path.is_file() or path.stat().st_mtime < not_before:
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def regeneration_argv(doc: dict, out_dir: Path) -> list[str]:
    """The recorded ``command:`` with ``--out-dir`` appended (R4)."""
    tokens = shlex.split(doc["command"])
    if tokens[: len(COMMAND_PREFIX)] != COMMAND_PREFIX:
        raise ValueError(
            f"command does not start with 'python -m raceline': "
            f"{doc['command']!r}"
        )
    if "--out-dir" in tokens:
        raise ValueError("command already carries an --out-dir")
    return [*tokens, "--out-dir", str(out_dir)]


def _differences(left, right, path: str = "") -> list[str]:
    if isinstance(left, dict) and isinstance(right, dict):
        found = []
        for key in sorted(set(left) | set(right), key=str):
            where = f"{path}.{key}" if path else str(key)
            if key not in left or key not in right:
                found.append(f"{where}: present on one side only")
            else:
                found.extend(_differences(left[key], right[key], where))
        return found
    if isinstance(left, list) and isinstance(right, list):
        if len(left) != len(right):
            return [f"{path}: length {len(left)} vs {len(right)}"]
        found = []
        for index, (a, b) in enumerate(zip(left, right, strict=True)):
            found.extend(_differences(a, b, f"{path}[{index}]"))
        return found
    return (
        []
        if left == right and type(left) is type(right)
        else [f"{path}: {left!r} != {right!r}"]
    )


def _numeric_deltas(left, right, path: str = "") -> dict[str, float]:
    if isinstance(left, dict) and isinstance(right, dict):
        found = {}
        for key in left.keys() & right.keys():
            where = f"{path}.{key}" if path else str(key)
            found.update(_numeric_deltas(left[key], right[key], where))
        return found
    if isinstance(left, list) and isinstance(right, list):
        found = {}
        for index, (a, b) in enumerate(zip(left, right, strict=True)):
            found.update(_numeric_deltas(a, b, f"{path}[{index}]"))
        return found
    numeric = (int, float)
    if (
        isinstance(left, numeric)
        and isinstance(right, numeric)
        and not isinstance(left, bool)
        and not isinstance(right, bool)
    ):
        return {path: right - left}
    return {}


def provenance_diff(committed: dict, fresh: dict) -> dict:
    """Exact on the inputs; ``verdict`` and ``code`` are reported (R5, O4).

    ``differences``: input mismatches (the normalised ``command`` included).
    ``verdict_deltas``: fresh minus committed for every numeric verdict leaf.
    ``code_drift``: ``code:`` entries that differ, as (committed, fresh).
    """
    differences = []
    for key in INPUT_KEYS:
        if key in committed or key in fresh:
            differences.extend(
                _differences(committed.get(key), fresh.get(key), key)
            )
    if normalise_command(committed["command"]) != normalise_command(
        fresh["command"]
    ):
        differences.append("command: differs after normalisation")
    old_code, new_code = committed.get("code", {}), fresh.get("code", {})
    drift = {
        key: (old_code.get(key), new_code.get(key))
        for key in sorted(set(old_code) | set(new_code))
        if old_code.get(key) != new_code.get(key)
    }
    return {
        "differences": differences,
        "verdict_deltas": _numeric_deltas(
            committed.get("verdict", {}), fresh.get("verdict", {})
        ),
        "code_drift": drift,
    }


# -- rendering --------------------------------------------------------------


def _cell(value, fmt: str = "{:.2f}", missing: str = "-") -> str:
    return missing if value is None else fmt.format(value)


def _laps(row: dict) -> str:
    if row["lap_min"] is None:
        return "DNF"
    return (
        f"{row['lap_min']:.2f} / {row['lap_median']:.2f} / {row['lap_max']:.2f}"
    )


def _dnf(row: dict) -> str:
    if row["dnf_min_m"] is None:
        return "-"
    return f"{row['dnf_min_m']:.1f} - {row['dnf_max_m']:.1f}"


def render(rows, regeneration, reference=None) -> str:
    """The report as markdown. JSON is the caller's ``json.dumps`` of the same
    arguments, written beside it."""
    out = [
        "# Raceline comparison",
        "",
        "Research output, not a gate (ADR 0005): no ranking is asserted.",
        "",
        "| vehicle | line | completed | lap s (min / median / max) | "
        "DNF distance m | collisions | min clearance m (metrics) | "
        "min clearance m (verifier) | path m | max kappa | TUM lap est s | "
        "solve s |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for row in rows:
        solve = row.get("solve_seconds")
        out.append(
            "| {vehicle} | {line} | {done}/{total} | {laps} | {dnf} | "
            "{coll} | {mwc} | {vmc} | {length} | {kappa} | {est} | "
            "{solve} |".format(
                vehicle=row["vehicle"],
                line=row["line"],
                done=row["n_completed"],
                total=row["n_total"],
                laps=_laps(row),
                dnf=_dnf(row),
                coll=_cell(row["collisions"], "{}"),
                mwc=_cell(row["min_wall_clearance"], "{:.3f}"),
                vmc=_cell(row.get("verifier_min_clearance"), "{:.3f}"),
                length=_cell(row.get("length_m"), "{:.1f}"),
                kappa=_cell(row.get("max_abs_kappa"), "{:.3f}"),
                est=_cell(row.get("lap_estimate_s")),
                solve=_cell(solve, "{:.0f}", "not measured"),
            )
        )
    out += ["", "## Upstream reference", ""]
    if reference is None:
        out.append("upstream reference: not available")
    else:
        out.append(
            f"{reference['label']}: path {reference['length_m']:.1f} m, "
            f"max kappa {reference['max_abs_kappa']:.3f}, "
            f"lap estimate {reference['lap_estimate_s']:.2f} s "
            f"(`{reference['path']}`)"
        )
    out += ["", "## Regeneration (SIM-5040)", ""]
    if not regeneration:
        out.append("not measured")
    else:
        out += [
            "| raceline | solve s | max dxy m | max dv m/s | code drift |",
            "|---|---|---|---|---|",
        ]
        for name, record in sorted(regeneration.items()):
            out.append(
                f"| {name} | {record['solve_seconds']:.0f} | "
                f"{record['max_xy_m']:.4f} | {record['max_dv_mps']:.4f} | "
                f"{', '.join(record['code_drift']) or '-'} |"
            )
    return "\n".join(out) + "\n"
