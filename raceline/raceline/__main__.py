"""``python -m raceline``: optimize a raceline, verify it, write it.

Exit codes: 0 written, 2 rejected by the verifier (nothing written), 3 the
extractor refused the input (nothing written, TUM never run).
"""

from __future__ import annotations

import argparse
import hashlib
import sys
import tomllib
from pathlib import Path

import yaml

from raceline import verify
from raceline.extract import (
    Pose2D,
    extract_track,
    load_map,
    observed_mask,
    track_bounds,
)
from raceline.optimize import (
    DEFAULT_MARGIN,
    OPTIMIZERS,
    Raceline,
    optimize,
    write_raceline_csv,
)
from raceline.track import load_track
from raceline.vehicle import VehicleLimits, load_vehicle

PACKAGE = Path(__file__).resolve().parents[1]
DEFAULT_OUT = PACKAGE.parent / "config" / "racelines"
CODE_FILES = (
    "raceline/__main__.py",
    "raceline/extract.py",
    "raceline/geometry.py",
    "raceline/optimize.py",
    "raceline/track.py",
    "raceline/tum.py",
    "raceline/vehicle.py",
    "raceline/verify.py",
    "tum_env/run_tum.py",
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _blob(path: Path) -> str:
    """Git's blob SHA, equal to ``git rev-parse HEAD:<path>`` when clean."""
    data = Path(path).read_bytes()
    return hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()


def _tum_pin() -> dict[str, str]:
    deps = tomllib.loads((PACKAGE / "tum_env" / "pyproject.toml").read_text())[
        "project"
    ]["dependencies"]
    return {
        d.split("@")[0].split("=")[0].strip(): d
        for d in deps
        if "global-racetrajectory" in d or "trajectory-planning" in d
    }


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m raceline", description=__doc__)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--track", type=Path, help="a committed track YAML")
    src.add_argument("--map", type=Path, help="a map_server YAML")
    p.add_argument(
        "--start-pose", type=float, nargs=3, metavar=("X", "Y", "YAW")
    )
    p.add_argument("--direction", choices=("cw", "ccw"))
    p.add_argument("--walls", choices=("observed", "non_free"), default=None)
    p.add_argument("--vehicle", type=Path, required=True)
    p.add_argument("--optimizer", required=True, choices=sorted(OPTIMIZERS))
    p.add_argument("--ground-truth-track", type=Path)
    p.add_argument(
        "--ground-truth-map",
        type=Path,
        help="a map_server YAML whose occupied cells judge a sharp tip (D16)",
    )
    p.add_argument("--margin", type=float, default=DEFAULT_MARGIN)
    p.add_argument("--passes", type=int, default=2)
    p.add_argument("--name")
    p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    p.add_argument("--timeout", type=float)
    return p


def _input_map(args, name: str):
    """(track, input WallSet), or raises ValueError (the extractor's)."""
    grid = load_map(args.map)
    x, y, yaw = args.start_pose
    track = extract_track(
        grid,
        grid.resolution,
        grid.origin,
        Pose2D(x, y, yaw),
        args.direction,
        name=name,
        walls=args.walls,
    )
    right, left = track_bounds(grid, track.xy, walls=args.walls)
    return track, verify.WallSet(
        "input",
        left,
        right,
        left_observed=observed_mask(grid, left),
        right_observed=observed_mask(grid, right),
    )


def _verdict_doc(v: verify.Verdict) -> dict:
    return {
        "ok": v.ok,
        "min_clearance": {k: float(x) for k, x in v.min_clearance.items()},
        "max_abs_kappa": float(v.max_abs_kappa),
        "max_ay_ratio": float(v.max_ay_ratio),
        "max_ax_ratio": float(v.max_ax_ratio),
        "unobserved_share": {
            k: float(x) for k, x in v.unobserved_share.items()
        },
        "tips": v.tips,
    }


def _provenance(args, vehicle, track, line, verdict, argv) -> dict:
    source = args.map or args.track
    src = {"path": str(source), "sha256": _sha256(source)}
    if args.map:
        image = yaml.safe_load(Path(args.map).read_text())["image"]
        src["image_sha256"] = _sha256(Path(args.map).parent / image)
    gt = args.ground_truth_track
    gt_map = args.ground_truth_map
    gt_map_doc = None
    if gt_map:
        image = yaml.safe_load(Path(gt_map).read_text())["image"]
        gt_map_doc = {
            "path": str(gt_map),
            "sha256": _sha256(gt_map),
            "image_sha256": _sha256(Path(gt_map).parent / image),
        }
    tables = {
        "ggv_sha256": _sha256(vehicle.ggv_path),
        "ax_max_machines_sha256": _sha256(vehicle.ax_max_machines_path),
    }
    params = {
        "margin": args.margin,
        "safety_width": vehicle.safety_width(args.margin),
        "passes": args.passes,
        "allowance": verify.EXTRACTION_ALLOWANCE,
        "walls": args.walls,
    }
    if args.map:
        params["start_pose"] = list(args.start_pose)
        params["direction"] = args.direction
    from raceline.optimize import tum_ini

    params["ini"] = tum_ini(vehicle)
    return {
        "format_version": 1,
        "track": track.name,
        "vehicle_version": vehicle.version,
        "optimizer": args.optimizer,
        "source": src,
        "ground_truth": (
            {"path": str(gt), "sha256": _sha256(gt)} if gt else None
        ),
        "ground_truth_map": gt_map_doc,
        "vehicle": {
            "path": str(args.vehicle),
            "sha256": _sha256(args.vehicle),
            **tables,
        },
        "code": {p: _blob(PACKAGE / p) for p in CODE_FILES},
        "tum": _tum_pin(),
        "parameters": params,
        "verdict": _verdict_doc(verdict),
        "command": "python -m raceline " + " ".join(argv),
    }


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    args = _parser().parse_args(argv)
    if args.map:
        if args.start_pose is None or args.direction is None:
            sys.exit("--map needs --start-pose and --direction")
        args.walls = args.walls or "observed"
    elif args.walls or args.start_pose or args.direction:
        sys.exit("--walls, --start-pose and --direction go with --map")
    if args.ground_truth_map and not (args.ground_truth_track or args.track):
        print(
            "input: --ground-truth-map needs --ground-truth-track or --track",
            file=sys.stderr,
        )
        return 3
    stem = (args.map or args.track).stem.lower()
    name = args.name or stem
    vehicle: VehicleLimits = load_vehicle(args.vehicle)

    try:
        if args.map:
            track, input_walls = _input_map(args, name)
        else:
            track = load_track(args.track)
            input_walls = verify.track_walls(track, "input")
    except ValueError as err:
        print(f"input: {err}", file=sys.stderr)
        return 3
    cells = None
    if args.ground_truth_map:
        cells = verify.observed_cells(
            load_map(args.ground_truth_map), str(args.ground_truth_map)
        )
    if args.ground_truth_track:
        truth = verify.track_walls(
            load_track(args.ground_truth_track), "ground_truth", cells
        )
    elif args.track:
        truth = verify.track_walls(track, "ground_truth", cells)
    else:
        truth = None
        print(
            "warning: no --ground-truth-track; ground truth is unchecked",
            file=sys.stderr,
        )

    line: Raceline = optimize(
        track,
        vehicle,
        args.optimizer,
        margin=args.margin,
        passes=args.passes,
        timeout=args.timeout,
    )
    required = verify.required_clearances(vehicle, args.margin)
    pairs = [(input_walls, required["input"])]
    if truth is not None:
        pairs.append((truth, required["ground_truth"]))
    verdict = verify.verify(line, vehicle, pairs, track)
    if not verdict.ok:
        print("rejected by the verifier:", file=sys.stderr)
        for failure in verdict.failures:
            print(f"  {failure}", file=sys.stderr)
        return 2

    folder = Path(args.out_dir) / name / vehicle.version
    write_raceline_csv(line, folder / f"{args.optimizer}.csv")
    doc = _provenance(args, vehicle, track, line, verdict, argv)
    (folder / f"{args.optimizer}.yaml").write_text(
        yaml.safe_dump(doc, sort_keys=False)
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
