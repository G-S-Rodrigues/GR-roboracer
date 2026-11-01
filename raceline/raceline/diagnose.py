"""Diagnostic plots for a raceline: where it is tight, and what binds it.

``write`` is what the generation CLI calls after ``verify`` (on success and on
rejection) and ``python -m raceline.diagnose LINE.csv ...`` calls on a
committed line. Every number comes from ``verify.profile``, the arrays the
verifier judges, so a plot cannot disagree with a verdict. Nothing here is a
verdict: the binding classes are a reading aid with the tolerances below.

Binding is read from the line alone. The wall bound is the optimizer's own
corridor, ``wall - width_opt / 2`` (``width_opt`` = 2 * (half width +
margin)), which is the required clearance plus the input-bound tolerance. The
power limit is not shown: it needs the drive force, which a line CSV lacks.
"""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path

import numpy as np

from raceline import verify
from raceline.optimize import Raceline, load_raceline_csv
from raceline.track import Track
from raceline.vehicle import VehicleLimits

# A limit counts as binding within this share of it (plotting only).
LIMIT_TOL = 0.01
# A node counts as touching the wall bound within this slack (m): about two
# of TUM's 0.025 m dn steps; the verifier's own tolerance is
# ``verify.INPUT_BOUND_TOL``.
WALL_ACTIVE_M = 0.05

PNG_NAMES = (
    "01_overview.png",
    "02_curvature.png",
    "03_usage.png",
    "04_clearance_speed.png",
    "05_binding.png",
)
# Priority when a node binds several: the first listed is its colour.
BINDING = (
    ("wall bound", "tab:red"),
    ("tyre (GGV)", "tab:purple"),
    ("speed cap", "tab:blue"),
)


def _ds_error(line: Raceline) -> np.ndarray:
    """``|ds - |dxy||`` per point, the closing step included."""
    s = np.append(line.s, 2 * line.s[-1] - line.s[-2])
    step = np.diff(s)
    nxt = np.roll(line.xy, -1, axis=0) - line.xy
    return np.abs(step - np.hypot(nxt[:, 0], nxt[:, 1]))


def binding_masks(
    prof: dict[str, np.ndarray], vehicle: VehicleLimits
) -> dict[str, np.ndarray]:
    """Which nodes sit on which limit (``BINDING``'s names)."""
    base = prof["required_input"] + verify.INPUT_BOUND_TOL
    return {
        "wall bound": prof["clearance_input"] - base <= WALL_ACTIVE_M,
        "tyre (GGV)": np.hypot(prof["ax_ratio"], prof["ay_ratio"])
        >= 1 - LIMIT_TOL,
        "speed cap": prof["v"] >= vehicle.v_max * (1 - LIMIT_TOL),
    }


def worst_points(prof: dict[str, np.ndarray]) -> dict[str, int]:
    """Indices of the points a reader looks at first, computed per line."""
    return {
        "max a_y": int(np.argmax(prof["ay_ratio"])),
        "max a_x": int(np.argmax(prof["ax_ratio"])),
        "min clearance": int(np.argmin(prof["clearance_input"])),
        "max |ds - |dxy||": int(np.argmax(prof["ds_error"])),
    }


def profile_with_extras(
    line: Raceline,
    vehicle: VehicleLimits,
    walls: Sequence[tuple[verify.WallSet, float]],
) -> dict[str, np.ndarray]:
    """``verify.profile`` plus ``ds_error`` and the binding masks."""
    prof = verify.profile(line, vehicle, walls)
    prof["ds_error"] = _ds_error(line)
    for name, mask in binding_masks(prof, vehicle).items():
        prof["binding_" + name.split()[0]] = mask
    return prof


def _close(poly: np.ndarray) -> np.ndarray:
    return np.vstack([poly, poly[:1]])


def _pyplot():
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt


def _save(fig, path: Path, plt) -> Path:
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def _draw_walls(ax, walls: Sequence[tuple[verify.WallSet, float]]) -> None:
    styles = {"input": ("k", "-"), "ground_truth": ("tab:green", "--")}
    for wall_set, _ in walls:
        colour, style = styles.get(wall_set.name, ("0.5", ":"))
        for k, poly in enumerate((wall_set.left, wall_set.right)):
            ax.plot(
                *_close(poly).T,
                color=colour,
                ls=style,
                lw=1,
                label=f"{wall_set.name} walls" if k == 0 else None,
            )


def write(
    line: Raceline,
    vehicle: VehicleLimits,
    walls: Sequence[tuple[verify.WallSet, float]],
    track: Track,
    out_dir: Path,
) -> list[Path]:
    """Write the five PNGs and ``profile.npz`` to ``out_dir``; return them."""
    plt = _pyplot()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    prof = profile_with_extras(line, vehicle, walls)
    worst = worst_points(prof)
    s, xy = prof["s"], line.xy
    paths = []

    fig, ax = plt.subplots(figsize=(11, 8))
    _draw_walls(ax, walls)
    ax.plot(*_close(track.xy).T, color="0.7", lw=0.8, label="centerline")
    sc = ax.scatter(xy[:, 0], xy[:, 1], c=prof["v"], s=4, cmap="viridis")
    fig.colorbar(sc, ax=ax, label="v [m/s]")
    for k, (name, i) in enumerate(worst.items()):
        ax.plot(
            *xy[i],
            marker="oxs^"[k],
            ms=9,
            ls="none",
            mfc="none",
            label=f"{name} (s = {s[i]:.1f} m)",
        )
    ax.set_aspect("equal")
    ax.legend(fontsize=7)
    ax.set_title("Line, walls and the worst points")
    paths.append(_save(fig, out / PNG_NAMES[0], plt))

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(s, prof["kappa"], lw=0.9, label="kappa (line)")
    ax.plot(s, prof["kappa_geometry"], lw=0.7, label="kappa (geometry)")
    for sign in (1, -1):
        ax.axhline(sign * vehicle.kappa_max, color="tab:red", ls="--", lw=0.8)
    ax.set_xlabel("s [m]")
    ax.set_ylabel("1/m")
    ax.legend(fontsize=8)
    ax.set_title("Curvature against the vehicle limit")
    paths.append(_save(fig, out / PNG_NAMES[1], plt))

    fig, (ax, gg) = plt.subplots(
        1, 2, figsize=(14, 4.5), gridspec_kw={"width_ratios": [2.2, 1]}
    )
    ax.plot(s, prof["ay_ratio"], lw=0.9, label="a_y / limit")
    ax.plot(s, prof["ax_ratio"], lw=0.9, label="a_x / limit")
    ax.axhline(1.0, color="tab:red", ls="--", lw=0.8)
    ax.set_xlabel("s [m]")
    ax.legend(fontsize=8)
    ax.set_title("Share of the GGV limits used")
    gg.scatter(prof["ay_ratio"], prof["ax_ratio"], s=5)
    angle = np.linspace(0, 2 * np.pi, 200)
    gg.plot(np.cos(angle), np.sin(angle), color="tab:red", lw=0.8)
    gg.set_xlabel("a_y / limit")
    gg.set_ylabel("a_x / limit")
    gg.set_aspect("equal")
    paths.append(_save(fig, out / PNG_NAMES[2], plt))

    fig, (ax, sp) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for wall_set, required in walls:
        ax.plot(
            s, prof[f"clearance_{wall_set.name}"], lw=0.9, label=wall_set.name
        )
        ax.axhline(required, ls="--", lw=0.7, color="0.4")
    ax.set_ylabel("clearance [m] (dashed: required)")
    ax.legend(fontsize=8)
    sp.plot(s, prof["v"], lw=0.9)
    sp.axhline(vehicle.v_max, color="tab:blue", ls="--", lw=0.8)
    sp.set_ylabel("v [m/s]")
    sp.set_xlabel("s [m]")
    paths.append(_save(fig, out / PNG_NAMES[3], plt))

    masks = binding_masks(prof, vehicle)
    primary = np.full(len(s), len(BINDING))
    for k in reversed(range(len(BINDING))):
        primary[masks[BINDING[k][0]]] = k
    colours = [c for _, c in BINDING] + ["0.8"]
    names = [n for n, _ in BINDING] + ["none"]
    fig, (ax, strip) = plt.subplots(
        2, 1, figsize=(11, 10), gridspec_kw={"height_ratios": [4, 1]}
    )
    _draw_walls(ax, walls)
    for k in range(len(names)):
        pick = primary == k
        ax.scatter(
            xy[pick, 0],
            xy[pick, 1],
            s=9,
            c=colours[k],
            label=f"{names[k]} ({pick.sum()} nodes, {100 * pick.mean():.0f}%)",
        )
    ax.set_aspect("equal")
    ax.legend(fontsize=8)
    ax.set_title(
        "Nodes by primary limit (wall bound > tyre > speed cap); "
        "bound = wall - width_opt / 2"
    )
    for k, (name, colour) in enumerate(BINDING):
        strip.plot(
            s[masks[name]], np.full(masks[name].sum(), k), "|", color=colour
        )
    strip.set_yticks(range(len(BINDING)))
    strip.set_yticklabels([n for n, _ in BINDING])
    strip.set_xlabel("s [m] (a node may bind several)")
    paths.append(_save(fig, out / PNG_NAMES[4], plt))

    npz = out / "profile.npz"
    np.savez(npz, **prof)
    return [*paths, npz]


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="python -m raceline.diagnose")
    p.add_argument("line", type=Path, help="a raceline CSV")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--track", type=Path, help="a committed track YAML")
    src.add_argument("--map", type=Path, help="a map_server YAML")
    p.add_argument(
        "--start-pose", type=float, nargs=3, metavar=("X", "Y", "YAW")
    )
    p.add_argument("--direction", choices=("cw", "ccw"))
    p.add_argument("--walls", choices=("observed", "non_free"), default=None)
    p.add_argument("--vehicle", type=Path, required=True)
    p.add_argument("--ground-truth-track", type=Path)
    p.add_argument(
        "--margin",
        type=float,
        help="the margin the line was generated with (default: the "
        "optimizer's, taken from the CSV's name)",
    )
    p.add_argument("--out", type=Path)
    return p


def main(argv: list[str] | None = None) -> int:
    # Imported here: the runner imports this module for ``write``.
    from raceline.__main__ import PACKAGE, _input_map
    from raceline.optimize import DEFAULT_MARGINS
    from raceline.track import load_track
    from raceline.vehicle import load_vehicle

    args = _parser().parse_args(argv)
    if args.map:
        if args.start_pose is None or args.direction is None:
            sys.exit("--map needs --start-pose and --direction")
        args.walls = args.walls or "observed"
    elif args.walls or args.start_pose or args.direction:
        sys.exit("--walls, --start-pose and --direction go with --map")
    optimizer = args.line.stem
    margin = args.margin
    if margin is None:
        if optimizer not in DEFAULT_MARGINS:
            sys.exit(f"--margin is needed: {optimizer!r} is not an optimizer")
        margin = DEFAULT_MARGINS[optimizer]
    vehicle = load_vehicle(args.vehicle)
    name = (args.map or args.track).stem.lower()
    try:
        if args.map:
            track, input_walls = _input_map(args, name)
        else:
            track = load_track(args.track)
            input_walls = verify.track_walls(track, "input")
    except ValueError as err:
        print(f"input: {err}", file=sys.stderr)
        return 3
    required = verify.required_clearances(vehicle, margin)
    pairs = [(input_walls, required["input"])]
    truth_track = args.ground_truth_track or args.track
    if truth_track:
        truth = verify.track_walls(
            load_track(truth_track) if args.ground_truth_track else track,
            "ground_truth",
        )
        pairs.append((truth, required["ground_truth"]))
    out = args.out or (
        PACKAGE.parent / "log" / "raceline" / name / vehicle.version / optimizer
    )
    for path in write(load_raceline_csv(args.line), vehicle, pairs, track, out):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
