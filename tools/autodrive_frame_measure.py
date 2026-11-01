#!/usr/bin/env python3
"""Measure the AutoDRIVE world-to-map frame from a SIM-3730 run (#44 part 10).

    python3 tools/autodrive_frame_measure.py <out>/porto_run.npz

Run in `dev` with the overlay sourced. Fits the rigid correction that puts the
run's scan endpoints on Porto's walls (racing_evaluation.frame_fit), also on
the first, middle and last third of the poses (the spread says how well the
along-track shift is pinned), and prints the `world_to_map` that
`config/autodrive/porto_frame.yaml` should carry. Exit 0 when the correction
is inside the derived uncertainty (x 0.15 m, y 0.03 m, yaw 0.02 rad), else 1.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import yaml
from racing_evaluation.frame_fit import compose, fit_correction
from racing_evaluation.map_accuracy import load_grid

REPO = Path(__file__).resolve().parents[1]
MAP = REPO / "config/scenarios/maps/Porto/Porto.yaml"
FRAME = REPO / "config/autodrive/porto_frame.yaml"
LASER_X = 0.2733  # config/vehicles/autodrive_roboracer.yaml
UNCERTAINTY = (0.15, 0.03, 0.02)


def main(argv: list[str]) -> int:
    run = np.load(argv[1])
    grid = load_grid(MAP)
    frame = yaml.safe_load(FRAME.read_text(encoding="utf-8"))["world_to_map"]
    current = (frame["x"], frame["y"], frame["yaw"])
    kwargs = {
        "angle_min": float(run["angle_min"]),
        "angle_increment": float(run["angle_increment"]),
        "laser_x": LASER_X,
        "range_max": float(run["range_max"]) or 30.0,
    }
    poses, scans = run["poses"], run["scans"]
    full = fit_correction(poses, scans, grid, **kwargs)
    c = full.correction
    print(
        f"correction dx={c.dx:+.4f} dy={c.dy:+.4f} dyaw={c.dyaw:+.5f}"
        f" residual={full.residual:.4f} m points={full.points}"
    )
    cuts = np.array_split(np.arange(len(poses)), 3)
    for k, idx in enumerate(cuts):
        part = fit_correction(poses[idx], scans[idx], grid, **kwargs).correction
        print(
            f"  third {k}: dx={part.dx:+.4f} dy={part.dy:+.4f}"
            f" dyaw={part.dyaw:+.5f}"
        )
    x, y, yaw = compose(c, current)
    print(f"world_to_map: x: {x:.4f}  y: {y:.4f}  yaw: {yaw:.5f}")
    inside = (
        abs(c.dx) <= UNCERTAINTY[0]
        and abs(c.dy) <= UNCERTAINTY[1]
        and abs(c.dyaw) <= UNCERTAINTY[2]
    )
    print("inside the derived uncertainty" if inside else "OUTSIDE it")
    return 0 if inside else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
