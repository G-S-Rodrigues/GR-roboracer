"""FTG-1070: the controller on Porto's real geometry.

Dev ground truth, test only: 1081 beams are ray-marched in the official
``Porto.pgm`` (what the AutoDRIVE lidar sees, no noise) at every 10th point of
the official clockwise centerline, the lidar 0.1180 m ahead of the pose
(lidar x 0.2733 minus the rear axle's 0.15532 behind the CoG, gotcha 26).
At each point the command must be finite with a positive speed, its target
beam must be free for at least ``bubble_radius``, and where the centerline's
curvature exceeds 1.0 1/m the steering must turn the same way. The fraction
of points that agree is at least ``AGREEMENT`` (D6).

The fixture ``porto_stream.jsonl.gz`` cannot test turning (the car barely
moves in it), so this uses the map. Nothing here is production code and
nothing in the controller reads the map.
"""

import math
import sys
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / "ros_ws/src/racing_evaluation"))
from racing_controller_follow_the_gap import gap  # noqa: E402
from racing_evaluation.map_accuracy import load_grid  # noqa: E402

ANGLE_MIN = -2.35619
INCREMENT = 0.004363323
BEAMS = 1081
RANGE_MIN = 0.06
RANGE_MAX = 10.0
LIDAR_AHEAD_OF_COG = 0.2733 - 0.15532
SAMPLE_EVERY = 10
TURN_KAPPA = 1.0  # 1/m: below this the steering sign is not asserted
# Fraction of sampled points that must agree. Measured 2026-10-31 with the
# bound at 1.0: 33 of 33 sampled points agree (fraction 1.000; one of them is
# a turn with |kappa| > 1.0, max |kappa| 1.45 1/m). D6's lower-bound rule is
# the worst measurement x 0.8, rounded down.
AGREEMENT = 0.8


def ray_march(grid, x, y, yaw):
    angles = ANGLE_MIN + INCREMENT * np.arange(BEAMS)
    steps = np.arange(0.05, RANGE_MAX, 0.02)
    theta = yaw + angles
    px = x + np.outer(np.cos(theta), steps)
    py = y + np.outer(np.sin(theta), steps)
    cols = np.floor((px - grid.origin[0]) / grid.resolution).astype(int)
    rows = np.floor((py - grid.origin[1]) / grid.resolution).astype(int)
    h, w = grid.occupied.shape
    outside = (cols < 0) | (cols >= w) | (rows < 0) | (rows >= h)
    hit = outside.copy()
    inside = ~outside
    hit[inside] = ~grid.free[rows[inside], cols[inside]]
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(steps) - 1)
    out = steps[first]
    out[~hit.any(axis=1)] = math.inf
    return out


def centerline():
    data = yaml.safe_load((REPO / "config/tracks/porto.yaml").read_text())
    return np.array([row[:2] for row in data["centerline"]], dtype=float)


def test_ftg_1070_porto_geometry():
    grid = load_grid(REPO / "config/scenarios/maps/Porto/Porto.yaml")
    pts = centerline()
    n = len(pts)
    nxt, prv = np.roll(pts, -1, axis=0), np.roll(pts, 1, axis=0)
    heading = np.arctan2(*(nxt - prv).T[::-1])
    dtheta = np.arctan2(
        np.sin(np.roll(heading, -1) - np.roll(heading, 1)),
        np.cos(np.roll(heading, -1) - np.roll(heading, 1)),
    )
    ds = np.hypot(*(nxt - prv).T)
    kappa = dtheta / ds
    p = gap.Parameters()
    failures = []
    sampled = list(range(0, n, SAMPLE_EVERY))
    for i in sampled:
        x, y = pts[i]
        yaw = heading[i]
        lx = x + LIDAR_AHEAD_OF_COG * math.cos(yaw)
        ly = y + LIDAR_AHEAD_OF_COG * math.sin(yaw)
        scan = ray_march(grid, lx, ly, yaw)
        steering, speed = gap.command(scan, ANGLE_MIN, INCREMENT, RANGE_MIN, p)
        angles, cleaned = gap.clean_ranges(
            scan, ANGLE_MIN, INCREMENT, RANGE_MIN, p
        )
        target = gap.choose_target(
            angles, gap.apply_bubble(angles, cleaned, p), p
        )
        ok = (
            math.isfinite(steering)
            and speed > 0.0
            and target is not None
            and cleaned[target] >= p.bubble_radius
        )
        if ok and abs(kappa[i]) > TURN_KAPPA:
            ok = steering * kappa[i] > 0.0
        if not ok:
            failures.append((i, round(float(kappa[i]), 2), round(steering, 3)))
    fraction = 1.0 - len(failures) / len(sampled)
    assert fraction >= AGREEMENT, (
        f"agreement {fraction:.3f} over {len(sampled)} points; first"
        f" disagreeing centerline index {failures[0][0]}; failures"
        f" (index, kappa, steering): {failures}"
    )
