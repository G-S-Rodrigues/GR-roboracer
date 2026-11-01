"""Follow-the-gap: one lidar scan in, one steering and speed out.

ROS-free (numpy only). No pose, no map, no centerline: it can drive a track
nobody has mapped, which is why it is the mapping driver (part 11a). Angles
are REP-103: positive to the left, so an opening on the left gives a
positive steering.

Per scan: clean the ranges, zero a bubble round the nearest return, take the
widest run of free beams (the "gap") and aim at its deepest beam. Speed falls
with steering and rises with the clearance straight ahead. ``launch_limit``
is the speed cap a node applies after a standing start (see Parameters).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

CLEARANCE_HALF_ANGLE = 0.2  # rad either side of straight ahead


@dataclass(frozen=True)
class Parameters:
    # Zeroed round the nearest return, m. About twice the car's half width
    # plus the sensing noise; the Porto corridor is >= 1.18 m wide.
    bubble_radius: float = 0.3
    # Ranges are clipped here, m: a far wall is not a better target than a
    # near free one, and `inf` (no return) must not win by being infinite.
    max_range: float = 2.0
    # Beams outside +-fov/2 are ignored, rad (200 degrees).
    fov: float = 3.49
    # A beam is free when its range exceeds this, m.
    gap_threshold: float = 1.0
    maximum_steering_angle: float = 0.5236
    steering_gain: float = 1.0
    minimum_speed: float = 0.8
    maximum_speed: float = 2.0
    clearance_for_full_speed: float = 2.0
    # A standing start spins the wheels (part 11a measured 0.9 m of phantom
    # odometry from a full-throttle launch), so the speed is capped at
    # minimum_speed + launch_acceleration * t after release.
    launch_acceleration: float = 1.0  # m/s^2

    def validate(self) -> None:
        positive = (
            "bubble_radius",
            "max_range",
            "gap_threshold",
            "maximum_steering_angle",
            "steering_gain",
            "clearance_for_full_speed",
            "launch_acceleration",
        )
        for name in positive:
            if not getattr(self, name) > 0.0:
                raise ValueError(f"{name} must be > 0")
        if not 0.0 < self.fov <= math.tau:
            raise ValueError("fov must be in (0, 2*pi]")
        if not self.gap_threshold < self.max_range:
            raise ValueError("gap_threshold must be < max_range")
        if not 0.0 <= self.minimum_speed <= self.maximum_speed:
            raise ValueError("need 0 <= minimum_speed <= maximum_speed")
        if not self.maximum_speed > 0.0:
            raise ValueError("maximum_speed must be > 0")


def clean_ranges(ranges, angle_min, angle_increment, range_min, p):
    """(angles, ranges) of the beams inside the field of view, cleaned.

    ``inf``, NaN and anything under ``range_min`` (no usable return) become
    ``max_range``; the rest is clipped to it.
    """
    r = np.asarray(ranges, dtype=float).copy()
    angles = angle_min + angle_increment * np.arange(len(r))
    keep = np.abs(angles) <= p.fov / 2.0 + 1e-9
    r = r[keep]
    bad = ~np.isfinite(r) | (r < range_min)
    r[bad] = p.max_range
    return angles[keep], np.minimum(r, p.max_range)


def apply_bubble(angles, ranges, p):
    """``ranges`` with the beams within the bubble of the nearest return zeroed.

    The bubble's half-angle is ``asin(min(1, radius / d))`` for a nearest
    return at distance ``d``.
    """
    out = np.asarray(ranges, dtype=float).copy()
    i = int(np.argmin(out))
    half = math.asin(min(1.0, p.bubble_radius / max(out[i], 1e-9)))
    out[np.abs(angles - angles[i]) <= half] = 0.0
    return out


def choose_target(angles, ranges, p):
    """Index of the deepest beam of the widest free gap; None if none is free.

    Equal widths go to the gap nearer straight ahead. Among equally deep beams
    the one nearest the middle of that depth plateau is taken, so an open
    straight aims down its middle.
    """
    free = np.asarray(ranges) > p.gap_threshold
    if not free.any():
        return None
    edges = np.diff(np.concatenate(([0], free.astype(int), [0])))
    starts = np.nonzero(edges == 1)[0]
    ends = np.nonzero(edges == -1)[0]  # exclusive

    def key(k):
        a = angles[starts[k] : ends[k]]
        ahead = float(np.min(np.abs(a)))
        return (-(ends[k] - starts[k]), ahead)

    k = min(range(len(starts)), key=key)
    lo, hi = starts[k], ends[k]
    segment = np.asarray(ranges)[lo:hi]
    deepest = np.nonzero(segment >= segment.max() - 1e-9)[0] + lo
    centre = float(np.mean(angles[deepest]))
    return int(deepest[np.argmin(np.abs(angles[deepest] - centre))])


def speed(steering, clearance, p):
    """Speed for a steering angle and the clearance straight ahead, m/s."""
    steer = 1.0 - min(abs(steering) / p.maximum_steering_angle, 1.0)
    room = min(max(clearance / p.clearance_for_full_speed, 0.0), 1.0)
    return p.minimum_speed + (p.maximum_speed - p.minimum_speed) * steer * room


def command(ranges, angle_min, angle_increment, range_min, p):
    """(steering, speed) for one scan. No free gap: (0.0, 0.0), stop."""
    angles, cleaned = clean_ranges(
        ranges, angle_min, angle_increment, range_min, p
    )
    target = choose_target(angles, apply_bubble(angles, cleaned, p), p)
    if target is None:
        return 0.0, 0.0
    steering = float(
        np.clip(
            p.steering_gain * angles[target],
            -p.maximum_steering_angle,
            p.maximum_steering_angle,
        )
    )
    ahead = cleaned[np.abs(angles) <= CLEARANCE_HALF_ANGLE]
    clearance = float(ahead.min()) if len(ahead) else 0.0
    return steering, speed(steering, clearance, p)


def launch_limit(elapsed, p):
    """Speed cap ``elapsed`` seconds after a standing start was released."""
    return min(
        p.minimum_speed + p.launch_acceleration * max(elapsed, 0.0),
        p.maximum_speed,
    )


def limit_speed(value, elapsed, p):
    return min(value, launch_limit(elapsed, p))
