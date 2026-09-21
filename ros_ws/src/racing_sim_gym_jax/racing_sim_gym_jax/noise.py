"""ROS-free, seeded sensor and odometry noise.

Applied on top of the simulator's exact outputs, which stay published
unchanged on /ground_truth/*. Everything is drawn from one
`numpy.random.Generator` seeded from the run seed, so a seeded run stays
reproducible.

Scan noise and odometry noise deliberately share no helper. Scan noise is a
per-sample perturbation; odometry noise is an *integrated* random walk plus a
per-run calibration error. A per-sample-jitter odometry has bounded error, so
`map -> odom` would have nothing to correct and every estimator's correction
would be identically zero - green tests, meaningless numbers (SIMJAX-1020).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


def apply_scan_noise(
    ranges: np.ndarray,
    rng: np.random.Generator,
    sigma_m: float,
    dropout_probability: float,
) -> np.ndarray:
    """Return a noisy copy of `ranges`: gaussian range error, then dropouts.

    A dropped beam reads +inf - REP 117's "no return", which is what
    slam_toolbox and nav2_amcl both treat as "nothing seen".
    """
    if sigma_m < 0.0 or not 0.0 <= dropout_probability <= 1.0:
        raise ValueError("sigma_m >= 0 and 0 <= dropout_probability <= 1")
    exact = np.asarray(ranges, dtype=float)
    noisy = exact + rng.normal(0.0, 1.0, exact.shape) * sigma_m
    noisy = np.maximum(noisy, 0.0)
    dropped = rng.random(exact.shape) < dropout_probability
    noisy[dropped] = math.inf
    return noisy


@dataclass(frozen=True)
class OdometryState:
    """A planar pose in the odometry frame."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class OdometryDelta:
    """True motion between two ticks, in the earlier pose's body frame."""

    forward: float
    lateral: float
    yaw: float


@dataclass(frozen=True)
class OdometryCalibration:
    """One run's wheel-odometry error: fixed scale errors plus the densities
    of the per-step random walk."""

    distance_scale_error: float
    yaw_scale_error: float
    distance_noise_density: float
    yaw_noise_density: float


@dataclass(frozen=True)
class OdometryNoise:
    """Noise magnitudes, as configured in a scenario file."""

    # Std of the per-run distance scale error (a residual speed_to_erpm_gain).
    distance_scale_sigma: float
    # Std of the per-run yaw-increment scale error (a residual steering gain).
    yaw_scale_sigma: float
    # Random-walk density of distance error, m per sqrt(m) travelled.
    distance_noise_density: float
    # Random-walk density of heading error, rad per sqrt(rad) turned.
    yaw_noise_density: float

    def draw_calibration(self, rng: np.random.Generator) -> OdometryCalibration:
        """Draw this run's calibration error - once per run, not per tick."""
        for value in (
            self.distance_scale_sigma,
            self.yaw_scale_sigma,
            self.distance_noise_density,
            self.yaw_noise_density,
        ):
            if value < 0.0:
                raise ValueError("odometry noise magnitudes must be >= 0")
        return OdometryCalibration(
            distance_scale_error=float(
                rng.normal(0.0, 1.0) * self.distance_scale_sigma
            ),
            yaw_scale_error=float(rng.normal(0.0, 1.0) * self.yaw_scale_sigma),
            distance_noise_density=self.distance_noise_density,
            yaw_noise_density=self.yaw_noise_density,
        )


def body_frame_delta(
    previous: OdometryState, current: OdometryState
) -> OdometryDelta:
    """The motion from `previous` to `current`, in `previous`'s body frame."""
    dx = current.x - previous.x
    dy = current.y - previous.y
    cos_yaw = math.cos(previous.yaw)
    sin_yaw = math.sin(previous.yaw)
    yaw = math.atan2(
        math.sin(current.yaw - previous.yaw),
        math.cos(current.yaw - previous.yaw),
    )
    return OdometryDelta(
        forward=cos_yaw * dx + sin_yaw * dy,
        lateral=-sin_yaw * dx + cos_yaw * dy,
        yaw=yaw,
    )


def integrate_noisy_odometry(
    previous: OdometryState,
    delta: OdometryDelta,
    rng: np.random.Generator,
    params: OdometryCalibration,
) -> OdometryState:
    """Compose one noisy increment onto the dead-reckoned pose.

    Noise variance is proportional to the distance (or angle) of the
    increment, so the accumulated error does not depend on the tick rate.
    With every magnitude zero this is exact SE(2) composition.
    """
    distance = math.hypot(delta.forward, delta.lateral)
    distance_scale = (
        1.0
        + params.distance_scale_error
        + (
            rng.normal(0.0, 1.0)
            * params.distance_noise_density
            * math.sqrt(distance)
            / distance
            if distance > 0.0
            else 0.0
        )
    )
    yaw = delta.yaw * (1.0 + params.yaw_scale_error) + rng.normal(
        0.0, 1.0
    ) * params.yaw_noise_density * math.sqrt(abs(delta.yaw))
    forward = delta.forward * distance_scale
    lateral = delta.lateral * distance_scale
    cos_yaw = math.cos(previous.yaw)
    sin_yaw = math.sin(previous.yaw)
    return OdometryState(
        x=previous.x + cos_yaw * forward - sin_yaw * lateral,
        y=previous.y + sin_yaw * forward + cos_yaw * lateral,
        yaw=math.atan2(
            math.sin(previous.yaw + yaw), math.cos(previous.yaw + yaw)
        ),
    )
