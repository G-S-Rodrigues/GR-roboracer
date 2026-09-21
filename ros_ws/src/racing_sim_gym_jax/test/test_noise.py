"""Pure tests for the seeded sensor and odometry noise model."""

import math

import numpy as np
from racing_sim_gym_jax.noise import (
    OdometryNoise,
    OdometryState,
    apply_scan_noise,
    body_frame_delta,
    integrate_noisy_odometry,
)

# The reference magnitudes config/scenarios/*.yaml carry.
REFERENCE_NOISE = OdometryNoise(
    distance_scale_sigma=0.01,
    yaw_scale_sigma=0.014,
    distance_noise_density=0.0144,
    yaw_noise_density=0.0248,
)


def test_simjax_1010_scan_noise_is_seeded():
    """SIMJAX-1010: one seed reproduces the noisy ranges byte for byte; a
    different seed does not."""
    ranges = np.linspace(0.5, 9.5, 64)

    first = apply_scan_noise(ranges, np.random.default_rng(7), 0.03, 0.01)
    again = apply_scan_noise(ranges, np.random.default_rng(7), 0.03, 0.01)
    other = apply_scan_noise(ranges, np.random.default_rng(8), 0.03, 0.01)

    assert first.tobytes() == again.tobytes()
    assert first.tobytes() != other.tobytes()
    assert not np.array_equal(first, ranges), "noise must change the scan"
    # Noise is on top of the exact scan, never a different scan.
    kept = np.isfinite(first)
    assert np.max(np.abs(first[kept] - ranges[kept])) < 0.2
    exact = apply_scan_noise(ranges, np.random.default_rng(7), 0.0, 0.0)
    np.testing.assert_array_equal(exact, ranges)


def _circle_path(length_m: float, step_m: float = 0.03):
    radius = 10.0
    count = int(length_m / step_m)
    for index in range(count + 1):
        angle = index * step_m / radius
        yield OdometryState(
            radius * math.sin(angle),
            radius - radius * math.cos(angle),
            angle,
        )


def _position_error_after(length_m: float, seed: int) -> float:
    rng = np.random.default_rng(seed)
    calibration = REFERENCE_NOISE.draw_calibration(rng)
    truth = list(_circle_path(length_m))
    estimate = truth[0]
    for previous, current in zip(truth, truth[1:], strict=False):
        estimate = integrate_noisy_odometry(
            estimate,
            body_frame_delta(previous, current),
            rng,
            calibration,
        )
    return math.hypot(estimate.x - truth[-1].x, estimate.y - truth[-1].y)


def test_simjax_1020_odometry_error_accumulates_with_distance():
    """SIMJAX-1020: dead-reckoning error after 100 m is far larger than after
    10 m. Asserts growth, not a magnitude: a per-sample-jitter odometry has
    bounded error, so map -> odom would have nothing to correct and every
    estimator's correction would be identically zero while tests stay green.
    """
    seeds = range(20)
    short = np.mean([_position_error_after(10.0, seed) for seed in seeds])
    long = np.mean([_position_error_after(100.0, seed) for seed in seeds])

    assert short > 0.0
    assert long > 5.0 * short, (short, long)


def test_simjax_1025_noise_free_odometry_is_exact():
    """SIMJAX-1025: with every magnitude zero, integration reproduces the
    true path - so any drift is the noise model's, not the integrator's."""
    rng = np.random.default_rng(0)
    calibration = OdometryNoise(0.0, 0.0, 0.0, 0.0).draw_calibration(rng)
    truth = list(_circle_path(30.0))
    estimate = truth[0]
    for previous, current in zip(truth, truth[1:], strict=False):
        estimate = integrate_noisy_odometry(
            estimate, body_frame_delta(previous, current), rng, calibration
        )
    assert math.isclose(estimate.x, truth[-1].x, abs_tol=1e-9)
    assert math.isclose(estimate.y, truth[-1].y, abs_tol=1e-9)
    assert math.isclose(estimate.yaw, truth[-1].yaw, abs_tol=1e-9)
