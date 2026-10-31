"""Pure tests for the seeded sensor and odometry noise model."""

import math

import numpy as np
from racing_sim_gym_jax.noise import (
    ImuNoise,
    NoiseStreams,
    OdometryNoise,
    OdometryState,
    apply_imu_noise,
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


IMU_NOISE = ImuNoise(
    gyro_sigma=0.01, accel_sigma=0.2, gyro_bias=0.002, accel_bias=0.05
)
NO_IMU_NOISE = ImuNoise(0.0, 0.0, 0.0, 0.0)
ACCELERATION = (0.5, -1.0, 9.81)


def _imu_sequence(seed, noise, count=20):
    streams = NoiseStreams.from_seed(seed, REFERENCE_NOISE, noise)
    return [streams.apply_imu(0.3, ACCELERATION) for _ in range(count)]


def test_simjax_1560_imu_noise_is_seeded_and_reseeding_restarts_it():
    """SIMJAX-1560: one seed gives one IMU noise sequence (bias included),
    another seed a different one, and a reseed - the `~/reset` path - starts
    the sequence over."""
    first = _imu_sequence(7, IMU_NOISE)
    assert first == _imu_sequence(7, IMU_NOISE)
    assert first != _imu_sequence(8, IMU_NOISE)
    assert first[0] != (0.3, ACCELERATION), "noise must change the reading"

    streams = NoiseStreams.from_seed(7, REFERENCE_NOISE, IMU_NOISE)
    drawn = [streams.apply_imu(0.3, ACCELERATION) for _ in range(5)]
    streams = NoiseStreams.from_seed(7, REFERENCE_NOISE, IMU_NOISE)
    assert [streams.apply_imu(0.3, ACCELERATION) for _ in range(5)] == drawn


def test_simjax_1560_zero_imu_noise_is_exact_and_draws_nothing():
    """Off means exact, and off draws nothing, so turning it off cannot shift
    any stream."""
    rng = np.random.default_rng(3)
    untouched = np.random.default_rng(3)
    bias = NO_IMU_NOISE.draw_bias(rng)
    assert apply_imu_noise(rng, 0.3, ACCELERATION, NO_IMU_NOISE, bias) == (
        0.3,
        ACCELERATION,
    )
    assert rng.random() == untouched.random()


def test_simjax_1570_imu_noise_leaves_scan_and_odometry_streams_alone():
    """SIMJAX-1570: with IMU noise on, the scan and odometry streams are
    bit-identical to IMU noise off. The IMU has its own third child of the
    run seed's SeedSequence; a draw from either of the others would shift
    every later scan or odometry sample."""
    on = NoiseStreams.from_seed(7, REFERENCE_NOISE, IMU_NOISE)
    off = NoiseStreams.from_seed(7, REFERENCE_NOISE, NO_IMU_NOISE)
    scan_on, scan_off, odometry_on, odometry_off = [], [], [], []
    for _ in range(50):
        on.apply_imu(0.3, ACCELERATION)
        scan_on.append(on.scan_rng.normal())
        scan_off.append(off.scan_rng.normal())
        odometry_on.append(on.odometry_rng.normal())
        odometry_off.append(off.odometry_rng.normal())
    assert on.odometry_calibration == off.odometry_calibration
    assert np.array_equal(scan_on, scan_off)
    assert np.array_equal(odometry_on, odometry_off)
