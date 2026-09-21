import math
from pathlib import Path

import numpy as np
import pytest
from racing_sim_gym_jax.laser import LaserMount, laser_poses

# SIMJAX-1090: the float32 cast against the float64 reference measured a
# worst 1.9e-6 m over 3 poses x 64 beams (plan D6); the gym's cast measured
# 1.23 m. 1e-4 m is float32 headroom, and still far below what one theta_dis
# bin (2pi/2000 rad) of angle error does to a grazing beam.
SCAN_ANGLE_TOLERANCE_M = 1e-4
# SIMJAX-1100: the gym's scan against the reference at the gym spacing
# measured a worst 0.112 m (its own 0.01 m noise plus a truncated theta_dis
# table angle on grazing beams; plan D6); at the old fov/(N-1) label, 4.97 m.
GROUND_TRUTH_SPACING_TOLERANCE_M = 0.15


def test_simjax_1080_laser_pose_is_the_vehicle_pose_moved_by_the_mount():
    """SIMJAX-1080: /scan is cast from the laser, not from base_link.

    The mount is base_link -> laser, so it is rotated by the vehicle's yaw
    before it is added: 0.275 m ahead is +y for a car facing +y. Casting from
    base_link while stamping frame `laser` puts every beam 0.275 m ahead of
    where the sim cast it, once TF places it - a bias every estimator reads
    as a localization error.
    """
    mount = LaserMount(x=0.275, y=0.1, yaw=0.3)
    poses = np.array(
        [
            [0.0, 0.0, 0.0],
            [2.0, -1.0, math.pi / 2.0],
            [-3.0, 4.0, -3.0 * math.pi / 4.0],
        ],
        dtype=np.float32,
    )

    shifted = np.asarray(laser_poses(poses, mount))

    for (x, y, yaw), (lx, ly, lyaw) in zip(poses, shifted, strict=True):
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        assert lx == pytest.approx(
            x + cos_yaw * 0.275 - sin_yaw * 0.1, abs=1e-6
        )
        assert ly == pytest.approx(
            y + sin_yaw * 0.275 + cos_yaw * 0.1, abs=1e-6
        )
        assert lyaw == pytest.approx(yaw + 0.3, abs=1e-6)
    assert shifted[1, :2] == pytest.approx([2.0 - 0.1, -1.0 + 0.275], abs=1e-6)


def test_simjax_1080_the_zero_mount_is_the_vehicle_pose():
    """SIMJAX-1080: a mount at base_link casts from base_link, unchanged."""
    poses = np.array([[1.5, -2.5, 0.7]], dtype=np.float32)

    assert np.asarray(laser_poses(poses, LaserMount())) == pytest.approx(poses)
    assert LaserMount().is_origin
    assert not LaserMount(x=0.275).is_origin


SPIELBERG = (
    Path(__file__).resolve().parents[4]
    / "config"
    / "scenarios"
    / "spielberg.yaml"
)
# base_link -> laser, as config/vehicles/f1tenth_default.yaml states it.
MOUNT = LaserMount(x=0.275)


def _sphere_trace(grid, x, y, angle, eps, max_range):
    """jax_pf's ray march on the env's own distance transform, in float64,
    at an exact angle - the reference the cast is measured against."""
    dt, origin_x, origin_y, origin_c, origin_s, height, width, resolution = grid

    def distance(px, py):
        dx, dy = px - origin_x, py - origin_y
        row = int(
            np.clip((-dx * origin_s + dy * origin_c) / resolution, 0, height)
        )
        col = int(
            np.clip((dx * origin_c + dy * origin_s) / resolution, 0, width)
        )
        return float(dt[min(row, height - 1), min(col, width - 1)])

    cos_a, sin_a = math.cos(angle), math.sin(angle)
    step = distance(x, y)
    total = step
    while step > eps and total <= max_range:
        x, y = x + step * cos_a, y + step * sin_a
        step = distance(x, y)
        total += step
    return min(total, max_range)


@pytest.fixture(scope="module")
def spielberg_backend():
    """One backend per process: jax_pf asserts it traces its scan once."""
    from racing_sim_gym_jax.backend import DriveCommand, GymBackend
    from racing_sim_gym_jax.scenario import load_scenario

    backend = GymBackend(load_scenario(SPIELBERG), 42, MOUNT)
    backend.set_command(DriveCommand(speed=2.0))
    return backend


def test_simjax_1090_scan_beams_lie_where_the_laser_scan_declares(
    spielberg_backend,
):
    """SIMJAX-1090: beam k of /scan is cast at angle_min + k *
    angle_increment, the angles its LaserScan declares.

    The node declares angle_min = -fov/2 and angle_increment = fov/(N-1)
    (`node.py`'s LaserScan). jax_pf's `get_scan` spreads its N beams with
    `linspace(start, start + inc * N, num=N, endpoint=True)`, i.e. at
    inc * N/(N-1) apart, and f1tenth_gym_jax passes inc = fov/(N-1): its
    sweep is fov * N/(N-1) = 4.775 rad for 64 beams, not the declared 4.7,
    so the last beam sits 0.075 rad (4.3 deg) from where /scan says it is.
    It also reads each angle from a table `linspace(0, 2pi, theta_dis)`
    (endpoint included) at a truncated index, a further error of up to
    2pi/theta_dis. slam_toolbox mapped Spielberg into a warped, unclosed
    loop on that scan (plan D6). Measured on the gym's cast: up to 0.66 m
    per beam against this reference.

    The reference is an exact-angle float64 ray march on the env's own
    distance transform, from the laser's pose. The bound is the float32
    march's own measured disagreement with it (plan D6), not a noise
    allowance: /scan's cast is noise-free before the node adds the
    scenario's sensor noise.
    """
    backend = spielberg_backend
    env = backend._env
    grid = (
        np.asarray(env.distance_transform),
        float(env.orig_x),
        float(env.orig_y),
        float(env.orig_c),
        float(env.orig_s),
        int(env.height),
        int(env.width),
        float(env.resolution),
    )
    fov, beams = float(env.fov), int(env.num_beams)
    declared = -fov / 2.0 + np.arange(beams) * fov / (beams - 1)
    worst = 0.0
    for steps in (0, 100, 200):
        snapshot = backend.reset(42) if steps == 0 else None
        for _ in range(steps):
            snapshot = backend.step()
        ((lx, ly, lyaw),) = np.asarray(
            laser_poses(
                np.array([[snapshot.x, snapshot.y, snapshot.yaw]]), MOUNT
            ),
            dtype=float,
        )
        reference = np.array(
            [
                _sphere_trace(
                    grid,
                    lx,
                    ly,
                    lyaw + angle,
                    float(env.eps),
                    float(env.max_range),
                )
                for angle in declared
            ]
        )
        worst = max(
            worst, float(np.max(np.abs(snapshot.laser_scan - reference)))
        )
    assert worst <= SCAN_ANGLE_TOLERANCE_M, worst


def test_simjax_1100_ground_truth_scan_declares_the_gym_spacing(
    spielberg_backend,
):
    """SIMJAX-1100: /ground_truth/scan's angle_increment is the spacing
    the gym actually casts at, fov/(N-1) * N/(N-1).

    Its ranges are the gym's own and stay so: racing_metrics and every
    golden read them. Only the label is made honest, as D4 did for its
    frame; no consumer reads its angles (racing_metrics uses ranges only).
    Measured against the exact-angle reference: the gym's scan carries its
    own N(0, 0.01 m) noise and a truncated theta_dis-table angle (up to
    2pi/2000 rad), so the bound is looser than SIMJAX-1090's.
    """
    from racing_sim_gym_jax.laser import gym_angle_increment

    backend = spielberg_backend
    env = backend._env
    grid = (
        np.asarray(env.distance_transform),
        float(env.orig_x),
        float(env.orig_y),
        float(env.orig_c),
        float(env.orig_s),
        int(env.height),
        int(env.width),
        float(env.resolution),
    )
    fov, beams = float(env.fov), int(env.num_beams)
    declared = -fov / 2.0 + np.arange(beams) * gym_angle_increment(fov, beams)
    worst = 0.0
    for steps in (0, 100, 200):
        snapshot = backend.reset(42) if steps == 0 else None
        for _ in range(steps):
            snapshot = backend.step()
        reference = np.array(
            [
                _sphere_trace(
                    grid,
                    snapshot.x,
                    snapshot.y,
                    snapshot.yaw + angle,
                    float(env.eps),
                    float(env.max_range),
                )
                for angle in declared
            ]
        )
        worst = max(worst, float(np.max(np.abs(snapshot.scan - reference))))
    assert worst <= GROUND_TRUTH_SPACING_TOLERANCE_M, worst
