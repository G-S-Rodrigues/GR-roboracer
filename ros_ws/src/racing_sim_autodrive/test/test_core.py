"""Tier 1: the ROS-free AutoDRIVE core (SIMAD-*), against the part-1 fixture."""

import base64
import gzip
import importlib.util
import math
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from racing_sim_autodrive import control, core, lidar  # noqa: F401

REPO = Path(__file__).resolve().parents[4]
FIXTURE = Path(__file__).parent / "fixtures" / "porto_stream.jsonl.gz"
LIDAR_KEY = "V1 LIDAR Range Array"
# 1e-3 rad: the telemetry prints 4 decimals, so quaternion and Euler yaw
# agree to a few 1e-4 rad at most.
YAW_TOL = 1e-3


def _read_records(path):
    """Load tools/autodrive_record.py by path and read a recording."""
    spec = importlib.util.spec_from_file_location(
        "autodrive_record", REPO / "tools" / "autodrive_record.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.read_records(path)


def _fixture():
    header, events = _read_records(FIXTURE)
    sim = [e["data"] for e in events if e["dir"] == "sim"]
    bridge = [e["data"] for e in events if e["dir"] == "bridge"]
    return header, sim, bridge


def _sim_events():
    """(t, raw) of every sim frame."""
    _, events = _read_records(FIXTURE)
    return [(e["t"], e["data"]) for e in events if e["dir"] == "sim"]


def _encode_lidar(values):
    text = "\n".join(str(v) for v in values)
    return base64.b64encode(gzip.compress(text.encode())).decode()


def test_simad_1010_core_imports_without_rclpy():
    """SIMAD-1010: the modules import with no rclpy; CartesianPose is there."""
    code = (
        "import sys, racing_sim_autodrive.core, racing_sim_autodrive.lidar, "
        "racing_sim_autodrive.control; "
        "assert not any(m.startswith('rclpy') for m in sys.modules)"
    )
    result = subprocess.run(
        [sys.executable, "-c", code], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stderr
    pose = core.Pose2D(1.0, 2.0, 0.5)
    assert (pose.x, pose.y, pose.psi) == (1.0, 2.0, 0.5)


def test_simad_1050_decode_lidar_reads_count_keeps_inf_rejects_bad():
    """SIMAD-1050: beam count comes from the payload; inf survives."""
    values = [0.5, 1.25, float("inf"), 3.0, 0.0, 2.5, 10.0]
    out = lidar.decode_lidar(_encode_lidar(values))
    assert out.shape == (7,)
    assert np.array_equal(out, np.array(values))
    assert math.isinf(out[2])
    for bad in ("nan", "-0.5"):
        with pytest.raises(ValueError):
            lidar.decode_lidar(_encode_lidar([1.0, bad, 2.0]))
    _, sim, _ = _fixture()
    real = lidar.decode_lidar(sim[1][LIDAR_KEY])
    assert real.shape == (1081,)
    assert np.isinf(real).any()


def test_simad_1020_decode_frame_every_fixture_frame():
    """SIMAD-1020: all 617 sim frames decode; only frame 0 lacks lidar."""
    _, sim, _ = _fixture()
    assert len(sim) == 617
    frames = [core.decode_frame(raw) for raw in sim]
    assert frames[0].ranges is None
    for frame in frames[1:]:
        assert frame.ranges is not None
        assert frame.ranges.shape == (1081,)
    for frame in frames:
        assert isinstance(frame.lap_count, int)
        assert isinstance(frame.collisions, int)


def test_simad_1030_frame_zero_known_values():
    """SIMAD-1030: the spawn frame, read straight off the recording."""
    _, sim, _ = _fixture()
    frame = core.decode_frame(sim[0])
    assert frame.rear_axle == pytest.approx((0.8, 3.1599, 0.0599))
    assert frame.yaw == pytest.approx(-math.pi / 2, abs=YAW_TOL)
    assert math.isinf(frame.last_lap_time)
    assert math.isinf(frame.best_lap_time)
    assert frame.lap_count == 0
    assert frame.collisions == 0
    assert frame.lidar_scan_rate == 40.0
    assert frame.encoder_ticks == (0, 0)


def test_simad_1040_missing_key_raises_camera_and_euler_optional():
    """SIMAD-1040: a missing non-lidar key is a FrameError naming it."""
    _, sim, _ = _fixture()
    raw = dict(sim[1])
    del raw["V1 Position"]
    with pytest.raises(core.FrameError, match="V1 Position"):
        core.decode_frame(raw)
    raw = dict(sim[1])
    del raw["V1 Front Camera Image"]
    del raw["V1 Orientation Euler Angles"]
    assert core.decode_frame(raw).ranges is not None


def test_simad_1060_quaternion_yaw_matches_euler_yaw():
    """SIMAD-1060: quaternion yaw equals the Euler yaw on every frame."""
    _, sim, _ = _fixture()
    for raw in sim:
        euler_yaw = float(raw["V1 Orientation Euler Angles"].split()[2])
        want = math.remainder(euler_yaw, math.tau)
        got = core.decode_frame(raw).yaw
        assert abs(math.remainder(got - want, math.tau)) < YAW_TOL


def _pose(x, y, psi):
    return core.Pose2D(x, y, psi)


def test_simad_1070_rear_axle_to_cog_moves_lr_along_yaw():
    """SIMAD-1070: CoG = rear axle + lr along heading; input untouched."""
    lr = 0.17145
    for psi, dx, dy in [
        (0.0, lr, 0.0),
        (math.pi / 2, 0.0, lr),
        (-math.pi, -lr, 0.0),
    ]:
        pose = _pose(1.0, 2.0, psi)
        cog = core.rear_axle_to_cog(pose, lr)
        assert cog.x == pytest.approx(1.0 + dx, abs=1e-12)
        assert cog.y == pytest.approx(2.0 + dy, abs=1e-12)
        assert cog.psi == psi
        assert (pose.x, pose.y, pose.psi) == (1.0, 2.0, psi)
    same = core.rear_axle_to_cog(_pose(1.0, 2.0, 0.7), 0.0)
    assert (same.x, same.y, same.psi) == (1.0, 2.0, 0.7)
    _, sim, _ = _fixture()
    spawn = core.frame_pose(core.decode_frame(sim[0]))
    cog = core.rear_axle_to_cog(spawn, lr)
    assert cog.x == pytest.approx(0.8, abs=1e-3)
    assert cog.y == pytest.approx(3.1599 - lr, abs=1e-3)


def test_simad_1080_sim_dt_none_when_lap_time_does_not_increase():
    """SIMAD-1080: dt from the sim's lap clock; None across the reset."""
    events = _sim_events()
    frames = [(t, core.decode_frame(raw)) for t, raw in events]
    none_at = []
    positive = 0
    for (_, prev), (t, cur) in zip(frames, frames[1:]):
        dt = core.sim_dt(prev, cur)
        if dt is None:
            none_at.append(t)
        else:
            assert dt > 0
            positive += 1
    # Measured on the fixture: 25 drops + 15 repeats, all while the reset
    # level was held (13.03-13.51 s).
    assert len(none_at) == 40
    assert positive == 576
    assert min(none_at) >= 13.03 and max(none_at) <= 13.51


# 0.02 m/s: lap-time dt measured 0.006 median / 0.008 max against
# V1 Linear Velocity x on the 3.5-4.9 s straight (16 pairs, r = 0.059).
# Receive-time dt measured 0.106 median / 0.305 max on the same pairs
# (gotcha 35), which is why dt is the caller's and not read here.
SPEED_TOL = 0.02
WHEEL_RADIUS = 0.059  # test input only: z of V1 Position at rest


def test_simad_1090_encoder_speed_matches_linear_velocity():
    """SIMAD-1090: encoder speed with the sim's dt tracks V1 Linear Velocity."""
    frames = [(t, core.decode_frame(raw)) for t, raw in _sim_events()]
    checked = 0
    for (_, prev), (t, cur) in zip(frames, frames[1:]):
        if not 3.5 <= t <= 4.9:
            continue
        dt = core.sim_dt(prev, cur)
        speed = core.encoder_speed(prev, cur, WHEEL_RADIUS, dt)
        assert speed == pytest.approx(cur.linear_velocity[0], abs=SPEED_TOL)
        checked += 1
    assert checked >= 10
    a, b = frames[10][1], frames[11][1]
    assert core.encoder_speed(a, b, WHEEL_RADIUS, None) is None
    assert core.encoder_speed(a, b, WHEEL_RADIUS, 0.0) is None
    # encoders jumping backwards is a reset, never a negative speed
    fast = [f for _, f in frames if f.encoder_angles[0] > 1.0][0]
    assert core.encoder_speed(fast, frames[0][1], WHEEL_RADIUS, 0.05) is None


def test_simad_1100_throttle_loop_feedforward_zero_and_clamp():
    """SIMAD-1100: feedforward exact; v_cmd 0 is exactly 0.0; output clamped."""
    ff = control.ThrottleLoop(gain=24.9, kp=0.0, ki=0.0)
    assert ff.update(2.49, 1.0, 0.02) == pytest.approx(0.1)
    assert ff.update(2.49, None, None) == pytest.approx(0.1)
    assert ff.update(100.0, 0.0, 0.02) == 1.0
    assert ff.update(-100.0, 0.0, 0.02) == -1.0
    loop = control.ThrottleLoop(gain=24.9, kp=0.0, ki=1.0)
    for _ in range(20):
        loop.update(5.0, 0.0, 0.02)
    assert loop.integral > 0.0
    assert loop.update(0.0, 3.0, 0.02) == 0.0
    assert loop.integral == 0.0
    with pytest.raises(ValueError):
        control.ThrottleLoop(gain=0.0, kp=0.0, ki=0.0)


def test_simad_1110_anti_windup_and_closed_loop_with_wrong_gain():
    """SIMAD-1110: no windup while saturated; a 20 % wrong gain still settles."""
    # v_cmd / gain = 0.4 < limit, so the feedforward alone never saturates;
    # kp * e = 2.0 does.
    loop = control.ThrottleLoop(gain=24.9, kp=0.2, ki=0.5)
    for _ in range(50):
        assert loop.update(10.0, 0.0, 0.02) == 1.0
    assert abs(loop.integral) < 0.1
    out = loop.update(10.0, 11.0, 0.02)  # error reverses: e = -1
    assert out < 1.0
    # first-order plant, true gain 24.9 m/s per unit throttle, tau 0.3 s;
    # the loop is told a gain 20 % too high.
    true_gain, tau, dt = 24.9, 0.3, 0.02
    loop = control.ThrottleLoop(gain=true_gain * 1.2, kp=0.02, ki=0.1)
    speed = 0.0
    for _ in range(int(3.0 / dt)):
        u = loop.update(3.0, speed, dt)
        speed += dt / tau * (true_gain * u - speed)
    assert speed == pytest.approx(3.0, rel=0.01)


# 0.5236 rad: the sim echoes +0.1571 for a commanded +0.3, i.e. 0.3 x 0.5236.
# A fixture-derived test input, not a vehicle value (part 4 measures that).
MAX_STEER = 0.5236


def test_simad_1120_steering_to_normalized():
    """SIMAD-1120: plain ratio, clamped, sign kept; the echo maps back."""
    assert control.steering_to_normalized(0.2618, MAX_STEER) == pytest.approx(
        0.5, abs=1e-4
    )
    assert control.steering_to_normalized(9.0, MAX_STEER) == 1.0
    assert control.steering_to_normalized(-9.0, MAX_STEER) == -1.0
    with pytest.raises(ValueError):
        control.steering_to_normalized(0.1, 0.0)
    frames = [core.decode_frame(raw) for _, raw in _sim_events()]
    # The echo plateaus at {0, +-0.1571} (>= 97 frames each) and ramps
    # between them through 6 single-frame values: 0.1571 is the 0.3 x
    # 0.5236 the schedule commanded.
    echoes = [f.steering_echo for f in frames]
    plateaus = {
        round(control.steering_to_normalized(e, MAX_STEER), 2)
        for e in set(echoes)
        if echoes.count(e) >= 10
    }
    assert plateaus == {0.0, 0.3, -0.3}
    peak = max(abs(control.steering_to_normalized(e, MAX_STEER)) for e in echoes)
    assert peak == pytest.approx(0.3, abs=1e-3)
    moving = [f for f in frames if f.linear_velocity[0] > 0.5]
    positive = [f for f in moving if f.steering_echo > 0.1]
    assert positive
    assert all(f.angular_velocity[2] >= 0.0 for f in positive)


def test_simad_1130_reset_latch_rising_edge():
    """SIMAD-1130: one True per rising edge of the recorded reset level.

    The fixture held the level for 59 replies, so this does not show that
    one True alone resets the sim; that is verified live in part 7.
    """
    events = _read_records(FIXTURE)[1]
    levels = [
        (e["t"], e["data"]["V1 Reset"] == "True")
        for e in events
        if e["dir"] == "bridge"
    ]
    assert sum(1 for _, level in levels if level) == 59
    latch = control.ResetLatch()
    fired = [t for t, level in levels if latch.next(level)]
    assert len(fired) == 1
    assert latch.next(False) is False
    assert latch.next(True) is True
    assert latch.next(True) is False
    sim = [(t, core.decode_frame(e)) for t, e in _sim_events()]
    after = [f for t, f in sim if t > fired[0]]
    spawn = after[1]  # the sim acts within two frames
    assert spawn.encoder_angles == (0.0, 0.0)
    assert spawn.rear_axle[0] == pytest.approx(0.8, abs=0.05)
    assert spawn.rear_axle[1] == pytest.approx(3.16, abs=0.05)
