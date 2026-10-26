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
