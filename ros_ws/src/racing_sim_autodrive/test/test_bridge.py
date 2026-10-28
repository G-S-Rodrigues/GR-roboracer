"""Tier 1: the ROS-free bridge session (SIMAD-1160 to 1230, 1260).

Everything the Socket.IO server node does between "a frame arrived" and "a
reply and some states to publish", against the part-1 fixture. No socket,
no rclpy, no clock: the node stamps (ADR 0006, free_running).

Vehicle values are the part-4 numbers copied with their anchors
(`config/vehicles/autodrive_roboracer/{source,identified}.yaml`, branch
44-autodrive-sim-4): lr 0.15532 (source.yaml com_x_lr_m), h 0.07354, wheel
radius 0.059, throttle gain 24.859772, max steering 0.523633. kp/ki are
UNTUNED test values (no gains exist yet; tuning is part 10).
"""

import math
import re
from pathlib import Path

import numpy as np
import pytest
from racing_common import CartesianPose as Pose2D
from racing_common import Track
from racing_sim_autodrive import bridge, control, core, replay

REPO = Path(__file__).resolve().parents[4]
FIXTURE = Path(__file__).parent / "fixtures" / "porto_stream.jsonl.gz"
PORTO = REPO / "config" / "tracks" / "porto.yaml"

VEHICLE = bridge.VehicleValues(
    lr=0.15532,
    h=0.07354,
    wheel_radius=0.059,
    throttle_gain=24.859772,
    max_steering_angle=0.523633,
    kp=0.02,  # untuned
    ki=0.05,  # untuned
)
IDENTITY = Pose2D(0.0, 0.0, 0.0)
DRIVE = bridge.DriveRequest(speed=2.0, steering_angle=0.1)


@pytest.fixture(scope="module")
def raws():
    return [raw for _, raw in replay.load_frames(FIXTURE)]


@pytest.fixture(scope="module")
def reset_start(raws):
    """Index of the first frame the recording answered with Reset True."""
    import gzip
    import json

    with gzip.open(FIXTURE, "rt") as handle:
        rows = [json.loads(line) for line in handle]
    replies = [r["data"] for r in rows if r.get("dir") == "bridge"]
    return next(i for i, r in enumerate(replies) if r["V1 Reset"] == "True")


def _session(**kwargs):
    kwargs.setdefault("world_to_map", IDENTITY)
    kwargs.setdefault("track", None)
    return bridge.BridgeSession(VEHICLE, **kwargs)


def test_simad_1160_every_frame_gets_a_wellformed_reply(raws):
    """SIMAD-1160: every fixture frame, the lidar-less first included,
    yields a reply with exactly the three keys the sim reads, as strings
    that parse (the recorded replies use the same three keys)."""
    session = _session()
    for k, raw in enumerate(raws):
        reply = session.on_frame(raw, DRIVE).reply
        assert set(reply) == {"V1 Throttle", "V1 Steering", "V1 Reset"}, k
        assert all(isinstance(v, str) for v in reply.values()), k
        assert math.isfinite(float(reply["V1 Throttle"])), k
        assert math.isfinite(float(reply["V1 Steering"])), k
        assert reply["V1 Reset"] in ("True", "False"), k
    assert "V1 LIDAR Range Array" not in raws[0]


def test_simad_1160_malformed_frame_raises_and_fallback_is_stopped(raws):
    """SIMAD-1160: a frame missing a key raises FrameError (the node then
    sends `fallback_reply`, so the sim is never left waiting, gotcha 29);
    the fallback commands nothing and does not reset."""
    broken = dict(raws[5])
    del broken["V1 Position"]
    with pytest.raises(core.FrameError):
        _session().on_frame(broken, DRIVE)
    assert bridge.fallback_reply() == {
        "V1 Throttle": "0.0000",
        "V1 Steering": "0.0000",
        "V1 Reset": "False",
    }


def test_simad_1170_ground_truth_is_the_cog_in_the_map_frame(raws):
    """SIMAD-1170: truth = V1 Position moved lr along heading
    (`rear_axle_to_cog`), then through world_to_map, at z = h; the twist is
    `V1 Linear Velocity` as reported (already the CoG, part 5 finding c)."""
    world_to_map = Pose2D(1.5, -2.0, 0.4)
    session = _session(world_to_map=world_to_map)
    k = 300
    step = None
    for raw in raws[: k + 1]:
        step = session.on_frame(raw, DRIVE)
    frame = core.decode_frame(raws[k])
    cog = core.rear_axle_to_cog(core.frame_pose(frame), VEHICLE.lr)
    c, s = math.cos(world_to_map.psi), math.sin(world_to_map.psi)
    truth = step.ground_truth
    assert truth.x == pytest.approx(c * cog.x - s * cog.y + world_to_map.x)
    assert truth.y == pytest.approx(s * cog.x + c * cog.y + world_to_map.y)
    assert truth.yaw == pytest.approx(
        math.remainder(cog.psi + world_to_map.psi, math.tau)
    )
    assert truth.z == VEHICLE.h
    assert (truth.vx, truth.vy) == frame.linear_velocity[:2]
    assert truth.wz == frame.angular_velocity[2]


def _manual_throttle(raws, first, last, speed_cmd, dts):
    """Hand computation of the throttle over raws[first..last] with the
    given per-pair dt (None = unknown), using only core/control."""
    loop = control.ThrottleLoop(VEHICLE.throttle_gain, VEHICLE.kp, VEHICLE.ki)
    prev = core.decode_frame(raws[first])
    out = []
    for raw, dt in zip(raws[first + 1 : last + 1], dts, strict=True):
        cur = core.decode_frame(raw)
        speed = core.encoder_speed(prev, cur, VEHICLE.wheel_radius, dt)
        out.append(loop.update(speed_cmd, speed, dt))
        prev = cur
    return out, loop


def test_simad_1180_default_dt_is_sim_dt_and_ignores_receipt_time(raws):
    """SIMAD-1180: by default (dt_source "lap_time", plan D4 / owner ruling
    2026-10-28) the throttle's dt is the `V1 Lap Time` delta: the same
    frames with different receive times give identical throttle, and the
    throttle equals a hand computation with `core.sim_dt`."""
    first, last = 120, 180
    frames = [core.decode_frame(r) for r in raws]
    dts = [
        core.sim_dt(frames[i - 1], frames[i])
        for i in range(first + 1, last + 1)
    ]
    assert any(d is not None for d in dts)
    expected, _ = _manual_throttle(raws, first, last, DRIVE.speed, dts)

    outs = []
    for stamps in (lambda i: i * 0.01, lambda i: i * 0.3 + 5.0):
        session = _session()
        session.on_frame(raws[first], DRIVE, received_at=stamps(first))
        got = [
            float(
                session.on_frame(raws[i], DRIVE, received_at=stamps(i)).reply[
                    "V1 Throttle"
                ]
            )
            for i in range(first + 1, last + 1)
        ]
        outs.append(got)
    assert outs[0] == outs[1]
    assert outs[0] == pytest.approx(expected, abs=6e-5)  # 4-decimal reply


def test_simad_1180_receipt_source_uses_consecutive_receive_stamps(raws):
    """SIMAD-1180: with dt_source "receipt" (the race-time path: never a
    sim-clock field) dt is the difference of consecutive receive stamps; the
    throttle equals the hand computation with those dts, and a missing stamp
    means no dt (feedforward plus the held integral)."""
    first, last = 120, 180
    period = 0.05
    dts = [period] * (last - first)
    expected, _ = _manual_throttle(raws, first, last, DRIVE.speed, dts)
    session = _session(dt_source="receipt")
    session.on_frame(raws[first], DRIVE, received_at=first * period)
    got = [
        float(
            session.on_frame(raws[i], DRIVE, received_at=i * period).reply[
                "V1 Throttle"
            ]
        )
        for i in range(first + 1, last + 1)
    ]
    assert got == pytest.approx(expected, abs=6e-5)
    # a frame without a stamp: dt None, integral unchanged
    before = session.integral
    step = session.on_frame(raws[last + 1], DRIVE, received_at=None)
    assert step.dt is None
    assert session.integral == before


def test_simad_1180_unknown_dt_is_feedforward_plus_held_integral(raws):
    """SIMAD-1180: when the lap clock does not rise (a repeated frame) the
    dt is None: the throttle is feedforward plus the integral built so far,
    and the integral is unchanged (control.py: not feedforward alone)."""
    session = _session()
    for raw in raws[120:160]:
        session.on_frame(raw, DRIVE)
    integral = session.integral
    assert integral != 0.0, "need a built integral for this test"
    step = session.on_frame(raws[159], DRIVE)  # same frame again
    assert step.dt is None and step.speed is None
    feedforward = DRIVE.speed / VEHICLE.throttle_gain
    assert float(step.reply["V1 Throttle"]) == pytest.approx(
        feedforward + integral, abs=6e-5
    )
    assert session.integral == integral


def test_simad_1190_reset_replies_true_once_and_reanchors(raws, reset_start):
    """SIMAD-1190: a reset request makes the next reply `V1 Reset True` and
    the one after False; the throttle integral is cleared; the dead
    reckoning is re-anchored to the truth until the sim has reset (the
    fixture's reset segment holds the lap clock for tens of frames)."""
    session = _session()
    for raw in raws[:reset_start]:
        session.on_frame(raw, DRIVE)
    assert session.integral != 0.0
    session.request_reset()
    first = session.on_frame(raws[reset_start], DRIVE)
    assert first.reply["V1 Reset"] == "True"
    assert session.integral == 0.0
    second = session.on_frame(raws[reset_start + 1], DRIVE)
    assert second.reply["V1 Reset"] == "False"
    # through the reset segment to a frame after the car is back at the start
    last = None
    for raw in raws[reset_start + 2 : reset_start + 70]:
        last = session.on_frame(raw, DRIVE)
    assert last.ground_truth.x == pytest.approx(0.8, abs=0.1)
    assert last.odometry.x == pytest.approx(last.ground_truth.x, abs=0.05)
    assert last.odometry.y == pytest.approx(last.ground_truth.y, abs=0.05)
    # ...and no further reset is replied without a request
    assert last.reply["V1 Reset"] == "False"


def test_simad_1190_a_reset_the_sim_never_shows_gives_up_with_an_error(raws):
    """SIMAD-1190: when the sim never shows the reset (a frame sequence in
    which the lap clock keeps running and the encoders keep rising), the
    session logs one ERROR after `RESET_WAIT_FRAMES` frames and returns to
    control: /odom dead-reckons again and the throttle loop integrates
    again, instead of copying the truth until the next lap line.

    The cap is read off the fixture (see the constant): the reset is fired in
    the driving segment, whose frames never contain a reset until 449."""
    import logging

    class Collect(logging.Handler):
        def __init__(self):
            super().__init__(logging.ERROR)
            self.records = []

        def emit(self, record):
            self.records.append(record)

    seen = Collect()  # a handler of our own: seen is not wired here
    cap = bridge.RESET_WAIT_FRAMES
    fire = 140
    assert fire + cap + 40 < 449
    session = _session()
    for raw in raws[:fire]:
        session.on_frame(raw, DRIVE)
    session.request_reset()
    assert session.on_frame(raws[fire], DRIVE).reply["V1 Reset"] == "True"
    logger = logging.getLogger(bridge.__name__)
    logger.addHandler(seen)
    try:
        for k in range(fire + 1, fire + cap):
            step = session.on_frame(raws[k], DRIVE)
            assert step.odometry.x == step.ground_truth.x, k  # still waiting
            assert session.integral == 0.0, k
        assert not seen.records
        step = session.on_frame(raws[fire + cap], DRIVE)
        assert [r.levelno for r in seen.records] == [logging.ERROR]
        assert "reset" in seen.records[0].getMessage()
        for k in range(fire + cap + 1, fire + cap + 40):
            step = session.on_frame(raws[k], DRIVE)
    finally:
        logger.removeHandler(seen)
    assert len(seen.records) == 1  # logged once, not per frame
    assert session.integral != 0.0  # the loop is back
    assert (
        step.odometry.x != step.ground_truth.x
        or step.odometry.y != step.ground_truth.y
    )  # dead reckoning again


def test_simad_1190_two_requests_before_a_frame_give_one_true(raws):
    """SIMAD-1190: requests are a level, not a count."""
    session = _session()
    session.on_frame(raws[0], None)
    session.request_reset()
    session.request_reset()
    assert session.on_frame(raws[1], None).reply["V1 Reset"] == "True"
    assert session.on_frame(raws[2], None).reply["V1 Reset"] == "False"


def test_simad_1200_drive_maps_to_normalized_steering_and_clamps(raws):
    """SIMAD-1200: steering = angle / max_steering_angle clamped to [-1, 1];
    no /drive yet: throttle 0 and steering 0."""
    k = 200
    for angle, want in (
        (0.1, 0.1 / VEHICLE.max_steering_angle),
        (-0.2, -0.2 / VEHICLE.max_steering_angle),
        (5.0, 1.0),
        (-5.0, -1.0),
    ):
        session = _session()
        for raw in raws[:k]:
            session.on_frame(raw, bridge.DriveRequest(1.0, angle))
        reply = session.on_frame(raws[k], bridge.DriveRequest(1.0, angle)).reply
        assert float(reply["V1 Steering"]) == pytest.approx(want, abs=6e-5)
    session = _session()
    for raw in raws[:k]:
        reply = session.on_frame(raw, None).reply
        assert float(reply["V1 Throttle"]) == 0.0
        assert float(reply["V1 Steering"]) == 0.0


def test_simad_1210_dead_reckoning_drifts_but_stays_bounded(raws):
    """SIMAD-1210: /odom is dead reckoning (encoder speed, yaw rate), so over
    the driving segment it differs from the truth by more than 1e-4 m (a
    ground-truth copy would not: ADAPT-2110) yet stays within 2 x MEASURED.

    The window is frames 132..336 (a session started at 132, commanded
    driving, before the reset segment). The recorded ground truth itself
    teleports at frames 83 and 131 (implied speed 7.3 m/s while the encoders
    read 0.92 m/s, yaw snapping back by -0.43 and -0.51 rad), so no
    dead-reckoned track can follow it there; an earlier window from frame 37
    measured 4.38 m for the correct integrator and let frozen odometry (7.12
    m) and a flipped yaw-rate sign (3.41 m) pass an 8.76 m bound. From 132
    the correct integrator measures MEASURED = 0.172 m, and the broken
    variants measure at least 0.49 m (speed x1.1), 0.79 (yaw rate 0), 1.14
    (yaw-rate sign flipped), 1.70 (speed x0.5) and 3.51 (frozen odometry),
    all above the bound 2 x 0.172 = 0.344 m.
    """
    session = _session()
    worst = 0.0
    for raw in raws[132:337]:
        step = session.on_frame(raw, DRIVE)
        worst = max(
            worst,
            math.hypot(
                step.odometry.x - step.ground_truth.x,
                step.odometry.y - step.ground_truth.y,
            ),
        )
    MEASURED = 0.172  # m
    assert worst > 1e-4
    assert worst < 2 * MEASURED


def test_simad_1220_scan_geometry_is_the_devkits_in_the_contract_frame():
    """SIMAD-1220: 1081 beams over the devkit's +-2.35619 rad at
    0.004363323 rad, range 0.06..10.0, in the contract's scan frame
    (contract.hpp kLaserFrame, parsed here; owner ruling 2026-10-28: the
    frame is `lidar` everywhere once part R1 renames it, and this follows
    the header). N comes from the array."""
    header = (
        REPO
        / "ros_ws/src/racing_sim_adapter/include/racing_sim_adapter"
        / "contract.hpp"
    ).read_text()
    contract_frame = re.search(r'kLaserFrame\{"([^"]+)"\}', header).group(1)
    geometry = bridge.scan_geometry(1081)
    assert geometry.frame_id == contract_frame
    assert geometry.angle_increment == pytest.approx(0.004363323)
    assert geometry.angle_min == pytest.approx(-2.35619, abs=1e-5)
    assert geometry.angle_max == pytest.approx(2.35619, abs=1e-5)
    assert geometry.range_min == 0.06 and geometry.range_max == 10.0
    assert (
        geometry.angle_min + geometry.angle_increment * 1080
        == pytest.approx(geometry.angle_max)
    )
    assert bridge.scan_geometry(541).angle_max == pytest.approx(
        geometry.angle_min + geometry.angle_increment * 540
    )


def test_simad_1220_scan_keeps_inf_and_is_absent_on_the_first_frame(raws):
    """SIMAD-1220: the first (lidar-less) frame yields no scan; later ones
    keep `inf` (no return) and carry no NaN."""
    session = _session()
    first = session.on_frame(raws[0], None)
    assert first.frame.ranges is None
    second = session.on_frame(raws[1], None)
    ranges = second.frame.ranges
    assert ranges is not None and len(ranges) == 1081
    assert np.isinf(ranges).any() and not np.isnan(ranges).any()


def test_simad_1230_track_state_structure(raws):
    """SIMAD-1230: with a track the step carries finite s, d, heading_error
    from `Track.to_frenet` on the map pose, and v_s, v_d from the speed
    (structure only: the world-to-map transform is part 10's); without a
    track it is None."""
    assert _session().on_frame(raws[200], DRIVE).track_state is None
    track = Track.from_yaml(str(PORTO))
    session = _session(track=track)
    step = None
    for raw in raws[:240]:
        step = session.on_frame(raw, DRIVE)
    state = step.track_state
    assert state is not None
    for value in (state.s, state.d, state.heading_error, state.v_s, state.v_d):
        assert math.isfinite(value)
    speed = step.ground_truth.vx
    assert state.v_s == pytest.approx(speed * math.cos(state.heading_error))
    assert state.v_d == pytest.approx(speed * math.sin(state.heading_error))


def test_simad_1260_reset_and_step_mode_handlers():
    """SIMAD-1260: reset accepts scenario_id "" or the configured one and
    rejects others; step_mode CONTINUOUS (0) succeeds, STEPPED (1) and
    anything else do not (the capability is false)."""
    assert bridge.handle_reset("", "porto")[0] is True
    assert bridge.handle_reset("porto", "porto")[0] is True
    ok, message = bridge.handle_reset("spielberg", "porto")
    assert ok is False and "spielberg" in message
    assert bridge.handle_step_mode(0)[0] is True
    ok, message = bridge.handle_step_mode(1)
    assert ok is False and message
    assert bridge.handle_step_mode(7)[0] is False
