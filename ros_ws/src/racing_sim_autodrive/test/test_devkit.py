"""Tier 1: front end B's ROS-free core (SIMAD-1420 to 1470).

The frame assembler pairs the devkit's 13 per-frame topics by DDS
`publication_sequence_number` (Step 1 measured: no loss at recorded pace,
the sequence identical across topics per frame, burst spread p100 7.2 ms
against inter-burst gaps down to 4.2 ms, so a time window cannot separate
frames and the sequence can). These tests feed it synthetic bursts: every
message of frame f carries f in its value, so a mispairing shows in the
emitted sample.
"""

import dataclasses
import logging
import random
from pathlib import Path
from types import SimpleNamespace

import pytest
from racing_common import CartesianPose as Pose2D
from racing_sim_autodrive import bridge, core, devkit, devkit_standin, replay

FIXTURE = Path(__file__).parent / "fixtures" / "porto_stream.jsonl.gz"
VEHICLE = bridge.VehicleValues(
    lr=0.15532,
    h=0.07354,
    wheel_radius=0.059,
    throttle_gain=24.859772,
    max_steering_angle=0.523633,
    kp=0.02,
    ki=0.05,
)
IDENTITY = Pose2D(0.0, 0.0, 0.0)
DRIVE = bridge.DriveRequest(speed=2.0, steering_angle=0.1)
PERIOD_NS = 18_000_000

# A per-topic sequence offset, so no test relies on them being equal.
BASE = {t: 1000 * i + 7 for i, t in enumerate(devkit.REQUIRED)}


def values_for(f):
    """The per-topic values the node extracts, all marked with frame f."""
    return {
        "throttle": 0.1 * f,
        "steering": 0.2 * f,
        "left_encoder": 1.0 * f,
        "right_encoder": 2.0 * f,
        "ips": (float(f), 2.0 * f, 0.05),
        "imu": ((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.1 * f), (0.0, 0.0, 9.8)),
        "odom": ((float(f), 2.0 * f, 0.05), (0.5, 0.0, 0.0)),
        "lidar": ([1.0, float(f)], 0.025),
        "lap_count": f,
        "lap_time": 0.02 * f,
        "last_lap_time": float("inf"),
        "best_lap_time": float("inf"),
        "collision_count": f,
    }


HEADERED = ("lidar", "imu", "left_encoder", "right_encoder", "odom")


def burst(f, skip=(), tweak=None, order=None, period=PERIOD_NS):
    """(topic, kwargs) for frame f, in `order` (default the devkit's)."""
    values = values_for(f)
    out = []
    for topic in order or devkit.REQUIRED:
        k = devkit.REQUIRED.index(topic)
        if topic in skip:
            continue
        src = f * period + k * 200_000
        hdr = src if topic in HEADERED else None
        item = {
            "value": values[topic],
            "source_stamp_ns": src,
            "sequence": BASE[topic] + f,
            "header_stamp_ns": hdr,
        }
        if tweak:
            tweak(topic, item)
        out.append((topic, item))
    return out


def feed(assembler, items):
    got = []
    for topic, item in items:
        got.extend(assembler.offer(topic, **item))
    return got


def frame_of(assembled):
    return assembled.sample.lap_count


def test_simad_1420_pairs_by_sequence_in_any_order_after_learning():
    """SIMAD-1420: the first burst is consumed to learn the offsets, the
    second (identical offsets) syncs and is emitted; from then on every
    complete burst, in any topic order, gives exactly one frame whose 13
    values are all from that frame. A partial burst gives none. The offsets
    are not assumed equal (BASE differs per topic)."""
    assembler = devkit.DevkitAssembler()
    rng = random.Random(7)
    assert feed(assembler, burst(0)) == []
    emitted = []
    for f in range(1, 40):
        order = list(devkit.REQUIRED)
        rng.shuffle(order)
        got = feed(assembler, burst(f, order=order))
        assert [frame_of(a) for a in got] == [f], f
        emitted += got
    sample = emitted[10].sample
    assert sample.collisions == 11
    assert sample.lap_time == pytest.approx(0.02 * 11)
    assert sample.left_encoder == 11.0 and sample.steering == pytest.approx(2.2)
    lidar_k = devkit.REQUIRED.index("lidar")
    assert emitted[10].stamp_ns == 11 * PERIOD_NS + lidar_k * 200_000
    assert assembler.resyncs == 0 and assembler.dropped_frames == 0
    partial = devkit.DevkitAssembler()
    assert feed(partial, burst(0)) == []
    assert feed(partial, burst(1, skip=("odom",))) == []


def test_simad_1420_frames_interleaving_across_bursts_pair_by_frame():
    """SIMAD-1420: frame k+1's messages arriving before frame k's last one
    (the executor serves subscriptions in any order) still pair k with k."""
    assembler = devkit.DevkitAssembler()
    feed(assembler, burst(0))
    feed(assembler, burst(1))
    late = burst(2)[-1]
    got = feed(assembler, burst(2)[:-1] + burst(3)[:-1])
    assert got == []  # frame 2 is one message short, frame 3 waits behind
    got = feed(assembler, [late])
    assert [frame_of(a) for a in got] == [2]
    got = feed(assembler, [burst(3)[-1]])
    assert [frame_of(a) for a in got] == [3]


def _after_learning(assembler, frames=(0, 1)):
    for f in frames:
        feed(assembler, burst(f))


def test_simad_1430_ips_not_equal_to_odom_position_resyncs():
    """SIMAD-1430: `ips` and `odom.position` are one parse of `V1 Position`;
    a frame where they differ is dropped, counted, logged ERROR once, and
    the assembler relearns (two bursts) and recovers."""
    assembler = devkit.DevkitAssembler()
    _after_learning(assembler)

    def tweak(topic, item):
        if topic == "ips":
            item["value"] = (99.0, 2.0, 0.05)

    class Collect(logging.Handler):
        def __init__(self):
            super().__init__(logging.ERROR)
            self.records = []

        def emit(self, record):
            self.records.append(record)

    seen = Collect()  # a handler of our own: pytest's is not wired here
    logger = logging.getLogger(devkit.__name__)
    logger.addHandler(seen)
    try:
        got = feed(assembler, burst(2, tweak=tweak))
        got += feed(assembler, burst(3, tweak=tweak))
    finally:
        logger.removeHandler(seen)
    assert got == []
    assert assembler.resyncs >= 1
    assert len(seen.records) == 1
    got = []
    for f in range(4, 10):
        got += feed(assembler, burst(f))
    assert [frame_of(a) for a in got][-3:] == [7, 8, 9]


@pytest.mark.parametrize("which", ["source", "header"])
def test_simad_1430_spread_beyond_skew_resyncs(which):
    """SIMAD-1430: a frame whose `source_timestamp` spread (or header-stamp
    spread) is beyond SKEW_S is dropped and resyncs."""
    assembler = devkit.DevkitAssembler()
    _after_learning(assembler)
    shift = int(3 * devkit.SKEW_S * 1e9)

    def tweak(topic, item):
        if topic == "imu":
            if which == "source":
                item["source_stamp_ns"] += shift
            else:
                item["header_stamp_ns"] += shift

    got = feed(assembler, burst(2, tweak=tweak))
    assert got == [] and assembler.resyncs == 1
    got = []
    for f in range(3, 9):
        got += feed(assembler, burst(f))
    assert [frame_of(a) for a in got][-2:] == [7, 8]


def test_simad_1430_one_dropped_lap_time_loses_one_frame_only():
    """SIMAD-1430: `lap_time` has no header, so only its sequence number can
    show a lost message. One lost message must lose that frame and nothing
    else: the next emitted frame is the one after it and all 13 of its
    values are from that frame, with no resync."""
    assembler = devkit.DevkitAssembler()
    _after_learning(assembler)
    got = []
    for f in range(2, 20):
        skip = ("lap_time",) if f == 9 else ()
        got += feed(assembler, burst(f, skip=skip))
    assert [frame_of(a) for a in got] == [f for f in range(2, 20) if f != 9]
    for a in got:
        assert a.sample.lap_time == pytest.approx(0.02 * a.sample.lap_count)
        assert a.sample.collisions == a.sample.lap_count
    assert assembler.lost["lap_time"] == 1
    assert assembler.resyncs == 0
    assert assembler.dropped_frames == 1


def test_simad_1430_a_loss_mid_burst_loses_at_most_one_frame():
    """SIMAD-1430: the lost message is detected when the next frame's
    message of that topic arrives, with frame k+1's other topics already
    queued; the broken frame goes and the rest still pair frame to frame."""
    assembler = devkit.DevkitAssembler()
    _after_learning(assembler)
    items = burst(2) + burst(3, skip=("odom",)) + burst(4)[:5]
    items += burst(4)[5:] + burst(5)
    got = feed(assembler, items)
    assert [frame_of(a) for a in got] == [2, 4, 5]
    assert assembler.lost["odom"] == 1 and assembler.resyncs == 0


def test_simad_1430_a_stalled_topic_that_resumes_with_a_jump():
    """SIMAD-1430: a topic silent for frames 10-14 resumes with a sequence
    jump of 6. The five frames are lost and counted, every message queued
    for them is dropped from the other topics, and from frame 15 on the
    pairing is right again with no resync."""
    assembler = devkit.DevkitAssembler()
    _after_learning(assembler)
    got = []
    for f in range(2, 25):
        skip = ("collision_count",) if 10 <= f <= 14 else ()
        got += feed(assembler, burst(f, skip=skip))
    assert [frame_of(a) for a in got] == [
        f for f in range(2, 25) if not 10 <= f <= 14
    ]
    for a in got:
        assert a.sample.collisions == a.sample.lap_count
    assert assembler.lost["collision_count"] == 5
    assert assembler.resyncs == 0
    assert assembler.dropped_frames == 5


def test_simad_1430_a_sequence_that_restarts_is_a_resync():
    """SIMAD-1430: a sequence number that does not rise (the devkit
    restarted) resyncs; two bursts later frames flow again."""
    assembler = devkit.DevkitAssembler()
    _after_learning(assembler)
    feed(assembler, burst(2))

    def restart(topic, item):
        item["sequence"] -= 500

    got = feed(assembler, burst(3, tweak=restart))
    assert assembler.resyncs == 1 and got == []
    got = feed(assembler, burst(4, tweak=restart))
    got += feed(assembler, burst(5, tweak=restart))
    assert [frame_of(a) for a in got] == [4, 5]


def test_simad_1440_session_is_receipt_only():
    """SIMAD-1440: B refuses `dt_source=lap_time` naming D4, and with no
    reset requested its replies do not depend on `lap_time` at all (the
    reset handshake is the one reader, `core.sim_dt`)."""
    with pytest.raises(ValueError, match="D4"):
        devkit.make_session(VEHICLE, IDENTITY, None, dt_source="lap_time")
    frames = replay.load_frames(FIXTURE)[1:300]

    def run(perturb):
        session = devkit.make_session(VEHICLE, IDENTITY, None)
        out = []
        for t, raw in frames:
            values = devkit_standin.devkit_values(raw)
            if perturb:
                values = dataclasses.replace(
                    values, lap_time=float(values.lap_time * 3.7 + 11)
                )
            frame = core.frame_from_devkit(devkit_standin.devkit_sample(values))
            step = session.on_sim_frame(frame, DRIVE, received_at=t)
            out.append((step.throttle, step.steering, step.reset, step.dt))
        return out

    assert run(False) == run(True)


def test_simad_1450_commands_and_the_reset_hold():
    """SIMAD-1450 (E7 b): throttle and steering follow the step; a reset the
    step fires publishes True once and holds it until the session reports
    the reset's effect seen, then False once; a repeated reset while
    holding is the same hold. Shutdown is zeros, plus False if a reset is
    pending."""
    commands = devkit.DevkitCommands()

    def step(reset, throttle=0.3, steering=-0.2):
        return SimpleNamespace(
            throttle=throttle, steering=steering, reset=reset
        )

    assert commands.from_step(step(False), False) == (0.3, -0.2, None)
    assert commands.from_step(step(True), True) == (0.3, -0.2, True)
    assert commands.pending_reset
    assert commands.from_step(step(False), True) == (0.3, -0.2, None)
    assert commands.from_step(step(True), True) == (0.3, -0.2, None)
    assert commands.from_step(step(False), True) == (0.3, -0.2, None)
    assert commands.from_step(step(False), False) == (0.3, -0.2, False)
    assert not commands.pending_reset
    assert commands.from_step(step(False), False) == (0.3, -0.2, None)
    assert devkit.shutdown_commands(False) == (0.0, 0.0, None)
    assert devkit.shutdown_commands(True) == (0.0, 0.0, False)


def test_simad_1450_the_hold_follows_the_real_session():
    """SIMAD-1450: through a real session over the fixture with a reset
    requested at frame 300, `reset_or_None` is True once and False once,
    and False comes only after the session left the awaiting state."""
    session = devkit.make_session(VEHICLE, IDENTITY, None)
    commands = devkit.DevkitCommands()
    seen = []
    for k, (t, raw) in enumerate(replay.load_frames(FIXTURE)[1:], start=1):
        if k == 300:
            session.request_reset()
        values = devkit_standin.devkit_values(raw)
        frame = core.frame_from_devkit(devkit_standin.devkit_sample(values))
        step = session.on_sim_frame(frame, DRIVE, received_at=t)
        _, _, reset = commands.from_step(step, session.awaiting_reset)
        if reset is not None:
            seen.append((k, reset, session.awaiting_reset))
    assert [r for _, r, _ in seen] == [True, False]
    assert seen[0][0] == 300 and seen[1][0] > 300
    assert seen[1][2] is False


def test_simad_1460_topic_lists_match_the_2026_guide():
    """SIMAD-1460: 13 required topics; the guide restricts `ips`, `odom`,
    the four lap/collision topics (7 of the 13, the guide's other two names
    being `reset_command` and `/tf`); the other 6 are allowed; B subscribes
    neither `front_camera` nor `/tf`."""
    assert len(devkit.REQUIRED) == len(set(devkit.REQUIRED)) == 13
    assert set(devkit.RESTRICTED) == {
        "ips",
        "odom",
        "lap_count",
        "lap_time",
        "last_lap_time",
        "best_lap_time",
        "collision_count",
    }
    assert set(devkit.ALLOWED) == {
        "lidar",
        "imu",
        "left_encoder",
        "right_encoder",
        "throttle",
        "steering",
    }
    assert set(devkit.ALLOWED) | set(devkit.RESTRICTED) == set(devkit.REQUIRED)
    assert not set(devkit.ALLOWED) & set(devkit.RESTRICTED)
    assert "front_camera" not in devkit.REQUIRED
    assert not any(t == "tf" or t.endswith("/tf") for t in devkit.REQUIRED)


def test_simad_1470_both_paths_give_the_same_session():
    """SIMAD-1470: the fixture through the socket path (`decode_frame`) and
    through the devkit path (stand-in values -> `frame_from_devkit`) into
    twin sessions with the same receipt stamps (the recorded `t`): throttle
    and steering within 1e-3 (the devkit rounds commands to 3 dp), ground
    truth within 1e-4, frames 1..616."""
    socket_side = devkit.make_session(VEHICLE, IDENTITY, None)
    devkit_side = devkit.make_session(VEHICLE, IDENTITY, None)
    for k, (t, raw) in enumerate(replay.load_frames(FIXTURE)):
        if k == 0:
            continue
        a = socket_side.on_sim_frame(core.decode_frame(raw), DRIVE, t)
        values = devkit_standin.devkit_values(raw)
        b = devkit_side.on_sim_frame(
            core.frame_from_devkit(devkit_standin.devkit_sample(values)),
            DRIVE,
            t,
        )
        assert abs(a.throttle - b.throttle) <= 1e-3, k
        assert abs(a.steering - b.steering) <= 1e-3, k
        for name in ("x", "y", "z", "yaw", "vx", "vy", "wz"):
            got, want = (
                getattr(b.ground_truth, name),
                getattr(a.ground_truth, name),
            )
            assert abs(got - want) <= 1e-4, (k, name)


def one_frame(a):
    """True when all 13 values of an assembled frame are from one frame."""
    f = a.sample.lap_count
    want = values_for(f)
    s = a.sample
    return (
        s.throttle == pytest.approx(want["throttle"])
        and s.steering == pytest.approx(want["steering"])
        and s.left_encoder == want["left_encoder"]
        and s.right_encoder == want["right_encoder"]
        and s.ips == want["ips"]
        and s.imu_angular_velocity == want["imu"][1]
        and s.ranges[1] == want["lidar"][0][1]
        and s.lap_time == pytest.approx(want["lap_time"])
        and s.collisions == f
    )


def assert_single_frames(got, last, minimum):
    """Every emitted frame is one burst, in order, ending at `last`, and at
    least `minimum` frames came out (recovery resumed output)."""
    frames = [frame_of(a) for a in got]
    assert all(one_frame(a) for a in got), frames
    assert frames == sorted(set(frames)), frames
    assert len(frames) >= minimum and frames[-1] == last, frames


@pytest.mark.parametrize("period", [PERIOD_NS, 8_000_000])
@pytest.mark.parametrize("missed", [3, 9])
def test_simad_1420_joining_mid_burst_never_mixes_frames(missed, period):
    """SIMAD-1420: a reader that joins after the first `missed` topics of a
    burst were published (docs/running.md starts the devkit first) sees the
    queue heads of different bursts. The devkit publishes each burst in
    REQUIRED order from one handler (autodrive_bridge.py `bridge`), so the
    alignment is by that order: no emitted frame mixes two bursts."""
    assembler = devkit.DevkitAssembler()
    skipped = devkit.REQUIRED[:missed]
    got = feed(assembler, burst(0, skip=skipped, period=period))
    for f in range(1, 30):
        got += feed(assembler, burst(f, period=period))
    assert_single_frames(got, 29, 25)
    assert assembler.resyncs == 0


@pytest.mark.parametrize("late", ["throttle", "lap_time", "collision_count"])
def test_simad_1420_a_topic_three_frames_late_still_syncs(late):
    """SIMAD-1420: a topic whose writer matched 3 frames after the rest
    (sequence numbers are the writer's own count, so it joins at 3) is
    aligned to its burst, not paired by queue position."""
    assembler = devkit.DevkitAssembler()
    got = []
    for f in range(30):
        got += feed(assembler, burst(f, skip=(late,) if f < 3 else ()))
    assert_single_frames(got, 29, 24)


def test_simad_1430_a_stall_past_the_backlog_then_recovery():
    """SIMAD-1430: a topic silent for 100 frames (more than _MAX_FRAMES)
    resumes. The backlog resync fires, and the frames after it are emitted
    again, each one a single burst."""
    assembler = devkit.DevkitAssembler()
    _after_learning(assembler)
    got = []
    for f in range(2, 100 + 250):
        skip = ("collision_count",) if 10 <= f < 110 else ()
        got += feed(assembler, burst(f, skip=skip))
    assert_single_frames(got, 349, 200)
    assert assembler.resyncs >= 1
    assert max(frame_of(a) for a in got if frame_of(a) < 10) == 9
    assert all(not 10 <= frame_of(a) < 110 for a in got)


def test_simad_1430_a_restart_whose_writers_rejoin_at_different_frames():
    """SIMAD-1430: the devkit restarts (sequences start over) and its 13
    writers rejoin the reader at frames 0..3 after the first. Frames after
    the restart are single bursts."""
    assembler = devkit.DevkitAssembler()
    _after_learning(assembler)
    got = []
    for f in range(2, 12):
        got += feed(assembler, burst(f))
    join = {t: i % 4 for i, t in enumerate(devkit.REQUIRED)}
    join["throttle"] = 0

    def restart(topic, item):
        # restarted writers count from 1 again
        item["sequence"] = item["sequence"] - BASE[topic] - 11 + 3 * len(topic)

    after = []
    for f in range(12, 50):
        skip = tuple(t for t in devkit.REQUIRED if f - 12 < join[t])
        after += feed(assembler, burst(f, skip=skip, tweak=restart))
    assert_single_frames(after, 49, 30)
    assert assembler.resyncs >= 1


def test_simad_1430_a_topic_that_never_comes_keeps_memory_bounded():
    """SIMAD-1430: with one topic absent from the start the assembler never
    syncs, emits nothing, and holds at most a bounded backlog per topic."""
    assembler = devkit.DevkitAssembler()
    got = []
    for f in range(500):
        got += feed(assembler, burst(f, skip=("collision_count",)))
    assert got == []
    assert all(len(q) <= 64 for q in assembler._pending.values())
