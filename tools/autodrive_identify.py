#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12,<3.13"
# dependencies = [
#   "python-socketio==4.2.0",
#   "python-engineio==3.13.0",
#   "gevent==26.9.0",
#   "gevent-websocket==0.10.1",
#   "pyyaml==6.0.3",
# ]
# ///
"""Identify the AutoDRIVE RoboRacer's vehicle model from a scripted drive.

Two subcommands:

    record   serve the sim exactly as tools/autodrive_record.py does, but
             answer with the identification drive (`identify_schedule`) and
             write the frames without the camera and the lidar.
    analyse  turn that recording into `identified.yaml`.

Run `record` first, then start the sim at it (the sim connects once and never
retries, docs/agents/repo-gotchas.md 30):

    env -u PYTHONPATH uv run --locked --script tools/autodrive_identify.py \
        record --config config/autodrive/simulator.yaml --out rec.jsonl.gz

The pure parts import only the standard library so tier 1 can load this file
by path. Every analysis function takes time from `V1 Lap Time`, the sim's own
clock, never from arrival time (the message rate follows host load,
gotcha 31).
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

SCHEDULE_ID = "porto-identify-v1"
REPEATS = 3
RESET_PULSE = 0.5
REST = 1.5
TAIL = 1.0
# Throttles held on the straight. Step 1's live probe (2026-10-26, Porto,
# spawn (0.8, 3.16) heading -y) found >= 7.9 m of clear straight: 0.02 to
# 0.12 reach a steady speed (spread < 0.1 % over 0.5 s), 0.2 is still
# accelerating (2 %) and is recorded for the gain's linearity only.
THROTTLES = (0.02, 0.04, 0.08, 0.12, 0.2)
# The steering step runs at 0.04 only: at 0.08 the car hit the wall during
# the second half of the step (probe segment 8, collision at lap time 3.25).
STEER_THROTTLE = 0.04


@dataclass(frozen=True)
class Segment:
    duration: float
    throttle: float
    steering: float
    reset: bool
    name: str


def _pre() -> list[Segment]:
    return [
        Segment(RESET_PULSE, 0.0, 0.0, True, "reset"),
        Segment(REST, 0.0, 0.0, False, "rest"),
    ]


def _steer_at_rest() -> list[Segment]:
    return [
        Segment(1.0, 0.0, 1.0, False, "steer_rest"),
        Segment(1.0, 0.0, -1.0, False, "steer_rest"),
        Segment(1.0, 0.0, 0.0, False, "coast"),
    ]


def _straight(throttle: float) -> list[Segment]:
    return [
        Segment(1.5, throttle, 0.0, False, "throttle"),
        Segment(1.5, 0.0, 0.0, False, "coast"),
    ]


def _burst() -> list[Segment]:
    return [
        Segment(0.3, 1.0, 0.0, False, "burst"),
        Segment(1.5, 0.0, 0.0, False, "coast"),
    ]


def _steer_step() -> list[Segment]:
    return [
        Segment(0.8, STEER_THROTTLE, 0.0, False, "throttle"),
        Segment(0.6, STEER_THROTTLE, 0.3, False, "steer_step"),
        Segment(0.6, STEER_THROTTLE, -0.3, False, "steer_step"),
        Segment(0.5, 0.0, 0.0, False, "coast"),
    ]


# One entry per manoeuvre; each is preceded by a reset pulse and a rest.
MANOEUVRES = (
    [_steer_at_rest]
    + [lambda th=th: _straight(th) for th in THROTTLES]
    + [_burst, _steer_step]
)


def build_schedule(repeats: int) -> list[Segment]:
    segments: list[Segment] = []
    for _ in range(repeats):
        for manoeuvre in MANOEUVRES:
            segments += _pre() + manoeuvre()
    return segments


_SEGMENTS = build_schedule(REPEATS)
SCHEDULE_END = sum(seg.duration for seg in _SEGMENTS) + TAIL


def _command(throttle: float, steering: float, reset: bool) -> dict[str, str]:
    """Strings exactly as the devkit forms them: str(float) and str(bool)."""
    return {
        "V1 Throttle": str(float(throttle)),
        "V1 Steering": str(float(steering)),
        "V1 Reset": str(bool(reset)),
    }


def identify_schedule(t: float) -> dict[str, str]:
    """The command for `t` seconds after the first sim message."""
    end = 0.0
    for seg in _SEGMENTS:
        end += seg.duration
        if t < end:
            return _command(seg.throttle, seg.steering, seg.reset)
    return _command(0.0, 0.0, False)


DROPPED_KEYS = ("V1 Front Camera Image", "V1 LIDAR Range Array")


def strip_frame(data: dict) -> dict:
    """The frame without the camera and the lidar; the input is untouched."""
    return {k: v for k, v in data.items() if k not in DROPPED_KEYS}


# --- analysis ---------------------------------------------------------------
# Time is `V1 Lap Time`, the sim's own clock. Thresholds below cite the step 1
# probe (2026-10-26, host load 4-15) and the part 1 fixture (load 20-30).

# A reset is lap time falling from beyond RESET_FROM to below RESET_TO.
RESET_FROM = 0.5
RESET_TO = 0.1
# Steady = varies by less than STEADY_REL over STEADY_WINDOW seconds of lap
# time. Probe: 0.02-0.12 throttle straights vary < 0.1 % over 0.5 s, the
# 0.3-command turn's yaw rate < 0.2 % over 0.3 s.
STEADY_REL = 0.01
STEADY_WINDOW = 0.5
# A speed below this is "at rest" (probe: rest |v| < 0.01 after the settle).
REST_SPEED = 0.01
# A run shorter than this is an edge, not a window (the throttle takes ~0.2 s
# to reach its speed).
MIN_RUN = 1.0
# A stop is measured from at least this speed (probe: 0.5 m/s at throttle 0.02).
MIN_BRAKE_SPEED = 0.4
BRAKE_HIGH = 0.9
BRAKE_LOW = 0.1


@dataclass(frozen=True)
class Sample:
    """One sim frame. Position and velocity as the sim reports them: the pose
    is the rear axle, the velocity the body's, in the car frame."""

    lap_time: float
    throttle: float
    steering: float
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float
    wz: float
    collisions: int
    cmd_steering: float | None = None


@dataclass(frozen=True)
class Estimate:
    """A value, how many observations it rests on, and the relative spread
    (max - min over the mean) of the repeats that went into it."""

    value: float
    n: int
    spread: float = 0.0


@dataclass(frozen=True)
class SteeringEstimate:
    limit: Estimate
    slew: Estimate
    sign: int


@dataclass(frozen=True)
class YawEstimate:
    wheelbase: Estimate
    rise_time: Estimate | None


@dataclass(frozen=True)
class PointCheck:
    pose_lateral_speed: float
    velocity_lever: float
    n: int


def _floats(text: str) -> list[float]:
    return [float(part) for part in text.split()]


def parse_frame(
    data: dict[str, str], cmd_steering: float | None = None
) -> Sample:
    x, y, z = _floats(data["V1 Position"])
    vx, vy, _ = _floats(data["V1 Linear Velocity"])
    return Sample(
        lap_time=float(data["V1 Lap Time"]),
        throttle=float(data["V1 Throttle"]),
        steering=float(data["V1 Steering"]),
        x=x,
        y=y,
        z=z,
        yaw=_floats(data["V1 Orientation Euler Angles"])[2],
        vx=vx,
        vy=vy,
        wz=_floats(data["V1 Angular Velocity"])[2],
        collisions=int(float(data["V1 Collisions"])),
        cmd_steering=cmd_steering,
    )


def samples_from_events(events: list[dict]) -> list[Sample]:
    """Sim frames of a recording, each with the steering command the sim had
    been sent when it produced the frame (None before the first reply)."""
    samples: list[Sample] = []
    command: float | None = None
    for event in events:
        if event["dir"] == "bridge":
            command = float(event["data"]["V1 Steering"])
        elif event["dir"] == "sim":
            samples.append(parse_frame(event["data"], command))
    return samples


def segments(samples: list[Sample]) -> list[list[Sample]]:
    """Runs of frames with one clock and one collision count.

    A segment ends at a reset (lap time falls from beyond RESET_FROM to below
    RESET_TO) and where the collision count rises (the car is teleported to a
    checkpoint and restarts from rest). Within a segment lap time never goes
    back: the reset pulse jitters it downward 27 times, and those frames are
    dropped.
    """
    out: list[list[Sample]] = []
    current: list[Sample] = []
    for sample in samples:
        if current:
            last = current[-1]
            reset = last.lap_time > RESET_FROM and sample.lap_time < RESET_TO
            hit = sample.collisions != last.collisions
            if reset or hit:
                out.append(current)
                current = []
            elif sample.lap_time < last.lap_time:
                continue
        current.append(sample)
    if current:
        out.append(current)
    return out


def _estimate(values: list[float], groups: list[list[float]]) -> Estimate:
    """The mean of `values`, with the widest relative spread over `groups`
    (values of the same condition in different repeats)."""
    spread = 0.0
    for group in groups:
        mean = statistics.fmean(group)
        if len(group) > 1 and mean:
            spread = max(spread, (max(group) - min(group)) / abs(mean))
    return Estimate(statistics.fmean(values), len(values), spread)


def _runs(seg: list[Sample], same) -> list[list[Sample]]:
    """Maximal runs of consecutive frames for which `same(frame)` is equal."""
    runs: list[list[Sample]] = []
    for sample in seg:
        if runs and same(runs[-1][-1]) == same(sample):
            runs[-1].append(sample)
        else:
            runs.append([sample])
    return runs


def _tail(run: list[Sample], seconds: float) -> list[Sample]:
    return [s for s in run if s.lap_time >= run[-1].lap_time - seconds]


def throttle_gain(segs: list[list[Sample]]) -> Estimate:
    """Speed per unit throttle.

    Windows are the last STEADY_WINDOW seconds of a run at one throttle with
    the reported steering at zero (a steady turn is 0.7 % slower), kept when
    the speed varies by less than STEADY_REL. The gain is the least-squares
    slope through the origin of window speed on throttle.
    """
    points: list[tuple[float, float]] = []
    for seg in segs:
        for run in _runs(seg, lambda s: (s.throttle, s.steering == 0.0)):
            first = run[0]
            if first.throttle <= 0.0 or first.steering != 0.0:
                continue
            if run[-1].lap_time - first.lap_time < MIN_RUN:
                continue
            window = _tail(run, STEADY_WINDOW)
            speeds = [s.vx for s in window]
            mean = statistics.fmean(speeds)
            if mean <= 0 or (max(speeds) - min(speeds)) / mean >= STEADY_REL:
                continue
            points.append((first.throttle, mean))
    if not points:
        return Estimate(math.nan, 0)
    value = sum(t * v for t, v in points) / sum(t * t for t, _ in points)
    by_throttle: dict[float, list[float]] = {}
    for throttle, speed in points:
        by_throttle.setdefault(throttle, []).append(speed / throttle)
    groups = list(by_throttle.values())
    spread = 0.0
    for group in groups:
        mean = statistics.fmean(group)
        if len(group) > 1:
            spread = max(spread, (max(group) - min(group)) / mean)
    return Estimate(value, len(points), spread)


def _crossing(frames: list[Sample], level: float) -> float | None:
    """Lap time at which speed first falls through `level`."""
    for a, b in zip(frames, frames[1:], strict=False):
        if a.vx >= level > b.vx:
            return a.lap_time + (a.vx - level) / (a.vx - b.vx) * (
                b.lap_time - a.lap_time
            )
    return None


def braking(segs: list[list[Sample]]) -> Estimate:
    """Mean deceleration between the 90 % and 10 % crossings of the speed the
    car left throttle at, over every throttle-to-zero that coasts to a stop."""
    rates: list[float] = []
    for seg in segs:
        for i in range(1, len(seg)):
            if not (seg[i - 1].throttle > 0.0 and seg[i].throttle == 0.0):
                continue
            v0 = seg[i - 1].vx
            if v0 < MIN_BRAKE_SPEED:
                continue
            frames = [seg[i - 1]]
            for sample in seg[i:]:
                if sample.throttle > 0.0:
                    break
                frames.append(sample)
            high = _crossing(frames, BRAKE_HIGH * v0)
            low = _crossing(frames, BRAKE_LOW * v0)
            if high is None or low is None or low <= high:
                continue
            rates.append((BRAKE_HIGH - BRAKE_LOW) * v0 / (low - high))
    if not rates:
        return Estimate(math.nan, 0)
    return _estimate(rates, [rates])


def _slew_rates(seg: list[Sample]) -> list[float]:
    """Rates inside ramps of the reported steering: pairs with the same
    direction of change on both sides, so neither the first nor the last pair
    of a ramp (partial frames) and not the snap to zero."""
    pairs = []
    for a, b in zip(seg, seg[1:], strict=False):
        dt = b.lap_time - a.lap_time
        pairs.append((b.steering - a.steering) / dt if dt > 0 else 0.0)
    rates = []
    for before, here, after in zip(pairs, pairs[1:], pairs[2:], strict=False):
        if here and before * here > 0 and after * here > 0:
            rates.append(abs(here))
    return rates


def _turn_sign(seg: list[Sample]) -> float:
    """Sum of steering x the turning direction of the pose track."""
    total = 0.0
    for a, b, c in zip(seg, seg[1:], seg[2:], strict=False):
        d1 = (b.x - a.x, b.y - a.y)
        d2 = (c.x - b.x, c.y - b.y)
        if math.hypot(*d1) < 1e-3 or math.hypot(*d2) < 1e-3:
            continue
        total += b.steering * (d1[0] * d2[1] - d1[1] * d2[0])
    return total


def steering(segs: list[list[Sample]]) -> SteeringEstimate:
    """Steering limit, slew rate and sign.

    The limit is reported / command at the end of every held command of at
    least STEADY_WINDOW seconds; the slew the median ramp rate; the sign +1
    when a positive angle turns the pose track counter-clockwise (z up).
    """
    ratios: list[float] = []
    rates: list[float] = []
    sign_sum = 0.0
    for seg in segs:
        rates += _slew_rates(seg)
        sign_sum += _turn_sign(seg)
        for run in _runs(seg, lambda s: s.cmd_steering):
            cmd = run[0].cmd_steering
            if not cmd or run[-1].lap_time - run[0].lap_time < STEADY_WINDOW:
                continue
            window = _tail(run, 0.2)
            reported = statistics.fmean(s.steering for s in window)
            ratios.append(reported / cmd)
    limit = _estimate(ratios, [ratios]) if ratios else Estimate(math.nan, 0)
    slew = (
        Estimate(statistics.median(rates), len(rates))
        if rates
        else Estimate(math.nan, 0)
    )
    return SteeringEstimate(limit, slew, 1 if sign_sum >= 0 else -1)


# A steady turn: the angle held and the yaw rate settled, moving.
TURN_MIN_ANGLE = 0.05
TURN_MIN_SPEED = 0.3
TURN_MIN_RATE = 0.1


def _steady_turns(seg: list[Sample]) -> list[Sample]:
    frames = []
    for a, b in zip(seg, seg[1:], strict=False):
        if (
            abs(b.steering) > TURN_MIN_ANGLE
            and b.steering == a.steering
            and b.vx > TURN_MIN_SPEED
            and abs(b.wz) > TURN_MIN_RATE
            and abs(b.wz - a.wz) < 0.02 * abs(b.wz)
        ):
            frames.append(b)
    return frames


def _rise_time(seg: list[Sample]) -> list[float]:
    """10-90 % rise time of the yaw rate after the angle leaves zero."""
    times = []
    i = 0
    while i < len(seg) - 1:
        if abs(seg[i].steering) < 1e-6 and abs(seg[i + 1].steering) > 1e-6:
            start = seg[i].lap_time
            run = [s for s in seg[i + 1 :] if s.lap_time - start < 1.0]
            held = [s for s in run if s.steering == run[-1].steering]
            steady = _steady_turns(held)
            if steady:
                plateau = statistics.fmean(s.wz for s in steady)
                low = _first_through(run, 0.1 * plateau)
                high = _first_through(run, 0.9 * plateau)
                if low is not None and high is not None and high > low:
                    times.append(high - low)
        i += 1
    return times


def _first_through(frames: list[Sample], level: float) -> float | None:
    for a, b in zip(frames, frames[1:], strict=False):
        if (a.wz - level) * (b.wz - level) <= 0 and a.wz != b.wz:
            return a.lap_time + (level - a.wz) / (b.wz - a.wz) * (
                b.lap_time - a.lap_time
            )
    return None


def yaw_response(segs: list[list[Sample]]) -> YawEstimate:
    """Effective wheelbase v tan(delta) / omega over steady-turn frames, and
    the 10-90 % rise time of the yaw rate."""
    ratios = [
        s.vx * math.tan(s.steering) / s.wz
        for seg in segs
        for s in _steady_turns(seg)
    ]
    rises = [t for seg in segs for t in _rise_time(seg)]
    wheelbase = (
        Estimate(statistics.median(ratios), len(ratios))
        if ratios
        else Estimate(math.nan, 0)
    )
    rise = Estimate(statistics.median(rises), len(rises)) if rises else None
    return YawEstimate(wheelbase, rise)


def rest_height(segs: list[list[Sample]], com_z: float) -> Estimate:
    """CoG height: the median z of the car at rest, plus the COM's height
    above the rear axle (`com_z`, from the guide, not measurable here)."""
    heights = [
        s.z
        for seg in segs
        for s in seg
        if s.throttle == 0.0
        and abs(s.vx) < REST_SPEED
        and abs(s.wz) < REST_SPEED
    ]
    if not heights:
        return Estimate(math.nan, 0)
    return Estimate(statistics.median(heights) + com_z, len(heights))


def reference_points(segs: list[list[Sample]]) -> PointCheck:
    """Which point each field belongs to, in steady turns.

    `pose_lateral_speed`: sideways speed of the pose point in the body frame,
    from the pose track (0 for the rear axle). `velocity_lever`: vy / omega of
    the reported velocity, the distance from the rear axle to the point whose
    velocity it is (the CoG).
    """
    lateral: list[float] = []
    lever: list[float] = []
    for seg in segs:
        steady = {id(s) for s in _steady_turns(seg)}
        for a, b, c in zip(seg, seg[1:], seg[2:], strict=False):
            if id(b) not in steady or c.lap_time <= a.lap_time:
                continue
            dt = c.lap_time - a.lap_time
            dx, dy = (c.x - a.x) / dt, (c.y - a.y) / dt
            lateral.append(-math.sin(b.yaw) * dx + math.cos(b.yaw) * dy)
            lever.append(b.vy / b.wz)
    if not lateral:
        return PointCheck(math.nan, math.nan, 0)
    return PointCheck(
        statistics.median(lateral), statistics.median(lever), len(lateral)
    )


def _recorder():
    """tools/autodrive_record.py, loaded by path (the two share a folder)."""
    path = Path(__file__).resolve().with_name("autodrive_record.py")
    spec = importlib.util.spec_from_file_location("autodrive_record", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cmd_record(args: argparse.Namespace) -> int:
    rec = _recorder()
    header = rec._header(args.config, SCHEDULE_ID)
    header["host_load"] = list(os.getloadavg())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    writer = rec.RecordWriter(args.out, header)
    handler = rec.BridgeHandler(
        writer, identify_schedule, SCHEDULE_END, strip_frame
    )
    print(f"recording to {args.out}; start the sim now", flush=True)
    try:
        rec.serve(handler, args.port)
    finally:
        writer.close()
    print(f"{handler.sims} sim messages (need {rec.MIN_MESSAGES})", flush=True)
    if not rec.recording_usable(handler.sims):
        print("too few messages; the recording is not usable", file=sys.stderr)
        return 1
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    record = sub.add_parser("record", help="record the identification drive")
    record.add_argument("--config", type=Path, required=True)
    record.add_argument("--out", type=Path, required=True)
    record.add_argument("--port", type=int, default=4567)
    record.set_defaults(func=cmd_record)
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
