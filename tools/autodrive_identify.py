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
import dataclasses
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


def build_schedule(repeats: int, manoeuvres=MANOEUVRES) -> list[Segment]:
    segments: list[Segment] = []
    for _ in range(repeats):
        for manoeuvre in manoeuvres:
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


def identify_schedule(t: float, segments=_SEGMENTS) -> dict[str, str]:
    """The command for `t` seconds after the first sim message."""
    end = 0.0
    for seg in segments:
        end += seg.duration
        if t < end:
            return _command(seg.throttle, seg.steering, seg.reset)
    return _command(0.0, 0.0, False)


# --- the lateral schedule (part 1L increment 1) -------------------------------
# Step-steer doublets on Porto's spawn straight: bring the wheels to v, settle
# the slip, steer +A then -A, centre, brake with locked wheels. The rungs are
# the corridor table of the plan (A, v, hold); each is heading-balanced (the
# second lobe is held s longer, s the slew time) and its planned path stays
# inside CORRIDOR_OFFSET_M (CHECK-2650). Step 2a's live probe ran all ten
# rungs twice with no collision (max |dx| 0.11-0.22 m).
LATERAL_SCHEDULE_ID = "porto-lateral-v1"
LATERAL_REPEATS = 3
# 0.52 m is free on the narrowest side of the spawn straight; slew and drift
# take the rest (Step 2a's measured offsets were 0.11-0.22 m).
CORRIDOR_OFFSET_M = 0.35
# The part-4 probe's clear straight (Porto spawn, heading -y).
STRAIGHT_CLEAR_M = 7.9
MAX_FINAL_HEADING_DEG = 3.0
STEER_SLEW = 3.2  # rad/s, the identified actuator slew
STEER_LIMIT = 0.5236  # rad, the steering command 1.0
WHEEL_SETPOINT_GAIN = 24.86  # wheel rad/s per throttle (F1)
DRIVE_A0, DRIVE_C = 5.38, 0.273  # full throttle: a = 5.38 - 0.273 v (F2)
BRAKE_A0 = 5.40  # throttle 0 locks the wheels: a = 5.40 + 0.273 v (F2)
# (steering fraction A, speed v, hold T of the first lobe)
LATERAL_RUNGS = (
    (0.3, 3.5, 0.170),
    (0.3, 4.0, 0.140),
    (0.3, 4.5, 0.115),
    (0.3, 4.8, 0.100),
    (0.4, 2.8, 0.165),
    (0.4, 3.2, 0.130),
    (0.4, 3.5, 0.110),
    (0.5, 2.4, 0.150),
    (0.5, 2.8, 0.115),
    (0.6, 2.4, 0.105),
)


def _drive_time(v: float) -> float:
    """Seconds of full throttle from rest to v: v' = 5.38 - 0.273 v."""
    return -math.log(1.0 - v * DRIVE_C / DRIVE_A0) / DRIVE_C


def lateral_rung(
    a: float, v: float, hold: float, balanced: bool = True
) -> list[Segment]:
    """One doublet. `balanced` holds the second lobe `s` longer, so the
    steering's net area, and with it the heading, returns to ~zero."""
    slew = a * STEER_LIMIT / STEER_SLEW
    throttle = v / WHEEL_SETPOINT_GAIN
    return [
        Segment(_drive_time(v), 1.0, 0.0, False, "accel"),
        Segment(0.3, throttle, 0.0, False, "settle"),
        Segment(hold, throttle, a, False, "lobe1"),
        Segment(
            hold + (slew if balanced else 0.0), throttle, -a, False, "lobe2"
        ),
        Segment(slew + 0.02, throttle, 0.0, False, "return"),
        Segment(1.7, 0.0, 0.0, False, "stop"),
    ]


LATERAL_MANOEUVRES = tuple(
    (lambda rung=rung: lateral_rung(*rung)) for rung in LATERAL_RUNGS
)
_LATERAL_SEGMENTS = build_schedule(LATERAL_REPEATS, LATERAL_MANOEUVRES)
LATERAL_END = sum(seg.duration for seg in _LATERAL_SEGMENTS) + TAIL


def lateral_schedule(t: float) -> dict[str, str]:
    """The lateral drive's command `t` seconds after the first sim message."""
    return identify_schedule(t, _LATERAL_SEGMENTS)


def planned_path(
    manoeuvre: list[Segment], v: float, wheelbase: float, dt: float = 0.001
) -> dict[str, float]:
    """A kinematic bicycle driven through `manoeuvre` (the segments of one
    rung, `lateral_rung`): full throttle follows the drive law to v, a
    throttle holds v, throttle 0 is the locked-wheel stop, and the steering
    slews at STEER_SLEW. An upper bound on the path below the limit and in
    understeer, not in oversteer. Returns the largest |y|, the final heading
    (deg), the distance travelled (m)."""
    x = y = psi = delta = speed = 0.0
    max_y = 0.0
    for seg in manoeuvre:
        for _ in range(int(round(seg.duration / dt))):
            if seg.throttle >= 1.0:
                speed = min(v, speed + (DRIVE_A0 - DRIVE_C * speed) * dt)
            elif seg.throttle > 0.0:
                speed = v
            else:
                speed = max(0.0, speed - (BRAKE_A0 + DRIVE_C * speed) * dt)
            target = seg.steering * STEER_LIMIT
            delta += max(-STEER_SLEW * dt, min(STEER_SLEW * dt, target - delta))
            psi += speed * math.tan(delta) / wheelbase * dt
            x += speed * math.cos(psi) * dt
            y += speed * math.sin(psi) * dt
            max_y = max(max_y, abs(y))
    return {
        "max_y": max_y,
        "heading_deg": math.degrees(psi),
        "distance": x,
    }


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
# The sim clock is compared with the wall clock over segments this long.
CLOCK_FROM = 1.0
MIN_CLOCK_SPAN = 2.0
# A burst is full throttle from rest; its acceleration is read over this long.
FULL_THROTTLE = 0.99
BURST_WINDOW = 0.25
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
    wall: float = math.nan  # arrival time: only for the clock-ratio check
    # IMU linear acceleration in the car frame, gravity not included (its z
    # median is 0.001 m/s^2 over the committed recording), and the left and
    # right encoder angles, rad, accumulated (zeroed by a reset).
    ax: float = math.nan
    ay: float = math.nan
    wheel_angles: tuple[float, float] | None = None


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
    data: dict[str, str],
    cmd_steering: float | None = None,
    wall: float = math.nan,
) -> Sample:
    x, y, z = _floats(data["V1 Position"])
    vx, vy, _ = _floats(data["V1 Linear Velocity"])
    ax = ay = math.nan
    if "V1 Linear Acceleration" in data:
        ax, ay, _ = _floats(data["V1 Linear Acceleration"])
    angles = None
    if "V1 Encoder Angles" in data:
        left, right = _floats(data["V1 Encoder Angles"])
        angles = (left, right)
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
        wall=wall,
        ax=ax,
        ay=ay,
        wheel_angles=angles,
    )


def wheel_speed(prev: Sample, cur: Sample, wheel_radius: float) -> float | None:
    """Mean wheel speed from the encoder angle deltas, m/s, over the lap-time
    delta (gotcha 35: the sim's clock, never the arrival time).

    None when either frame has no angles, when the lap-time delta is not
    positive (a lap-line reset) or when either wheel's angle went backwards
    (the encoders zero on a reset). Duplicates
    `racing_sim_autodrive.core.encoder_speed` on purpose: this file is a uv
    script with no overlay (CHECK-2605 holds the two equal).
    """
    if prev.wheel_angles is None or cur.wheel_angles is None:
        return None
    dt = cur.lap_time - prev.lap_time
    if not dt > 0.0:
        return None
    deltas = [
        c - p for p, c in zip(prev.wheel_angles, cur.wheel_angles, strict=True)
    ]
    if any(d < 0.0 for d in deltas):
        return None
    return sum(deltas) / len(deltas) * wheel_radius / dt


def slip_ratio(wheel: float, body: float) -> float:
    """(w - v) / max(w, v): +1 is full spin, -1 locked wheels, 0 when both
    speeds are below REST_SPEED."""
    top = max(wheel, body)
    if max(abs(wheel), abs(body)) < REST_SPEED:
        return 0.0
    return (wheel - body) / top


@dataclass(frozen=True)
class SensorChecks:
    """Median IMU / kinematics ratios and the frames each rests on."""

    ax_ratio: float
    ax_n: int
    ay_ratio: float
    ay_n: int


# F5 (plan, 2026-11-08, committed recording): the IMU a_x is the body's
# acceleration, median ratio to dv_x/dt 1.0012 (n 2199); the IMU a_y is v_x w_z
# in turns, 1.0027 (n 623). Only frames where the kinematic value is well
# above the IMU's noise are used, and a central difference over a gap longer
# than MAX_DIFF_GAP (the frame rate follows host load, gotcha 31) is not.
AX_MIN = 0.5
AY_MIN = 0.2
MAX_DIFF_GAP = 0.2


def sensor_checks(segs: list[list[Sample]]) -> SensorChecks:
    """Median of IMU a_x over the central-difference dv_x/dt, and of IMU a_y
    over v_x w_z, per frame. Both are 1 when the IMU is the body's
    acceleration without gravity."""
    ax_ratios: list[float] = []
    ay_ratios: list[float] = []
    for seg in segs:
        for a, b, c in zip(seg, seg[1:], seg[2:], strict=False):
            span = c.lap_time - a.lap_time
            if math.isnan(b.ax) or math.isnan(b.ay):
                continue
            if 0.0 < span < MAX_DIFF_GAP:
                dv = (c.vx - a.vx) / span
                if abs(dv) > AX_MIN:
                    ax_ratios.append(b.ax / dv)
            lateral = b.vx * b.wz
            if abs(lateral) > AY_MIN:
                ay_ratios.append(b.ay / lateral)
    return SensorChecks(
        statistics.median(ax_ratios) if ax_ratios else math.nan,
        len(ax_ratios),
        statistics.median(ay_ratios) if ay_ratios else math.nan,
        len(ay_ratios),
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
            samples.append(parse_frame(event["data"], command, event["t"]))
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


def steady_windows(segs: list[list[Sample]]) -> list[tuple[float, float]]:
    """(throttle, mean speed) of every steady window: the last STEADY_WINDOW
    seconds of a run at one throttle with the reported steering at zero (a
    steady turn is 0.7 % slower), kept when the speed varies by less than
    STEADY_REL."""
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
    return points


def throttle_gain(segs: list[list[Sample]]) -> Estimate:
    """Speed per unit throttle: the least-squares slope through the origin of
    the steady windows' speed on throttle; `spread` is the widest relative
    spread of one throttle's gain over its repeats."""
    points = steady_windows(segs)
    if not points:
        return Estimate(math.nan, 0)
    value = sum(t * v for t, v in points) / sum(t * t for t, _ in points)
    by_throttle: dict[float, list[float]] = {}
    for throttle, speed in points:
        by_throttle.setdefault(throttle, []).append(speed / throttle)
    spread = _estimate([value], list(by_throttle.values())).spread
    return Estimate(value, len(points), spread)


def _crossing(frames: list[Sample], level: float) -> float | None:
    """Lap time at which speed first falls through `level`."""
    for a, b in zip(frames, frames[1:], strict=False):
        if a.vx >= level > b.vx:
            return a.lap_time + (a.vx - level) / (a.vx - b.vx) * (
                b.lap_time - a.lap_time
            )
    return None


def brake_events(segs: list[list[Sample]]) -> list[tuple[float, float]]:
    """(speed left throttle at, mean deceleration between the 90 % and 10 %
    crossings of it) for every throttle-to-zero that coasts to a stop."""
    events: list[tuple[float, float]] = []
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
            events.append((v0, (BRAKE_HIGH - BRAKE_LOW) * v0 / (low - high)))
    return events


def braking(segs: list[list[Sample]]) -> Estimate:
    """Mean of the brake events' decelerations. It is not constant (about 5.6
    then 9 m/s^2 within one stop) and depends on the speed: `spread` is
    therefore 0 here and the repeat spread is taken in `analyse`."""
    rates = [rate for _, rate in brake_events(segs)]
    if not rates:
        return Estimate(math.nan, 0)
    return Estimate(statistics.fmean(rates), len(rates))


def burst_acceleration(segs: list[list[Sample]]) -> Estimate:
    """Mean acceleration over the first BURST_WINDOW seconds of every
    full-throttle run that starts from rest (the machine's, as the sim lets
    the car use it)."""
    rates: list[float] = []
    for seg in segs:
        for i in range(1, len(seg)):
            if not (
                seg[i].throttle >= FULL_THROTTLE
                and seg[i - 1].throttle < FULL_THROTTLE
                and abs(seg[i - 1].vx) < MIN_BRAKE_SPEED
            ):
                continue
            window = []
            for sample in seg[i:]:
                if sample.throttle < FULL_THROTTLE:
                    break
                if sample.lap_time - seg[i].lap_time > BURST_WINDOW:
                    break
                window.append(sample)
            span = window[-1].lap_time - window[0].lap_time
            if span >= 0.8 * BURST_WINDOW:
                rates.append((window[-1].vx - window[0].vx) / span)
    if not rates:
        return Estimate(math.nan, 0)
    return Estimate(statistics.fmean(rates), len(rates))


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
# Frames an angle must stay put to be the step's plateau, not a ramp value.
MIN_HELD = 3


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
    """10-90 % rise time of the yaw rate after the angle leaves zero, to the
    plateau of the first angle held (the step, not a later reversal)."""
    times = []
    i = 0
    while i < len(seg) - 1:
        if abs(seg[i].steering) < 1e-6 and abs(seg[i + 1].steering) > 1e-6:
            start = seg[i].lap_time
            run = [s for s in seg[i + 1 :] if s.lap_time - start < 1.0]
            # The plateau is the first angle the drive holds (the step), not
            # the last one in the window (the reversal that follows it).
            first = next(
                (
                    r
                    for r in _runs(run, lambda s: s.steering)
                    if len(r) >= MIN_HELD
                    and abs(r[0].steering) > TURN_MIN_ANGLE
                ),
                None,
            )
            held = first or []
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


# --- lateral windows and radius bins (part 1L increment 1) --------------------
# A window is the frames of one steering lobe within WINDOW_BAND of its plateau.
# Constants cite Step 2a's probe (2026-11-10, 10 rungs x 2, load 1-2).
WINDOW_BAND = 0.9
MIN_WINDOW_FRAMES = 5
# Slip beyond this is longitudinal slip in the window, not a lateral limit.
MAX_WINDOW_SLIP = 0.05
# Linear-range a_y / a_kin was 0.94 (MAD 0.015, six rungs; P3): 0.94 - 3 MAD.
SATURATION_RATIO = 0.895
# The linear-range w_z ratio was 0.91-0.93 (P3): above 1.05 is over the
# kinematic rate, which an understeering car cannot be.
OVERSTEER_WZ_RATIO = 1.05
# Saturated rungs' w_z and a_y ratios differed by at most 0.15 (P1).
INCONSISTENT_GAP = 0.2
# a_y lags the steering (the lobes last 0.1-0.17 s): a window whose median is
# below the linear line but whose last third has reached it was still rising,
# not saturated (P1: A 0.3 at 4.0 m/s, max 7.9 against a_kin 8.0).


@dataclass(frozen=True)
class LateralWindow:
    rung: int
    repeat: int
    speed: float  # median wheel speed, m/s
    steering: float  # median |actual steering|, rad
    a_kin: float  # v^2 tan(d) / L, median over the frames
    r_kin: float  # L / tan(d), median
    ay: float  # IMU |a_y| window median
    ay_ratio: float  # ay / a_kin
    wz_ratio: float  # median |w_z| / (v tan(d) / L)
    radius: float  # median path radius v / |w_z|
    sideslip: float  # max |v_y / v_x|
    slip: float  # median slip ratio
    n: int
    rising: bool  # a_y reached the linear line by the end of the window
    label: str  # linear | saturated | oversteer | inconsistent


def _lobes(seg: list[Sample]) -> list[list[tuple[Sample, Sample]]]:
    """Runs of (previous, current) frame pairs with one steering sign and
    |steering| above TURN_MIN_ANGLE."""
    lobes: list[list[tuple[Sample, Sample]]] = []
    current: list[tuple[Sample, Sample]] = []
    sign = 0.0
    for prev, cur in zip(seg, seg[1:], strict=False):
        s = math.copysign(1.0, cur.steering)
        if abs(cur.steering) > TURN_MIN_ANGLE and (not current or s == sign):
            current.append((prev, cur))
            sign = s
        elif current:
            lobes.append(current)
            current = []
    if current:
        lobes.append(current)
    return lobes


def _window_label(ay_ratio: float, wz_ratio: float, rising: bool) -> str:
    if ay_ratio >= SATURATION_RATIO:
        return "linear"
    if wz_ratio > OVERSTEER_WZ_RATIO:
        return "oversteer"
    if rising:
        return "linear"  # understeer gradient: a_y has not settled
    if abs(ay_ratio - wz_ratio) > INCONSISTENT_GAP:
        return "inconsistent"
    return "saturated"


def lateral_windows(
    segs: list[list[Sample]],
    wheelbase: float,
    wheel_radius: float,
    rungs: int,
) -> list[LateralWindow]:
    """One window per steering lobe, in segment order. Segment i is rung
    i % rungs of repeat i // rungs (the layout `_by_repeat` assumes). Speed
    is the wheel speed, slip the wheel against the body; a window of fewer
    than MIN_WINDOW_FRAMES frames, or with |slip| >= MAX_WINDOW_SLIP, is not
    used. The sensors are read as F5 found them: a_y is the body's, so
    |a_y| / a_kin is 1 in the linear range (0.94 measured)."""
    out: list[LateralWindow] = []
    for index, seg in enumerate(segs):
        for lobe in _lobes(seg):
            peak = max(abs(cur.steering) for _, cur in lobe)
            frames = []
            for prev, cur in lobe:
                w = wheel_speed(prev, cur, wheel_radius)
                if w is None or abs(cur.steering) < WINDOW_BAND * peak:
                    continue
                frames.append((cur, w))
            if len(frames) < MIN_WINDOW_FRAMES:
                continue
            tans = [math.tan(abs(c.steering)) for c, _ in frames]
            a_kin = [
                w * w * t / wheelbase
                for (_, w), t in zip(frames, tans, strict=True)
            ]
            r_kin = [wheelbase / t for t in tans]
            wz_kin = [
                w * t / wheelbase
                for (_, w), t in zip(frames, tans, strict=True)
            ]
            slip = statistics.median(slip_ratio(w, c.vx) for c, w in frames)
            if abs(slip) >= MAX_WINDOW_SLIP:
                continue
            ay_values = [abs(c.ay) for c, _ in frames]
            third = max(1, len(ay_values) // 3)
            last = statistics.median(ay_values[-third:])
            ay = statistics.median(ay_values)
            a_kin_m = statistics.median(a_kin)
            wz_ratio = statistics.median(
                abs(c.wz) / k for (c, _), k in zip(frames, wz_kin, strict=True)
            )
            ay_ratio = ay / a_kin_m
            rising = last / a_kin_m >= SATURATION_RATIO
            speeds = [w for _, w in frames]
            out.append(
                LateralWindow(
                    rung=index % rungs,
                    repeat=index // rungs,
                    speed=statistics.median(speeds),
                    steering=statistics.median(
                        abs(c.steering) for c, _ in frames
                    ),
                    a_kin=a_kin_m,
                    r_kin=statistics.median(r_kin),
                    ay=ay,
                    ay_ratio=ay_ratio,
                    wz_ratio=wz_ratio,
                    radius=statistics.median(
                        w / abs(c.wz) for c, w in frames if abs(c.wz) > 1e-9
                    ),
                    sideslip=max(abs(c.vy / c.vx) for c, _ in frames if c.vx),
                    slip=slip,
                    n=len(frames),
                    rising=rising,
                    label=_window_label(ay_ratio, wz_ratio, rising),
                )
            )
    return out


@dataclass(frozen=True)
class SpeedCell:
    speed: float  # median wheel speed of the cell's windows, m/s
    plateau: float  # median a_y of its saturated windows
    n: int
    spread: float  # (max - min) / mean over repeats


@dataclass(frozen=True)
class RadiusBin:
    radius: float  # the bin's centre, m
    n: int  # windows in the bin
    peak: Estimate | None  # per-repeat peaks: value, repeats, spread
    bound: str  # exact | lower
    post_cliff: Estimate | None  # saturated windows' median a_y
    cells: tuple[SpeedCell, ...]
    speed_dependent: bool


def _bin_index(radius: float, centres: tuple[float, ...]) -> int:
    """Edges at the geometric means of neighbouring centres, open at both
    ends."""
    for i, (a, b) in enumerate(zip(centres, centres[1:], strict=False)):
        if radius < math.sqrt(a * b):
            return i
    return len(centres) - 1


def _peak(windows: list[LateralWindow]) -> tuple[float | None, bool]:
    """(peak a_y, exact): the larger of the highest linear window and the
    onset window (the lowest-demand saturated one). Exact iff saturation was
    seen and the onset window is the peak; otherwise a lower bound."""
    linear = [w.ay for w in windows if w.label == "linear"]
    saturated = [w for w in windows if w.label == "saturated"]
    if not linear and not saturated:
        return None, False
    if not saturated:
        return max(linear), False
    onset = min(saturated, key=lambda w: w.a_kin).ay
    top = max([*linear, onset])
    return top, onset >= top


def lateral_by_radius(
    windows: list[LateralWindow], centres: tuple[float, ...]
) -> list[RadiusBin]:
    """Windows binned by measured path radius, one `RadiusBin` per centre.
    `peak` is the median over repeats of each repeat's `_peak`; the bin is
    `exact` iff every repeat's peak is exact. `speed_dependent` is set when
    two speed cells' plateaus differ by more than the larger spread."""
    grouped: list[list[LateralWindow]] = [[] for _ in centres]
    for w in windows:
        grouped[_bin_index(w.radius, centres)].append(w)
    bins = []
    for centre, members in zip(centres, grouped, strict=True):
        repeats = sorted({w.repeat for w in members})
        peaks, exact = [], []
        for repeat in repeats:
            value, ok = _peak([w for w in members if w.repeat == repeat])
            if value is not None:
                peaks.append(value)
                exact.append(ok)
        peak = None
        if peaks:
            peak = Estimate(
                statistics.median(peaks),
                len(peaks),
                _spread(peaks) or 0.0,
            )
        saturated = [w for w in members if w.label == "saturated"]
        post = None
        if saturated:
            values = [w.ay for w in saturated]
            post = Estimate(
                statistics.median(values), len(values), _spread(values) or 0.0
            )
        by_speed: dict[float, list[LateralWindow]] = {}
        for w in saturated:
            by_speed.setdefault(round(w.speed, 1), []).append(w)
        cells = []
        for speed in sorted(by_speed):
            ws = by_speed[speed]
            plateaus = [
                statistics.median(x.ay for x in ws if x.repeat == r)
                for r in sorted({x.repeat for x in ws})
            ]
            cells.append(
                SpeedCell(
                    speed=statistics.median(w.speed for w in ws),
                    plateau=statistics.median(plateaus),
                    n=len(ws),
                    spread=_spread(plateaus) or 0.0,
                )
            )
        dependent = any(
            abs(a.plateau - b.plateau)
            > max(a.spread, b.spread) * (a.plateau + b.plateau) / 2.0
            for i, a in enumerate(cells)
            for b in cells[i + 1 :]
        )
        bins.append(
            RadiusBin(
                radius=centre,
                n=len(members),
                peak=peak,
                bound="exact" if peaks and all(exact) else "lower",
                post_cliff=post,
                cells=tuple(cells),
                speed_dependent=dependent,
            )
        )
    return bins


def _recorder():
    """tools/autodrive_record.py, loaded by path (the two share a folder)."""
    path = Path(__file__).resolve().with_name("autodrive_record.py")
    spec = importlib.util.spec_from_file_location("autodrive_record", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def schedule_for(name: str):
    """(id, command function, end, repeats) of a named recording schedule."""
    if name == "identify":
        return SCHEDULE_ID, identify_schedule, SCHEDULE_END, REPEATS
    if name == "lateral":
        return (
            LATERAL_SCHEDULE_ID,
            lateral_schedule,
            LATERAL_END,
            LATERAL_REPEATS,
        )
    raise ValueError(f"unknown schedule {name!r}")


def cmd_record(args: argparse.Namespace) -> int:
    rec = _recorder()
    schedule_id, command, end, repeats = schedule_for(args.schedule)
    header = rec._header(args.config, schedule_id)
    header["host_load"] = list(os.getloadavg())
    header["repeats"] = repeats
    args.out.parent.mkdir(parents=True, exist_ok=True)
    writer = rec.RecordWriter(args.out, header)
    handler = rec.BridgeHandler(writer, command, end, strip_frame)
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


def _clean(value):
    """Plain, rounded, YAML-safe values: NaN becomes None, floats keep 6
    decimals (the sim reports 4)."""
    if isinstance(value, float):
        return None if math.isnan(value) else round(value, 6)
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    return value


def _est(estimate: Estimate | None) -> dict | None:
    if estimate is None:
        return None
    return {
        "value": estimate.value,
        "n": estimate.n,
        "spread": estimate.spread,
    }


def _by_repeat(segs: list[list[Sample]], repeats: int) -> list[list]:
    """The segments of each repeat of the schedule, or [] when collisions or
    a missed reset broke the one-segment-per-manoeuvre layout."""
    per = len(MANOEUVRES)
    if len(segs) != per * repeats:
        return []
    return [segs[i * per : (i + 1) * per] for i in range(repeats)]


def _spread(values: list[float]) -> float | None:
    values = [v for v in values if not math.isnan(v)]
    if len(values) < 2:
        return None
    mean = statistics.fmean(values)
    return (max(values) - min(values)) / abs(mean) if mean else None


def analyse(recording: Path, source: Path) -> dict:
    """The contents of identified.yaml, from the recording and source.yaml
    alone (nothing about the analysing host)."""
    import hashlib

    import yaml

    rec = _recorder()
    header, events = rec.read_records(recording)
    com_z = yaml.safe_load(source.read_text())["values"][
        "com_z_above_rear_axle_m"
    ]["value"]
    samples = samples_from_events(events)
    segs = segments(samples)
    groups = _by_repeat(segs, header.get("repeats", REPEATS))

    gain = throttle_gain(segs)
    windows = steady_windows(segs)
    top_throttle, top_speed = max(windows) if windows else (math.nan,) * 2
    by_throttle: dict[float, list[float]] = {}
    for throttle, speed in windows:
        by_throttle.setdefault(throttle, []).append(speed)
    brake = braking(segs)
    burst = burst_acceleration(segs)
    steer = steering(segs)
    yaw = yaw_response(segs)
    height = rest_height(segs, com_z)
    points = reference_points(segs)

    def over_repeats(measure) -> dict:
        values = [measure(group) for group in groups]
        return {"values": values, "spread": _spread(values)}

    repeats = (
        {
            "throttle_gain": over_repeats(lambda g: throttle_gain(g).value),
            "braking": over_repeats(lambda g: braking(g).value),
            "burst_acceleration": over_repeats(
                lambda g: burst_acceleration(g).value
            ),
            "steering_slew": over_repeats(lambda g: steering(g).slew.value),
            "effective_wheelbase": over_repeats(
                lambda g: yaw_response(g).wheelbase.value
            ),
            "height": over_repeats(lambda g: rest_height(g, com_z).value),
        }
        if groups
        else None
    )
    if repeats and repeats["burst_acceleration"]["spread"] is not None:
        # The burst is one value per repeat: its spread is the repeats'.
        burst = dataclasses.replace(
            burst, spread=repeats["burst_acceleration"]["spread"]
        )
    # Frames before lap time 1 s include the reset pulse, where the clock
    # sits near 0 while the wall clock runs.
    ratios = []
    for seg in segs:
        run = [s for s in seg if s.lap_time >= CLOCK_FROM]
        if (
            len(run) > 1
            and run[-1].lap_time - run[0].lap_time >= MIN_CLOCK_SPAN
        ):
            ratios.append(
                (run[-1].lap_time - run[0].lap_time)
                / (run[-1].wall - run[0].wall)
            )
    out = {
        "provenance": {
            "edition": header["edition"],
            "images": header["images"],
            "libraries": header["libraries"],
            "schedule": header["schedule"],
            "recorded_at": header["recorded_at"],
            "host_load": header.get("host_load"),
            "recording_sha256": hashlib.sha256(
                recording.read_bytes()
            ).hexdigest(),
            "sim_frames": len(samples),
            "segments": len(segs),
            "collisions": max((s.collisions for s in samples), default=0),
            "lap_time_per_wall": {
                "min": min(ratios) if ratios else None,
                "max": max(ratios) if ratios else None,
            },
        },
        "measurements": {
            "throttle_gain": {
                **_est(gain),
                "unit": "m/s per unit throttle",
                "by_throttle": {
                    str(t): statistics.fmean(v) / t
                    for t, v in sorted(by_throttle.items())
                },
            },
            # Porto has no straight long enough: 22.88 m/s (the guide's,
            # source.yaml) at about 25 m/s per unit needs tens of metres, and
            # throttle 0.2 was still accelerating at 4.9 m/s.
            "top_speed": {
                "measured": False,
                "highest_steady_speed": top_speed,
                "at_throttle": top_throttle,
                "reason": "Porto's longest straight (about 8 m) is too "
                "short to reach a steady speed above the highest recorded",
            },
            "burst_acceleration": {
                **_est(burst),
                "unit": "m/s^2",
                "window_s": BURST_WINDOW,
            },
            "braking_deceleration": {
                **_est(brake),
                "unit": "m/s^2",
                # Not one number: it depends on the speed the car brakes from.
                "depends_on_speed": True,
                "by_initial_speed": [
                    {"speed": v0, "deceleration": rate}
                    for v0, rate in sorted(brake_events(segs))
                ],
            },
            "steering": {
                "limit": {**_est(steer.limit), "unit": "rad"},
                "slew": {**_est(steer.slew), "unit": "rad/s"},
                "sign": steer.sign,
            },
            "yaw_response": {
                "effective_wheelbase": {
                    **_est(yaw.wheelbase),
                    "unit": "m",
                },
                "rise_time": (
                    {**_est(yaw.rise_time), "unit": "s"}
                    if yaw.rise_time
                    else None
                ),
            },
            "rest_height": {
                **_est(height),
                "unit": "m",
                "com_z_above_rear_axle": com_z,
            },
            "reference_points": {
                "pose_lateral_speed": points.pose_lateral_speed,
                "velocity_lever": points.velocity_lever,
                "n": points.n,
            },
            "repeats": repeats,
        },
    }
    return _clean(out)


def cmd_analyse(args: argparse.Namespace) -> int:
    import yaml

    rec = _recorder()
    header, _ = rec.read_records(args.recording)
    schedule = header.get("schedule")
    if schedule == SCHEDULE_ID:
        if args.out is None or args.out_dir is not None:
            print(f"{SCHEDULE_ID} takes --out, not --out-dir", file=sys.stderr)
            return 2
        result = analyse(args.recording, args.source)
        args.out.write_text(
            "# Written by tools/autodrive_identify.py analyse; never edit by\n"
            "# hand (CHECK-1780 re-derives it from the committed recording).\n"
            + yaml.safe_dump(result, sort_keys=False)
        )
        print(f"wrote {args.out}", flush=True)
        return 0
    if schedule == LATERAL_SCHEDULE_ID:
        if args.out_dir is None or args.out is not None:
            print(
                f"{LATERAL_SCHEDULE_ID} takes --out-dir, not --out",
                file=sys.stderr,
            )
            return 2
        raise NotImplementedError("the lateral analysis arrives in Step 6")
    raise ValueError(f"unknown schedule {schedule!r} in {args.recording}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    record = sub.add_parser("record", help="record a scripted drive")
    record.add_argument("--config", type=Path, required=True)
    record.add_argument("--out", type=Path, required=True)
    record.add_argument("--port", type=int, default=4567)
    record.add_argument(
        "--schedule", choices=("identify", "lateral"), default="identify"
    )
    record.set_defaults(func=cmd_record)
    analyse_parser = sub.add_parser(
        "analyse", help="analyse a recording (identified.yaml or the limits)"
    )
    analyse_parser.add_argument("--recording", type=Path, required=True)
    analyse_parser.add_argument("--source", type=Path, required=True)
    analyse_parser.add_argument("--out", type=Path)
    analyse_parser.add_argument("--out-dir", type=Path)
    analyse_parser.set_defaults(func=cmd_analyse)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
