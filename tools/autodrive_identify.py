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
import os
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
