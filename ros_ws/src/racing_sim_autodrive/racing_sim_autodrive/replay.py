"""Replay the recorded sim stream at a Socket.IO server, as the sim would.

The stand-in for the Unity sim in tier-2 tests: it connects like the sim
(WebSocket, Engine.IO v3 via python-socketio 4.2), sends the recorded
`Bridge` frames, and checks every one gets a reply. The first recorded frame
has no lidar and is sent as it is (gotcha 29): a server that stalls on it
fails here.

`load_frames` and `schedule` are pure; socketio is imported inside `main`, so
tier 1 tests the schedule under plain python3 (the recorder's pattern).
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import statistics
import sys
import threading
import time
from collections.abc import Iterator, Sequence
from pathlib import Path

__all__ = [
    "load_frames",
    "schedule",
    "latency_summary",
    "build_parser",
    "main",
]

PACES = ("recorded", "lockstep")
REPLY_WAIT_S = 1.0  # a reply later than this is a stall, never a hang


def load_frames(path: Path | str) -> list[tuple[float, dict[str, str]]]:
    """The recording's sim->bridge frames, in order, as (time, raw dict)."""
    frames = []
    with gzip.open(path, "rt") as handle:
        for line in handle:
            row = json.loads(line)
            if row.get("dir") == "sim":
                frames.append((float(row["t"]), row["data"]))
    return frames


def schedule(
    frames: Sequence[tuple[float, dict[str, str]]],
    pace: str,
    start_frame: int = 0,
    loop: bool = False,
    count: int | None = None,
    end_frame: int | None = None,
) -> Iterator[tuple[int, float, dict[str, str]]]:
    """Yield (fixture index, delay before sending in s, raw frame).

    `lockstep` delays are 0: the previous reply paces the next send.
    `recorded` delays reproduce the recorded gaps; the first frame, and the
    one at `start_frame`, go at once. With `loop` the plan restarts at
    `start_frame` forever (the wrap waits the median recorded period, not
    the difference of the last and first stamps); `count` bounds the total.
    `end_frame` (exclusive, as a slice end; default the fixture's end) cuts
    the segment, so a loop can repeat only `start_frame..end_frame - 1`.
    """
    if pace not in PACES:
        raise ValueError(f"pace {pace!r} is not one of {PACES}")
    if not 0 <= start_frame < len(frames):
        raise ValueError(
            f"start_frame {start_frame} outside 0..{len(frames) - 1}"
        )
    end = len(frames) if end_frame is None else end_frame
    if not start_frame < end <= len(frames):
        raise ValueError(
            f"end_frame {end} outside {start_frame + 1}..{len(frames)}"
        )
    periods = [b[0] - a[0] for a, b in zip(frames, frames[1:], strict=False)]
    wrap = statistics.median(periods) if periods else 0.0
    sent = 0
    index = start_frame
    while count is None or sent < count:
        if sent == 0 or pace == "lockstep":
            delay = 0.0
        elif index == start_frame:
            delay = wrap
        else:
            delay = frames[index][0] - frames[index - 1][0]
        yield index, delay, frames[index][1]
        sent += 1
        index += 1
        if index == end:
            if not loop:
                return
            index = start_frame


def latency_summary(latencies: Sequence[float]) -> dict[str, float]:
    """n, median, nearest-rank p99 and max of a list of latencies.

    Part 8's rate bar is read from these; the unit is the caller's.
    """
    if not latencies:
        raise ValueError("no latencies to summarise")
    ordered = sorted(latencies)
    rank = math.ceil(0.99 * len(ordered))
    return {
        "n": len(ordered),
        "p50": statistics.median(ordered),
        "p99": ordered[rank - 1],
        "max": ordered[-1],
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--fixture", type=Path, required=True)
    parser.add_argument("--url", default="http://127.0.0.1:4567")
    parser.add_argument("--pace", choices=PACES, default="lockstep")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--end-frame", type=int, default=None)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--count", type=int, default=None)
    parser.add_argument("--replies-out", type=Path, default=None)
    parser.add_argument(
        "--timings-out",
        type=Path,
        default=None,
        help="write {frame, sent_ns, reply_ns} per frame (perf_counter_ns); "
        "lockstep only: sends and replies are paired by order, so a lost "
        "reply shifts every later row",
    )
    parser.add_argument(
        "--hold-s",
        type=float,
        default=0.0,
        help="keep the socket open and silent this long after the last reply",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    import socketio

    frames = load_frames(args.fixture)
    replies: list[tuple[int, dict]] = []
    sent_indices: list[int] = []
    sent_ns: list[int] = []
    reply_ns: list[int] = []
    arrived = threading.Condition()
    client = socketio.Client()

    @client.on("Bridge")
    def on_reply(data):
        with arrived:
            reply_ns.append(time.perf_counter_ns())
            replies.append((len(replies), data))
            arrived.notify_all()

    client.connect(args.url, transports=["websocket"])
    status = 0
    try:
        for index, delay, raw in schedule(
            frames,
            args.pace,
            args.start_frame,
            args.loop,
            args.count,
            end_frame=args.end_frame,
        ):
            if delay:
                time.sleep(delay)
            sent_indices.append(index)
            sent_ns.append(time.perf_counter_ns())
            client.emit("Bridge", raw)
            if args.pace == "lockstep":
                with arrived:
                    ok = arrived.wait_for(
                        lambda: len(replies) >= len(sent_indices),
                        timeout=REPLY_WAIT_S,
                    )
                if not ok:
                    print(
                        f"replay: no reply to frame {index} "
                        f"({len(sent_indices)} sent, {len(replies)} replies) "
                        f"within {REPLY_WAIT_S} s",
                        file=sys.stderr,
                    )
                    status = 2
                    break
        else:
            with arrived:
                arrived.wait_for(
                    lambda: len(replies) >= len(sent_indices),
                    timeout=REPLY_WAIT_S,
                )
            if len(replies) < len(sent_indices):
                print(
                    f"replay: {len(sent_indices)} sent, {len(replies)} replies",
                    file=sys.stderr,
                )
                status = 2
    except KeyboardInterrupt:
        pass
    finally:
        if args.hold_s > 0:
            time.sleep(args.hold_s)
        client.disconnect()
        if args.replies_out is not None:
            with args.replies_out.open("w") as out:
                for k, data in replies:
                    frame = sent_indices[k] if k < len(sent_indices) else None
                    out.write(
                        json.dumps({"frame": frame, "reply": data}) + "\n"
                    )
        if args.timings_out is not None:
            with args.timings_out.open("w") as out:
                for k, sent in enumerate(sent_ns):
                    if k < len(reply_ns):
                        row = {
                            "frame": sent_indices[k],
                            "sent_ns": sent,
                            "reply_ns": reply_ns[k],
                        }
                        out.write(json.dumps(row) + "\n")
    print(
        f"replay: {len(sent_indices)} sent, {len(replies)} replies",
        flush=True,
    )
    return status


if __name__ == "__main__":
    raise SystemExit(main())
