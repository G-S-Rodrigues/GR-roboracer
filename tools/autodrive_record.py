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
"""Record the AutoDRIVE simulator's Socket.IO stream under a scripted drive.

Serves what the official devkit bridge serves (python-socketio 4.2.0 /
python-engineio 3.13.0 on gevent: the only pair the sim talks to, see
config/autodrive/simulator.yaml), answers every `Bridge` message with the
scripted command, and writes both sides to a gzip JSONL file.

Run it first, then start the sim at it (the sim connects once and never
retries):

    env -u PYTHONPATH uv run --locked --script tools/autodrive_record.py \
        --config config/autodrive/simulator.yaml --out stream.jsonl.gz

The pure parts (`command_schedule`, `RecordWriter`, `read_records`) import
only the standard library so the tier-1 tests can load this file by path.
"""

from __future__ import annotations

import argparse
import gzip
import json
import sys
import time
from collections.abc import Callable
from datetime import UTC, datetime
from pathlib import Path

SCHEDULE_ID = "porto-v1"
SCHEDULE_END = 16.0
# The sim's rate follows host load (gotcha 31), so a recording is judged by
# how many messages it holds, never by a rate.
MIN_MESSAGES = 200


def recording_usable(sims: int) -> bool:
    return sims >= MIN_MESSAGES


def _command(throttle: float, steering: float, reset: bool) -> dict[str, str]:
    """Strings exactly as the devkit forms them: str(float) and str(bool)."""
    return {
        "V1 Throttle": str(float(throttle)),
        "V1 Steering": str(float(steering)),
        "V1 Reset": str(bool(reset)),
    }


def command_schedule(t: float) -> dict[str, str]:
    """The command for `t` seconds after the first sim message.

    Rest, straight at throttle 0.04 (about 1 m/s: 2.5 m/s straight from
    spawn hit a wall), left, right, brake, a reset pulse, rest.
    """
    if t < 2.0:
        return _command(0.0, 0.0, False)
    if t < 5.0:
        return _command(0.04, 0.0, False)
    if t < 8.0:
        return _command(0.04, 0.3, False)
    if t < 11.0:
        return _command(0.04, -0.3, False)
    if t < 13.0:
        return _command(0.0, 0.0, False)
    if t < 13.5:
        return _command(0.0, 0.0, True)
    return _command(0.0, 0.0, False)


class RecordWriter:
    """gzip JSONL: a header line, then one line per event."""

    def __init__(self, path: Path, header: dict) -> None:
        self._handle = gzip.open(path, "wt", encoding="utf-8")  # noqa: SIM115
        self._handle.write(json.dumps({"header": header}) + "\n")

    def write(self, t: float, direction: str, data: dict) -> None:
        line = {"t": round(t, 4), "dir": direction, "data": data}
        self._handle.write(json.dumps(line) + "\n")

    def close(self) -> None:
        self._handle.close()

    def __enter__(self) -> RecordWriter:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def read_records(path: Path) -> tuple[dict, list[dict]]:
    """(header, events) of a recording written by `RecordWriter`."""
    with gzip.open(path, "rt", encoding="utf-8") as handle:
        header = json.loads(handle.readline())["header"]
        events = [json.loads(line) for line in handle if line.strip()]
    return header, events


class BridgeHandler:
    """What one `Bridge` message does, with the clock injected as `now`.

    Time is measured from the first frame. The sim record is written (through
    `frame_filter`), then the reply; the reply is `schedule(t)`. Returns None
    once `t > end`: the caller stops the server.
    """

    def __init__(
        self,
        writer: RecordWriter,
        schedule: Callable[[float], dict[str, str]],
        end: float,
        frame_filter: Callable[[dict], dict] = lambda data: data,
    ) -> None:
        self._writer = writer
        self._schedule = schedule
        self._end = end
        self._filter = frame_filter
        self._t0: float | None = None
        self.sims = 0

    def __call__(self, now: float, data: dict) -> dict[str, str] | None:
        if self._t0 is None:
            self._t0 = now
        t = now - self._t0
        if t > self._end:
            return None
        self.sims += 1
        self._writer.write(t, "sim", self._filter(data))
        reply = self._schedule(t)
        self._writer.write(t, "bridge", reply)
        return reply


def serve(handler: BridgeHandler, port: int) -> None:
    """Serve the sim's Socket.IO stream through `handler` until it stops."""
    import socketio
    from gevent import pywsgi
    from geventwebsocket.handler import WebSocketHandler

    sio = socketio.Server(async_mode="gevent")
    server = pywsgi.WSGIServer(
        ("", port),
        socketio.WSGIApp(sio),
        handler_class=WebSocketHandler,
    )

    @sio.on("connect")
    def connect(sid, environ):
        print("connected", sid, flush=True)

    @sio.on("Bridge")
    def bridge(sid, data):
        reply = handler(time.monotonic(), data)
        if reply is None:
            server.stop()
            return
        sio.emit("Bridge", data=reply)

    server.serve_forever()


def _header(config_path: Path, schedule_id: str = SCHEDULE_ID) -> dict:
    import importlib.metadata as metadata

    import yaml

    config = yaml.safe_load(config_path.read_text())
    return {
        "edition": config["x-autodrive"]["edition"],
        "images": {k: v["image"] for k, v in config["services"].items()},
        "libraries": {
            name: metadata.version(name)
            for name in (
                "python-socketio",
                "python-engineio",
                "gevent",
                "gevent-websocket",
            )
        },
        "schedule": schedule_id,
        "recorded_at": datetime.now(UTC).isoformat(),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--port", type=int, default=4567)
    args = parser.parse_args(argv)

    header = _header(args.config)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    writer = RecordWriter(args.out, header)
    handler = BridgeHandler(writer, command_schedule, SCHEDULE_END)
    print(f"recording to {args.out}; start the sim now", flush=True)
    try:
        serve(handler, args.port)
    finally:
        writer.close()
    print(f"{handler.sims} sim messages (need {MIN_MESSAGES})", flush=True)
    if not recording_usable(handler.sims):
        print("too few messages; the recording is not usable", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
