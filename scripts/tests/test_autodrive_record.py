"""Tests of tools/autodrive_record.py and of the stream it recorded.

`tools/` is not a package, so the module is loaded by path inside a helper
each test calls: a missing file then fails the test, not the collection.
"""

from __future__ import annotations

import base64
import gzip
import importlib.util
import itertools
import json
import math
import re
import statistics
from pathlib import Path
from types import ModuleType

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOL = REPO_ROOT / "tools" / "autodrive_record.py"
CONFIG = REPO_ROOT / "config" / "autodrive" / "simulator.yaml"
FIXTURE = (
    REPO_ROOT
    / "ros_ws"
    / "src"
    / "racing_sim_autodrive"
    / "test"
    / "fixtures"
    / "porto_stream.jsonl.gz"
)
REPLY_KEYS = {"V1 Throttle", "V1 Steering", "V1 Reset"}
LIDAR = "V1 LIDAR Range Array"


def _tool() -> ModuleType:
    spec = importlib.util.spec_from_file_location("autodrive_record", TOOL)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_check_1640_command_schedule_is_deterministic_and_well_formed() -> None:
    """CHECK-1640: the scripted drive starts at rest, never leaves [-1, 1],
    writes strings as the devkit does (`str(float)`, `str(bool)`), is a pure
    function of time, and ends with a reset pulse followed by rest."""
    tool = _tool()
    times = [i * 0.05 for i in range(0, 340)]
    for t in times:
        command = tool.command_schedule(t)
        assert set(command) == REPLY_KEYS, t
        assert command == tool.command_schedule(t), "not deterministic"
        for key in ("V1 Throttle", "V1 Steering"):
            value = float(command[key])
            assert -1.0 <= value <= 1.0, (t, key)
            assert command[key] == str(value), (t, key, command[key])
        assert command["V1 Reset"] in ("True", "False"), t
    rest = {"V1 Throttle": "0.0", "V1 Steering": "0.0", "V1 Reset": "False"}
    assert tool.command_schedule(0.0) == rest
    assert tool.command_schedule(1.9) == rest
    assert float(tool.command_schedule(3.0)["V1 Throttle"]) > 0
    resets = [
        t for t in times if tool.command_schedule(t)["V1 Reset"] == "True"
    ]
    assert resets, "no reset pulse"
    assert max(resets) < tool.SCHEDULE_END
    assert tool.command_schedule(tool.SCHEDULE_END - 0.01) == rest
    assert tool.command_schedule(tool.SCHEDULE_END + 1.0) == rest


def test_check_1650_record_writer_round_trips_header_first(tmp_path) -> None:
    """CHECK-1650: a recording is gzip JSONL whose first line is the header
    (edition, both digests, library versions, schedule id); events follow in
    order with `t`, `dir` and the sim's dict kept verbatim."""
    tool = _tool()
    header = {
        "edition": "e",
        "images": {"sim": "s@sha256:" + "a" * 64},
        "libraries": {"python-socketio": "4.2.0"},
        "schedule": "porto-v1",
    }
    sim = {"V1 Throttle": "0.0000", "V1 LIDAR Range Array": "H4sI=="}
    bridge = {"V1 Throttle": "0.0", "V1 Steering": "0.0", "V1 Reset": "False"}
    path = tmp_path / "s.jsonl.gz"
    with tool.RecordWriter(path, header) as writer:
        writer.write(0.0, "sim", sim)
        writer.write(0.01, "bridge", bridge)
    with gzip.open(path, "rt") as handle:
        first = json.loads(handle.readline())
    assert first["header"]["edition"] == "e"
    got_header, events = tool.read_records(path)
    assert got_header["schedule"] == "porto-v1"
    assert got_header["libraries"] == {"python-socketio": "4.2.0"}
    assert [e["dir"] for e in events] == ["sim", "bridge"]
    assert events[0] == {"t": 0.0, "dir": "sim", "data": sim}
    assert events[1]["data"] == bridge


def _decode_ranges(encoded: str) -> list[float]:
    text = gzip.decompress(base64.b64decode(encoded)).decode("utf-8")
    return [float(x) for x in text.split("\n") if x]


def test_check_1660_committed_stream_is_a_well_formed_recording() -> None:
    """CHECK-1660: the committed fixture was recorded against the pinned
    images, its first sim message lacks the lidar and every later one has it
    (1081 ranges, each a number: +inf means no return, NaN and negatives are
    corrupt), every sim message has a reply, the period is sane, the car
    moved, and a reset is in it. Structure and sanity bounds only: the sim is
    wall-clock and unseeded, so this is a sample, not a golden, and it does
    not assert a rate. Measured period of the committed recording: median
    0.018 s, 10-90 % range 0.011-0.055 s, at host load 20-30."""
    tool = _tool()
    header, events = tool.read_records(FIXTURE)
    config = yaml.safe_load(CONFIG.read_text())
    images = {k: v["image"] for k, v in config["services"].items()}
    assert header["edition"] == config["x-autodrive"]["edition"]
    for key, image in images.items():
        digest = re.search(r"@(sha256:[0-9a-f]{64})$", image)[1]
        assert header["images"][key].endswith(digest), key
    sims = [e for e in events if e["dir"] == "sim"]
    replies = [e for e in events if e["dir"] == "bridge"]
    assert len(sims) >= 200, len(sims)
    assert len(replies) == len(sims), "every sim message needs a reply"
    for reply in replies:
        assert set(reply["data"]) == REPLY_KEYS
    assert LIDAR not in sims[0]["data"], "first message should lack lidar"
    for event in sims[1:]:
        assert LIDAR in event["data"], f"no lidar at t={event['t']}"
        ranges = _decode_ranges(event["data"][LIDAR])
        assert len(ranges) == 1081, len(ranges)
        assert not any(math.isnan(r) for r in ranges), event["t"]
        assert all(r >= 0.0 for r in ranges), event["t"]
    times = [e["t"] for e in sims]
    periods = [b - a for a, b in itertools.pairwise(times)]
    assert all(p > 0 for p in periods), "time does not advance"
    assert 0.005 <= statistics.median(periods) <= 0.2
    speeds = [
        math.hypot(*[float(v) for v in e["data"]["V1 Linear Velocity"].split()])
        for e in sims
    ]
    assert max(speeds) > 0.3, max(speeds)
    assert any(r["data"]["V1 Reset"] == "True" for r in replies)
    key_sets = {tuple(sorted(e["data"])) for e in sims[1:]}
    assert len(key_sets) == 1, "key set changes mid-stream"
