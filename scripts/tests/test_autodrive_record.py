"""Tests of tools/autodrive_record.py and of the stream it recorded.

`tools/` is not a package, so the module is loaded by path inside a helper
each test calls: a missing file then fails the test, not the collection.
"""

from __future__ import annotations

import gzip
import importlib.util
import json
from pathlib import Path
from types import ModuleType

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
