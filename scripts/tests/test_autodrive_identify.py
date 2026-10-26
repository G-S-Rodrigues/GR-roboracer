"""Tests of tools/autodrive_identify.py, the AutoDRIVE vehicle identification.

`tools/` is not a package, so the module is loaded by path inside a helper
each test calls: a missing file then fails the test, not the collection.
"""

from __future__ import annotations

import importlib.util
import sys
import tomllib
from pathlib import Path
from types import ModuleType

REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS = REPO_ROOT / "tools"
TOOL = TOOLS / "autodrive_identify.py"
RECORDER = TOOLS / "autodrive_record.py"
REPLY_KEYS = {"V1 Throttle", "V1 Steering", "V1 Reset"}


def _tool() -> ModuleType:
    spec = importlib.util.spec_from_file_location("autodrive_identify", TOOL)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses look the module up
    spec.loader.exec_module(module)
    return module


def _script_block(path: Path) -> dict:
    """The PEP 723 `# /// script` block of a uv script, parsed."""
    lines = path.read_text().splitlines()
    start = lines.index("# /// script")
    end = lines.index("# ///", start + 1)
    body = "\n".join(line[2:] for line in lines[start + 1 : end])
    return tomllib.loads(body)


def test_check_1710_identify_schedule_is_deterministic_and_resets_first() -> (
    None
):
    """CHECK-1710: the identification drive is a pure function of time, writes
    strings as the devkit does, never leaves [-1, 1], puts a 0.5 s reset pulse
    and 1.5 s of rest before every manoeuvre (the sim connects once, so a reset
    is the only way back to spawn), repeats the whole list `REPEATS` times and
    ends at rest."""
    tool = _tool()
    segments = tool.build_schedule(tool.REPEATS)
    assert segments == tool.build_schedule(tool.REPEATS)
    total = sum(seg.duration for seg in segments)
    assert abs(tool.SCHEDULE_END - (total + tool.TAIL)) < 1e-9

    # Pure function of time, devkit string forms, bounded.
    steps = int(tool.SCHEDULE_END / 0.05) + 20
    for i in range(steps):
        t = i * 0.05
        command = tool.identify_schedule(t)
        assert set(command) == REPLY_KEYS, t
        assert command == tool.identify_schedule(t), t
        for key in ("V1 Throttle", "V1 Steering"):
            value = float(command[key])
            assert -1.0 <= value <= 1.0, (t, key)
            assert command[key] == str(value), (t, key, command[key])
        assert command["V1 Reset"] in ("True", "False"), t

    # Every manoeuvre is preceded by reset pulse + rest, in that order.
    manoeuvres = [
        i
        for i, s in enumerate(segments)
        if s.name != "reset" and s.name != "rest"
    ]
    assert manoeuvres
    starts = [i for i in manoeuvres if segments[i - 1].name == "rest"]
    for first in starts:
        assert segments[first - 2].name == "reset"
        assert segments[first - 2].reset is True
        assert abs(segments[first - 2].duration - 0.5) < 1e-9
        assert abs(segments[first - 1].duration - 1.5) < 1e-9
        assert (segments[first - 1].throttle, segments[first - 1].steering) == (
            0.0,
            0.0,
        )
    # No command is issued to a manoeuvre that is not preceded by a reset.
    names = [s.name for s in segments]
    one = len(tool.MANOEUVRES)
    assert names.count("reset") == one * tool.REPEATS
    assert tool.REPEATS >= 3
    once = tool.build_schedule(1)
    assert segments == once * tool.REPEATS
    # The straight, the burst and the steering step are in the list.
    assert {"throttle", "burst", "steer_step", "steer_rest"} <= set(names)
    # Ends at rest, and rest is not a reset.
    rest = {"V1 Throttle": "0.0", "V1 Steering": "0.0", "V1 Reset": "False"}
    assert tool.identify_schedule(tool.SCHEDULE_END - 0.01) == rest
    assert tool.identify_schedule(tool.SCHEDULE_END + 1.0) == rest
    assert tool.identify_schedule(0.0)["V1 Reset"] == "True"


def test_check_1720_identify_pins_the_recorders_libraries() -> None:
    """CHECK-1720: the identify script serves the sim with exactly the
    recorder's library set (python-socketio 4.2.0 / engineio 3.13.0 is the
    only pair the sim talks to) plus pyyaml, and both lock files exist."""
    identify = _script_block(TOOL)
    recorder = _script_block(RECORDER)
    assert identify["requires-python"] == recorder["requires-python"]
    assert sorted(identify["dependencies"]) == sorted(recorder["dependencies"])
    for dep in identify["dependencies"]:
        assert "==" in dep, f"{dep} is not pinned"
    assert (TOOLS / "autodrive_record.py.lock").is_file()
    assert (TOOLS / "autodrive_identify.py.lock").is_file()


def test_check_1725_strip_frame_drops_camera_and_lidar_only() -> None:
    """CHECK-1725: the recording keeps every field of a sim frame verbatim
    except the camera image and the lidar ranges (megabytes nobody analyses);
    the input frame is not mutated."""
    tool = _tool()
    frame = {
        "V1 Front Camera Image": "x" * 1000,
        "V1 LIDAR Range Array": "y" * 1000,
        "V1 Position": "0.8000 3.1600 0.0600",
        "V1 Lap Time": "1.5",
        "V1 Collisions": "0",
    }
    original = dict(frame)
    stripped = tool.strip_frame(frame)
    assert set(stripped) == {"V1 Position", "V1 Lap Time", "V1 Collisions"}
    for key, value in stripped.items():
        assert value == original[key]
    assert frame == original
    # A frame without a lidar (the first one) is not an error.
    assert tool.strip_frame({"V1 Lap Time": "0"}) == {"V1 Lap Time": "0"}
