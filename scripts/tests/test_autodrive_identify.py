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


# --- analysis, against the committed Porto stream (part 1's fixture) --------
# The oracles below were read from the raw rows of the fixture by hand (plan
# 2026-10-26-autodrive-roboracer-vehicle-file-tdd.md, "Facts measured"); none
# of them is a constant in the tool, so a copied constant fails the
# negative case beside it.

FIXTURE = (
    REPO_ROOT
    / "ros_ws"
    / "src"
    / "racing_sim_autodrive"
    / "test"
    / "fixtures"
    / "porto_stream.jsonl.gz"
)
COM_Z = 0.01434  # the guide's COM height above the rear axle (source.yaml)


def _samples(tool: ModuleType) -> list:
    import gzip
    import json

    with gzip.open(FIXTURE, "rt", encoding="utf-8") as handle:
        events = [json.loads(line) for line in handle if line.strip()][1:]
    return tool.samples_from_events(events)


def _segments(tool: ModuleType) -> list[list]:
    return tool.segments(_samples(tool))


def _scaled(segments: list[list], **factors: float) -> list[list]:
    """Segments with named Sample fields multiplied (or shifted by `add_`)."""
    import dataclasses

    out = []
    for seg in segments:
        new = []
        for s in seg:
            changes = {}
            for name, factor in factors.items():
                if name.startswith("add_"):
                    field = name[4:]
                    changes[field] = getattr(s, field) + factor
                else:
                    changes[name] = getattr(s, name) * factor
            new.append(dataclasses.replace(s, **changes))
        out.append(new)
    return out


def _approx(value: float, expected: float, rel: float) -> bool:
    return abs(value - expected) <= rel * abs(expected)


def test_check_1730_segments_split_on_reset_and_collision() -> None:
    """CHECK-1730: a segment ends at a reset (lap time falls from > 0.5 s to
    < 0.1 s) and at a collision (the count rises: the car is teleported to a
    checkpoint). The reset pulse jitters lap time downward 27 times, so a
    splitter on every decrease would make ~28 segments, and a pulse frame
    belongs to none. On the fixture: exactly four segments, the three before
    the reset split at the two collisions, each with a non-decreasing lap
    time and one collision count."""
    tool = _tool()
    samples = _samples(tool)
    segs = tool.segments(samples)
    assert len(segs) == 4, [(s[0].lap_time, s[-1].lap_time) for s in segs]
    spans = [(s[0].lap_time, s[-1].lap_time) for s in segs]
    assert spans[0][0] < 0.03 and abs(spans[0][1] - 5.6268) < 1e-4
    assert abs(spans[1][0] - 5.6838) < 1e-4
    assert abs(spans[1][1] - 6.8427) < 1e-4
    assert abs(spans[2][0] - 6.8657) < 1e-4
    assert abs(spans[2][1] - 12.9146) < 1e-4
    assert spans[3][0] < 0.03 and spans[3][1] > 2.0
    assert [s[0].collisions for s in segs] == [0, 1, 2, 0]
    for seg in segs:
        assert len({s.collisions for s in seg}) == 1
        laps = [s.lap_time for s in seg]
        assert laps == sorted(laps)
    # The only frames lost are the reset pulse's, where lap time is near 0.
    kept = {id(f) for seg in segs for f in seg}
    lost = [f for f in samples if id(f) not in kept]
    assert lost and all(f.lap_time < 0.1 for f in lost), len(lost)


def test_check_1740_throttle_gain_from_steering_zero_windows() -> None:
    """CHECK-1740: speed per unit throttle from steady windows with the
    reported steering at zero (a steady turn is 0.7 % slower): the fixture's
    straight at 0.04 holds 1.0009 m/s, 25.02 per unit. Velocities x 1.1 give a
    gain x 1.1, so the number is computed, not remembered."""
    tool = _tool()
    segs = _segments(tool)
    gain = tool.throttle_gain(segs)
    assert gain.n >= 1
    assert abs(gain.value - 25.02) < 0.1, gain
    scaled = tool.throttle_gain(_scaled(segs, vx=1.1))
    assert _approx(scaled.value, gain.value * 1.1, 0.002), scaled


def test_check_1745_burst_acceleration_from_rest() -> None:
    """CHECK-1745: the machine's acceleration is the speed gained in the first
    BURST_WINDOW seconds of a full-throttle run that starts from rest (the
    drive's 0.3 s burst: v 0.08 to 1.4 m/s in 0.25 s, about 5 m/s^2). Lower
    throttles are not a burst; the fixture has none; speeds x 2 give x 2."""
    import dataclasses

    tool = _tool()

    def frame(t: float, v: float, throttle: float):
        return tool.Sample(
            lap_time=t, throttle=throttle, steering=0.0, x=0.0, y=0.0,
            z=0.06, yaw=0.0, vx=v, vy=0.0, wz=0.0, collisions=0,
        )  # fmt: skip

    rest = [frame(i * 0.02, 0.0, 0.0) for i in range(5)]
    burst = [frame(0.1 + i * 0.02, 4.0 * i * 0.02, 1.0) for i in range(30)]
    coast = [frame(0.7 + i * 0.02, 0.5, 0.0) for i in range(5)]
    seg = rest + burst + coast
    accel = tool.burst_acceleration([seg])
    assert accel.n == 1
    assert abs(accel.value - 4.0) < 1e-6, accel
    doubled = [dataclasses.replace(f, vx=f.vx * 2) for f in seg]
    assert abs(tool.burst_acceleration([doubled]).value - 8.0) < 1e-6
    gentle = [dataclasses.replace(f, throttle=f.throttle * 0.5) for f in seg]
    assert tool.burst_acceleration([gentle]).n == 0
    assert tool.burst_acceleration(_segments(tool)).n == 0


def test_check_1750_braking_deceleration_between_90_and_10_percent() -> None:
    """CHECK-1750: mean deceleration between the 90 % and 10 % crossings of
    the speed the car left throttle at (linear interpolation between frames,
    lap time): the fixture's stop from 0.9937 m/s takes 10.8631 -> 10.9865 s,
    6.44 m/s^2 (not 6.56: that skips the odd rows). Lap time x 2 halves it."""
    tool = _tool()
    segs = _segments(tool)
    brake = tool.braking(segs)
    assert brake.n >= 1
    assert _approx(brake.value, 6.44, 0.02), brake
    slow = tool.braking(_scaled(segs, lap_time=2.0))
    assert _approx(slow.value, brake.value / 2, 0.001), slow


def test_check_1760_steering_limit_slew_and_sign() -> None:
    """CHECK-1760: the reported steering is the actual (slewed) angle, so the
    limit is reported / command on a held command (0.3 -> 0.1571: 0.5236), the
    slew is the rate of the interior of a reversal ramp (+0.1571 -> -0.1571 at
    lap 7.80-7.91: 3.2 rad/s), and the sign is whether the pose track turns
    the way the angle says (+1: left is positive, z up). Mirroring y gives -1;
    lap time x 2 halves the slew."""
    tool = _tool()
    segs = _segments(tool)
    est = tool.steering(segs)
    assert abs(est.limit.value - 0.5236) < 0.001, est
    assert abs(est.slew.value - 3.2) < 0.2, est
    assert est.sign == 1
    mirrored = tool.steering(_scaled(segs, y=-1.0))
    assert mirrored.sign == -1
    slow = tool.steering(_scaled(segs, lap_time=2.0))
    assert _approx(slow.slew.value, est.slew.value / 2, 0.001), slow


def test_check_1770_yaw_response_and_rest_height() -> None:
    """CHECK-1770: the effective wheelbase v tan(delta) / omega in steady turns
    (0.3388 against the 0.33 geometric: mild understeer), and the CoG height
    h = median z at rest + the COM's height above the rear axle (0.0592 +
    0.01434). Yaw rate x 1.1 divides the first by 1.1; z + 0.01 adds 0.01."""
    tool = _tool()
    segs = _segments(tool)
    yaw = tool.yaw_response(segs)
    assert abs(yaw.wheelbase.value - 0.3388) < 0.003, yaw
    faster = tool.yaw_response(_scaled(segs, wz=1.1))
    assert _approx(faster.wheelbase.value, yaw.wheelbase.value / 1.1, 0.002)
    height = tool.rest_height(segs, COM_Z)
    assert abs(height.value - 0.0735) < 0.0005, height
    higher = tool.rest_height(_scaled(segs, add_z=0.01), COM_Z)
    assert abs(higher.value - (height.value + 0.01)) < 1e-4, higher


def test_check_1770_rise_time_is_the_step_not_the_reversal() -> None:
    """CHECK-1770 (rise time): the drive steps +0.3 for 0.5 s and then
    reverses to -0.3, both inside the 1.0 s window after the angle leaves
    zero. The 10-90 % rise time is the step's: a first-order yaw response
    with tau 0.05 s rises in 0.05 ln 9 = 0.110 s (the tool's plateau is the
    mean of the frames it calls steady, so it reads a little under: 0.097).
    The reversal here is three times slower (tau 0.15 s, 0.33 s), so a
    plateau taken from the last angle in the window - the reversal's -
    reports 0.19 s, not under 0.15."""
    import math

    tool = _tool()
    dt = 0.005
    frames = [(i * dt, 0.0, 0.0) for i in range(5)]
    wz = 0.0
    for i in range(100):  # the step, to +0.9 rad/s
        wz += (0.9 - wz) * (1 - math.exp(-dt / 0.05))
        frames.append(((5 + i) * dt, 0.3, wz))
    for i in range(100):  # the reversal, to -0.9 rad/s, slower
        wz += (-0.9 - wz) * (1 - math.exp(-dt / 0.15))
        frames.append(((105 + i) * dt, -0.3, wz))
    seg = [
        tool.Sample(
            lap_time=t,
            throttle=0.1,
            steering=angle,
            x=0.0,
            y=0.0,
            z=0.06,
            yaw=0.0,
            vx=1.0,
            vy=0.0,
            wz=w,
            collisions=0,
        )
        for t, angle, w in frames
    ]
    yaw = tool.yaw_response([seg])
    assert yaw.rise_time is not None
    assert 0.07 < yaw.rise_time.value < 0.15, yaw.rise_time


def test_check_1775_pose_is_the_rear_axle_velocity_is_the_body() -> None:
    """CHECK-1775: in a steady turn the pose point has no lateral speed in the
    body frame (it is the rear axle: < 0.01 m/s) while `V1 Linear Velocity`
    does (vy = omega x 0.160: the body, about the CoG). A pose shifted 0.155 m
    forward along the heading - the CoG - fails the first check."""
    tool = _tool()
    segs = _segments(tool)
    check = tool.reference_points(segs)
    assert check.n >= 3
    assert abs(check.pose_lateral_speed) < 0.01, check
    assert abs(check.velocity_lever - 0.160) < 0.01, check
    import math

    shifted = [
        [
            __import__("dataclasses").replace(
                s,
                x=s.x + 0.155 * math.cos(s.yaw),
                y=s.y + 0.155 * math.sin(s.yaw),
            )
            for s in seg
        ]
        for seg in segs
    ]
    assert abs(tool.reference_points(shifted).pose_lateral_speed) > 0.03


# --- the committed identification ------------------------------------------

VEHICLE_DIR = REPO_ROOT / "config" / "vehicles" / "autodrive_roboracer"
RECORDING = VEHICLE_DIR / "identification" / "2026-iros-practice.jsonl.gz"
SOURCE = VEHICLE_DIR / "source.yaml"
IDENTIFIED = VEHICLE_DIR / "identified.yaml"


def test_check_1780_committed_recording_reanalyses_to_identified() -> None:
    """CHECK-1780: `identified.yaml` is exactly what the tool computes from the
    committed recording and `source.yaml` - no hand edit survives, and nothing
    in it comes from the analysing host (provenance is the recording's header
    and bytes). The live run is a sample (wall-clock, unseeded); this pins the
    analysis to it."""
    import hashlib

    import yaml

    tool = _tool()
    first = tool.analyse(RECORDING, SOURCE)
    assert first == tool.analyse(RECORDING, SOURCE), "not deterministic"
    committed = yaml.safe_load(IDENTIFIED.read_text())
    assert first == committed
    provenance = committed["provenance"]
    digest = hashlib.sha256(RECORDING.read_bytes()).hexdigest()
    assert provenance["recording_sha256"] == digest
    assert provenance["schedule"] == tool.SCHEDULE_ID
    assert "@sha256:" in provenance["images"]["sim"]
    # The burst's spread is measured over the repeats like the others', not
    # the Estimate default of 0.0.
    burst = committed["measurements"]["burst_acceleration"]
    repeats = committed["measurements"]["repeats"]["burst_acceleration"]
    assert len(repeats["values"]) == 3
    assert burst["spread"] == repeats["spread"]


VEHICLE_YAML = REPO_ROOT / "config" / "vehicles" / "autodrive_roboracer.yaml"
DEFAULT_YAML = REPO_ROOT / "config" / "vehicles" / "f1tenth_default.yaml"
GRAVITY = 9.81


def _node(path: Path, node: str) -> dict:
    import yaml

    return yaml.safe_load(path.read_text())[node]["ros__parameters"]


def test_check_1790_vehicle_file_states_the_autodrive_car() -> None:
    """CHECK-1790: the vehicle file's numbers are the ones `source.yaml`
    read (geometry, laser mount) or the default's reference-stack caps
    (speed, acceleration, steering rate: owner ruling D2), the tables are
    TUM's, and nothing in them claims a top speed or a single deceleration
    the identification did not establish (owner rulings D3, D4)."""
    import numpy as np
    import yaml

    values = yaml.safe_load(SOURCE.read_text())["values"]

    def read(name: str) -> float:
        return float(values[name]["value"])

    sim = _node(VEHICLE_YAML, "racing_sim")
    assert (sim["laser_x"], sim["laser_y"], sim["laser_yaw"]) == (
        read("lidar_x_m"),
        read("lidar_y_m"),
        read("lidar_yaw_rad"),
    )
    controller = _node(VEHICLE_YAML, "racing_controller_baseline")
    supervisor = _node(VEHICLE_YAML, "racing_safety_supervisor")
    assert controller["wheelbase"] == read("wheelbase_table_m")
    assert supervisor["vehicle_half_width"] == read("half_width_m")
    assert supervisor["maximum_steering_angle"] == read("steering_limit_rad")
    default_controller = _node(DEFAULT_YAML, "racing_controller_baseline")
    default_supervisor = _node(DEFAULT_YAML, "racing_safety_supervisor")
    for key in (
        "maximum_speed",
        "maximum_acceleration",
        "maximum_steering_rate",
    ):
        assert supervisor[key] == default_supervisor[key], key
    assert controller["maximum_speed"] == default_controller["maximum_speed"]
    assert supervisor["stale_input_timeout_ms"] == 100
    for node in ("racing_metrics", "racing_recording"):
        version = _node(VEHICLE_YAML, node)["vehicle_parameter_version"]
        assert version == "autodrive-roboracer-v2", node

    # The tables: ax is the cap (the measured machine, 5.2 m/s^2 over the
    # first 0.25 s, is above it); ay is mu * g with mu from the prefab.
    identified = yaml.safe_load(IDENTIFIED.read_text())["measurements"]
    machine = identified["burst_acceleration"]["value"]
    cap = supervisor["maximum_acceleration"]
    mu = read("mu_lateral")
    tables = VEHICLE_DIR
    ggv = np.loadtxt(tables / "ggv.csv", delimiter=",", comments="#")
    machines = np.loadtxt(
        tables / "ax_max_machines.csv", delimiter=",", comments="#"
    )
    assert ggv[0, 0] == 0.0 and ggv[-1, 0] == 10.0
    assert np.allclose(ggv[:, 1], min(cap, machine))
    assert np.allclose(ggv[:, 2], mu * GRAVITY, atol=1e-5)
    assert np.allclose(machines[:, 1], min(cap, machine))

    # D4: the top speed is recorded as not measured, and braking is not a
    # single number.
    top = identified["top_speed"]
    assert top["measured"] is False
    assert top["highest_steady_speed"] < 3.0
    assert identified["braking_deceleration"]["depends_on_speed"] is True
    header = (tables / "ggv.csv").read_text().splitlines()[:6]
    assert any("not measured" in line for line in header)
    import re

    text = re.sub(r"\s*#\s*", " ", VEHICLE_YAML.read_text())
    assert "not measured" in text and "depends on speed" in text


def test_check_1795_autodrive_wheelbase_is_the_specification() -> None:
    """CHECK-1795: the AutoDRIVE car's wheelbase is the specification's
    (guide table :180 = the prefab's Ackermann field, 324 mm, :31695; owner
    2026-11-01), not the TF/collider front-wheel x (`wheelbase_m`). The TUM
    split takes lr from the image's CoG (`com_x_lr_m`) and lf as the rest."""
    import pytest
    import yaml

    values = yaml.safe_load(SOURCE.read_text())["values"]

    def read(name: str) -> float:
        return float(values[name]["value"])

    spec = read("wheelbase_table_m")
    prefab = read("wheelbase_prefab_field_mm") / 1000
    assert spec == pytest.approx(prefab, abs=1e-12)
    controller = _node(VEHICLE_YAML, "racing_controller_baseline")
    assert controller["wheelbase"] == spec
    dynamics = yaml.safe_load((VEHICLE_DIR / "tum_mintime.yaml").read_text())
    assert dynamics["wheelbase_rear"] == read("com_x_lr_m")
    assert dynamics["wheelbase_front"] + dynamics["wheelbase_rear"] == (
        pytest.approx(spec, abs=1e-9)
    )


# --- part 1L increment 1: the readers the lateral work needs -----------------
WHEEL_RADIUS = 0.059  # source.yaml wheel_radius_m


def test_check_2600_parse_frame_carries_imu_and_encoder_angles() -> None:
    """CHECK-2600: `parse_frame` keeps the IMU a_x / a_y and both encoder
    angles (the recorder strips only camera and lidar, so every frame has
    them); a frame without them gives nan / None; a hand-built `Sample` still
    constructs without them. Oracle: fixture frame 300, read by hand."""
    import math

    tool = _tool()
    samples = _samples(tool)
    s = samples[300]
    assert s.ax == -0.0346 and s.ay == -0.4620
    assert s.wheel_angles == (133.9628, 134.5518)
    assert samples[299].wheel_angles == (133.6846, 134.2835)
    bare = tool.parse_frame(
        {
            "V1 Position": "0 0 0",
            "V1 Linear Velocity": "1 0 0",
            "V1 Orientation Euler Angles": "0 0 0",
            "V1 Angular Velocity": "0 0 0",
            "V1 Lap Time": "1.0",
            "V1 Throttle": "0",
            "V1 Steering": "0",
            "V1 Collisions": "0",
        }
    )
    assert math.isnan(bare.ax) and math.isnan(bare.ay)
    assert bare.wheel_angles is None
    hand = tool.Sample(
        lap_time=0.0, throttle=0.0, steering=0.0, x=0.0, y=0.0, z=0.0,
        yaw=0.0, vx=0.0, vy=0.0, wz=0.0, collisions=0,
    )  # fmt: skip
    assert math.isnan(hand.ax) and hand.wheel_angles is None


def test_check_2605_wheel_speed_is_the_encoder_delta_over_lap_time() -> None:
    """CHECK-2605: wheel speed = mean encoder angle delta x r / lap-time dt
    (gotcha 35: never the receive time). None on dt <= 0 (a lap-line reset)
    and on a backward angle (the encoders zero on a reset), never a negative
    speed. It equals `racing_sim_autodrive.core.encoder_speed` on every
    fixture pair: the tool is stdlib-only and duplicates those 8 lines, and
    this is what stops the copy drifting. Hand oracle: frames 300 -> 301,
    1.0521 rad mean in 0.062 s is 1.0012 m/s."""
    import dataclasses
    from types import SimpleNamespace

    from racing_sim_autodrive import core

    tool = _tool()
    samples = _samples(tool)
    speed = tool.wheel_speed(samples[300], samples[301], WHEEL_RADIUS)
    assert _approx(speed, 1.0012, 0.001), speed
    assert not _approx(
        tool.wheel_speed(samples[300], samples[301], WHEEL_RADIUS * 1.1),
        1.0012,
        0.001,
    )
    n = 0
    for prev, cur in zip(samples, samples[1:], strict=False):
        dt = cur.lap_time - prev.lap_time
        expected = core.encoder_speed(
            SimpleNamespace(encoder_angles=prev.wheel_angles),
            SimpleNamespace(encoder_angles=cur.wheel_angles),
            WHEEL_RADIUS,
            dt if dt > 0.0 else None,
        )
        got = tool.wheel_speed(prev, cur, WHEEL_RADIUS)
        assert got == expected, (prev.lap_time, cur.lap_time)
        n += got is not None
    assert n > 300, n
    back = dataclasses.replace(samples[301], wheel_angles=(0.0, 0.0))
    assert tool.wheel_speed(samples[300], back, WHEEL_RADIUS) is None
    same = dataclasses.replace(samples[301], lap_time=samples[300].lap_time)
    assert tool.wheel_speed(samples[300], same, WHEEL_RADIUS) is None
    none = dataclasses.replace(samples[301], wheel_angles=None)
    assert tool.wheel_speed(samples[300], none, WHEEL_RADIUS) is None


def test_check_2610_slip_ratio_signs_and_rest() -> None:
    """CHECK-2610: slip = (w - v) / max(w, v): +1 for full spin (v 0), -1 for
    locked wheels (w 0), 0 when both are below REST_SPEED, and 0.15 for a
    wheel at 5.0 against a body at 4.25."""
    tool = _tool()
    assert tool.slip_ratio(5.0, 0.0) == 1.0
    assert tool.slip_ratio(0.0, 3.0) == -1.0
    assert tool.slip_ratio(0.0, 0.0) == 0.0
    assert tool.slip_ratio(0.004, 0.002) == 0.0
    assert abs(tool.slip_ratio(5.0, 4.25) - 0.15) < 1e-12
    assert abs(tool.slip_ratio(4.25, 5.0) + 0.15) < 1e-12


def test_check_2615_sensor_checks_imu_against_the_kinematics() -> None:
    """CHECK-2615: on the committed recording the IMU a_x is the body
    acceleration (median ratio to dv_x/dt ~1.001, no gravity) and the IMU a_y
    is v_x w_z in turns (median ratio ~1.003): the facts F5 rests the lateral
    work on. This re-derives a committed recording (CHECK-1780's class), it is
    not a bound on the car. v_x x 1.1 moves the first to ~1/1.1 and the
    second to ~1/1.1 as well."""
    tool = _tool()
    rec = tool._recorder()
    _, events = rec.read_records(RECORDING)
    segs = tool.segments(tool.samples_from_events(events))
    checks = tool.sensor_checks(segs)
    assert checks.ax_n > 1000 and checks.ay_n > 300, checks
    assert _approx(checks.ax_ratio, 1.001, 0.02), checks
    assert _approx(checks.ay_ratio, 1.003, 0.02), checks
    scaled = tool.sensor_checks(_scaled(segs, vx=1.1))
    assert _approx(scaled.ax_ratio, checks.ax_ratio / 1.1, 0.02), scaled
    assert _approx(scaled.ay_ratio, checks.ay_ratio / 1.1, 0.02), scaled
    assert not _approx(scaled.ax_ratio, 1.001, 0.02)


# --- part 1L increment 1: the lateral schedule --------------------------------
SOURCE = (
    REPO_ROOT / "config" / "vehicles" / "autodrive_roboracer" / "source.yaml"
)


def _wheelbase() -> float:
    import yaml

    values = yaml.safe_load(SOURCE.read_text())["values"]
    return values["wheelbase_table_m"]["value"]


def test_check_2650_lateral_schedule_stays_in_the_corridor() -> None:
    """CHECK-2650: `lateral_schedule` is a pure function of time, writes the
    devkit's strings, stays in [-1, 1], opens every manoeuvre with the reset
    pulse and rest and ends at rest. For every rung a kinematic bicycle
    (`planned_path`: L from source.yaml, steering slewed at 3.2 rad/s, the
    locked-wheel stop) keeps max |y| <= CORRIDOR_OFFSET_M, ends within 3 deg
    of straight and fits STRAIGHT_CLEAR_M. Oracles come from independent
    throwaway code (plan.py of the step: the same bicycle, its distance
    short by the schedule's 0.02 s return tail, 0.02 v). Three negative cases
    fail one condition each. The v1 schedule is unchanged."""
    tool = _tool()
    steps = int(tool.LATERAL_END / 0.05) + 20
    for i in range(steps):
        t = i * 0.05
        command = tool.lateral_schedule(t)
        assert set(command) == REPLY_KEYS, t
        assert command == tool.lateral_schedule(t), t
        assert abs(float(command["V1 Throttle"])) <= 1.0, t
        assert abs(float(command["V1 Steering"])) <= 1.0, t
    assert tool.lateral_schedule(tool.LATERAL_END + 5.0) == {
        "V1 Throttle": "0.0",
        "V1 Steering": "0.0",
        "V1 Reset": "False",
    }
    assert tool.LATERAL_SCHEDULE_ID == "porto-lateral-v1"
    assert tool.LATERAL_REPEATS == 3 and tool.CORRIDOR_OFFSET_M == 0.35
    assert tool.STRAIGHT_CLEAR_M == 7.9

    segments = tool.build_schedule(
        tool.LATERAL_REPEATS, tool.LATERAL_MANOEUVRES
    )
    assert len(tool.LATERAL_MANOEUVRES) == len(tool.LATERAL_RUNGS) == 10
    assert (
        abs(tool.LATERAL_END - (sum(s.duration for s in segments) + tool.TAIL))
        < 1e-9
    )
    for first in range(0, len(segments), 8):  # 2 pre + 6 rung segments
        assert segments[first].reset and segments[first].duration == 0.5
        assert segments[first + 1].duration == 1.5
        assert not segments[first + 1].reset
        stop = segments[first + 7]
        assert stop.name == "stop" and stop.throttle == 0.0
        assert stop.steering == 0.0

    wheelbase = _wheelbase()
    for a, v, hold in tool.LATERAL_RUNGS:
        path = tool.planned_path(tool.lateral_rung(a, v, hold), v, wheelbase)
        assert path["max_y"] <= tool.CORRIDOR_OFFSET_M, (a, v, path)
        assert abs(path["heading_deg"]) <= 3.0, (a, v, path)
        assert path["distance"] <= tool.STRAIGHT_CLEAR_M, (a, v, path)

    # Hand oracles (independent bicycle; distance + 0.02 s x v).
    for (a, v, hold), max_y, distance in (
        ((0.3, 3.5, 0.170), 0.221, 4.872 + 0.02 * 3.5),
        ((0.3, 4.8, 0.100), 0.167, 7.267 + 0.02 * 4.8),
        ((0.4, 3.2, 0.130), 0.170, 4.119 + 0.02 * 3.2),
        ((0.5, 2.8, 0.115), 0.144, 3.397 + 0.02 * 2.8),
        ((0.6, 2.4, 0.105), 0.121, 2.760 + 0.02 * 2.4),
    ):
        path = tool.planned_path(tool.lateral_rung(a, v, hold), v, wheelbase)
        assert _approx(path["max_y"], max_y, 0.02), (a, v, path)
        assert _approx(path["distance"], distance, 0.02), (a, v, path)
        scaled = tool.planned_path(
            tool.lateral_rung(a, v, hold), v, wheelbase * 1.1
        )
        assert not _approx(scaled["max_y"], max_y, 0.001), (a, v, scaled)

    unbalanced = tool.planned_path(
        tool.lateral_rung(0.5, 3.5, 0.065, balanced=False), 3.5, wheelbase
    )
    assert abs(unbalanced["heading_deg"]) > 3.0, unbalanced  # 8.4 deg
    wide = tool.planned_path(tool.lateral_rung(1.0, 2.7, 0.0), 2.7, wheelbase)
    assert wide["max_y"] > tool.CORRIDOR_OFFSET_M, wide  # 0.77 m
    long_run = tool.planned_path(
        tool.lateral_rung(0.3, 5.2, 0.1), 5.2, wheelbase
    )
    assert long_run["distance"] > tool.STRAIGHT_CLEAR_M, long_run  # 8.4 m

    # The v1 schedule is untouched (CHECK-1710 holds it too).
    assert tool.build_schedule(tool.REPEATS) == tool.build_schedule(
        tool.REPEATS, tool.MANOEUVRES
    )
    assert tool.SCHEDULE_END == 115.9 and len(tool._SEGMENTS) == 105
    assert tool.identify_schedule(3.0) == tool.identify_schedule(
        3.0, tool._SEGMENTS
    )


def test_check_2655_record_and_analyse_know_the_lateral_schedule(
    tmp_path,
) -> None:
    """CHECK-2655: `record --schedule lateral` is accepted and writes header
    `schedule: porto-lateral-v1` with `repeats: 3`; the default stays
    `identify`; an unknown schedule exits 2; `analyse` dispatches on the
    header (`--out` for identify, `--out-dir` for lateral, the wrong one for
    the header's schedule exits 2); a header naming no schedule raises."""
    import pytest

    tool = _tool()
    parser = tool.build_parser()
    args = parser.parse_args(["record", "--config", "c.yaml", "--out", "o.gz"])
    assert args.schedule == "identify"
    args = parser.parse_args(
        [
            "record",
            "--config",
            "c.yaml",
            "--out",
            "o.gz",
            "--schedule",
            "lateral",
        ]
    )
    assert args.schedule == "lateral"
    with pytest.raises(SystemExit) as exit_info:
        parser.parse_args(
            ["record", "--config", "c", "--out", "o", "--schedule", "bogus"]
        )
    assert exit_info.value.code == 2
    assert tool.schedule_for("identify") == (
        tool.SCHEDULE_ID,
        tool.identify_schedule,
        tool.SCHEDULE_END,
        tool.REPEATS,
    )
    assert tool.schedule_for("lateral") == (
        tool.LATERAL_SCHEDULE_ID,
        tool.lateral_schedule,
        tool.LATERAL_END,
        tool.LATERAL_REPEATS,
    )
    with pytest.raises(ValueError, match="bogus"):
        tool.schedule_for("bogus")
    # analyse: the wrong output option for the header's schedule exits 2.
    rec = tool._recorder()
    for schedule_id, wrong in (
        (tool.LATERAL_SCHEDULE_ID, ["--out", str(tmp_path / "x.yaml")]),
        (tool.SCHEDULE_ID, ["--out-dir", str(tmp_path)]),
    ):
        path = tmp_path / f"{schedule_id}.jsonl.gz"
        writer = rec.RecordWriter(path, {"schedule": schedule_id, "repeats": 3})
        writer.close()
        code = tool.main(
            ["analyse", "--recording", str(path), "--source", str(SOURCE)]
            + wrong
        )
        assert code == 2, schedule_id
    bad = tmp_path / "bad.jsonl.gz"
    rec.RecordWriter(bad, {"schedule": "bogus"}).close()
    with pytest.raises(ValueError, match="bogus"):
        tool.main(
            ["analyse", "--recording", str(bad), "--source", str(SOURCE),
             "--out", str(tmp_path / "y.yaml")]
        )  # fmt: skip
