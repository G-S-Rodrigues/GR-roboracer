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


# --- part 1L increment 1: steady-delta windows and radius bins ----------------
LAT_L = 0.324
LAT_R = 0.059
LAT_DT = 0.01


def _lat_segment(
    tool,
    v: float,
    delta: float,
    ay,
    wz,
    n: int = 12,
    sign: float = 1.0,
    wheel_scale: float = 1.0,
):
    """One lobe: 3 rest frames, a two-frame steering ramp, `n` frames at the
    steering `delta` (rad) with IMU a_y `ay` and yaw rate `wz` (a number, or
    one per frame), a two-frame return, the car at `v` with its wheels at
    `wheel_scale` x v. Lap time runs 0.01 s per frame."""
    ays = list(ay) if isinstance(ay, list | tuple) else [ay] * n
    wzs = list(wz) if isinstance(wz, list | tuple) else [wz] * n
    steering = [0.0] * 3 + [0.4 * delta, 0.7 * delta] + [delta] * n
    steering += [0.5 * delta, 0.0]
    frames = []
    angle = 0.0
    for i, st in enumerate(steering):
        k = i - 5
        in_window = 0 <= k < n
        frames.append(
            tool.Sample(
                lap_time=i * LAT_DT, throttle=0.1, steering=sign * st,
                x=0.0, y=0.0, z=0.07, yaw=0.0, vx=v, vy=0.0,
                wz=sign * (wzs[k] if in_window else 0.0), collisions=0,
                ax=0.0, ay=sign * (ays[k] if in_window else 0.0),
                wheel_angles=(angle, angle),
            )
        )  # fmt: skip
        angle += wheel_scale * v * LAT_DT / LAT_R
    return frames


def _kin(v: float, delta: float) -> tuple[float, float]:
    import math

    return v * v * math.tan(delta) / LAT_L, LAT_L / math.tan(delta)


def test_check_2665_lateral_windows_label_linear_saturated_oversteer() -> None:
    """CHECK-2665: per steady-steering window (frames within 90 % of the
    lobe's plateau, at least 5, |slip| < 0.05), the kinematic demand a_kin =
    v^2 tan(d) / L and r_kin = L / tan(d) (v the wheel speed), the IMU a_y
    window median, the w_z ratio, the path radius v / w_z, the sideslip and
    the label. Hand oracle: v 3.0, d 0.15708 (A 0.3): tan d = 0.158384, a_kin
    4.3996, r_kin 2.0457. A flat 7.0 at v 4.5 is `saturated` (a_kin 9.899,
    ratio 0.707); the same demand met by an a_y that is still rising and
    reaches the linear line (0.895 a_kin) in the window's last third is
    `linear` (understeer gradient: the lobe ended before a_y settled); w_z
    above the kinematic rate with a_y below it is `oversteer`; a one-frame
    4.4 spike moves no median; a spinning wheel (slip 0.23) or four frames
    give no window."""
    import math

    tool = _tool()
    assert tool.SATURATION_RATIO == 0.895
    delta = 0.3 * 0.5236

    # Linear: a_y 0.94 a_kin, w_z 0.92 of the kinematic rate.
    a_kin, r_kin = _kin(3.0, delta)
    assert _approx(a_kin, 4.3996, 0.002) and _approx(r_kin, 2.0457, 0.002)
    seg = _lat_segment(tool, 3.0, delta, 0.94 * a_kin, 0.92 * 3.0 / r_kin)
    (w,) = tool.lateral_windows([seg], LAT_L, LAT_R, rungs=1)
    assert w.label == "linear" and w.n == 12 and w.rung == 0
    assert _approx(w.a_kin, 4.3996, 0.002) and _approx(w.r_kin, 2.0457, 0.002)
    assert _approx(w.ay, 0.94 * 4.3996, 0.002) and _approx(
        w.ay_ratio, 0.94, 0.002
    )
    assert _approx(w.wz_ratio, 0.92, 0.002)
    assert _approx(w.radius, 2.0457 / 0.92, 0.002) and abs(w.slip) < 1e-6
    assert _approx(w.speed, 3.0, 0.002)
    assert not _approx(w.a_kin, 4.3996 * 1.1, 0.002)
    # The sign of the lobe does not matter.
    mirrored = _lat_segment(
        tool, 3.0, delta, 0.94 * a_kin, 0.92 * 3.0 / r_kin, sign=-1.0
    )
    (n,) = tool.lateral_windows([mirrored], LAT_L, LAT_R, rungs=1)
    assert _approx(n.ay, w.ay, 1e-6) and n.label == "linear"

    # Saturated flat at 7.0 (v 4.5): peak 7.0, and a one-frame spike is no
    # median.
    a_kin, r_kin = _kin(4.5, delta)
    assert _approx(a_kin, 9.899, 0.002)
    flat = [7.0] * 12
    spiked = list(flat)
    spiked[6] = 4.4
    for ay in (flat, spiked):
        seg = _lat_segment(tool, 4.5, delta, ay, [a / 4.5 for a in ay])
        (s,) = tool.lateral_windows([seg], LAT_L, LAT_R, rungs=1)
        assert s.label == "saturated", s
        assert _approx(s.ay, 7.0, 0.001) and _approx(s.ay_ratio, 0.707, 0.002)
        assert not s.rising

    # A slow rise that never reaches the linear line is still saturated.
    creep = [6.5 + 0.06 * k for k in range(12)]  # 6.5 .. 7.16
    seg = _lat_segment(tool, 4.5, delta, creep, [a / 4.5 for a in creep])
    (c,) = tool.lateral_windows([seg], LAT_L, LAT_R, rungs=1)
    assert c.label == "saturated" and not c.rising, c

    # Understeer gradient: a_y still rising through the window while the
    # ratio is below the threshold: linear, not saturated.
    rising = [5.0 + 0.5 * k for k in range(12)]  # 5.0 .. 10.5, median 7.75
    seg = _lat_segment(tool, 4.5, delta, rising, [a / 4.5 for a in rising])
    (r,) = tool.lateral_windows([seg], LAT_L, LAT_R, rungs=1)
    assert r.ay_ratio < tool.SATURATION_RATIO and r.rising
    assert r.label == "linear", r

    # Oversteer: w_z at 1.2 of the kinematic rate, a_y at 0.6 of a_kin.
    seg = _lat_segment(tool, 4.5, delta, 0.6 * a_kin, 1.2 * 4.5 / r_kin)
    (o,) = tool.lateral_windows([seg], LAT_L, LAT_R, rungs=1)
    assert o.label == "oversteer", o
    # a_y low but w_z low by a different amount: the two disagree.
    seg = _lat_segment(tool, 4.5, delta, 0.5 * a_kin, 0.8 * 4.5 / r_kin)
    (i,) = tool.lateral_windows([seg], LAT_L, LAT_R, rungs=1)
    assert i.label == "inconsistent", i

    # Not used: spinning wheels (slip 0.23), or fewer than 5 frames.
    spin = _lat_segment(tool, 3.0, delta, 4.0, 1.0, wheel_scale=1.3)
    assert tool.lateral_windows([spin], LAT_L, LAT_R, rungs=1) == []
    short = _lat_segment(tool, 3.0, delta, 4.0, 1.0, n=4)
    assert tool.lateral_windows([short], LAT_L, LAT_R, rungs=1) == []
    # rung and repeat come from the segment's place in the layout.
    segs = [_lat_segment(tool, 3.0, delta, 4.0, 1.0) for _ in range(6)]
    got = tool.lateral_windows(segs, LAT_L, LAT_R, rungs=3)
    assert [(g.repeat, g.rung) for g in got] == [
        (0, 0), (0, 1), (0, 2), (1, 0), (1, 1), (1, 2),
    ]  # fmt: skip
    assert math.isfinite(got[0].sideslip)


def _bin_segments(tool, factor: float = 1.0, repeats: int = 2):
    """For each radius-bin centre R_c (1.00, 1.21, 1.52, 2.05 m) and f(R) =
    9 + 1.5 R: a linear window (a_y 0.94 a_kin, 0.25 of f) and a saturated
    one (a_y f, demand 1.4 f) at the speed whose path radius v^2 / f is R_c.
    `factor` scales every IMU a_y but not the yaw rate, so the path radius
    (and the bin) stays. One rung per (centre, kind), `repeats` times."""
    import math

    segs = []
    for _ in range(repeats):
        for centre in (1.00, 1.21, 1.52, 2.05):
            f = 9.0 + 1.5 * centre
            v = math.sqrt(centre * f)
            for kind in ("linear", "saturated"):
                if kind == "linear":
                    speed = 0.5 * v
                    r_kin = 0.94 * centre
                    demand = speed * speed / r_kin
                    ay = 0.94 * demand
                else:
                    speed = v
                    r_kin = centre / 1.4
                    demand = speed * speed / r_kin
                    ay = f
                delta = math.atan(LAT_L / r_kin)
                segs.append(
                    _lat_segment(
                        tool, speed, delta, factor * ay, ay / speed,
                    )
                )  # fmt: skip
    return segs


def test_check_2675_lateral_by_radius_recovers_f_of_r() -> None:
    """CHECK-2675: windows binned by their measured path radius v / w_z, bins
    centred on the schedule's R_kin (1.00, 1.21, 1.52, 2.05 m) with edges at
    their geometric means (open at both ends). Per bin: `peak` (the larger of
    the highest linear window and the onset window, the lowest-demand
    saturated one), `bound` (`exact` iff saturation was seen and the onset
    window is the peak, else `lower`), `post_cliff` (median of the saturated
    windows), the relative spread of the peak over repeats, `n`, and per-speed
    cells with `speed_dependent` when two cells' plateaus in one bin differ
    by more than their spreads. A synthetic f(R) = 9 + 1.5 R is recovered at
    each centre; a_y x 2 doubles the peak (nothing saturates then, so it is
    a lower bound)."""
    tool = _tool()
    centres = (1.00, 1.21, 1.52, 2.05)
    segs = _bin_segments(tool)
    windows = tool.lateral_windows(segs, LAT_L, LAT_R, rungs=8)
    assert len(windows) == 16
    bins = tool.lateral_by_radius(windows, centres)
    assert [b.radius for b in bins] == list(centres)
    for b, centre in zip(bins, centres, strict=True):
        f = 9.0 + 1.5 * centre
        assert b.n == 4 and b.bound == "exact", b
        assert _approx(b.peak.value, f, 0.02), (centre, b.peak, f)
        assert _approx(b.post_cliff.value, f, 0.02) and b.post_cliff.n == 2
        assert b.peak.spread == 0.0 and not b.speed_dependent
        assert b.peak.n == 2  # one value per repeat
    doubled = tool.lateral_by_radius(
        tool.lateral_windows(
            _bin_segments(tool, factor=2.0), LAT_L, LAT_R, rungs=8
        ),
        centres,
    )
    for d, centre in zip(doubled, centres, strict=True):
        f = 9.0 + 1.5 * centre
        assert d.bound == "lower" and d.post_cliff is None, d
        assert _approx(d.peak.value, 2.0 * f, 0.02), (centre, d.peak)

    # Speed dependence: two plateaus at one radius (R 1.5: v 3.0 at 6.0 and
    # v 3.3 at 7.26 m/s^2) differ by more than their spreads (0, one value
    # per repeat); equal plateaus do not.
    def cell(v, ay, delta=0.5):
        return _lat_segment(tool, v, delta, ay, ay / v)

    split = [cell(3.0, 6.0), cell(3.3, 7.26)] * 2
    (b,) = tool.lateral_by_radius(
        tool.lateral_windows(split, LAT_L, LAT_R, rungs=2), (1.52,)
    )
    assert b.speed_dependent and len(b.cells) == 2, b
    assert [round(c.speed, 1) for c in b.cells] == [3.0, 3.3]
    same = [cell(3.0, 6.0), cell(3.1, 6.0)] * 2  # R 1.50 and 1.60
    (b,) = tool.lateral_by_radius(
        tool.lateral_windows(same, LAT_L, LAT_R, rungs=2), (1.52,)
    )
    assert not b.speed_dependent, b


TABLE_LINEAR = tuple((r, 9.0 + 1.5 * r) for r in (0.1, 1.0, 3.0, 6.0, 12.0))


def test_check_2680_ggv_from_radius_solves_a_equals_f_of_v2_over_a() -> None:
    """CHECK-2680: a*(v) is the root of a = f(v^2 / a). Constant f = 10 gives
    10 at every v; f(R) = 9 + 1.5 R gives the hand-solved roots of
    a^2 - 9a - 1.5 v^2 = 0 (9.623 at v 2, 11.152 at 4, 13.117 at 6). A falling
    f flags multiple_roots and takes the smallest; v^2 / a past the measured
    radii flags extrapolated."""
    tool = _tool()
    flat = tuple((r, 10.0) for r in (0.5, 2.0, 8.0))
    rows = tool.ggv_from_radius(flat, (0.0, 2.0, 4.0, 6.0))
    assert [round(r.ay, 6) for r in rows] == [10.0] * 4
    rows = tool.ggv_from_radius(TABLE_LINEAR, (0.0, 2.0, 4.0, 6.0))
    for row, want in zip(rows[1:], (9.6235, 11.1521, 13.1168), strict=True):
        assert abs(row.ay - want) < 1e-3, (row, want)
        assert not row.extrapolated and not row.multiple_roots, row
    assert rows[0].ay == 9.0 + 1.5 * 0.1 and not rows[0].extrapolated
    # R = v^2 / a = 22.88^2 / 15 > 12 m: past the table.
    (top,) = tool.ggv_from_radius(TABLE_LINEAR, (22.88,))
    assert top.extrapolated, top
    # f falls steeply with R (12 at 0.5 m to 3 at 1.0 m, then flat): at v 2
    # a - f(4 / a) crosses zero at 3.0, near 4.4 and at 12; the smallest wins.
    falling = ((0.5, 12.0), (1.0, 3.0), (8.0, 3.0))
    (row,) = tool.ggv_from_radius(falling, (2.0,))
    assert row.multiple_roots and abs(row.ay - 3.0) < 1e-6, row


def _bins(tool, peak=9.0, post=5.5):
    est = tool.Estimate
    return [
        tool.RadiusBin(
            radius=r,
            n=10,
            peak=est(peak + r, 3, 0.01),
            bound="exact" if r < 2 else "lower",
            post_cliff=est(post, 3, 0.02),
            cells=(),
            speed_dependent=False,
        )
        for r in (1.0, 1.5, 2.0)
    ]


def test_check_2690_write_limits_writes_labelled_files(tmp_path) -> None:
    """CHECK-2690: both files, every value a {value, label, source} mapping
    with label in {measured, inferred, guide}; top speed read from
    source.yaml, never typed; ggv/ax rows sorted by v, non-negative, <= 50,
    ending at >= 22.88; the increment-1 a_x laws are inferred; rows past a
    law's range are extrapolated and the drive column is 0 with clamped above
    19.7 m/s; the closed-loop fields are null with `pending` until a sweep."""
    import yaml

    tool = _tool()
    source = tmp_path / "source.yaml"
    source.write_text(
        "car:\n  top_speed_m_s: {value: 22.88, source: guide, line: 204}\n"
    )
    tool.write_limits(
        tmp_path, _bins(tool), source=source, recording_sha256="ab" * 32
    )
    docs = {
        k: yaml.safe_load((tmp_path / f"limits_{k}.yaml").read_text())
        for k in ("sliding", "peak")
    }

    def walk(node):
        if isinstance(node, dict):
            if "label" in node:
                assert node["label"] in {"measured", "inferred", "guide"}
                assert "source" in node and "value" in node, node
            for v in node.values():
                walk(v)

    for kind, doc in docs.items():
        walk(doc)
        assert doc["top_speed"]["value"] == 22.88
        assert doc["top_speed"]["label"] == "guide"
        assert doc["top_speed_predicted"]["label"] == "inferred"
        for table in ("ggv", "ax_max_machines"):
            rows = doc[table]["rows"]
            vs = [r["v"] for r in rows]
            assert vs == sorted(vs) and vs[-1] >= 22.88, (kind, table)
            assert all(r["label"] == "inferred" for r in rows)
            assert all(x >= 0 for r in rows for x in r["values"]), rows
            assert all(x <= 50 for r in rows for x in r["values"]), rows
        top = doc["ax_max_machines"]["rows"][-1]
        assert top["extrapolated"], top
        if kind == "sliding":  # 5.38 - 0.273 * 22.88 < 0
            assert top["values"] == [0.0] and top["clamped"], top
        else:  # 8.15 - 0.273 * 22.88 = 1.90 > 0: extrapolated, not clamped
            assert (
                abs(top["values"][0] - 1.9038) < 1e-3 and "clamped" not in top
            )
        assert doc["drive_ax"]["label"] == "inferred"
        assert "plan F2-F4" in doc["drive_ax"]["assumption"]
    sliding, peak = docs["sliding"], docs["peak"]
    assert sliding["lateral"]["ay_reached"]["pending"].startswith(
        "closed-loop sweep"
    )
    assert sliding["lateral"]["ay_reached"]["value"] is None
    assert peak["braking_ax"]["value"] is None
    assert "sliding used" in peak["ggv"]["braking_note"]
    assert peak["drive_ax"]["intercept"] > sliding["drive_ax"]["intercept"]


def _sweep_file(path: Path, drop: set[str] | None = None) -> None:
    """A synthetic corner-sweep channels file (the Step 5b schema): a header,
    four frames, a lap-line reset (lap_time falls, lap_count rises) and a
    clamped run of three frames 1 microsecond apart. `drop` names a stream to
    leave out entirely."""
    import gzip
    import json

    drop = drop or set()
    stamps = [100.0, 100.01, 100.02, 100.03, 100.04]
    stamps += [100.05, 100.05 + 1e-6, 100.05 + 2e-6]
    lap_time = [1.0, 1.01, 1.02, 0.0, 0.01, 0.02, 0.03, 0.04]
    lines: list[dict] = [{"meta": {"sweep": {"target_ay": 6.0}}}]
    for i, t in enumerate(stamps):
        left = 10.0 * i if i != 3 else 0.0  # encoders zero at the reset
        lines += [
            {
                "stream": "imu",
                "t": t,
                "ax": 0.1 * i,
                "ay": 2.0 + i,
                "az": 0.0,
                "wz": 0.5,
            },
            {"stream": "enc", "t": t, "left": left, "right": left},
            {"stream": "truth", "t": t, "vx": 3.0, "vy": 0.1 * i},
            {
                "stream": "track",
                "t": t,
                "s": float(i),
                "d": 0.0,
                "heading_error": 0.0,
            },
            {
                "stream": "counters",
                "t": t,
                "lap_count": 1 if i >= 3 else 0,
                "lap_time": lap_time[i],
                "collisions": 0,
            },
        ]
    # one /imu frame whose stamp has no counters message is dropped
    lines.append(
        {"stream": "imu", "t": 200.0, "ax": 0, "ay": 0, "az": 0, "wz": 0}
    )
    lines += [
        {
            "stream": "drive",
            "t": 99.995 + 0.01 * k,
            "steer": 0.1 * k,
            "speed": 3.0,
        }
        for k in range(8)
    ]
    lines = [ln for ln in lines if "meta" in ln or ln["stream"] not in drop]
    with gzip.open(path, "wt", encoding="utf-8") as handle:
        for ln in lines:
            handle.write(json.dumps(ln) + "\n")


def test_check_2685_parse_sweep_joins_one_frame_per_stamp(tmp_path) -> None:
    """CHECK-2685: `parse_sweep` joins the IMU, encoder, truth, track and
    counters messages that carry the same stamp into one `Sample` with the
    counters' `lap_time`; the commanded steering is the nearest preceding
    `/drive` message; `wheel_speed` across the lap-line reset is None, not a
    negative speed; a frame without counters is dropped and counted; three
    frames 1 microsecond apart stay three samples; a missing channel raises
    naming it."""
    import math

    import pytest

    tool = _tool()
    path = tmp_path / "sweep.jsonl.gz"
    _sweep_file(path)
    parsed = tool.parse_sweep(path)
    samples = parsed.samples
    assert len(samples) == 8, len(samples)
    assert parsed.dropped == 1
    assert parsed.meta["sweep"]["target_ay"] == 6.0
    assert [s.lap_time for s in samples] == [
        1.0,
        1.01,
        1.02,
        0.0,
        0.01,
        0.02,
        0.03,
        0.04,
    ]
    first = samples[0]
    assert (first.ax, first.ay, first.wz) == (0.0, 2.0, 0.5)
    assert (first.vx, first.vy) == (3.0, 0.0)
    assert first.wheel_angles == (0.0, 0.0)
    assert first.collisions == 0
    # the stamp 100.0 follows the /drive message at 99.995 (steer 0.0)
    assert first.cmd_steering == 0.0
    assert math.isclose(samples[2].cmd_steering, 0.2)
    assert samples[3].steering == samples[3].cmd_steering
    assert tool.wheel_speed(samples[2], samples[3], 0.059) is None
    assert tool.wheel_speed(samples[0], samples[1], 0.059) is not None
    assert tool.wheel_speed(samples[0], samples[1], 0.059) > 0.0
    with pytest.raises(ValueError, match="enc"):
        broken = tmp_path / "no_enc.jsonl.gz"
        _sweep_file(broken, drop={"enc"})
        tool.parse_sweep(broken)


LATERAL_RECORDING = (
    VEHICLE_DIR / "identification" / "2026-iros-practice-lateral.jsonl.gz"
)


SWEEP_RECORDING = (
    VEHICLE_DIR / "identification" / "2026-iros-practice-corner-sweep.jsonl.gz"
)


def test_check_2670_committed_lateral_recording_reanalyses_to_limits(
    tmp_path,
) -> None:
    """CHECK-2670: `limits_sliding.yaml` and `limits_peak.yaml` are exactly
    what the tool computes from the committed lateral recording and
    `source.yaml` (CHECK-1780's pattern): no hand edit survives, and the
    recording's bytes are their provenance."""
    import hashlib

    tool = _tool()
    assert LATERAL_RECORDING.is_file(), "lateral recording not committed"
    digest = hashlib.sha256(LATERAL_RECORDING.read_bytes()).hexdigest()
    outs = [tmp_path / f"write{i}" for i in range(2)]
    for out in outs:  # deterministic: both writes are compared
        out.mkdir()
        tool.write_limits(
            out,
            tool.lateral_bins(LATERAL_RECORDING, SOURCE),
            source=SOURCE,
            recording_sha256=digest,
            closed_loop=tool.closed_loop_from_sweep(
                SWEEP_RECORDING,
                SOURCE,
                hashlib.sha256(SWEEP_RECORDING.read_bytes()).hexdigest(),
            ),
        )
    for name in ("limits_sliding.yaml", "limits_peak.yaml"):
        committed = VEHICLE_DIR / name
        assert committed.is_file(), f"{name} not committed"
        for out in outs:
            assert (out / name).read_text() == committed.read_text(), (
                out.name,
                name,
            )
        assert digest in committed.read_text(), name


def test_check_2670_sweep_half_closed_loop_fields_rederive(tmp_path) -> None:
    """CHECK-2670, sweep half: with the committed corner sweep the two files
    carry the closed-loop a_y (reached / closed-loop max, per radius bin and
    overall), exactly what `analyse --sweep` computes from the two recordings,
    and no `pending:` note survives. Only the ladder's kept rungs count; the
    collision rung is in the file, labelled, and moves no value."""
    import hashlib

    import yaml

    tool = _tool()
    assert SWEEP_RECORDING.is_file(), "corner sweep not committed"
    lateral = hashlib.sha256(LATERAL_RECORDING.read_bytes()).hexdigest()
    sweep = hashlib.sha256(SWEEP_RECORDING.read_bytes()).hexdigest()
    closed = tool.closed_loop_from_sweep(SWEEP_RECORDING, SOURCE, sweep)
    tool.write_limits(
        tmp_path,
        tool.lateral_bins(LATERAL_RECORDING, SOURCE),
        source=SOURCE,
        recording_sha256=lateral,
        closed_loop=closed,
    )
    for name in ("limits_sliding.yaml", "limits_peak.yaml"):
        committed = VEHICLE_DIR / name
        assert (tmp_path / name).read_text() == committed.read_text(), name
        assert sweep in committed.read_text(), name
        assert "pending" not in committed.read_text(), name
    sliding = yaml.safe_load((VEHICLE_DIR / "limits_sliding.yaml").read_text())
    assert sliding["lateral"]["ay_reached"]["value"] > 0.0
    assert sliding["lateral"]["ay_reached"]["n"] == len(
        [
            w
            for _, w in tool.closed_loop_windows(SWEEP_RECORDING, SOURCE)
            if _.get("status") == "kept" and w.label == "saturated"
        ]
    )
    peak = yaml.safe_load((VEHICLE_DIR / "limits_peak.yaml").read_text())
    assert peak["lateral"]["ay_closed_loop_max"]["n"] >= 1
    kept = tool.sweep_rungs(SWEEP_RECORDING)
    assert [r["status"] for r in kept] == ["kept"] * 3 + ["collision"]


def test_check_2690_writer_closed_loop_fills_both_files(tmp_path) -> None:
    """CHECK-2690, closed-loop half: `closed_loop` fills the overall field
    and the per-bin one, in the file that owns it, labelled measured and
    closed-loop."""
    import yaml

    tool = _tool()
    closed = {
        "ay_reached": 4.5,
        "ay_closed_loop_max": 6.4,
        "bins": {2.05: {"reached": (4.4, 3), "max": (6.3, 2)}},
        "recording_sha256": "abc",
    }
    bins = tool.lateral_bins(LATERAL_RECORDING, SOURCE)
    tool.write_limits(
        tmp_path,
        bins,
        source=SOURCE,
        recording_sha256="def",
        closed_loop=closed,
    )
    sliding = yaml.safe_load((tmp_path / "limits_sliding.yaml").read_text())
    peak = yaml.safe_load((tmp_path / "limits_peak.yaml").read_text())
    top = sliding["lateral"]["ay_reached"]
    assert top["value"] == 4.5 and top["source"] == "closed-loop"
    assert top["recording_sha256"] == "abc"
    row = next(b for b in sliding["lateral"]["bins"] if b["radius"] == 2.05)
    assert row["ay_reached"]["value"] == 4.4 and row["ay_reached"]["n"] == 3
    assert peak["lateral"]["ay_closed_loop_max"]["value"] == 6.4
    prow = next(b for b in peak["lateral"]["bins"] if b["radius"] == 2.05)
    assert prow["ay_closed_loop_max"]["value"] == 6.3
    assert "ay_reached" not in prow


def test_check_2690_ggv_bin_rule_and_chosen_field(tmp_path) -> None:
    """CHECK-2690, ggv half: the peak table's bin takes the larger of the
    open-loop peak and the closed-loop max and names which; the sliding
    table keeps the open-loop post-cliff value (call #22) and names it, the
    closed-loop `ay_reached` stays beside it. Both `rule:` texts say so."""
    import yaml

    tool = _tool()
    bins = [
        b
        for b in tool.lateral_bins(LATERAL_RECORDING, SOURCE)
        if b.peak is not None and b.post_cliff is not None
    ]
    assert len(bins) >= 2
    big, small = 99.0, 0.5
    closed = {
        "ay_reached": 4.5,
        "ay_closed_loop_max": 6.4,
        "bins": {
            bins[0].radius: {"reached": (big, 3), "max": (big, 2)},
            bins[1].radius: {"reached": (small, 3), "max": (small, 2)},
        },
        "recording_sha256": "abc",
    }
    tool.write_limits(
        tmp_path,
        bins,
        source=SOURCE,
        recording_sha256="def",
        closed_loop=closed,
    )
    sliding = yaml.safe_load((tmp_path / "limits_sliding.yaml").read_text())
    peak = yaml.safe_load((tmp_path / "limits_peak.yaml").read_text())
    by_r = {b["radius"]: b for b in peak["lateral"]["bins"]}
    assert by_r[bins[0].radius]["chosen"] == "closed_loop_max"
    assert by_r[bins[1].radius]["chosen"] == "open_loop_peak"
    for b in sliding["lateral"]["bins"]:
        if "ay_post_cliff" in b:
            assert b["chosen"] == "open_loop_post_cliff"
    s0 = next(
        b for b in sliding["lateral"]["bins"] if b["radius"] == bins[0].radius
    )
    assert s0["ay_reached"]["value"] == big
    assert "closed" in peak["ggv"]["rule"] and "max" in peak["ggv"]["rule"]
    assert "post-cliff" in sliding["ggv"]["rule"]
    assert "open-loop" in sliding["ggv"]["rule"]
