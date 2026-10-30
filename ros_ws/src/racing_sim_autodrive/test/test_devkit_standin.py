"""Tier 1: the devkit stand-in's parse mirrors the devkit (SIMAD-1410).

`devkit_values` is a deliberate copy of the official bridge's `Bridge`
handler (`autodrive_bridge.py:309-375`, a third-party Humble file this repo
cannot import). It must turn every fixture frame into the values the devkit
would publish, and front end B then turns those back into the same
`SimFrame` the Socket.IO path decodes. SIM-3720 holds the copy to the pinned
image; this file holds it to the fixture.
"""

import math
from pathlib import Path

import numpy as np
import pytest
from racing_sim_autodrive import core, devkit_standin, replay

FIXTURE = Path(__file__).parent / "fixtures" / "porto_stream.jsonl.gz"


@pytest.fixture(scope="module")
def raws():
    return [raw for _, raw in replay.load_frames(FIXTURE)]


def _close(a, b, tol=1e-4):
    if isinstance(a, float | int) and not isinstance(a, bool):
        if math.isinf(a) or math.isinf(b):
            return a == b
        return abs(a - b) <= tol
    return all(_close(x, y, tol) for x, y in zip(a, b, strict=True))


def test_simad_1410_every_frame_matches_the_socket_decode(raws):
    """SIMAD-1410: per fixture frame 1..616, the stand-in's values ->
    `frame_from_devkit` equal `decode_frame` within 1e-4 (the telemetry's
    printed 4 dp; float32 ranges are about 1e-6 at 10 m), and the devkit path
    has no ticks."""
    assert len(raws) == 617
    for k in range(1, len(raws)):
        values = devkit_standin.devkit_values(raws[k])
        assert values is not None, k
        got = core.frame_from_devkit(devkit_standin.devkit_sample(values))
        want = core.decode_frame(raws[k])
        assert got.encoder_ticks is None and want.encoder_ticks is not None
        for field in got.__dataclass_fields__:
            if field == "encoder_ticks":
                continue
            a, b = getattr(got, field), getattr(want, field)
            if field == "ranges":
                assert a.shape == b.shape == (1081,), k
                finite = np.isfinite(b)
                assert np.array_equal(np.isfinite(a), finite), k
                assert np.abs(a[finite] - b[finite]).max() <= 1e-4, k
            elif field == "lidar_scan_rate":
                assert abs(a - b) <= 1e-4 * b, k
            else:
                assert _close(a, b), (k, field, a, b)


def test_simad_1410_values_have_the_devkits_dtypes(raws):
    """SIMAD-1410: the devkit publishes Float32 for the two echoes and the
    three lap times and the scan's ranges, doubles for the rest, and an int
    for the two counts (`int(float())`); the stand-in's values already
    carry exactly those precisions."""
    values = devkit_standin.devkit_values(raws[300])
    assert values.ranges.dtype == np.float32
    for name in ("throttle", "steering", "lap_time", "last_lap_time"):
        v = getattr(values, name)
        assert float(np.float32(v)) == v, name
    assert isinstance(values.lap_count, int)
    assert isinstance(values.collision_count, int)
    assert not isinstance(values.lap_count, bool)


def test_simad_1410_a_lidar_less_frame_publishes_nothing(raws):
    """SIMAD-1410: frame 0 has no lidar; the devkit's KeyError means no
    publish and no reply (gotcha 29), so the stand-in returns None. The
    camera key is read unguarded too, and so is every other key."""
    assert "V1 LIDAR Range Array" not in raws[0]
    assert devkit_standin.devkit_values(raws[0]) is None
    for key in ("V1 Front Camera Image", "V1 Position", "V1 Lap Count"):
        broken = dict(raws[5])
        del broken[key]
        assert devkit_standin.devkit_values(broken) is None, key


def test_simad_1410_reply_strings_are_the_devkits():
    """SIMAD-1410: the devkit stores commands as `float(np.round(x, 3))` and
    replies `str()` of each, `str(bool)` for the reset."""
    assert devkit_standin.latch_command(0.30000001192092896) == 0.3
    assert devkit_standin.latch_command(-0.12345) == -0.123
    assert devkit_standin.devkit_reply(0.3, -0.123, True) == {
        "V1 Throttle": "0.3",
        "V1 Steering": "-0.123",
        "V1 Reset": "True",
    }
    assert devkit_standin.devkit_reply(0.0, 0.0, False) == {
        "V1 Throttle": "0.0",
        "V1 Steering": "0.0",
        "V1 Reset": "False",
    }
