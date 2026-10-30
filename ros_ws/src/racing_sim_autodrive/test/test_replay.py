"""Tier 1: the replay schedule (SIMAD-1250). Pure, no socket, no ROS."""

import gzip
import itertools
import json
from pathlib import Path

import pytest
from racing_sim_autodrive import replay

FIXTURE = Path(__file__).parent / "fixtures" / "porto_stream.jsonl.gz"


def _sim_records():
    with gzip.open(FIXTURE, "rt") as handle:
        rows = [json.loads(line) for line in handle]
    return [r for r in rows if r.get("dir") == "sim"]


@pytest.fixture(scope="module")
def records():
    return replay.load_frames(FIXTURE)


def test_simad_1250_fixture_loads_in_order_first_frame_lidarless(records):
    """SIMAD-1250: load_frames returns every recorded sim frame in order,
    the lidar-less first one included, as (time, raw dict) pairs."""
    sim = _sim_records()
    assert len(records) == len(sim) > 600
    assert [t for t, _ in records] == [r["t"] for r in sim]
    assert records[0][1] == sim[0]["data"]
    assert "V1 LIDAR Range Array" not in records[0][1]
    assert "V1 LIDAR Range Array" in records[1][1]


def test_simad_1250_lockstep_sends_in_order_with_no_delay(records):
    """SIMAD-1250: lock-step sends every frame once, in order, delay 0
    (the reply, not a timer, paces it)."""
    plan = list(replay.schedule(records, pace="lockstep"))
    assert [i for i, _, _ in plan] == list(range(len(records)))
    assert {d for _, d, _ in plan} == {0.0}
    assert plan[0][2] is records[0][1]


def test_simad_1250_recorded_reproduces_recorded_periods(records):
    """SIMAD-1250: recorded pacing waits the recorded gap before each
    frame after the first; the first goes at once."""
    plan = list(replay.schedule(records, pace="recorded"))
    assert plan[0][1] == 0.0
    for k in range(1, len(plan)):
        expected = records[k][0] - records[k - 1][0]
        assert plan[k][1] == pytest.approx(expected, abs=1e-9)


def test_simad_1250_start_frame_skips_and_keeps_indices(records):
    """SIMAD-1250: --start-frame N begins at frame N (delay 0 there) and
    keeps the fixture index, so a reply can be matched to its frame."""
    plan = list(replay.schedule(records, pace="recorded", start_frame=100))
    assert plan[0][0] == 100 and plan[0][1] == 0.0
    assert len(plan) == len(records) - 100
    assert plan[1][1] == pytest.approx(records[101][0] - records[100][0])


def test_simad_1250_loop_wraps_and_count_bounds(records):
    """SIMAD-1250: --loop restarts at the start frame after the last one,
    forever; --count bounds the total. Without --loop the plan ends."""
    n = len(records)
    looped = list(
        itertools.islice(
            replay.schedule(
                records, pace="lockstep", start_frame=n - 2, loop=True
            ),
            5,
        )
    )
    assert [i for i, _, _ in looped] == [n - 2, n - 1, n - 2, n - 1, n - 2]
    counted = list(replay.schedule(records, pace="lockstep", count=7))
    assert len(counted) == 7
    wrapped = list(
        replay.schedule(
            records, pace="lockstep", start_frame=n - 1, loop=True, count=3
        )
    )
    assert [i for i, _, _ in wrapped] == [n - 1, n - 1, n - 1]


def test_simad_1250_wrap_delay_is_a_recorded_period_not_a_jump(records):
    """SIMAD-1250: the delay across a wrap is the median recorded period,
    never the negative or huge difference of last minus first stamps."""
    plan = list(
        itertools.islice(
            replay.schedule(
                records,
                pace="recorded",
                start_frame=len(records) - 1,
                loop=True,
            ),
            3,
        )
    )
    assert plan[0][1] == 0.0
    assert 0.0 < plan[1][1] < 0.5


def test_simad_1250_bad_arguments_raise(records):
    """SIMAD-1250: unknown pace or an out-of-range start frame is an error,
    not an empty plan."""
    with pytest.raises(ValueError):
        list(replay.schedule(records, pace="fast"))
    with pytest.raises(ValueError):
        list(
            replay.schedule(records, pace="lockstep", start_frame=len(records))
        )


def test_simad_1310_end_frame_bounds_the_loop(records):
    """SIMAD-1310: `end_frame` (exclusive) ends the segment: a loop restarts
    at `start_frame` after `end_frame - 1`, recorded gaps hold inside it and
    the wrap waits a recorded period; without `loop` the plan stops at
    `end_frame - 1`; `end_frame=None` is today's behaviour; a bad end
    raises."""
    looped = list(
        itertools.islice(
            replay.schedule(
                records,
                pace="lockstep",
                start_frame=2,
                loop=True,
                end_frame=5,
            ),
            7,
        )
    )
    assert [i for i, _, _ in looped] == [2, 3, 4, 2, 3, 4, 2]
    recorded = list(
        itertools.islice(
            replay.schedule(
                records,
                pace="recorded",
                start_frame=2,
                loop=True,
                end_frame=5,
            ),
            5,
        )
    )
    assert recorded[0][1] == 0.0
    for k in (1, 2):
        expected = records[recorded[k][0]][0] - records[recorded[k][0] - 1][0]
        assert recorded[k][1] == pytest.approx(expected, abs=1e-9)
    assert 0.0 < recorded[3][1] < 0.5
    once = list(
        replay.schedule(records, pace="lockstep", start_frame=2, end_frame=5)
    )
    assert [i for i, _, _ in once] == [2, 3, 4]
    assert list(
        replay.schedule(records, pace="recorded", start_frame=100)
    ) == list(
        replay.schedule(
            records, pace="recorded", start_frame=100, end_frame=None
        )
    )
    for bad in (2, 0, len(records) + 1):
        with pytest.raises(ValueError):
            list(
                replay.schedule(
                    records, pace="lockstep", start_frame=2, end_frame=bad
                )
            )
