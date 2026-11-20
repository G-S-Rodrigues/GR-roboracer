"""slam.launch.py: a hand-driven map of the track with one SLAM method."""

from pathlib import Path

import pytest
from racing_bringup import slam

REPOSITORY_ROOT = Path(__file__).parents[1].parents[2]
SERIALIZE_COMMAND = (
    "ros2 service call /slam_toolbox/serialize_map "
    "slam_toolbox/srv/SerializePoseGraph"
)


def test_bringup_1991_far_lio_is_refused_as_not_integrated() -> None:
    """BRINGUP-1991: `method:=far_lio` is refused in either mode with the
    reason, and the note it points to exists."""
    for mode in slam.MODES:
        with pytest.raises(RuntimeError) as raised:
            slam.check("far_lio", mode, "")
        assert str(raised.value) == (
            "method:=far_lio is not integrated yet (no launch, no map save; "
            "see docs/agents_knowledge/localization-slam.md)"
        )
    assert (
        REPOSITORY_ROOT / "docs" / "agents_knowledge" / "localization-slam.md"
    ).is_file()


def test_bringup_1991_unknown_method_or_mode_is_refused() -> None:
    """BRINGUP-1991: an unknown method or mode is refused, naming the
    argument and its choices."""
    with pytest.raises(RuntimeError) as raised:
        slam.check("cartographer", "mapping", "")
    message = str(raised.value)
    assert "method:=cartographer" in message
    assert all(method in message for method in slam.METHODS)

    with pytest.raises(RuntimeError) as raised:
        slam.check("slam_toolbox", "racing", "")
    message = str(raised.value)
    assert "mode:=racing" in message
    assert all(mode in message for mode in slam.MODES)


def test_bringup_1991_mapping_needs_no_map() -> None:
    """BRINGUP-1991: every integrated method maps with no `map:=`."""
    for method in ("slam_toolbox", "mola"):
        slam.check(method, "mapping", "")


def test_bringup_1993_slam_toolbox_localization_needs_a_map() -> None:
    """BRINGUP-1993: slam_toolbox localization with an empty `map:=` is
    refused, and the message names the serialize_map command and says
    `map:=` is its base path with no extension."""
    with pytest.raises(RuntimeError) as raised:
        slam.check("slam_toolbox", "localization", "")
    message = str(raised.value)
    assert "map:=" in message
    assert SERIALIZE_COMMAND in message
    assert "no extension" in message


def test_bringup_1993_slam_toolbox_map_must_name_both_graph_files(
    tmp_path: Path,
) -> None:
    """BRINGUP-1993: slam_toolbox's `map:=` is the base path serialize_map
    wrote `<base>.posegraph` and `<base>.data` under; a missing one is
    refused by name, both present is accepted."""
    base = tmp_path / "porto_manual"
    with pytest.raises(RuntimeError) as raised:
        slam.check("slam_toolbox", "localization", str(base))
    assert f"{base}.posegraph" in str(raised.value)

    Path(f"{base}.posegraph").write_bytes(b"")
    with pytest.raises(RuntimeError) as raised:
        slam.check("slam_toolbox", "localization", str(base))
    assert f"{base}.data" in str(raised.value)

    Path(f"{base}.data").write_bytes(b"")
    slam.check("slam_toolbox", "localization", str(base))


def test_bringup_1993_mola_localization_defaults_or_needs_the_file(
    tmp_path: Path,
) -> None:
    """BRINGUP-1993: MOLA localization with an empty `map:=` is accepted
    (the committed Porto `.mm` is the default); a `map:=` naming a missing
    `.mm` is refused by name, an existing one is accepted."""
    slam.check("mola", "localization", "")

    missing = tmp_path / "missing.mm"
    with pytest.raises(RuntimeError) as raised:
        slam.check("mola", "localization", str(missing))
    assert str(missing) in str(raised.value)

    present = tmp_path / "porto.mm"
    present.write_bytes(b"")
    slam.check("mola", "localization", str(present))
