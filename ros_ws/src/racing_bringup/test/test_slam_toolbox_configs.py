"""The slam_toolbox parameter files: one per lidar density (part 8)."""

from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
LOCALIZATION = REPOSITORY_ROOT / "config" / "localization"
FILE_64 = LOCALIZATION / "slam_toolbox_online.yaml"
FILE_1080 = LOCALIZATION / "slam_toolbox_online_1080.yaml"
FILE_LOCALIZATION_1080 = LOCALIZATION / "slam_toolbox_localization_1080.yaml"

# The 64-beam file's keys on origin/main, copied here so the test reads no git.
KEYS_64 = {
    "angle_variance_penalty",
    "base_frame",
    "coarse_angle_resolution",
    "coarse_search_angle_offset",
    "correlation_search_space_dimension",
    "distance_variance_penalty",
    "do_loop_closing",
    "map_frame",
    "map_update_interval",
    "max_laser_range",
    "minimum_angle_penalty",
    "minimum_distance_penalty",
    "minimum_time_interval",
    "minimum_travel_distance",
    "minimum_travel_heading",
    "mode",
    "odom_frame",
    "resolution",
    "scan_buffer_maximum_scan_distance",
    "scan_buffer_size",
    "scan_topic",
    "stack_size_to_use",
    "transform_publish_period",
    "use_sim_time",
}
# The keys the 1080 file changes; each carries its measurement in a comment
# on its own line in the file (the pattern is BRINGUP-1560).
CHANGED_KEYS_1080 = {"coarse_angle_resolution"}
FRAME_KEYS = (
    "odom_frame",
    "map_frame",
    "base_frame",
    "scan_topic",
    "use_sim_time",
    "mode",
)
# The keys the localization file shares with the online 1080 file it loads a
# graph from: the scan it reads, the frames it names, the grid resolution and
# the matcher's 1080-beam angle step.
SHARED_KEYS_LOCALIZATION_1080 = (
    "scan_topic",
    "max_laser_range",
    "odom_frame",
    "map_frame",
    "base_frame",
    "resolution",
    "coarse_angle_resolution",
)


def _parameters(path: Path) -> dict:
    return yaml.safe_load(path.read_text())["slam_toolbox"]["ros__parameters"]


def test_bringup_1820_the_1080_file_differs_by_its_measured_keys() -> None:
    """BRINGUP-1820: the 1080-beam file is the 64-beam file plus the keys the
    part 8 sweep chose, each with the measurement that chose it beside it.

    The 64 file keeps every key it has on origin/main: it is the baseline the
    next comparison needs (ADR 0005). The frames, the scan topic, the clock
    and the mode are equal, so the two files cannot select different graphs.
    """
    small = _parameters(FILE_64)
    large = _parameters(FILE_1080)

    assert set(small) >= KEYS_64, KEYS_64 - set(small)

    differing = {
        key
        for key in set(small) | set(large)
        if small.get(key, object()) != large.get(key, object())
    }
    assert differing == CHANGED_KEYS_1080

    lines = FILE_1080.read_text().splitlines()
    for key in CHANGED_KEYS_1080:
        own = [line for line in lines if line.strip().startswith(f"{key}:")]
        assert own, key
        for line in own:
            assert "#" in line.split(f"{key}:", 1)[1], line

    for key in FRAME_KEYS:
        assert small[key] == large[key], key


def test_bringup_1997_the_1080_localization_file_matches_online() -> None:
    """BRINGUP-1997: the 1080-beam localization file runs slam_toolbox in
    localization mode, starting at the graph's first node, on the online 1080
    file's scan, frames and resolution.

    Shared keys: scan_topic, max_laser_range, odom_frame, map_frame,
    base_frame, resolution and coarse_angle_resolution. It names no
    map_file_name: slam.launch.py passes the graph as `map:=`.
    """
    assert FILE_LOCALIZATION_1080.is_file(), FILE_LOCALIZATION_1080
    online = _parameters(FILE_1080)
    localization = _parameters(FILE_LOCALIZATION_1080)

    assert localization["mode"] == "localization"
    assert localization["map_start_at_dock"] is True
    assert "map_file_name" not in localization

    for key in SHARED_KEYS_LOCALIZATION_1080:
        assert localization[key] == online[key], key
