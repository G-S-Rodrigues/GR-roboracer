"""The slam_toolbox parameter files: one per lidar density (part 8)."""

from pathlib import Path

import yaml

PACKAGE_ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[2]
LOCALIZATION = REPOSITORY_ROOT / "config" / "localization"
FILE_64 = LOCALIZATION / "slam_toolbox_online.yaml"
FILE_1080 = LOCALIZATION / "slam_toolbox_online_1080.yaml"

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

    assert KEYS_64 <= set(small), KEYS_64 - set(small)

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
