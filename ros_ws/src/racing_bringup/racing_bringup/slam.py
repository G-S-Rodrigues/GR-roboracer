"""The argument checks of slam.launch.py: which SLAM method runs, in which
mode, on which map."""

from pathlib import Path

METHODS = ("slam_toolbox", "mola", "far_lio")
MODES = ("mapping", "localization")

# What a slam_toolbox `map:=` comes from: serialize_map writes
# `<filename>.posegraph` and `<filename>.data`, and the localization node
# reads the pair back from the same base path.
SERIALIZE_COMMAND = (
    "ros2 service call /slam_toolbox/serialize_map "
    "slam_toolbox/srv/SerializePoseGraph \"{filename: '<base path>'}\""
)
POSE_GRAPH_SUFFIXES = (".posegraph", ".data")


def check(method: str, mode: str, map_path: str) -> None:
    """Refuse a method, mode or map the launch cannot run.

    Mapping takes no map. In localization, slam_toolbox needs `map:=`, the
    base path of a serialized pose graph; MOLA takes a `.mm` and falls back
    to the committed Porto one when `map:=` is empty.
    """
    if method not in METHODS:
        raise RuntimeError(
            f"method:={method} is not one of {', '.join(METHODS)}"
        )
    if mode not in MODES:
        raise RuntimeError(f"mode:={mode} is not one of {', '.join(MODES)}")
    if method == "far_lio":
        raise RuntimeError(
            "method:=far_lio is not integrated yet (no launch, no map save; "
            "see docs/agents_knowledge/localization-slam.md)"
        )
    if mode != "localization":
        return
    if method == "slam_toolbox":
        if not map_path:
            raise RuntimeError(
                "method:=slam_toolbox mode:=localization needs map:=, the "
                "base path (no extension) of a pose graph serialized by a "
                f"mapping run: {SERIALIZE_COMMAND}"
            )
        for suffix in POSE_GRAPH_SUFFIXES:
            if not Path(f"{map_path}{suffix}").is_file():
                raise RuntimeError(
                    f"map:={map_path} names no serialized pose graph: "
                    f"{map_path}{suffix} does not exist"
                )
    elif map_path and not Path(map_path).is_file():
        raise RuntimeError(f"map:={map_path} does not exist")
