"""The viewer slam.launch.py starts beside the graph."""

from launch_ros.actions import Node

# "foxglove" is added here later, beside these.
VIEWERS = ("rviz", "none")


def nodes(kind: str, rviz_config: str, use_sim_time: bool) -> list:
    """The nodes `visualization:=<kind>` adds; none for `none`.

    Every member of VIEWERS has a branch here: one added to the choices
    without one is refused rather than silently starting nothing.
    """
    if kind == "none":
        return []
    if kind == "rviz":
        return [
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    raise RuntimeError(
        f"visualization:={kind} is not one of {', '.join(VIEWERS)}"
    )
