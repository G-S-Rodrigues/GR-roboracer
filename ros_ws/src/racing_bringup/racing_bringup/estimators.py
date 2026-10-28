"""Launch actions for the pose estimators the bringup can start.

slam_toolbox (jazzy) is a lifecycle node: nothing subscribes, publishes or
broadcasts until it is configured and activated. Its own launch files drive
those transitions with launch events rather than a lifecycle manager, and so
does this module, so the node is active - and publishing - before the
scenario runner's deterministic reset (repo-gotchas #19).

nav2_amcl and nav2_map_server are lifecycle nodes too, but nav2 ships a
manager for exactly this and orders the transitions across both of them
(map_server before amcl, which reads the map on configure). That manager is
used rather than a second hand-rolled copy of the event chain above.
"""

from __future__ import annotations

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

SLAM_TOOLBOX_EXECUTABLES = {
    "mapping": "async_slam_toolbox_node",
    "localization": "localization_slam_toolbox_node",
}


def autostarted(node: LifecycleNode) -> list:
    """`node`, configured on launch and activated once it is inactive."""
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    return [node, activate, configure]


def slam_toolbox(mode: str, parameters: list) -> list:
    """slam_toolbox in `mode` ("mapping" | "localization"), autostarted."""
    node = LifecycleNode(
        package="slam_toolbox",
        executable=SLAM_TOOLBOX_EXECUTABLES[mode],
        name="slam_toolbox",
        namespace="",
        parameters=[*parameters, {"use_lifecycle_manager": False}],
        output="screen",
    )
    return autostarted(node)


def amcl(parameters: list, map_yaml: str) -> list:
    """nav2_amcl localizing against `map_yaml`, served by nav2_map_server.

    The grid is the track's committed survey
    (config/scenarios/maps/<map>/slam/), named by the caller from the
    scenario, so a run can only ever be localized against its own track.
    """
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[*parameters, {"yaml_filename": map_yaml}],
            output="screen",
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            parameters=parameters,
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="amcl_lifecycle_manager",
            parameters=parameters,
            output="screen",
        ),
    ]


def mola_map_files(grid_yaml: str) -> tuple[str, str]:
    """(.mm, provenance yaml) of the MOLA map that belongs to `grid_yaml`.

    `grid_yaml` is the scenario's occupancy grid,
    config/scenarios/maps/<map>/slam/<map>.yaml; MOLA's own map lives beside
    it in maps/<map>/mola/<map>.{mm,yaml}. Derived from the scenario, as
    amcl's grid is, so a run can only be localized on its own track. Raises
    FileNotFoundError, naming the script that builds it, when it is absent.
    """
    grid = Path(grid_yaml)
    mola_dir = grid.parent.parent / "mola"
    mm = mola_dir / f"{grid.stem}.mm"
    provenance = mola_dir / f"{grid.stem}.yaml"
    for path in (mm, provenance):
        if not path.is_file():
            raise FileNotFoundError(
                f"no MOLA map {path}; build it with scripts/mola_map_track.sh"
            )
    return str(mm), str(provenance)


def mola_launch_arguments(
    configuration: str, mm_file: str | None = None
) -> dict:
    """The launch arguments of MOLA-LO's ros2-lidar-odometry launch file.

    `configuration` is config/localization/mola_lidar2d.yaml (or the mapping
    copy); a relative `mola_lo_pipeline` in it is relative to that file.
    `mm_file` is the map MOLA localizes on; None, the mapping form, names no
    initial map. Pure: it reads committed files and starts nothing.
    """
    path = Path(configuration).resolve()
    arguments = {
        key: str(value)
        for key, value in yaml.safe_load(path.read_text("utf-8")).items()
    }
    arguments["mola_lo_pipeline"] = str(
        path.parent / arguments["mola_lo_pipeline"]
    )
    if mm_file is not None:
        arguments["mola_initial_map_mm_file"] = mm_file
    return arguments


def mola_alignment_arguments(provenance_yaml: str) -> list[str]:
    """`static_transform_publisher` arguments for `map -> mola_map`: the
    provenance's T, which relates MOLA's map frame (wherever its mapping lap
    started) to the world frame. Used to score against ground truth, and the
    only way `/initialpose` (frame `map`) reaches MOLA."""
    transform = yaml.safe_load(Path(provenance_yaml).read_text("utf-8"))[
        "T_map_mola"
    ]
    return [
        "--x",
        str(transform["x"]),
        "--y",
        str(transform["y"]),
        "--z",
        str(transform["z"]),
        "--yaw",
        str(transform["yaw"]),
        "--frame-id",
        "map",
        "--child-frame-id",
        "mola_map",
    ]


def mola(configuration: str, grid_yaml: str) -> list:
    """MOLA-LO localizing on the map it built itself for `grid_yaml`'s track.

    MOLA owns `mola_map -> odom`; the static `map -> mola_map` joins that to
    the world frame (BRINGUP-2010), so the support node's `map -> odom`
    lookup resolves through the chain unchanged. Reads committed files only
    and spawns nothing at description time.
    """
    mm_file, provenance = mola_map_files(grid_yaml)
    share = Path(get_package_share_directory("mola_lidar_odometry"))
    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            # The stock executable lives under /opt/ros, so the documented
            # `pkill -f install/racing_` never matched it; the name is on its
            # command line (gotcha #10).
            name="racing_mola_map_alignment",
            arguments=mola_alignment_arguments(provenance),
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "ros2-launchs" / "ros2-lidar-odometry.launch.py")
            ),
            launch_arguments=mola_launch_arguments(
                configuration, mm_file
            ).items(),
        ),
    ]


def mola_mapping(configuration: str, simplemap_output: str) -> list:
    """MOLA-LO mapping one lap, saving its simplemap to `simplemap_output`.

    `configuration` is config/localization/mola_lidar2d_mapping.yaml. MOLA
    defines the frame here, so there is no alignment publisher and no initial
    map. `MOLA_SIMPLEMAP_OUTPUT` is an environment variable the pipeline reads
    (pipelines/blocks/simplemap.yaml), not a launch argument. Every value that
    selects a time base (`use_sim_time`) comes from the yaml and is never set
    here. Reads committed files only and spawns nothing at description time.
    """
    share = Path(get_package_share_directory("mola_lidar_odometry"))
    return [
        SetEnvironmentVariable("MOLA_SIMPLEMAP_OUTPUT", simplemap_output),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(share / "ros2-launchs" / "ros2-lidar-odometry.launch.py")
            ),
            launch_arguments=mola_launch_arguments(configuration).items(),
        ),
    ]
