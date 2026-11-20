"""A map of the AutoDRIVE Porto track, driven by hand, with one SLAM method.

Starts the AutoDRIVE server, robot_state_publisher, one SLAM method
(slam_toolbox or MOLA, mapping or localizing) and a viewer. The controller,
the safety supervisor and the support node are deliberately absent: nothing
publishes /drive, and the car is driven from the AutoDRIVE window in Manual
mode. Without the support node the SLAM method is the only owner of
`map -> odom`.

Run from the repository root: the config paths here are relative to the
working directory, as in sim_pure_pursuit.launch.py. The simulator connects
once and never reconnects (repo-gotchas #30), so restart it for every launch.
Arguments: `ros2 launch racing_bringup slam.launch.py -s`.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racing_bringup import autodrive, estimators, slam, visualization
from racing_bringup.scenario_parameters import track_parameters
from racing_bringup.simulators import sim_time_for
from racing_bringup.vehicle_description import description_for

SCENARIO = "config/scenarios/porto.yaml"
VEHICLE = "config/vehicles/autodrive_roboracer.yaml"
# The AutoDRIVE lidar has 1081 beams whatever porto.yaml's num_beams says,
# so the 1080-beam files, never scan_beam_count.
SLAM_TOOLBOX_CONFIGS = {
    "mapping": "config/localization/slam_toolbox_online_1080.yaml",
    "localization": "config/localization/slam_toolbox_localization_1080.yaml",
}
# The files mola_mapping.launch.py and sim_pure_pursuit's pose_source:=mola
# hand MOLA-LO.
MOLA_MAPPING_CONFIG = "config/localization/mola_lidar2d_mapping.yaml"
MOLA_LOCALIZATION_CONFIG = "config/localization/mola_lidar2d.yaml"
SIMPLEMAP_OUTPUT = "log/maps/mola_manual.simplemap"


def _method_nodes(
    method: str,
    mode: str,
    map_path: str,
    tracks: dict,
    use_sim_time: bool,
    context,
) -> list:
    """The nodes of the SLAM method, already checked by slam.check."""
    if method == "slam_toolbox":
        config = (
            LaunchConfiguration("slam_toolbox_config").perform(context)
            or SLAM_TOOLBOX_CONFIGS[mode]
        )
        parameters = (
            [config, {"map_file_name": map_path}]
            if mode == "localization"
            else [config]
        )
        return estimators.slam_toolbox(mode, parameters, use_sim_time)
    if mode == "mapping":
        output = LaunchConfiguration("simplemap_output").perform(context)
        # MOLA does not create the folder, and fails to write without it.
        Path(output).parent.mkdir(parents=True, exist_ok=True)
        return estimators.mola_mapping(
            MOLA_MAPPING_CONFIG, output, use_sim_time
        )
    return estimators.mola(
        MOLA_LOCALIZATION_CONFIG,
        tracks["track_data"]["directory"],
        use_sim_time,
        mm_file=map_path or None,
    )


def _launch_described(context, description_share: Path) -> list:
    method = LaunchConfiguration("method").perform(context)
    mode = LaunchConfiguration("mode").perform(context)
    map_path = LaunchConfiguration("map").perform(context)
    vehicle = LaunchConfiguration("vehicle").perform(context)
    autodrive.check(vehicle, autodrive.AUTODRIVE_SCENARIO)
    slam.check(method, mode, map_path)
    if not Path(vehicle).is_file():
        raise RuntimeError(f"vehicle file not found: {vehicle}")

    use_sim_time = sim_time_for("autodrive")
    try:
        tracks = track_parameters(Path(SCENARIO), vehicle_path=Path(vehicle))
    except (ValueError, FileNotFoundError) as error:
        raise RuntimeError(str(error)) from error

    server_node = Node(
        package="racing_sim_autodrive",
        executable="racing_sim_autodrive_server",
        name="racing_sim",
        parameters=autodrive.server_parameters(
            tracks, LaunchConfiguration("port").perform(context)
        ),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": description_for(
                    vehicle, description_share
                ).read_text(encoding="utf-8"),
                "use_sim_time": use_sim_time,
            }
        ],
    )
    return [
        server_node,
        robot_state_publisher_node,
        *_method_nodes(method, mode, map_path, tracks, use_sim_time, context),
        *visualization.nodes(
            LaunchConfiguration("visualization").perform(context),
            str(description_share / "rviz" / "slam.rviz"),
            use_sim_time,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    description_share = Path(
        get_package_share_directory("racing_vehicle_description")
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "method",
                default_value="slam_toolbox",
                choices=list(slam.METHODS),
                description=(
                    "The SLAM method. The car is driven by hand from the "
                    "AutoDRIVE window (Manual mode); this launch starts no "
                    "controller. far_lio is listed but not integrated yet."
                ),
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="mapping",
                choices=list(slam.MODES),
                description="Build a map, or localize on the one map:= names.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description=(
                    "localization only. slam_toolbox: the base path (no "
                    "extension) of a pose graph serialized by a mapping run "
                    "(required). mola: a .mm (empty = the committed Porto "
                    "map); a <same stem>.yaml beside it gives map -> "
                    "mola_map, else the identity."
                ),
            ),
            DeclareLaunchArgument(
                "slam_toolbox_config",
                default_value="",
                description=(
                    "slam_toolbox's parameter file (empty = "
                    f"{SLAM_TOOLBOX_CONFIGS['mapping']} for mapping, "
                    f"{SLAM_TOOLBOX_CONFIGS['localization']} for "
                    "localization)."
                ),
            ),
            DeclareLaunchArgument(
                "simplemap_output",
                default_value=SIMPLEMAP_OUTPUT,
                description=(
                    "mola mapping only: where MOLA writes the .simplemap "
                    "on Ctrl+C (its folder is created)."
                ),
            ),
            DeclareLaunchArgument(
                "vehicle",
                default_value=VEHICLE,
                description=(
                    "An AutoDRIVE vehicle file: "
                    f"{' or '.join(autodrive.AUTODRIVE_VEHICLES)}."
                ),
            ),
            DeclareLaunchArgument(
                "port",
                default_value=autodrive.AUTODRIVE_PORT,
                description="The port the AutoDRIVE server listens on.",
            ),
            DeclareLaunchArgument(
                "visualization",
                default_value="rviz",
                choices=list(visualization.VIEWERS),
                description="The viewer: rviz on slam.rviz, or none.",
            ),
            OpaqueFunction(
                function=_launch_described, args=[description_share]
            ),
        ]
    )
