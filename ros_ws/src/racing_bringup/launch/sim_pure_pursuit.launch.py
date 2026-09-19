"""Composes sim adapter + support + controller + supervisor + metrics +
recording + robot_state_publisher (+ RViz) into the vertical slice.

Run from the repository root, so the relative config paths this launch file
and the nodes it starts both use (``config/vehicles/...``, the launch
arguments below) resolve the same way ``scripts/check.sh`` resolves them.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racing_bringup.scenario_parameters import track_parameters

VEHICLE_CONFIG = "config/vehicles/f1tenth_default.yaml"
SCENARIO = "config/scenarios/contract_test.yaml"
REC_DEFAULT = "log/recording.jsonl"


def _launch_nodes(context, robot_description: str, rviz_config: str):
    scenario = LaunchConfiguration("scenario")
    seed = LaunchConfiguration("seed")
    recording_path = LaunchConfiguration("recording_path")
    use_rviz = LaunchConfiguration("use_rviz")
    time_scale = LaunchConfiguration("time_scale")
    start_held = LaunchConfiguration("start_held")

    # The scenario names the world; every track-aware node loads that same
    # world (BRINGUP-1010), over the vehicle file's defaults.
    tracks = track_parameters(Path(scenario.perform(context)))

    # The simulator owns the clock (ADR 0006): it publishes /clock from its
    # own simulated-time counter and stays off use_sim_time so its wall
    # timer is not driven by the clock it is itself responsible for
    # advancing. Every other node reads time through /clock instead.
    sim_node = Node(
        package="racing_sim_gym_jax",
        executable="racing_sim_gym_jax_node",
        parameters=[
            {
                "scenario_path": scenario,
                "seed": seed,
                "time_scale": time_scale,
                "start_held": start_held,
            }
        ],
    )
    support_node = Node(
        package="racing_bringup",
        executable="racing_bringup_support",
        parameters=[
            VEHICLE_CONFIG,
            tracks["racing_bringup_support"],
            {"use_sim_time": True},
        ],
    )
    controller_node = Node(
        package="racing_controller_baseline",
        executable="racing_controller_baseline_node",
        parameters=[VEHICLE_CONFIG, {"use_sim_time": True}],
    )
    supervisor_node = Node(
        package="racing_safety_supervisor",
        executable="racing_safety_supervisor_node",
        parameters=[
            VEHICLE_CONFIG,
            tracks["racing_safety_supervisor"],
            {"use_sim_time": True},
        ],
    )
    metrics_node = Node(
        package="racing_metrics",
        executable="racing_metrics_node",
        parameters=[
            VEHICLE_CONFIG,
            tracks["racing_metrics"],
            {"seed": seed, "use_sim_time": True},
        ],
    )
    recording_node = Node(
        package="racing_recording",
        executable="racing_recording_node",
        parameters=[
            VEHICLE_CONFIG,
            tracks["racing_recording"],
            {
                "seed": seed,
                "output_path": recording_path,
                "use_sim_time": True,
            },
        ],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description, "use_sim_time": True}
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )
    return [
        sim_node,
        support_node,
        controller_node,
        supervisor_node,
        metrics_node,
        recording_node,
        robot_state_publisher_node,
        rviz_node,
    ]


def generate_launch_description() -> LaunchDescription:
    description_share = Path(
        get_package_share_directory("racing_vehicle_description")
    )
    robot_description = (description_share / "urdf" / "f1tenth.urdf").read_text(
        encoding="utf-8"
    )
    rviz_config = str(description_share / "rviz" / "sim_pure_pursuit.rviz")

    # Launch-time controls: which scenario runs, its seed, where the replay
    # log lands, whether RViz starts alongside the graph, and how fast
    # simulated time advances relative to wall clock (ADR 0006).
    scenario_arg = DeclareLaunchArgument("scenario", default_value=SCENARIO)
    seed_arg = DeclareLaunchArgument("seed", default_value="0")
    rec_arg = DeclareLaunchArgument("recording_path", default_value=REC_DEFAULT)
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    time_scale_arg = DeclareLaunchArgument("time_scale", default_value="1.0")
    start_held_arg = DeclareLaunchArgument("start_held", default_value="false")

    return LaunchDescription(
        [
            scenario_arg,
            seed_arg,
            rec_arg,
            use_rviz_arg,
            time_scale_arg,
            start_held_arg,
            OpaqueFunction(
                function=_launch_nodes,
                args=[robot_description, rviz_config],
            ),
        ]
    )
