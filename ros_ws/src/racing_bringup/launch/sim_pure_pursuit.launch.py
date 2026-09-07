"""Composes sim adapter + support + controller + supervisor + metrics +
recording + robot_state_publisher (+ RViz) into the vertical slice.

Run from the repository root, so the relative config paths this launch file
and the nodes it starts both use (``config/vehicles/...``, the launch
arguments below) resolve the same way ``scripts/check.sh`` resolves them.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VEHICLE_CONFIG = "config/vehicles/f1tenth_default.yaml"
SCENARIO = "config/scenarios/contract_test.yaml"
REC_DEFAULT = "log/recording.jsonl"


def generate_launch_description() -> LaunchDescription:
    description_share = Path(
        get_package_share_directory("racing_vehicle_description")
    )
    robot_description = (description_share / "urdf" / "f1tenth.urdf").read_text(
        encoding="utf-8"
    )
    rviz_config = str(description_share / "rviz" / "sim_pure_pursuit.rviz")

    # Launch-time controls: which scenario runs, its seed, where the replay
    # log lands, and whether RViz starts alongside the graph.
    scenario_arg = DeclareLaunchArgument("scenario", default_value=SCENARIO)
    seed_arg = DeclareLaunchArgument("seed", default_value="0")
    rec_arg = DeclareLaunchArgument("recording_path", default_value=REC_DEFAULT)
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")

    scenario = LaunchConfiguration("scenario")
    seed = LaunchConfiguration("seed")
    recording_path = LaunchConfiguration("recording_path")
    use_rviz = LaunchConfiguration("use_rviz")

    sim_node = Node(
        package="racing_sim_gym_jax",
        executable="racing_sim_gym_jax_node",
        parameters=[{"scenario_path": scenario, "seed": seed}],
    )
    support_node = Node(
        package="racing_bringup",
        executable="racing_bringup_support",
        parameters=[VEHICLE_CONFIG],
    )
    controller_node = Node(
        package="racing_controller_baseline",
        executable="racing_controller_baseline_node",
        parameters=[VEHICLE_CONFIG],
    )
    supervisor_node = Node(
        package="racing_safety_supervisor",
        executable="racing_safety_supervisor_node",
        parameters=[VEHICLE_CONFIG],
    )
    metrics_node = Node(
        package="racing_metrics",
        executable="racing_metrics_node",
        parameters=[VEHICLE_CONFIG, {"seed": seed}],
    )
    recording_node = Node(
        package="racing_recording",
        executable="racing_recording_node",
        parameters=[
            VEHICLE_CONFIG,
            {"seed": seed, "output_path": recording_path},
        ],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            scenario_arg,
            seed_arg,
            rec_arg,
            use_rviz_arg,
            sim_node,
            support_node,
            controller_node,
            supervisor_node,
            metrics_node,
            recording_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
