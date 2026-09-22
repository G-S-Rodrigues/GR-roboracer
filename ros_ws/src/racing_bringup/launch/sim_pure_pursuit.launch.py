"""Composes sim adapter + support + controller + supervisor + metrics +
evaluation + recording + robot_state_publisher (+ RViz) into the vertical
slice.

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
from racing_bringup.estimators import amcl, slam_toolbox
from racing_bringup.scenario_parameters import track_parameters

VEHICLE_CONFIG = "config/vehicles/f1tenth_default.yaml"
SCENARIO = "config/scenarios/contract_test.yaml"
REC_DEFAULT = "log/recording.jsonl"
# The pose the controller drives on, and so the estimate racing_evaluation
# scores by default: ground truth, the reference stack's pose source
# (config/reference_stack.yaml).
GROUND_TRUTH_POSE = "/ground_truth/odom"
# What an estimator's pose reaches the rest of the stack as: the active
# map -> odom composed with /odom's dead reckoning at control rate, by the
# support node (BRINGUP-1040). An estimator's own pose topic updates once
# per graph node, far slower than the controller's period.
ESTIMATED_POSE = "/localization/odom"
# pose_source -> the configuration of the node that owns map -> odom.
# slam_toolbox maps online, with no prior map (plan D7): `map` is then the
# pose it starts at, which is the true start pose. nav2_amcl instead
# localizes against the track's committed occupancy grid, which the scenario
# names (scenario_parameters.track_parameters).
POSE_SOURCE_CONFIGS = {
    "slam_toolbox": "config/localization/slam_toolbox_online.yaml",
    "amcl": "config/localization/amcl.yaml",
}
# Where nav2_amcl's initial pose comes from. The sim places the car at an
# arc length drawn from the run's seed (f110_env.reset), so no static pose
# can be configured; the support node publishes the first ground-truth pose
# here, once and latched, with the covariance of a human placing the car
# roughly - the simulation's stand-in for RViz's 2D Pose Estimate
# (BRINGUP-2020). slam_toolbox needs none: it defines `map` as where it
# starts.
INITIAL_POSE_TOPIC = "/initialpose"


def _launch_nodes(context, robot_description: str, rviz_config: str):
    scenario = LaunchConfiguration("scenario")
    seed = LaunchConfiguration("seed")
    recording_path = LaunchConfiguration("recording_path")
    use_rviz = LaunchConfiguration("use_rviz")
    time_scale = LaunchConfiguration("time_scale")
    start_held = LaunchConfiguration("start_held")
    estimate_topic = LaunchConfiguration("estimate_topic")
    # Which node owns map -> odom, and so which pose the controller drives
    # on. The safety supervisor stays on ground truth in simulation
    # whatever this is: see the comment in its subscription.
    pose_source = LaunchConfiguration("pose_source").perform(context)
    if pose_source not in ("ground_truth", *POSE_SOURCE_CONFIGS):
        raise RuntimeError(f"unknown pose_source: {pose_source}")
    estimating = pose_source != "ground_truth"
    # Whether the controller drives on that estimate or keeps ground truth
    # while the estimator is merely scored. Open loop is how an estimator
    # too rough to race on is still measured (SIM-3070); it changes nothing
    # about who owns map -> odom.
    driving_on_estimate = estimating and LaunchConfiguration(
        "drive_on_estimate"
    ).perform(context) in ("true", "True", "1")
    # The estimate scored and recorded is the pose the controller drives
    # on, unless the caller named another topic.
    if estimating and estimate_topic.perform(context) == GROUND_TRUTH_POSE:
        estimate_topic = ESTIMATED_POSE

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
        # The vehicle file carries the laser mount /scan is cast from
        # (BRINGUP-1050).
        parameters=[
            VEHICLE_CONFIG,
            {
                "scenario_path": scenario,
                "seed": seed,
                "time_scale": time_scale,
                "start_held": start_held,
            },
        ],
    )
    support_node = Node(
        package="racing_bringup",
        executable="racing_bringup_support",
        parameters=[
            VEHICLE_CONFIG,
            tracks["racing_bringup_support"],
            {
                "use_sim_time": True,
                # An active estimator owns map -> odom; the support node
                # then composes it with /odom into the control-rate pose.
                "publish_map_to_odom": not estimating,
                "composed_odometry_topic": ESTIMATED_POSE if estimating else "",
                # Only AMCL needs to be told where it starts.
                "initial_pose_topic": (
                    INITIAL_POSE_TOPIC if pose_source == "amcl" else ""
                ),
            },
        ],
    )
    controller_node = Node(
        package="racing_controller_baseline",
        executable="racing_controller_baseline_node",
        parameters=[VEHICLE_CONFIG, {"use_sim_time": True}],
        # The pose the controller drives on is selected here rather than in
        # its C++, which keeps the standard /odom: this remap is the seam
        # pose_source moves. Its default stays ground truth - driving on
        # /odom's drifting dead reckoning would move both goldens.
        remappings=[("/odom", GROUND_TRUTH_POSE)]
        if not driving_on_estimate
        else [("/odom", ESTIMATED_POSE)],
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
    # Scores estimate_topic against ground truth, live. With the default
    # (ground truth scored against itself) every error is zero: SIM-3060,
    # the check that the ruler itself is straight.
    evaluation_node = Node(
        package="racing_evaluation",
        executable="racing_evaluation_node",
        parameters=[
            tracks["racing_evaluation"],
            {"estimate_topic": estimate_topic, "use_sim_time": True},
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
                "estimate_topic": estimate_topic,
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
        evaluation_node,
        recording_node,
        robot_state_publisher_node,
        rviz_node,
        *_pose_source_nodes(pose_source, tracks),
    ]


def _pose_source_nodes(pose_source: str, tracks: dict) -> list:
    """The nodes `pose_source` adds; none for ground truth, which the
    support node already serves."""
    if pose_source == "ground_truth":
        return []
    configuration = POSE_SOURCE_CONFIGS[pose_source]
    if pose_source == "amcl":
        # The grid comes from the scenario, like every other track-aware
        # parameter (BRINGUP-1060): a hand-set path is a second copy of
        # which world this is, and localizing against another track's walls
        # is silent, not an error.
        return amcl([configuration], tracks["map_server"]["yaml_filename"])
    return slam_toolbox("mapping", [configuration])


def generate_launch_description() -> LaunchDescription:
    description_share = Path(
        get_package_share_directory("racing_vehicle_description")
    )
    robot_description = (description_share / "urdf" / "f1tenth.urdf").read_text(
        encoding="utf-8"
    )
    rviz_config = str(description_share / "rviz" / "sim_pure_pursuit.rviz")

    # Launch-time controls: which scenario runs, its seed, where the replay
    # log lands, whether RViz starts alongside the graph, how fast
    # simulated time advances relative to wall clock (ADR 0006), and which
    # pose racing_evaluation scores.
    scenario_arg = DeclareLaunchArgument("scenario", default_value=SCENARIO)
    seed_arg = DeclareLaunchArgument("seed", default_value="0")
    rec_arg = DeclareLaunchArgument("recording_path", default_value=REC_DEFAULT)
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")
    time_scale_arg = DeclareLaunchArgument("time_scale", default_value="1.0")
    start_held_arg = DeclareLaunchArgument("start_held", default_value="false")
    estimate_topic_arg = DeclareLaunchArgument(
        "estimate_topic", default_value=GROUND_TRUTH_POSE
    )
    pose_source_arg = DeclareLaunchArgument(
        "pose_source", default_value="ground_truth"
    )
    drive_on_estimate_arg = DeclareLaunchArgument(
        "drive_on_estimate", default_value="true"
    )

    return LaunchDescription(
        [
            scenario_arg,
            seed_arg,
            rec_arg,
            use_rviz_arg,
            time_scale_arg,
            start_held_arg,
            estimate_topic_arg,
            pose_source_arg,
            drive_on_estimate_arg,
            OpaqueFunction(
                function=_launch_nodes,
                args=[robot_description, rviz_config],
            ),
        ]
    )
