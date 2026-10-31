"""Composes sim adapter + support + controller + supervisor + metrics +
evaluation + recording + robot_state_publisher (+ RViz) into the vertical
slice.

Run from the repository root, so the relative config paths this launch file
and the nodes it starts both use (``config/vehicles/...``, the launch
arguments below) resolve the same way ``scripts/check.sh`` resolves them.
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from racing_bringup.estimators import amcl, mola, slam_toolbox
from racing_bringup.scenario_parameters import (
    AUTODRIVE_BRIDGES,
    GYM_INCOMPATIBLE_VEHICLES,
    PLANNERS,
    RACELINE_OPTIMIZERS,
    SIMULATORS,
    track_parameters,
)
from racing_bringup.vehicle_description import description_for
from racing_sim_autodrive.capabilities import CAPABILITIES as AUTODRIVE_CAPS
from racing_sim_gym_jax.capabilities import CAPABILITIES as GYM_CAPS

LOGGER = get_logger("sim_pure_pursuit.launch")
VEHICLE_CONFIG = "config/vehicles/f1tenth_default.yaml"
SCENARIO = "config/scenarios/contract_test.yaml"
# The simulator capability tables, by `simulator:=` value; `capability.clock`
# decides whether the rest of the graph reads /clock (ADR 0006).
SIMULATOR_CAPABILITIES = {
    "f1tenth_gym_jax": GYM_CAPS,
    "autodrive": AUTODRIVE_CAPS,
}
# What `simulator:=autodrive` runs: the only vehicle and the only track the
# AutoDRIVE build has active, the server's values and the world_to_map it
# publishes under (config/autodrive/porto_frame.yaml: derived, not measured,
# until part 10's lap measures it; plan D3). Paths are repository-relative,
# like the rest of this file.
AUTODRIVE_VEHICLE = "autodrive_roboracer"
AUTODRIVE_SCENARIO = "porto"
AUTODRIVE_SERVER_CONFIG = "config/vehicles/autodrive_roboracer/server.yaml"
AUTODRIVE_FRAME = "config/autodrive/porto_frame.yaml"
# config/autodrive/simulator.yaml bridge_port (BRINGUP-1600 keeps them equal).
AUTODRIVE_PORT = "4567"
# How the controller's command speed is chosen: its own curvature schedule, or
# the speed each /trajectory point carries (a raceline's).
SPEED_SOURCES = ("curvature_schedule", "trajectory")
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
# names (scenario_parameters.track_parameters). MOLA-LO localizes on the .mm
# it built itself in a mapping lap (scripts/mola_map_track.sh), found beside
# that grid in maps/<map>/mola/.
POSE_SOURCE_CONFIGS = {
    "slam_toolbox": "config/localization/slam_toolbox_online.yaml",
    "amcl": "config/localization/amcl.yaml",
    "mola": "config/localization/mola_lidar2d.yaml",
}
# Where nav2_amcl's and MOLA's initial pose comes from. The sim places the
# car at an arc length drawn from the run's seed (f110_env.reset), so no
# static pose can be configured; the support node publishes the first
# ground-truth pose
# here, once and latched, with the covariance of a human placing the car
# roughly - the simulation's stand-in for RViz's 2D Pose Estimate
# (BRINGUP-2020). MOLA's bridge transforms it into `mola_map` through the
# static map -> mola_map edge. slam_toolbox needs none: it defines `map` as
# where it starts.
INITIAL_POSE_TOPIC = "/initialpose"


def _world_to_map() -> dict:
    """The server's `world_to_map.*` parameters, from AUTODRIVE_FRAME."""
    document = yaml.safe_load(Path(AUTODRIVE_FRAME).read_text(encoding="utf-8"))
    return {
        f"world_to_map.{key}": float(value)
        for key, value in document["world_to_map"].items()
    }


def _devkit_domain(context) -> int:
    """`autodrive_devkit_domain` as an int 0-101 that differs from this
    launch's own ROS domain, or a RuntimeError naming the argument.

    The devkit publishes a global /tf (world -> roboracer_1 -> lidar); on the
    stack's domain it would join the stack's TF tree and break base_link ->
    lidar, so front end B bridges across two domains instead.
    """
    text = LaunchConfiguration("autodrive_devkit_domain").perform(context)
    try:
        domain = int(text)
    except ValueError:
        domain = None
    if domain is None or not 0 <= domain <= 101:
        raise RuntimeError(
            "autodrive_bridge:=official needs autodrive_devkit_domain, the "
            f"ROS domain the devkit runs on, as an integer 0-101 (got {text!r};"
            " it defaults to $AUTODRIVE_ROS_DOMAIN_ID)"
        )
    own = int(os.environ.get("ROS_DOMAIN_ID", "0") or "0")
    if domain == own:
        raise RuntimeError(
            f"autodrive_devkit_domain {domain} is this launch's own ROS "
            "domain: the devkit's global /tf (world -> roboracer_1 -> lidar)"
            " would join the stack's TF tree. Run the devkit on its own "
            "domain."
        )
    return domain


def _check_autodrive(context, vehicle: str, bridge: str) -> None:
    """Refuse what the AutoDRIVE backend cannot do, naming the argument.

    The capabilities it lacks (time scaling, a held start) are refused, not
    ignored: a run that quietly drops `time_scale:=2` measures something else.
    The build has one car and one active track, so no other is offered.
    """
    if bridge == "official":
        _devkit_domain(context)
    if Path(vehicle).stem != AUTODRIVE_VEHICLE:
        raise RuntimeError(
            f"simulator:=autodrive runs vehicle {AUTODRIVE_VEHICLE} only, "
            f"not {vehicle}"
        )
    scenario = LaunchConfiguration("scenario").perform(context)
    if Path(scenario).stem != AUTODRIVE_SCENARIO:
        raise RuntimeError(
            f"simulator:=autodrive runs scenario {AUTODRIVE_SCENARIO} only "
            f"(the build has one active track), not {scenario}"
        )
    if LaunchConfiguration("start_held").perform(context).lower() in (
        "true",
        "1",
    ):
        raise RuntimeError(
            "simulator:=autodrive has no start_held (capability.start_held "
            "is false): the sim starts when it connects"
        )
    if float(LaunchConfiguration("time_scale").perform(context)) != 1.0:
        raise RuntimeError(
            "simulator:=autodrive has no time_scale (capability.time_scale "
            "is false): the sim runs in real time"
        )
    LOGGER.info(
        "simulator:=autodrive: reset is initial_pose_only, so seed is "
        "accepted and has no effect on the start"
    )


def _launch_nodes(context, robot_description: str, rviz_config: str):
    # Which vehicle file every vehicle-taking node reads. Checked first, so
    # a wrong path is the error raised whatever the other arguments hold.
    vehicle = LaunchConfiguration("vehicle").perform(context)
    if not Path(vehicle).is_file():
        raise RuntimeError(f"vehicle file not found: {vehicle}")
    simulator = LaunchConfiguration("simulator").perform(context)
    if simulator not in SIMULATORS:
        raise RuntimeError(f"unknown simulator: {simulator}")
    autodrive_bridge = LaunchConfiguration("autodrive_bridge").perform(context)
    if autodrive_bridge not in AUTODRIVE_BRIDGES:
        raise RuntimeError(f"unknown autodrive_bridge: {autodrive_bridge}")
    use_autodrive = simulator == "autodrive"
    if not use_autodrive and Path(vehicle).stem in GYM_INCOMPATIBLE_VEHICLES:
        raise RuntimeError(
            f"vehicle {vehicle} is not a car the gym can cast a scan for; "
            "run it with simulator:=autodrive"
        )
    if use_autodrive:
        _check_autodrive(context, vehicle, autodrive_bridge)
    # Every node but the simulator reads /clock exactly when the simulator
    # owns it (ADR 0006; the capability, not the simulator's name).
    use_sim_time = (
        SIMULATOR_CAPABILITIES[simulator]["capability.clock"] == "owns_clock"
    )
    scenario = LaunchConfiguration("scenario")
    seed = LaunchConfiguration("seed")
    laps = LaunchConfiguration("laps")
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
    if pose_source == "mola" and driving_on_estimate:
        # Warned, not refused: a closed-loop experiment stays possible
        # (ADR 0005).
        LOGGER.warning(
            "pose_source:=mola with drive_on_estimate:=true: no closed-loop "
            "lap is backed for MOLA (worst open-loop error 3.1 m), and the "
            "controller may latch a stop before the estimate is held to the "
            "start pose (repo-gotchas #19). Pass drive_on_estimate:=false "
            "to score it open loop."
        )
    # The estimate scored and recorded is the pose the controller drives
    # on, unless the caller named another topic.
    if estimating and estimate_topic.perform(context) == GROUND_TRUTH_POSE:
        estimate_topic = ESTIMATED_POSE

    # The scenario names the world; every track-aware node loads that same
    # world (BRINGUP-1010), over the vehicle file's defaults.
    # planner / raceline_optimizer pick which line /trajectory carries; the
    # resolver fails closed, so a wrong value is an error, never a fallback to
    # the centerline.
    planner = LaunchConfiguration("planner").perform(context)
    raceline_optimizer = LaunchConfiguration("raceline_optimizer").perform(
        context
    )
    # Forwarded to the controller only when given (empty = unset), so the
    # precedence is: argument > vehicle file > the node's default.
    speed_source = LaunchConfiguration("speed_source").perform(context)
    if speed_source and speed_source not in SPEED_SOURCES:
        raise RuntimeError(f"unknown speed_source: {speed_source}")
    try:
        tracks = track_parameters(
            Path(scenario.perform(context)),
            planner=planner,
            raceline_optimizer=raceline_optimizer,
            vehicle_path=Path(vehicle),
        )
    except (ValueError, FileNotFoundError) as error:
        raise RuntimeError(str(error)) from error
    if (
        speed_source == "trajectory"
        and tracks["racing_bringup_support"]["raceline_path"] == ""
    ):
        # track_centerline on a map with no raceline publishes the
        # centerline at speed 0: the car would never move, with no error.
        raise RuntimeError(
            "speed_source:=trajectory on "
            f"{Path(scenario.perform(context)).stem}: the map has no "
            "raceline, so track_centerline publishes its centerline at "
            "speed 0 and the car would not move; use the default "
            "speed_source (curvature_schedule)"
        )

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
            vehicle,
            {
                "scenario_path": scenario,
                "seed": seed,
                "time_scale": time_scale,
                "start_held": start_held,
            },
        ],
    )
    # Front end A: our Socket.IO server the sim connects to. It listens before
    # the sim exists and never owns /clock (free-running, ADR 0006). The values
    # file is the server's, not the vehicle file (BRINGUP-1670).
    autodrive_node = None
    if use_autodrive and autodrive_bridge == "official":
        # Front end B: the official devkit bridge. The devkit runs its own
        # bridge process on its own ROS domain; this node reads it there. No
        # port: the sim connects to the devkit, not to us.
        autodrive_node = Node(
            package="racing_sim_autodrive",
            executable="racing_sim_autodrive_devkit",
            name="racing_sim",
            parameters=[
                AUTODRIVE_SERVER_CONFIG,
                {
                    **_world_to_map(),
                    "track_path": tracks["racing_safety_supervisor"][
                        "track_path"
                    ],
                    "scenario_id": AUTODRIVE_SCENARIO,
                    "devkit_domain_id": _devkit_domain(context),
                },
            ],
        )
    elif use_autodrive:
        autodrive_node = Node(
            package="racing_sim_autodrive",
            executable="racing_sim_autodrive_server",
            name="racing_sim",
            parameters=[
                AUTODRIVE_SERVER_CONFIG,
                {
                    **_world_to_map(),
                    "track_path": tracks["racing_safety_supervisor"][
                        "track_path"
                    ],
                    "scenario_id": AUTODRIVE_SCENARIO,
                    "port": int(
                        LaunchConfiguration("autodrive_port").perform(context)
                    ),
                },
            ],
        )
    support_node = Node(
        package="racing_bringup",
        executable="racing_bringup_support",
        parameters=[
            vehicle,
            tracks["racing_bringup_support"],
            {
                "use_sim_time": use_sim_time,
                # An active estimator owns map -> odom; the support node
                # then composes it with /odom into the control-rate pose.
                "publish_map_to_odom": not estimating,
                "composed_odometry_topic": ESTIMATED_POSE if estimating else "",
                # Only AMCL and MOLA need to be told where they start.
                "initial_pose_topic": (
                    INITIAL_POSE_TOPIC
                    if pose_source in ("amcl", "mola")
                    else ""
                ),
                # MOLA applies it after its first scan (BRINGUP-2500).
                "hold_estimate_until_initial_pose": pose_source == "mola",
            },
        ],
    )
    controller_node = Node(
        package="racing_controller_baseline",
        executable="racing_controller_baseline_node",
        parameters=[
            vehicle,
            {
                "use_sim_time": use_sim_time,
                **({"speed_source": speed_source} if speed_source else {}),
            },
        ],
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
            vehicle,
            tracks["racing_safety_supervisor"],
            {"use_sim_time": use_sim_time},
        ],
    )
    metrics_node = Node(
        package="racing_metrics",
        executable="racing_metrics_node",
        parameters=[
            vehicle,
            tracks["racing_metrics"],
            {"seed": seed, "laps": laps, "use_sim_time": use_sim_time},
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
            {"estimate_topic": estimate_topic, "use_sim_time": use_sim_time},
        ],
    )
    recording_node = Node(
        package="racing_recording",
        executable="racing_recording_node",
        parameters=[
            vehicle,
            tracks["racing_recording"],
            {
                "seed": seed,
                "output_path": recording_path,
                "estimate_topic": estimate_topic,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )
    return [
        autodrive_node if autodrive_node is not None else sim_node,
        support_node,
        controller_node,
        supervisor_node,
        metrics_node,
        evaluation_node,
        recording_node,
        robot_state_publisher_node,
        rviz_node,
        *_pose_source_nodes(pose_source, tracks, use_sim_time),
    ]


def _pose_source_nodes(
    pose_source: str, tracks: dict, use_sim_time: bool
) -> list:
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
        return amcl(
            [configuration],
            tracks["map_server"]["yaml_filename"],
            use_sim_time,
        )
    if pose_source == "mola":
        # MOLA's map is derived from the same grid, for the same reason.
        return mola(
            configuration,
            tracks["map_server"]["yaml_filename"],
            use_sim_time,
        )
    return slam_toolbox("mapping", [configuration], use_sim_time)


def _launch_described(context, description_share: Path, rviz_config: str):
    """The vehicle file's own URDF, then every node (_launch_nodes)."""
    vehicle = LaunchConfiguration("vehicle").perform(context)
    if not Path(vehicle).is_file():
        raise RuntimeError(f"vehicle file not found: {vehicle}")
    robot_description = description_for(vehicle, description_share).read_text(
        encoding="utf-8"
    )
    return _launch_nodes(context, robot_description, rviz_config)


def generate_launch_description() -> LaunchDescription:
    description_share = Path(
        get_package_share_directory("racing_vehicle_description")
    )
    rviz_config = str(description_share / "rviz" / "sim_pure_pursuit.rviz")

    # Launch-time controls: which scenario runs, which vehicle file it runs
    # with, its seed, where the replay
    # log lands, whether RViz starts alongside the graph, how fast
    # simulated time advances relative to wall clock (ADR 0006), and which
    # pose racing_evaluation scores.
    scenario_arg = DeclareLaunchArgument("scenario", default_value=SCENARIO)
    vehicle_arg = DeclareLaunchArgument("vehicle", default_value=VEHICLE_CONFIG)
    simulator_arg = DeclareLaunchArgument(
        "simulator", default_value="f1tenth_gym_jax", choices=list(SIMULATORS)
    )
    autodrive_bridge_arg = DeclareLaunchArgument(
        "autodrive_bridge",
        default_value="ours",
        choices=list(AUTODRIVE_BRIDGES),
    )
    autodrive_devkit_domain_arg = DeclareLaunchArgument(
        "autodrive_devkit_domain",
        default_value=os.environ.get("AUTODRIVE_ROS_DOMAIN_ID", ""),
        description=(
            "autodrive_bridge:=official: the ROS domain the AutoDRIVE devkit "
            "runs on; must differ from this launch's ROS_DOMAIN_ID"
        ),
    )
    autodrive_port_arg = DeclareLaunchArgument(
        "autodrive_port", default_value=AUTODRIVE_PORT
    )
    seed_arg = DeclareLaunchArgument("seed", default_value="0")
    laps_arg = DeclareLaunchArgument(
        "laps",
        default_value="1",
        description="Laps that complete the run (racing_metrics `laps`).",
    )
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
    planner_arg = DeclareLaunchArgument(
        "planner", default_value="track_centerline", choices=list(PLANNERS)
    )
    raceline_optimizer_arg = DeclareLaunchArgument(
        "raceline_optimizer",
        default_value="tum_mincurv_iqp",
        choices=list(RACELINE_OPTIMIZERS),
    )
    speed_source_arg = DeclareLaunchArgument(
        "speed_source",
        default_value="",
        choices=["", *SPEED_SOURCES],
    )

    return LaunchDescription(
        [
            scenario_arg,
            vehicle_arg,
            simulator_arg,
            autodrive_bridge_arg,
            autodrive_port_arg,
            autodrive_devkit_domain_arg,
            seed_arg,
            laps_arg,
            rec_arg,
            use_rviz_arg,
            time_scale_arg,
            start_held_arg,
            estimate_topic_arg,
            pose_source_arg,
            drive_on_estimate_arg,
            planner_arg,
            raceline_optimizer_arg,
            speed_source_arg,
            OpaqueFunction(
                function=_launch_described,
                args=[description_share, rviz_config],
            ),
        ]
    )
