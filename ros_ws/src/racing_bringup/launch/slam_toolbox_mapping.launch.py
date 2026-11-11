"""The survey run: the reference stack laps on ground truth while
slam_toolbox's async mapper lays /scan at those exact poses, which is how
a track's committed occupancy grid is made (plan D7).

`scripts/map_track.sh` drives this launch and saves /map afterwards; it is
the supported entry point. The mapper never publishes map -> odom
(config/localization/slam_toolbox_survey.yaml says why), so the driving
stack is exactly the default one, and every argument of
sim_pure_pursuit.launch.py passes through.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from racing_bringup.estimators import slam_toolbox
from racing_bringup.simulators import sim_time_for

SURVEY_CONFIG = "config/localization/slam_toolbox_survey.yaml"


def _mapper(context: LaunchContext) -> list:
    """slam_toolbox on the simulator's time base: /clock exactly when the
    simulator owns it (the gym), wall time on AutoDRIVE (ADR 0006)."""
    return slam_toolbox(
        "mapping",
        [LaunchConfiguration("slam_toolbox_config").perform(context)],
        use_sim_time=sim_time_for(
            LaunchConfiguration("simulator").perform(context)
        ),
    )


def generate_launch_description() -> LaunchDescription:
    bringup = (
        Path(get_package_share_directory("racing_bringup"))
        / "launch"
        / "sim_pure_pursuit.launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simulator",
                default_value="f1tenth_gym_jax",
                description="The simulator, which decides use_sim_time.",
            ),
            DeclareLaunchArgument(
                "slam_toolbox_config",
                default_value=SURVEY_CONFIG,
                description="slam_toolbox's parameter yaml.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup))
            ),
            OpaqueFunction(function=_mapper),
        ]
    )
