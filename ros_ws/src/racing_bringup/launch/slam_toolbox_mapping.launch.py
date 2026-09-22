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
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from racing_bringup.estimators import slam_toolbox

SURVEY_CONFIG = "config/localization/slam_toolbox_survey.yaml"


def generate_launch_description() -> LaunchDescription:
    bringup = (
        Path(get_package_share_directory("racing_bringup"))
        / "launch"
        / "sim_pure_pursuit.launch.py"
    )
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(bringup))
            ),
            *slam_toolbox("mapping", [SURVEY_CONFIG]),
        ]
    )
