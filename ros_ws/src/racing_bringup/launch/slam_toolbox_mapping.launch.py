"""The survey run: the driving stack laps while slam_toolbox's async mapper
lays /scan at the poses /odom gives it, which is how a track's committed
occupancy grid is made (plan D7).

On the gym the driver is Pure Pursuit on ground truth and the poses are
exact. On AutoDRIVE (`simulator:=autodrive controller:=follow_the_gap`) the
driver reads /scan only and the poses are the sim's dead-reckoned /odom
(encoder speed plus an exact yaw rate: a perfect-gyro odometry, part 11a).

`scripts/map_track.sh` drives this launch and saves /map afterwards; it is
the supported entry point. The mapper never publishes map -> odom
(config/localization/slam_toolbox_survey.yaml says why), so the driving
stack is exactly the default one, and every argument of
sim_pure_pursuit.launch.py passes through, `controller:=` included.
The mapper reads /clock exactly when the simulator owns it (ADR 0006).
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from racing_bringup.estimators import slam_toolbox
from racing_sim_autodrive.capabilities import CAPABILITIES as AUTODRIVE_CAPS
from racing_sim_gym_jax.capabilities import CAPABILITIES as GYM_CAPS

SURVEY_CONFIG = "config/localization/slam_toolbox_survey.yaml"
# By `simulator:=` value, as sim_pure_pursuit.launch.py's own table.
SIMULATOR_CAPABILITIES = {
    "f1tenth_gym_jax": GYM_CAPS,
    "autodrive": AUTODRIVE_CAPS,
}


def _mapper(context) -> list:
    simulator = LaunchConfiguration(
        "simulator", default="f1tenth_gym_jax"
    ).perform(context)
    if simulator not in SIMULATOR_CAPABILITIES:
        raise RuntimeError(
            f"simulator {simulator!r} is not one of "
            f"{sorted(SIMULATOR_CAPABILITIES)}"
        )
    owns_clock = (
        SIMULATOR_CAPABILITIES[simulator]["capability.clock"] == "owns_clock"
    )
    return slam_toolbox("mapping", [SURVEY_CONFIG], use_sim_time=owns_clock)


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
            OpaqueFunction(function=_mapper),
        ]
    )
