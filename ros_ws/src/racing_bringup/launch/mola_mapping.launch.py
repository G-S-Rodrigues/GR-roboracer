"""MOLA-LO's mapping lap, and nothing else: the 2D lidar pipeline runs
online in mapping mode and saves its simplemap when it shuts down.

`scripts/mola_map_track.sh` drives this launch beside the reference stack
(which it starts separately, held) and is the supported entry point. It is
MOLA-only on purpose: the script ends the lap by sending SIGINT to
mola-cli, which saves the simplemap on that signal, and that shutdown needs
MOLA in a process tree of its own. Defaults are relative to the repository
root, as for slam_toolbox_mapping.launch.py.
"""

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from racing_bringup.estimators import mola_mapping

MAPPING_CONFIG = "config/localization/mola_lidar2d_mapping.yaml"


def _mapping(context: LaunchContext) -> list:
    return mola_mapping(
        LaunchConfiguration("configuration").perform(context),
        LaunchConfiguration("simplemap_output").perform(context),
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "simplemap_output",
                description="Where MOLA writes the lap's .simplemap.",
            ),
            DeclareLaunchArgument(
                "configuration",
                default_value=MAPPING_CONFIG,
                description="MOLA-LO's launch arguments, as a yaml.",
            ),
            OpaqueFunction(function=_mapping),
        ]
    )
