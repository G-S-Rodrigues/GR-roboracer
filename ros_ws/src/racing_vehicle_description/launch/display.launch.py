"""Static RViz display of the F1TENTH chassis -- no sim, nothing moves.

Complements racetrack driving slices like racing_bringup's sim_pure_pursuit
(the full vertical slice with a moving car); this launch is for inspecting the
model itself. It starts just robot_state_publisher (publishing the URDF's
fixed-joint TF tree) and RViz pointed at rviz/display.rviz.

Because every joint in f1tenth.urdf is fixed and no node publishes joint states,
the chassis sits motionless at base_link -- exactly what a geometry / placement
check wants, and the RViz twin of scripts/render_urdf.py's offline renders.

Run from anywhere (share path resolves handled below):

    ros2 launch racing_vehicle_description display.launch.py
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("racing_vehicle_description"))
    robot_desc = (share / "urdf" / "f1tenth.urdf").read_text(encoding="utf-8")
    rviz_cfg = str(share / "rviz" / "display.rviz")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_desc}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_cfg],
            ),
        ]
    )
