"""Static RViz display of a vehicle's description -- no sim, nothing moves.

Complements racetrack driving slices like racing_bringup's sim_pure_pursuit
(the full vertical slice with a moving car); this launch is for inspecting the
model itself. It starts just robot_state_publisher (publishing the URDF's
fixed-joint TF tree) and RViz pointed at rviz/display.rviz.

The URDF follows the vehicle: `vehicle:=` names a config/vehicles/ file whose
stem urdf/descriptions.yaml maps to a URDF (the file itself is never opened).
The default is the F1TENTH gym car. Every joint is fixed and no node publishes
joint states, so the model sits motionless in rear_axle (lr behind and h below
base_link: 0.17145 / 0.074 m for the gym car, 0.15532 / 0.07354 m for the
AutoDRIVE RoboRacer) -- exactly what a geometry / placement check wants, and
the RViz twin of scripts/render_urdf.py's offline renders.

Run from anywhere (share path resolves handled below):

    ros2 launch racing_vehicle_description display.launch.py
    ros2 launch racing_vehicle_description display.launch.py \
        vehicle:=config/vehicles/autodrive_roboracer.yaml
"""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def description_for(vehicle: str, share: Path) -> Path:
    """The URDF that draws `vehicle` (a vehicle file path; only its stem).

    Mirrors racing_bringup.vehicle_description.description_for: this package
    must not depend on racing_bringup. DESC-1090 and BRINGUP-1056 pin both to
    urdf/descriptions.yaml.
    """
    share = Path(share)
    stem = Path(vehicle).stem
    mapping = yaml.safe_load(
        (share / "urdf" / "descriptions.yaml").read_text(encoding="utf-8")
    )
    if stem not in mapping:
        raise RuntimeError(f"no description for vehicle {stem}")
    return share / "urdf" / mapping[stem]


def _nodes(context, share: Path):
    vehicle = LaunchConfiguration("vehicle").perform(context)
    robot_desc = description_for(vehicle, share).read_text(encoding="utf-8")
    rviz_cfg = str(share / "rviz" / "display.rviz")
    return [
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


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("racing_vehicle_description"))
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle",
                default_value="config/vehicles/f1tenth_default.yaml",
                description="Vehicle file whose stem selects the URDF.",
            ),
            OpaqueFunction(function=_nodes, args=[share]),
        ]
    )
