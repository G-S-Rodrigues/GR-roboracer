"""What slam.launch.py starts to drive the car, beside the SLAM method."""

from launch_ros.actions import Node

from racing_bringup import teleop

# "joystick" is added here later, beside these.
DRIVES = ("window", "teleop")


def nodes(
    drive: str, config: str, vehicle_path: str, use_sim_time: bool
) -> list:
    """The nodes `drive:=<drive>` adds; none for `window`.

    `teleop` loads `config` and refuses a limit above the vehicle's caps
    before it returns a node. Every member of DRIVES has a branch here: one
    added to the choices without one is refused rather than silently
    starting nothing.
    """
    if drive == "window":
        return []
    if drive == "teleop":
        teleop.check_caps(teleop.load(config), vehicle_path)
        return [
            Node(
                package="racing_bringup",
                executable="racing_bringup_teleop",
                name="racing_bringup_teleop",
                parameters=[config, {"use_sim_time": use_sim_time}],
            )
        ]
    raise RuntimeError(f"drive:={drive} is not one of {', '.join(DRIVES)}")
