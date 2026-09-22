"""Launch actions for the pose estimators the bringup can start.

slam_toolbox (jazzy) is a lifecycle node: nothing subscribes, publishes or
broadcasts until it is configured and activated. Its own launch files drive
those transitions with launch events rather than a lifecycle manager, and so
does this module, so the node is active - and publishing - before the
scenario runner's deterministic reset (repo-gotchas #19).

nav2_amcl and nav2_map_server are lifecycle nodes too, but nav2 ships a
manager for exactly this and orders the transitions across both of them
(map_server before amcl, which reads the map on configure). That manager is
used rather than a second hand-rolled copy of the event chain above.
"""

from __future__ import annotations

from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

SLAM_TOOLBOX_EXECUTABLES = {
    "mapping": "async_slam_toolbox_node",
    "localization": "localization_slam_toolbox_node",
}


def autostarted(node: LifecycleNode) -> list:
    """`node`, configured on launch and activated once it is inactive."""
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )
    return [node, activate, configure]


def slam_toolbox(mode: str, parameters: list) -> list:
    """slam_toolbox in `mode` ("mapping" | "localization"), autostarted."""
    node = LifecycleNode(
        package="slam_toolbox",
        executable=SLAM_TOOLBOX_EXECUTABLES[mode],
        name="slam_toolbox",
        namespace="",
        parameters=[*parameters, {"use_lifecycle_manager": False}],
        output="screen",
    )
    return autostarted(node)


def amcl(parameters: list, map_yaml: str) -> list:
    """nav2_amcl localizing against `map_yaml`, served by nav2_map_server.

    The grid is the track's committed survey
    (config/scenarios/maps/<map>/slam/), named by the caller from the
    scenario, so a run can only ever be localized against its own track.
    """
    return [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[*parameters, {"yaml_filename": map_yaml}],
            output="screen",
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            parameters=parameters,
            output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="amcl_lifecycle_manager",
            parameters=parameters,
            output="screen",
        ),
    ]
