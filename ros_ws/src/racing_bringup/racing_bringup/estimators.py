"""Launch actions for the pose estimators the bringup can start.

slam_toolbox (jazzy) is a lifecycle node: nothing subscribes, publishes or
broadcasts until it is configured and activated. Its own launch files drive
those transitions with launch events rather than a lifecycle manager, and so
does this module, so the node is active - and publishing - before the
scenario runner's deterministic reset (repo-gotchas #19).
"""

from __future__ import annotations

from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
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
