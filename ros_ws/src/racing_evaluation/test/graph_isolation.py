"""Graph-isolation check for the tier-2 launch tests (EVAL-2030).

Not a test module: ``test_evaluation_node.py`` and
``test_evaluation_node_pose.py`` import it. It reads what a node publishes and
subscribes from the live graph, so a test can assert that no data topic of the
node under test, or of its own test node, sits outside the file's prefix.
"""

import time

import rclpy
from rclpy._rclpy_pybind11 import NodeNameNonExistentError

# Every rcl node holds these two on a global name by design. They carry no
# Odometry or LocalizationError, so they cannot reach the Aligner.
INFRASTRUCTURE_TOPICS = frozenset({"/rosout", "/parameter_events"})


def _topics(node, name: str, namespace: str) -> set[str]:
    publishers = node.get_publisher_names_and_types_by_node(name, namespace)
    subscribers = node.get_subscriber_names_and_types_by_node(name, namespace)
    return {topic for topic, _ in publishers + subscribers}


def data_topics(node, name: str, namespace: str, timeout: float = 10.0):
    """Every topic `name` in `namespace` publishes or subscribes, minus the
    two every rcl node holds; waits for the node to be discovered."""
    deadline = time.monotonic() + timeout
    found: set[str] = set()
    stable_since = None
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            current = _topics(node, name, namespace)
        except NodeNameNonExistentError:
            stable_since = None
            continue
        current -= INFRASTRUCTURE_TOPICS
        if current != found or stable_since is None:
            found = current
            stable_since = time.monotonic()
        elif time.monotonic() - stable_since >= 0.5:
            break
    return found


def outside(topics, prefix: str) -> list[str]:
    """Sorted topics not under `prefix + "/"`."""
    return sorted(
        topic for topic in topics if not topic.startswith(prefix + "/")
    )
