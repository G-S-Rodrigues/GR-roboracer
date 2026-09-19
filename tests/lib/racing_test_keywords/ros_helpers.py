"""Generic rclpy spin/publish/wait-for-message helpers.

Generalises the polling idiom already proven in
``racing_sim_adapter/test/test_adapter_contract.py`` (tier 2): spin a node with
short timeouts against a wall-clock deadline instead of blocking calls, because
a cold JAX process can take longer to come up than ordinary ROS discovery.
"""

from __future__ import annotations

import time
from collections.abc import Iterable
from typing import Any

import rclpy
from rclpy.node import Node

_rclpy_owns_init = False


def ensure_rclpy_initialized() -> None:
    """Initialise rclpy once per process if nothing else already has."""
    global _rclpy_owns_init
    if not rclpy.ok():
        rclpy.init()
        _rclpy_owns_init = True


def reset_rclpy() -> None:
    """Tear down and clear this module's rclpy context, if it owns one.

    Reusing one rclpy context/DDS participant across two sequential
    ``run_scenario`` launches in the same process is what SIM-3030's
    back-to-back runs exposed: the second launch's subscription silently
    never matched the second process's publisher, timing out at 90s even
    though the graph itself came up cleanly (visible in its own log). A
    fresh context per launch sidesteps that stale-participant state.
    """
    global _rclpy_owns_init
    if _rclpy_owns_init and rclpy.ok():
        rclpy.shutdown()
        _rclpy_owns_init = False


def wait_for_node(
    node: Node,
    node_name: str,
    required_topics: Iterable[str] = (),
    timeout: float = 15.0,
) -> None:
    """Block until ``node_name`` and its ``required_topics`` are discovered."""
    required = set(required_topics)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        topics = {name for name, _types in node.get_topic_names_and_types()}
        if node_name in node.get_node_names() and required.issubset(topics):
            return
        rclpy.spin_once(node, timeout_sec=0.05)
    raise TimeoutError(f"node not found within {timeout}s: {node_name}")


_ENDPOINT_GETTERS = {
    "publisher": "get_publishers_info_by_topic",
    "subscription": "get_subscriptions_info_by_topic",
}


def wait_for_endpoints(
    node: Node,
    node_name: str,
    topics: Iterable[str],
    role: str,
    timeout: float = 15.0,
) -> None:
    """Block until ``node_name`` holds a ``role`` endpoint on every topic.

    ``role`` is ``"publisher"`` or ``"subscription"``. ``wait_for_node``
    only proves a topic name exists somewhere in the graph and that some
    node with this name is present - it does not prove *this* node has
    actually matched that topic in DDS. Endpoint discovery
    (``get_publishers_info_by_topic`` / ``get_subscriptions_info_by_topic``)
    is as close to "matched" as a third node can observe: real
    subscription/publisher matching happens inside each participant's own
    DDS layer and is not otherwise exposed to an outside process.

    ``timeout`` is spent once across every topic in ``topics``, not reset
    per topic, so a caller sharing one deadline across several calls (as
    ``_reset_at_deterministic_t0`` does) gets one true deadline overall
    rather than a fresh budget per call.
    """
    getter = getattr(node, _ENDPOINT_GETTERS[role])
    deadline = time.monotonic() + timeout
    for topic in topics:
        while time.monotonic() < deadline:
            infos = getter(topic)
            if any(info.node_name == node_name for info in infos):
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        else:
            raise TimeoutError(
                f"{role} endpoint not found within {timeout}s: "
                f"{node_name} on {topic}"
            )


def wait_for_message(
    node: Node,
    message_type: Any,
    topic: str,
    qos: Any,
    timeout: float = 10.0,
) -> Any:
    """Block until one message arrives on ``topic``, then return it."""
    messages: list[Any] = []
    subscription = node.create_subscription(
        message_type, topic, messages.append, qos
    )
    deadline = time.monotonic() + timeout
    try:
        while not messages and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_subscription(subscription)
    if not messages:
        raise TimeoutError(f"no message on {topic} within {timeout}s")
    return messages[0]
