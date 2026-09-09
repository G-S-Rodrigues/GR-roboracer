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
