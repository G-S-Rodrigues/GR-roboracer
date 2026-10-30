"""Tier-2 conformance: a backend started held does not advance until reset.

A separate launch from `test_adapter_contract.py` because it needs the
backend launched with `start_held: true`, and every case in one launch_test
file shares one launch.
"""

import sys
import time
import unittest
from pathlib import Path

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from nav_msgs.msg import Odometry
from racing_interfaces.srv import Reset
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock

sys.path.insert(0, str(Path(__file__).resolve().parent))

import conformance  # noqa: E402

ADAPTER_NODE = conformance.ADAPTER_NAME
ODOMETRY_TOPIC = conformance.topic("/odom")
CLOCK_TOPIC = conformance.topic("/clock")
RESET_SERVICE = conformance.service("reset")

STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription(
        [
            *conformance.backend().actions(
                [
                    {
                        "start_held": True,
                        # ADAPT-2145: a capability is the backend's to
                        # declare; this override, the value the backend does
                        # NOT declare, must be ignored.
                        "capability.clock": (
                            conformance.backend().clock_override
                        ),
                    }
                ]
            ),
            launch_testing.actions.ReadyToTest(),
        ]
    )


def _stamp_ns(message: Clock) -> int:
    return message.clock.sec * 1_000_000_000 + message.clock.nanosec


class TestAdapterStartHeld(unittest.TestCase):
    """A deterministic t=0: nothing moves before the first reset."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("racing_sim_adapter_start_held_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _require_adapter(self) -> None:
        required_topics = {ODOMETRY_TOPIC}
        # Only a backend that owns the clock publishes /clock.
        if conformance.backend().expected_clock == conformance.OWNS_CLOCK:
            required_topics.add(CLOCK_TOPIC)
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            topics = {
                name for name, _types in self.node.get_topic_names_and_types()
            }
            if conformance.adapter_present(
                self.node
            ) and required_topics.issubset(topics):
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail(f"node not found: {conformance.node_fqn()}")

    def _collect(self, message_type, topic, qos, count, timeout=3.0):
        messages = []
        subscription = self.node.create_subscription(
            message_type, topic, messages.append, qos
        )
        deadline = time.monotonic() + timeout
        try:
            while len(messages) < count and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        finally:
            self.node.destroy_subscription(subscription)
        self.assertGreaterEqual(len(messages), count, f"no data on {topic}")
        return messages

    def _reset(self, seed: int) -> None:
        client = self.node.create_client(Reset, RESET_SERVICE)
        self.assertTrue(
            client.wait_for_service(timeout_sec=2.0), "reset unavailable"
        )
        future = client.call_async(Reset.Request(seed=seed, scenario_id=""))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=3.0)
        self.assertTrue(future.done(), "reset timed out")
        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.success, response.message)
        self.node.destroy_client(client)

    def _applicable(self, test_id: str) -> bool:
        return conformance.applicable(self, self.node, test_id)

    def test_adapt_2080_start_held_does_not_advance_until_reset(self):
        """ADAPT-2080: started held, /clock stays at 0 and the pose stays
        put - while still publishing, so consumers can be discovered - until
        the first `~/reset`, after which simulated time advances.

        Without this, t=0 is wherever DDS discovery lands, and a reset issued
        once the graph is up rewinds a clock consumers already followed:
        racing_metrics then aborts on a non-monotonic stamp.
        """
        self._require_adapter()
        if not self._applicable("ADAPT-2080"):
            return
        held_clock = self._collect(Clock, CLOCK_TOPIC, CLOCK_QOS, 10)
        self.assertEqual(
            [_stamp_ns(message) for message in held_clock],
            [0] * len(held_clock),
            "a held backend must not advance simulated time",
        )
        held_odometry = self._collect(Odometry, ODOMETRY_TOPIC, STATE_QOS, 5)
        for message in held_odometry[1:]:
            self.assertEqual(message.pose.pose, held_odometry[0].pose.pose)

        self._reset(seed=2080)

        released = self._collect(Clock, CLOCK_TOPIC, CLOCK_QOS, 20)
        stamps_ns = [_stamp_ns(message) for message in released]
        for earlier, later in zip(stamps_ns, stamps_ns[1:], strict=False):
            self.assertLess(earlier, later, "/clock must advance once released")

    def test_adapt_2145_a_launch_override_of_a_capability_is_ignored(self):
        """ADAPT-2145: this launch passes capability.clock as the value the
        backend does not declare (the row's clock_override); the backend
        still reads back its own, because a capability is what the backend
        is, not something a launch can configure."""
        self._require_adapter()
        declared = conformance.declared_capabilities(self.node)
        expected = conformance.backend().expected_clock
        self.assertNotEqual(conformance.backend().clock_override, expected)
        self.assertEqual(declared[conformance.CLOCK_PARAMETER], expected)
