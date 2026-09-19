"""Tier-2: `time_scale` actually changes /clock's wall-clock rate.

A separate launch from `test_adapter_contract.py`, modelled on
`test_adapter_start_held.py`: it needs the backend launched with
`time_scale: 5.0`, and every case in one launch_test file shares one launch.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock

ADAPTER_NODE = "racing_sim"
CLOCK_TOPIC = "/clock"
TIME_SCALE = 5.0
SAMPLE_COUNT = 60
# Discard the leading samples: JIT warm-up and process startup happen once,
# before the wall timer settles into steady state, and would drag the
# measured rate toward 1x regardless of time_scale (repo-gotchas #13).
STEADY_STATE_TAIL = 40
# repo-gotchas #4 / ADR 0006: the JAX step bounds how fast the wall timer
# can actually fire, so time_scale=5.0 never gets a clean 5x. Measured
# steady state on this host (dev container, CPU JAX, shared with another
# agent's test runs - see CLAUDE.md's task instructions): four runs gave
# 3.51x / 3.64x / 3.65x / 2.34x, the low one coinciding with host
# contention from that other agent. PR 1 measured ~2.5x effective across a
# *whole lap* including process startup and JIT warm-up at time_scale=5.0;
# this test excludes both, so in principle its floor could sit above that
# whole-lap figure, but the contended run means host load can pull it back
# down close to it. 2.0 stays clearly under every measured run - including
# the contended one - while still clearly failing at time_scale=1
# (rate ~1x), which is the point of the test.
MINIMUM_RATE = 2.0

CLOCK_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@pytest.mark.launch_test
def generate_test_description():
    backend = launch_ros.actions.Node(
        package="racing_sim_gym_jax",
        executable="racing_sim_gym_jax_node",
        parameters=[{"time_scale": TIME_SCALE}],
        output="screen",
    )
    return launch.LaunchDescription(
        [backend, launch_testing.actions.ReadyToTest()]
    )


def _stamp_ns(message: Clock) -> int:
    return message.clock.sec * 1_000_000_000 + message.clock.nanosec


class TestAdapterTimeScale(unittest.TestCase):
    """`time_scale` propagates from the launch argument to /clock's rate."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("racing_sim_adapter_time_scale_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _require_adapter(self) -> None:
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            topics = {
                name for name, _types in self.node.get_topic_names_and_types()
            }
            if (
                ADAPTER_NODE in self.node.get_node_names()
                and CLOCK_TOPIC in topics
            ):
                return
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.fail(f"node not found: {ADAPTER_NODE}")

    def test_adapt_2090_time_scale_propagates_to_clock_rate(self):
        """ADAPT-2090: launched with time_scale=5.0, /clock's steady-state
        simulated-seconds-per-wall-second rate is well above 1x.

        Without this, `time_scale` could be parsed and passed all the way
        into `wall_timer_period` (COMMON-level unit tests already cover that
        pure function) while never actually reaching the running timer -
        e.g. a launch-argument wiring mistake - and nothing would catch it,
        because no other launch-level test uses anything but the default
        time_scale=1.0.
        """
        self._require_adapter()
        messages: list[Clock] = []
        wall_times: list[float] = []

        def _record(message: Clock) -> None:
            messages.append(message)
            wall_times.append(time.monotonic())

        subscription = self.node.create_subscription(
            Clock, CLOCK_TOPIC, _record, CLOCK_QOS
        )
        deadline = time.monotonic() + 15.0
        try:
            while len(messages) < SAMPLE_COUNT and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.05)
        finally:
            self.node.destroy_subscription(subscription)
        self.assertGreaterEqual(len(messages), SAMPLE_COUNT, "no /clock data")

        stable_messages = messages[-STEADY_STATE_TAIL:]
        stable_wall_times = wall_times[-STEADY_STATE_TAIL:]
        sim_elapsed = (
            _stamp_ns(stable_messages[-1]) - _stamp_ns(stable_messages[0])
        ) / 1e9
        wall_elapsed = stable_wall_times[-1] - stable_wall_times[0]
        self.assertGreater(wall_elapsed, 0.0)
        measured_rate = sim_elapsed / wall_elapsed
        self.assertGreater(
            measured_rate,
            MINIMUM_RATE,
            f"time_scale={TIME_SCALE} should advance /clock well above "
            f"1x wall time in steady state; measured {measured_rate:.2f}x",
        )
