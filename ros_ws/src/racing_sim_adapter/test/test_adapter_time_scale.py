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
# can actually fire, so time_scale=5.0 never gets a clean 5x, and how far
# below 5x it lands depends on how loaded the host is. Measured steady
# state on this host (dev container, CPU JAX), from least to most loaded:
# 3.51x / 3.64x / 3.65x idle; 2.34x with another process contending for
# CPU; 1.80x during an actual `check.sh --full` run at load average ~4.
# The property under test is "time_scale reaches the running wall timer",
# not a throughput figure - a wiring bug (time_scale parsed but never
# passed to the timer) would show ~1x regardless of load, so the floor
# only needs to sit clearly below every measured run, contended included,
# and clearly above the ~1x a time_scale=1 run would show. 1.5 does both:
# it sits comfortably below the worst run actually observed (1.80x),
# while a wiring-bug run (time_scale never reaching the timer, rate ~1x)
# would still fail it by a wide margin.
MINIMUM_RATE = 1.5

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
