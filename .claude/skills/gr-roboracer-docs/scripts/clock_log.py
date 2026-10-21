"""Log (wall, /clock sim time) once per second. Best-effort subscriber."""

import contextlib
import pathlib
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

out = pathlib.Path(sys.argv[1]).open("w")  # noqa: SIM115 (open for the run)
out.write("wall,sim\n")
rclpy.init()
n = Node("clock_log", parameter_overrides=[])
last = [0.0]


def cb(m):
    now = time.time()
    if now - last[0] >= 1.0:
        last[0] = now
        out.write(f"{now:.4f},{m.clock.sec + m.clock.nanosec * 1e-9:.4f}\n")
        out.flush()


n.create_subscription(
    Clock,
    "/clock",
    cb,
    QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    ),
)
with contextlib.suppress(KeyboardInterrupt):
    rclpy.spin(n)
