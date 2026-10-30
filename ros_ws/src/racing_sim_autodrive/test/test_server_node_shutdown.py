"""Tier 2: SIMAD-2050, the server stops cleanly on SIGINT.

Both halves must stop: the gevent loop on the main thread and the executor
thread. A hang or a non-zero exit would read as a failed run rather than a
shutdown (the SIMJAX-1110 shape).
"""

import sys
import time
import unittest
from pathlib import Path

import launch_testing.asserts
import launch_testing.markers
import pytest
import rclpy
from rclpy._rclpy_pybind11 import NodeNameNonExistentError

sys.path.insert(0, str(Path(__file__).resolve().parent))

import _server_launch as launch_support  # noqa: E402

PORT = launch_support.free_port()


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    server, description = launch_support.server_description(PORT)
    return description, {"server": server}


class TestServerUp(unittest.TestCase):
    def test_simad_2050_server_is_listening(self, proc_output):
        proc_output.assertWaitFor("listening on", timeout=20)

    def test_simad_2100_server_names_are_namespaced(self, server, proc_output):
        """SIMAD-2100: the test-launched server publishes, subscribes and
        serves only names under its test prefix.

        colcon runs other packages' tier-2 tests on the same domain, so a
        global /ground_truth/odom or /scan from this server poisons them
        (EVAL-2020 flaked 2 of 5 runs). /rosout and /parameter_events exist
        on every node and are excepted.
        """
        proc_output.assertWaitFor("listening on", timeout=20)
        full = server.node_name
        namespace, name = full.rsplit("/", 1)
        # an unset namespace comes back as "<node_namespace_unspecified>"
        namespace = namespace if namespace.startswith("/") else "/"
        rclpy.init()
        node = rclpy.create_node("simad_2100_probe")
        try:
            publishers = subscribers = services = []

            def complete():
                return (
                    len(publishers) >= 8
                    and any(n.endswith("/drive") for n, _ in subscribers)
                    and any(n.endswith("/reset") for n, _ in services)
                )

            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                try:
                    publishers = node.get_publisher_names_and_types_by_node(
                        name, namespace
                    )
                    subscribers = node.get_subscriber_names_and_types_by_node(
                        name, namespace
                    )
                    services = node.get_service_names_and_types_by_node(
                        name, namespace
                    )
                except NodeNameNonExistentError:
                    continue  # not discovered yet
                if complete():
                    break
        finally:
            node.destroy_node()
            rclpy.shutdown()
        allowed = {"/rosout", "/parameter_events"}
        outside = sorted(
            n
            for n, _ in [*publishers, *subscribers, *services]
            if n not in allowed
            and (namespace == "/" or not n.startswith(namespace + "/"))
        )
        self.assertEqual(outside, [], f"{full}: names outside {namespace}")
        self.assertNotEqual(namespace, "/", f"{full}: root namespace")
        # never pass on an empty or partial graph: discovery must have finished
        self.assertTrue(
            complete(),
            f"{full}: graph incomplete after 10 s: pubs {publishers}, "
            f"subs {subscribers}, services {services}",
        )


@launch_testing.post_shutdown_test()
class TestServerShutdown(unittest.TestCase):
    def test_simad_2050_sigint_exits_cleanly(self, proc_info):
        """SIMAD-2050: SIGINT -> exit code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)
