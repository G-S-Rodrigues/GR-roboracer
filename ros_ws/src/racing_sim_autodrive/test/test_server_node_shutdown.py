"""Tier 2: SIMAD-2050, the server stops cleanly on SIGINT.

Both halves must stop: the gevent loop on the main thread and the executor
thread. A hang or a non-zero exit would read as a failed run rather than a
shutdown (the SIMJAX-1110 shape).
"""

import sys
import unittest
from pathlib import Path

import launch_testing.asserts
import launch_testing.markers
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import _server_launch as launch_support  # noqa: E402

PORT = launch_support.free_port()


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    return launch_support.server_description(PORT)[1]


class TestServerUp(unittest.TestCase):
    def test_simad_2050_server_is_listening(self, proc_output):
        proc_output.assertWaitFor("listening on", timeout=20)


@launch_testing.post_shutdown_test()
class TestServerShutdown(unittest.TestCase):
    def test_simad_2050_sigint_exits_cleanly(self, proc_info):
        """SIMAD-2050: SIGINT -> exit code 0."""
        launch_testing.asserts.assertExitCodes(proc_info)
