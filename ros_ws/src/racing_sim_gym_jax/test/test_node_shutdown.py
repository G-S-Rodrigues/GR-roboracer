"""SIMJAX-1110: the sim node stops cleanly on the launcher's SIGINT.

The same guard BRINGUP-1070 pins for the support node and EVAL-1050 for
the evaluation node. The sim node is the one process in the graph whose
non-zero exit would be read as a physics failure rather than a shutdown
race, which is why it carries the guard even though it has not yet been
caught by it.
"""

import signal

import pytest
from racing_sim_gym_jax import node as sim_node

TAKE_AFTER_SHUTDOWN = RuntimeError(
    "Unable to convert call argument '0' to Python object"
)


class _Context:
    def __init__(self, ok: bool) -> None:
        self._ok = ok

    def ok(self) -> bool:
        return self._ok


class _Node:
    """Only what spin_until_shutdown reads of a node."""

    def __init__(self, context_ok: bool) -> None:
        self.context = _Context(context_ok)


def _raising_spin(error: BaseException):
    def _spin(node, *args, **kwargs):
        raise error

    return _spin


def test_simjax_1110_sigint_does_not_raise_into_a_c_call() -> None:
    """SIGINT must not raise KeyboardInterrupt while rclpy is inside a call.

    Python's default handler raises at the next bytecode boundary, which
    for a spinning node is usually inside a pybind11 call; `take_message`
    then fails to convert its argument with an error already pending and
    surfaces as `RuntimeError: Unable to convert call argument '0' to
    Python object`. rclpy's own handler wakes the executor between calls
    instead, and is left to do the stopping.
    """
    previous = signal.getsignal(signal.SIGINT)
    try:
        sim_node.let_rclpy_own_sigint()
        handler = signal.getsignal(signal.SIGINT)

        assert handler is not signal.default_int_handler
        assert callable(handler)
        assert handler(signal.SIGINT, None) is None
    finally:
        signal.signal(signal.SIGINT, previous)


def test_simjax_1110_take_after_shutdown_is_a_clean_stop(monkeypatch) -> None:
    """A take that fails once the context is gone exits 0, not 1."""
    monkeypatch.setattr(
        sim_node.rclpy, "spin", _raising_spin(TAKE_AFTER_SHUTDOWN)
    )

    sim_node.spin_until_shutdown(_Node(context_ok=False))


def test_simjax_1110_a_live_context_still_raises(monkeypatch) -> None:
    """The same error with the context still up is a real fault.

    A sim node that swallowed every exception would keep publishing a
    frozen state and report success, which is the failure shape this
    repository's gotchas are made of.
    """
    monkeypatch.setattr(
        sim_node.rclpy, "spin", _raising_spin(TAKE_AFTER_SHUTDOWN)
    )

    with pytest.raises(RuntimeError):
        sim_node.spin_until_shutdown(_Node(context_ok=True))
