"""BRINGUP-1070: the support node stops cleanly on the launcher's SIGINT."""

import signal

import pytest
from racing_bringup import support_node

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


def test_bringup_1070_sigint_does_not_raise_into_a_c_call() -> None:
    """SIGINT must not raise KeyboardInterrupt while rclpy is inside a call.

    Python's default handler raises at the next bytecode boundary, which
    for a spinning node is usually inside a pybind11 call: `take_message`
    then fails to convert its argument with an error already pending and
    surfaces as `RuntimeError: Unable to convert call argument '0' to
    Python object`. The node exits 1 and launch reports `process has
    died` after a run whose results were complete - measured on every
    pose source, including ground truth. rclpy's own handler, which wakes
    the executor between calls rather than inside one, is left to do the
    stopping.
    """
    previous = signal.getsignal(signal.SIGINT)
    try:
        support_node.let_rclpy_own_sigint()
        handler = signal.getsignal(signal.SIGINT)

        assert handler is not signal.default_int_handler
        assert callable(handler)
        # The whole point: invoking it raises nothing.
        assert handler(signal.SIGINT, None) is None
    finally:
        signal.signal(signal.SIGINT, previous)


def test_bringup_1070_take_after_shutdown_is_a_clean_stop(monkeypatch) -> None:
    """A take that fails once the context is gone exits 0, not 1.

    The second half of the same guard: whatever order rclpy tears its
    handles down in, an error raised after the context is down is the
    shutdown, not a fault.
    """
    monkeypatch.setattr(
        support_node.rclpy, "spin", _raising_spin(TAKE_AFTER_SHUTDOWN)
    )

    support_node.spin_until_shutdown(_Node(context_ok=False))


def test_bringup_1070_a_live_context_still_raises(monkeypatch) -> None:
    """The same error with the context still up is a real fault.

    Swallowing it unconditionally would turn every runtime error in a
    callback into a silent, zero-exit run - the shape of failure this
    repository's gotchas are all made of. The context being down is what
    makes it a shutdown race rather than a bug.
    """
    monkeypatch.setattr(
        support_node.rclpy, "spin", _raising_spin(TAKE_AFTER_SHUTDOWN)
    )

    with pytest.raises(RuntimeError):
        support_node.spin_until_shutdown(_Node(context_ok=True))
