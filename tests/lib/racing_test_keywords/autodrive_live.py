"""Shared by the live and replayed AutoDRIVE tests (SIM-3610, SIM-3730)."""

from racing_test_keywords.scenario_runner import read_host_load


def judge_stale_input(
    latch, odom_stamps, timeout_s, stale_count, trace, end_latch_wait_s
):
    """None when the only STALE_INPUT is the end-of-stream latch, else why not.

    The window is anchored to the data, not to the replay process: the last
    /ground_truth/odom stamp is the last input the stack received. A latch
    stamped at or before it happened mid-stream. One stamped after it is the
    supervisor noticing that the input stopped (the commands stop with it).
    """
    if latch is None or stale_count == 0:
        return (
            f"no end-of-stream STALE_INPUT within {end_latch_wait_s} s "
            f"of the replay's exit (first non-NONE status {latch}): the "
            "supervisor is not judging the stream on wall time"
        )
    odom = sorted(odom_stamps)
    last = odom[-1]
    stamp = latch.header.stamp.sec + latch.header.stamp.nanosec * 1e-9
    if stamp > last:
        return None
    later = [i for i, t in enumerate(odom) if t > stamp]
    if later and later[0] > 0:
        index = later[0]
        gap = odom[index] - odom[index - 1]
        if gap >= timeout_s:
            return (
                f"stand-in stall, not a stack latch: the replay/server "
                f"delivered no /ground_truth/odom for {gap * 1000:.1f} ms "
                f"at index {index} (timeout {timeout_s * 1000:.0f} ms, host "
                f"load {read_host_load()['load1']}), and "
                f"STALE_INPUT x{stale_count} "
                f"latched at {stamp:.3f} inside it"
            )
    return (
        f"STALE_INPUT x{stale_count} mid-stream, {last - stamp:.3f} s before "
        f"the last /ground_truth/odom; first latch {latch}{trace}"
    )
