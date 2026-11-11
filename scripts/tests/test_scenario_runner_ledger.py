"""N1.2 / N1.5: what launch did to each process is part of the run's outcome
(CHECK-2370, CHECK-2371, CHECK-2377).

Real ``launch.LaunchService`` and real child processes (plain Python and
``sleep``, not ROS nodes: gotcha 10's stale graph does not apply), so the
event dispatch the ledger relies on is the real one. The nightly of
2026-11-05 lost SIM-3510's recording to a node launch had to SIGKILL, and the
log said only ``JSONDecodeError``.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import launch
import launch.actions
import pytest

LIB_ROOT = Path(__file__).resolve().parents[2] / "tests" / "lib"
if str(LIB_ROOT) not in sys.path:
    sys.path.insert(0, str(LIB_ROOT))

from racing_test_keywords import scenario_runner  # noqa: E402

# argv[1] is touched once both handlers are set: "started" in the ledger only
# means launch spawned the process, and a SIGINT that lands before the
# handlers would end it with -2 instead of the escalation under test.
STUBBORN = (
    "import signal,sys,time,pathlib; signal.signal(signal.SIGINT, "
    "signal.SIG_IGN); signal.signal(signal.SIGTERM, signal.SIG_IGN); "
    "pathlib.Path(sys.argv[1]).touch(); time.sleep(30)"
)
EARLY = "raise SystemExit(3)"


def _process(name: str, *argv: str):
    return launch.actions.ExecuteProcess(
        cmd=list(argv),
        name=name,
        sigterm_timeout="0.3",
        sigkill_timeout="0.3",
        output="log",
    )


def _stubborn(ready: Path):
    return _process("stubborn", sys.executable, "-c", STUBBORN, str(ready))


def _polite():
    return _process("polite", "sleep", "30")


def _early():
    return _process("early", sys.executable, "-c", EARLY)


def _wait_until(predicate, what: str, deadline: float = 30.0) -> None:
    """Poll to a bounded deadline; a load-proof stand-in for a fixed sleep."""
    end = time.monotonic() + deadline
    while not predicate():
        if time.monotonic() > end:
            raise AssertionError(f"timed out after {deadline}s: {what}")
        time.sleep(0.02)


def _ledger_exited(ledger, prefix: str):
    return lambda: any(n.startswith(prefix) for n in dict(ledger.exited))


def _run_launch(entities, ledger, ready=(), exited=()) -> int:
    """Run launch; shut it down once launch has started every entity, every
    ``ready`` file exists and every ``exited`` prefix is in the ledger."""
    description = launch.LaunchDescription([*ledger.actions(), *entities])
    service = launch.LaunchService()
    service.include_launch_description(description)

    def stop() -> None:
        ledger.shutting_down = True
        service.shutdown()

    def when_up() -> None:
        try:
            _wait_until(
                lambda: len(dict(ledger.started)) >= len(entities),
                "every stand-in started",
            )
            for path in ready:
                _wait_until(path.exists, f"{path} touched")
            for prefix in exited:
                _wait_until(_ledger_exited(ledger, prefix), f"{prefix} exited")
        finally:
            stop()

    thread = threading.Thread(target=when_up, daemon=True)
    thread.start()
    try:
        return service.run()
    finally:
        thread.join(timeout=35)


def test_check_2370_ledger_lists_only_the_sigkilled_process(tmp_path):
    ledger = scenario_runner._ProcessLedger()
    ready = tmp_path / "stubborn.ready"
    code = _run_launch(
        [_stubborn(ready), _polite(), _early()],
        ledger,
        ready=[ready],
        exited=["early"],
    )
    assert code == 0
    escalations = ledger.shutdown_escalations
    assert len(escalations) == 1, escalations
    assert escalations[0]["name"].startswith("stubborn")
    assert escalations[0]["signal"] == "SIGKILL"
    text = ledger.describe()
    assert "exited rc=3" in text
    assert "early" in text


def test_check_2370_a_malformed_event_never_fails_the_run():
    """A handler that raised would make launch shut a green run down."""
    ledger = scenario_runner._ProcessLedger()
    ledger._on_exit(SimpleNamespace(process_name="x"), None)  # no returncode
    ledger._on_start(SimpleNamespace(), None)  # no process_name
    assert "ledger error" in ledger.describe()
    assert _run_launch([_polite()], ledger) == 0


LAUNCH_FILE = """
import sys
import launch
import launch.actions


def _p(name, *argv):
    return launch.actions.ExecuteProcess(
        cmd=list(argv), name=name, sigterm_timeout="0.3",
        sigkill_timeout="0.3", output="log")


def generate_launch_description():
    return launch.LaunchDescription([ENTITIES])
"""


def _write_launch_file(path: Path, entities: str) -> Path:
    path.write_text(LAUNCH_FILE.replace("ENTITIES", entities))
    return path


def _stubborn_entity(ready: Path) -> str:
    return f'_p("stubborn", sys.executable, "-c", {STUBBORN!r}, {str(ready)!r})'


EARLY_ENTITY = f'_p("early", sys.executable, "-c", {EARLY!r})'
POLITE_ENTITY = '_p("polite", "sleep", "30")'


class _Observer:
    first_latch = None
    # F-T0's start record (a normal start); tests override on the class.
    held_seen = True
    tick = 0.01
    first_motion = 0.01
    metrics_start = 0.0

    def __init__(self, node) -> None:
        pass

    def describe(self) -> str:
        return "observer text"

    def close(self) -> None:
        pass


class _Node:
    def destroy_node(self) -> None:
        pass

    def get_node_names_and_namespaces(self):
        return [("racing_sim", "/")]


def _metrics_message() -> SimpleNamespace:
    return SimpleNamespace(
        **{
            field: (False if field == "lap_completed" else 0)
            for field in scenario_runner._METRICS_FIELDS
        }
    )


@pytest.fixture
def real_launch_run(tmp_path, monkeypatch):
    """The real ``run_scenario`` and ``LaunchService`` on a stand-in launch
    file, with rclpy, the t=0 reset and the metrics wait faked."""
    ready = tmp_path / "stubborn.ready"
    state = {"wait": None, "reset": None, "ledger": None, "reset_kwargs": None}

    class RecordingLedger(scenario_runner._ProcessLedger):
        def __init__(self) -> None:
            super().__init__()
            state["ledger"] = self

    def launch_file(entities: str) -> None:
        monkeypatch.setattr(
            scenario_runner,
            "BRINGUP_LAUNCH_FILE",
            _write_launch_file(tmp_path / "stand_in.launch.py", entities),
        )

    def wait_for_message(*args, **kwargs):
        _wait_until(ready.exists, "stubborn set its handlers")
        if state["wait"] is not None:
            raise state["wait"]
        return _metrics_message()

    def reset(node, seed, timeout, *args, **kwargs):
        state["reset_kwargs"] = kwargs
        if state["reset"] is not None:
            state["reset"]()
        return state.get("t0_waits")

    monkeypatch.setattr(scenario_runner, "ensure_rclpy_initialized", lambda: 0)
    monkeypatch.setattr(scenario_runner, "reset_rclpy", lambda: None)
    monkeypatch.setattr(
        scenario_runner.rclpy, "create_node", lambda *a, **k: _Node()
    )
    monkeypatch.setattr(scenario_runner, "_ProcessLedger", RecordingLedger)
    monkeypatch.setattr(scenario_runner, "_HangObserver", _Observer)
    monkeypatch.setattr(scenario_runner, "_reset_at_deterministic_t0", reset)
    monkeypatch.setattr(scenario_runner, "wait_for_message", wait_for_message)
    launch_file(_stubborn_entity(ready))
    return SimpleNamespace(
        state=state,
        launch_file=launch_file,
        stubborn=_stubborn_entity(ready),
    )


def test_check_2371_a_passing_run_reports_its_escalations(real_launch_run):
    outcome = scenario_runner.run_scenario(seed=1, timeout=30.0)
    assert [e["signal"] for e in outcome.shutdown_escalations] == ["SIGKILL"]
    assert outcome.shutdown_escalations[0]["name"].startswith("stubborn")
    assert set(outcome) == set(scenario_runner._METRICS_FIELDS)


def test_check_2564_run_scenario_hands_drive_on_estimate_to_t0(
    real_launch_run,
):
    scenario_runner.run_scenario(seed=1, timeout=30.0, drive_on_estimate=False)
    kwargs = real_launch_run.state["reset_kwargs"]
    assert kwargs["drive_on_estimate"] is False
    assert isinstance(kwargs["observer"], scenario_runner._HangObserver)


def test_check_2573_a_late_metrics_start_raises_start_missed(
    real_launch_run, monkeypatch
):
    monkeypatch.setattr(scenario_runner._HangObserver, "metrics_start", 0.43)
    with pytest.raises(scenario_runner.StartMissed) as raised:
        scenario_runner.run_scenario(seed=1, timeout=30.0)
    assert "verdict=metrics-start-missed" in str(raised.value)
    assert "0.43" in str(raised.value)


def test_check_2573_a_late_first_motion_raises_start_missed(
    real_launch_run, monkeypatch
):
    monkeypatch.setattr(scenario_runner._HangObserver, "first_motion", 0.05)
    with pytest.raises(scenario_runner.StartMissed) as raised:
        scenario_runner.run_scenario(seed=1, timeout=30.0)
    assert "verdict=drive-start-missed" in str(raised.value)


def test_check_2573_a_normal_start_rides_as_start_and_keeps_the_keys(
    real_launch_run,
):
    real_launch_run.state["t0_waits"] = {"metrics_started": 0.5}
    outcome = scenario_runner.run_scenario(seed=1, timeout=30.0)
    assert set(outcome) == set(scenario_runner._METRICS_FIELDS)
    assert outcome.start == {
        "metrics_start": 0.0,
        "first_motion": 0.01,
        "tick": 0.01,
        "t0_waits": {"metrics_started": 0.5},
    }


def test_check_2573_a_frozen_stop_with_a_normal_start_raises_nothing(
    real_launch_run,
):
    """lap_completed false and lap_time 0 (a DNF) must not read as a late
    start: the verdict comes from observer stamps, never lap_time."""
    message = _metrics_message()
    assert message.lap_completed is False
    assert message.lap_time == 0
    outcome = scenario_runner.run_scenario(seed=1, timeout=30.0)
    assert outcome["lap_completed"] is False
    assert outcome.start["metrics_start"] == 0.0


def test_check_2371_a_failure_carries_the_escalations(real_launch_run):
    message = "no message on /scenario/metrics within 1s"
    real_launch_run.state["wait"] = TimeoutError(message)
    with pytest.raises(TimeoutError) as raised:
        scenario_runner.run_scenario(seed=1, timeout=30.0)
    text = str(raised.value)
    assert text.startswith(message)
    assert "shutdown escalations: stubborn" in text
    assert "SIGKILL" in text


def test_check_2377_a_t0_stall_names_what_exited(real_launch_run):
    real_launch_run.launch_file(f"{EARLY_ENTITY}, {POLITE_ENTITY}")

    def stall() -> None:
        _wait_until(
            lambda: (
                real_launch_run.state["ledger"] is not None
                and _ledger_exited(real_launch_run.state["ledger"], "early")()
            ),
            "early exited",
        )
        raise TimeoutError("node not found within 1s: slam_toolbox")

    real_launch_run.state["reset"] = stall
    with pytest.raises(TimeoutError) as raised:
        scenario_runner.run_scenario(seed=1, timeout=30.0)
    text = str(raised.value)
    assert text.startswith("node not found within 1s: slam_toolbox")
    assert "exited rc=3" in text
    assert "early" in text
    assert "alive" in text
    assert "polite" in text
    assert "nodes: [/racing_sim]" in text
