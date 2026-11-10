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

STUBBORN = (
    "import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); "
    "signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(30)"
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


def _stubborn():
    return _process("stubborn", sys.executable, "-c", STUBBORN)


def _polite():
    return _process("polite", "sleep", "30")


def _early():
    return _process("early", sys.executable, "-c", EARLY)


def _run_launch(entities, ledger, shutdown_after=1.0) -> int:
    description = launch.LaunchDescription([*ledger.actions(), *entities])
    service = launch.LaunchService()
    service.include_launch_description(description)

    def stop() -> None:
        ledger.shutting_down = True
        service.shutdown()

    timer = threading.Timer(shutdown_after, stop)
    timer.start()
    try:
        return service.run()
    finally:
        timer.cancel()


def test_check_2370_ledger_lists_only_the_sigkilled_process():
    ledger = scenario_runner._ProcessLedger()
    code = _run_launch([_stubborn(), _polite(), _early()], ledger)
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


STUBBORN_ENTITY = f'_p("stubborn", sys.executable, "-c", {STUBBORN!r})'
EARLY_ENTITY = f'_p("early", sys.executable, "-c", {EARLY!r})'
POLITE_ENTITY = '_p("polite", "sleep", "30")'


class _Observer:
    first_latch = None

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
    state = {"entities": STUBBORN_ENTITY, "wait": None, "reset": None}

    def launch_file(entities: str) -> None:
        monkeypatch.setattr(
            scenario_runner,
            "BRINGUP_LAUNCH_FILE",
            _write_launch_file(tmp_path / "stand_in.launch.py", entities),
        )

    def wait_for_message(*args, **kwargs):
        time.sleep(1.5)  # the stand-ins are up and have set their handlers
        if state["wait"] is not None:
            raise state["wait"]
        return _metrics_message()

    def reset(node, seed, timeout, *args, **kwargs):
        if state["reset"] is not None:
            state["reset"]()

    monkeypatch.setattr(scenario_runner, "ensure_rclpy_initialized", lambda: 0)
    monkeypatch.setattr(scenario_runner, "reset_rclpy", lambda: None)
    monkeypatch.setattr(
        scenario_runner.rclpy, "create_node", lambda *a, **k: _Node()
    )
    monkeypatch.setattr(scenario_runner, "_HangObserver", _Observer)
    monkeypatch.setattr(scenario_runner, "_reset_at_deterministic_t0", reset)
    monkeypatch.setattr(scenario_runner, "wait_for_message", wait_for_message)
    launch_file(STUBBORN_ENTITY)
    return SimpleNamespace(state=state, launch_file=launch_file)


def test_check_2371_a_passing_run_reports_its_escalations(real_launch_run):
    outcome = scenario_runner.run_scenario(seed=1, timeout=30.0)
    assert [e["signal"] for e in outcome.shutdown_escalations] == ["SIGKILL"]
    assert outcome.shutdown_escalations[0]["name"].startswith("stubborn")
    assert set(outcome) == set(scenario_runner._METRICS_FIELDS)


def test_check_2371_a_failure_carries_the_escalations(real_launch_run):
    message = "no message on /scenario/metrics within 1s"
    real_launch_run.state["wait"] = TimeoutError(message)
    with pytest.raises(TimeoutError) as raised:
        scenario_runner.run_scenario(seed=1, timeout=30.0)
    text = str(raised.value)
    assert text.startswith(message)
    assert "shutdown escalations: stubborn" in text
    assert "SIGKILL" in text
