"""Tier 1: what the AutoDRIVE backend declares (SIMAD-1240)."""

import subprocess
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_simad_1240_capabilities_are_the_free_running_set():
    """SIMAD-1240: AutoDRIVE is free_running, resets to its start pose only,
    and cannot step, scale time or start held (spec Context)."""
    from racing_sim_autodrive.capabilities import CAPABILITIES

    assert CAPABILITIES == {
        "capability.clock": "free_running",
        "capability.reset": "initial_pose_only",
        "capability.step_mode": False,
        "capability.time_scale": False,
        "capability.start_held": False,
    }


def test_simad_1240_import_is_standard_library_only():
    """SIMAD-1240: a launch file may read the capabilities without ROS,
    numpy or racing_common (the gym's module has the same property)."""
    code = (
        "import sys; import racing_sim_autodrive.capabilities; "
        "bad = [m for m in ('numpy', 'rclpy', 'racing_common', 'socketio') "
        "if m in sys.modules]; print(bad)"
    )
    out = subprocess.run(
        [sys.executable, "-c", code],
        cwd=PACKAGE_ROOT,
        env={"PYTHONPATH": str(PACKAGE_ROOT)},
        capture_output=True,
        text=True,
        check=True,
    )
    assert out.stdout.strip() == "[]"
