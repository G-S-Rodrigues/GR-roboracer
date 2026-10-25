"""Tier-1 checks of the capability vocabulary (no ROS graph)."""

import ast
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import conformance  # noqa: E402

HEADER = (
    Path(__file__).parents[1]
    / "include"
    / "racing_sim_adapter"
    / "contract.hpp"
)


def _header_constants() -> dict[str, str]:
    """Every `kCapability*` / `kClock*` / `kReset*` string constant."""
    pattern = re.compile(
        r"inline\s+constexpr\s+std::string_view\s+"
        r"(k(?:Capability|Clock|Reset)\w*)\s*\{\s*\"([^\"]*)\"\s*\}"
    )
    return dict(pattern.findall(HEADER.read_text()))


def test_adapt_1020_python_vocabulary_matches_the_contract_header() -> None:
    """ADAPT-1020: the Python mirror of the capability names equals
    contract.hpp's, so the two cannot drift apart unseen."""
    constants = _header_constants()
    parameters = {
        value
        for name, value in constants.items()
        if name.startswith("kCapability") and name.endswith("Parameter")
    }
    clocks = {v for k, v in constants.items() if k.startswith("kClock")}
    resets = {
        v
        for k, v in constants.items()
        if k.startswith("kReset") and not k.endswith("Service")
    }
    assert parameters == set(conformance.CAPABILITY_PARAMETERS)
    assert clocks == set(conformance.CLOCK_VALUES)
    assert resets == set(conformance.RESET_VALUES)
    assert len(parameters) == 5


GYM_DECLARATION = {
    conformance.CLOCK_PARAMETER: conformance.OWNS_CLOCK,
    conformance.RESET_PARAMETER: conformance.SEEDED_POSE,
    conformance.STEP_MODE_PARAMETER: True,
    conformance.TIME_SCALE_PARAMETER: True,
    conformance.START_HELD_PARAMETER: True,
}
BARE_DECLARATION = {
    conformance.CLOCK_PARAMETER: conformance.FREE_RUNNING,
    conformance.RESET_PARAMETER: conformance.INITIAL_POSE_ONLY,
    conformance.STEP_MODE_PARAMETER: False,
    conformance.TIME_SCALE_PARAMETER: False,
    conformance.START_HELD_PARAMETER: False,
}


def test_adapt_1010_not_applicable_follows_the_declaration() -> None:
    """ADAPT-1010: a backend with every capability makes no test not
    applicable; one with none makes exactly the gated set so, and each
    reason names the test and the capability it lacks."""
    requirements = conformance.REQUIREMENTS
    assert conformance.not_applicable(requirements, GYM_DECLARATION) == {}
    missing = conformance.not_applicable(requirements, BARE_DECLARATION)
    assert set(missing) == set(requirements)
    reason = missing["ADAPT-2070"]
    assert "ADAPT-2070" in reason
    assert conformance.CLOCK_PARAMETER in reason
    assert conformance.OWNS_CLOCK in reason
    assert conformance.FREE_RUNNING in reason
    only_clock = dict(GYM_DECLARATION)
    only_clock[conformance.CLOCK_PARAMETER] = conformance.FREE_RUNNING
    clockless = conformance.not_applicable(requirements, only_clock)
    expected = {
        test_id
        for test_id, needs in requirements.items()
        if conformance.CLOCK_PARAMETER in needs
    }
    assert set(clockless) == expected
    assert "ADAPT-2050" not in clockless


def _launch_file_methods() -> dict[str, tuple[str, ast.FunctionDef]]:
    """ADAPT id -> (file name, test method) over the launch_test files."""
    found: dict[str, tuple[str, ast.FunctionDef]] = {}
    for path in sorted(Path(__file__).parent.glob("test_adapter_*.py")):
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if not isinstance(node, ast.FunctionDef):
                continue
            match = re.match(r"test_adapt_(\d{4})_", node.name)
            if match:
                found[f"ADAPT-{match.group(1)}"] = (path.name, node)
    return found


def _applicable_calls(method: ast.FunctionDef) -> list[str]:
    """The literal ids passed to `self._applicable(...)` in a method."""
    ids = []
    for node in ast.walk(method):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "_applicable"
        ):
            ids.append(ast.literal_eval(node.args[0]))
    return ids


def test_adapt_1030_every_conformance_test_is_classified_and_guarded() -> None:
    """ADAPT-1030: each ADAPT-2xxx test is in REQUIREMENTS or UNGATED, never
    both; a gated one asks `_applicable` with its own id, an ungated one does
    not ask at all. A test added without a classification cannot go unseen."""
    methods = _launch_file_methods()
    gated = set(conformance.REQUIREMENTS)
    ungated = set(conformance.UNGATED)
    assert not gated & ungated, f"both gated and ungated: {gated & ungated}"
    unclassified = sorted(set(methods) - gated - ungated)
    assert not unclassified, f"unclassified: {unclassified}"
    stale = sorted((gated | ungated) - set(methods))
    assert not stale, f"classified but no such test: {stale}"
    unguarded = sorted(
        test_id
        for test_id in gated
        if _applicable_calls(methods[test_id][1]) != [test_id]
    )
    assert not unguarded, f"gated without their own guard: {unguarded}"
    guarded = sorted(
        test_id for test_id in ungated if _applicable_calls(methods[test_id][1])
    )
    assert not guarded, f"ungated but guarded: {guarded}"
