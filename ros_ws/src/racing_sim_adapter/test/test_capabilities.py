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


def _has_guard(method: ast.FunctionDef, test_id: str) -> bool:
    """True when the method's first statement (after the docstring and any
    `self._require_adapter()`) is exactly `if not self._applicable(<id>):
    return`, and it asks about no other id. An inverted guard, an `else`, or
    a guard further down would pass a count of calls and never run the body."""
    if _applicable_calls(method) != [test_id]:
        return False
    body = list(method.body)
    if (
        body
        and isinstance(body[0], ast.Expr)
        and isinstance(body[0].value, ast.Constant)
    ):
        body = body[1:]
    while (
        body
        and isinstance(body[0], ast.Expr)
        and isinstance(body[0].value, ast.Call)
        and ast.unparse(body[0].value) == "self._require_adapter()"
    ):
        body = body[1:]
    if not body or not isinstance(body[0], ast.If):
        return False
    guard = body[0]
    return (
        ast.unparse(guard.test) == f"not self._applicable({test_id!r})"
        and len(guard.body) == 1
        and isinstance(guard.body[0], ast.Return)
        and guard.body[0].value is None
        and not guard.orelse
    )


def _parse_method(source: str) -> ast.FunctionDef:
    function = ast.parse(source).body[0]
    assert isinstance(function, ast.FunctionDef)
    return function


def test_adapt_1030_guard_shape_rejects_a_guard_that_would_not_run_the_body():
    """ADAPT-1030: the guard check accepts `if not self._applicable(own id):
    return` and rejects an inverted one (which returns early on the gym and
    reads as a pass), a wrong id, an `else`, and a late guard."""
    template = (
        "def test_adapt_2050_x(self):\n"
        '    """doc"""\n'
        "    self._require_adapter()\n"
        "{guard}"
        "    self.work()\n"
    )

    def guard(condition: str, body: str = "return") -> str:
        return f"    if {condition}:\n        {body}\n"

    good = guard("not self._applicable('ADAPT-2050')")
    assert _has_guard(_parse_method(template.format(guard=good)), "ADAPT-2050")
    bad = {
        "inverted": guard("self._applicable('ADAPT-2050')"),
        "wrong id": guard("not self._applicable('ADAPT-2070')"),
        "else": good + "    else:\n        pass\n",
        "no return": guard("not self._applicable('ADAPT-2050')", "pass"),
        "late": "    self.work()\n" + good,
        "absent": "",
    }
    for name, guard in bad.items():
        method = _parse_method(template.format(guard=guard))
        assert not _has_guard(method, "ADAPT-2050"), name


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
        if not _has_guard(methods[test_id][1], test_id)
    )
    assert not unguarded, f"gated without their own guard: {unguarded}"
    guarded = sorted(
        test_id for test_id in ungated if _applicable_calls(methods[test_id][1])
    )
    assert not guarded, f"ungated but guarded: {guarded}"


def _capabilities_by_path(path: Path) -> dict:
    """A backend's stdlib-only capabilities.py, loaded with no ROS and no
    package import (the gym's package pulls JAX in)."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.CAPABILITIES


SOURCES = Path(__file__).parents[2]
BACKEND_CAPABILITIES = {
    conformance.GYM: SOURCES
    / "racing_sim_gym_jax"
    / "racing_sim_gym_jax"
    / "capabilities.py",
    conformance.AUTODRIVE: SOURCES
    / "racing_sim_autodrive"
    / "racing_sim_autodrive"
    / "capabilities.py",
}


def _mismatches(backends: dict) -> dict[str, set[str]]:
    """Row name -> ids where its expected_not_applicable differs from what
    its capabilities module makes not applicable."""
    found = {}
    for name, row in backends.items():
        declared = _capabilities_by_path(BACKEND_CAPABILITIES[name])
        computed = set(
            conformance.not_applicable(conformance.REQUIREMENTS, declared)
        )
        if computed != set(row.expected_not_applicable):
            found[name] = computed ^ set(row.expected_not_applicable)
    return found


def test_adapt_1040_each_backend_row_expects_what_its_capabilities_imply():
    """ADAPT-1040: for every BACKENDS row, expected_not_applicable equals
    not_applicable(REQUIREMENTS, that backend's capabilities module). A row
    cannot drift from what the backend declares, and an extra id is named."""
    from dataclasses import replace

    assert set(conformance.BACKENDS) == set(BACKEND_CAPABILITIES)
    assert _mismatches(conformance.BACKENDS) == {}
    extra = dict(conformance.BACKENDS)
    extra[conformance.AUTODRIVE] = replace(
        extra[conformance.AUTODRIVE],
        expected_not_applicable=extra[
            conformance.AUTODRIVE
        ].expected_not_applicable
        | {"ADAPT-2010"},
    )
    assert _mismatches(extra) == {conformance.AUTODRIVE: {"ADAPT-2010"}}


def test_adapt_1040_backend_selection_defaults_to_the_gym_and_rejects_unknown(
    monkeypatch,
):
    """ADAPT-1040: RACING_SIM_BACKEND unset is the gym; a name not in the
    table raises rather than silently running the gym."""
    import pytest

    monkeypatch.delenv("RACING_SIM_BACKEND", raising=False)
    assert conformance.backend().name == conformance.GYM
    monkeypatch.setenv("RACING_SIM_BACKEND", conformance.AUTODRIVE)
    assert conformance.backend().name == conformance.AUTODRIVE
    monkeypatch.setenv("RACING_SIM_BACKEND", "nope")
    with pytest.raises(ValueError, match="nope"):
        conformance.backend()


CMAKE = Path(__file__).parents[1] / "CMakeLists.txt"


def _launch_registrations(text: str) -> dict[str, list[str]]:
    """CTest name of every add_launch_test -> the RACING_SIM_BACKEND values
    set on it by a `set_property(TEST <name> ... ENVIRONMENT ...)`."""
    names = []
    for call in re.findall(r"add_launch_test\(([^)]*)\)", text):
        target = re.search(r"TARGET\s+(\S+)", call)
        source = re.search(r"test/(\S+\.py)", call).group(1)
        names.append(target.group(1) if target else f"test_{source}")
    backends: dict[str, list[str]] = {name: [] for name in names}
    for name, tail in re.findall(
        r"set_property\(TEST\s+(\S+)\s+APPEND PROPERTY ENVIRONMENT\s+"
        r"([^)]*)\)",
        text,
    ):
        backends.setdefault(name, []).extend(
            re.findall(r"RACING_SIM_BACKEND=(\S+)", tail)
        )
    return {n: backends[n] for n in names}


def test_adapt_1050_every_launch_registration_runs_the_backend_it_names():
    """ADAPT-1050: each add_launch_test carries exactly one
    RACING_SIM_BACKEND, `autodrive` for a target ending `_autodrive` and
    `f1tenth_gym_jax` for the others. add_launch_test has no ENV argument, so
    a lost property would silently rerun the gym under an `_autodrive` name
    and pass (seen before the property route)."""
    found = _launch_registrations(CMAKE.read_text())
    assert len(found) == 8
    for name, values in found.items():
        want = (
            conformance.AUTODRIVE
            if name.endswith("_autodrive")
            else conformance.GYM
        )
        assert values == [want], name


def test_adapt_1050_negative_a_lost_property_is_reported():
    """ADAPT-1050 negative: the scan is not vacuous."""
    text = (
        "add_launch_test(test/test_adapter_contract.py\n"
        "  TARGET test_adapter_contract_autodrive TIMEOUT 30)\n"
        "set_property(TEST test_adapter_contract_autodrive APPEND PROPERTY"
        " ENVIRONMENT\n  RACING_SIM_BACKEND=autodrive)\n"
        "add_launch_test(test/test_adapter_start_held.py TIMEOUT 30)\n"
    )
    assert _launch_registrations(text) == {
        "test_adapter_contract_autodrive": ["autodrive"],
        "test_test_adapter_start_held.py": [],
    }


_NAME_SHAPED = re.compile(r"/[A-Za-z_~][\w/~]*")


def _is_topic_call(node: ast.AST) -> bool:
    """`topic(...)` or `<anything>.topic(...)`: the sanctioned way to name
    a topic."""
    if not isinstance(node, ast.Call):
        return False
    func = node.func
    return (isinstance(func, ast.Name) and func.id == "topic") or (
        isinstance(func, ast.Attribute) and func.attr == "topic"
    )


def _global_names(source: str) -> list[str]:
    """Every name-shaped absolute string in `source`: a str literal that is
    `/name`-shaped, or an f-string whose first part is a literal starting `/`.
    Not reported: a first argument of `topic(...)`, the inner parts of an
    f-string (it is judged once, by its first part), and prose such as
    "/clock must advance" (not name-shaped)."""
    tree = ast.parse(source)
    skipped: set[int] = set()
    found: list[str] = []
    for node in ast.walk(tree):
        if _is_topic_call(node) and node.args:
            skipped.add(id(node.args[0]))
        elif isinstance(node, ast.JoinedStr):
            for part in node.values:
                skipped.add(id(part))
            first = node.values[0] if node.values else None
            if (
                isinstance(first, ast.Constant)
                and isinstance(first.value, str)
                and first.value.startswith("/")
            ):
                found.append(ast.unparse(node))
        elif (
            isinstance(node, ast.Constant)
            and isinstance(node.value, str)
            and id(node) not in skipped
            and _NAME_SHAPED.fullmatch(node.value)
        ):
            found.append(node.value)
    return found


def test_adapt_1060_no_conformance_file_names_a_global_topic_or_service():
    """ADAPT-1060: no test_adapter_*.py names a topic or service by a global
    literal; they go through conformance.topic()/service() so the row's
    namespace applies. This is the receiving side of ADAPT-2170 (the test
    nodes hold no lasting subscription a live probe could see). Known false
    negative, accepted: a relative name ("scan") on the root-namespace test
    node also resolves globally; it fails functionally (no data under the
    prefix) instead.

    Other known blind spots, also accepted: remap strings such as
    "/clock:=/x"; names built as "/" + name; f-strings that start with a
    variable; and global names reached through conformance constants such
    as CLOCK_TOPIC."""
    paths = sorted(Path(__file__).parent.glob("test_adapter_*.py"))
    assert len(paths) == 4, paths
    leaks = {path.name: _global_names(path.read_text()) for path in paths}
    assert {name: names for name, names in leaks.items() if names} == {}


def test_adapt_1060_negative_a_planted_global_name_is_reported():
    """ADAPT-1060 negative: the scan is not vacuous."""
    source = (
        'X = "/scan"\n'
        'Y = f"/{a}/reset"\n'
        'Z = conformance.topic("/odom")\n'
        'M = "/clock must advance"\n'
    )
    assert _global_names(source) == ["/scan", "f'/{a}/reset'"]
