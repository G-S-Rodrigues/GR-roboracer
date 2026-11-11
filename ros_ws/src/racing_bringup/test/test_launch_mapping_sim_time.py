"""Tier 1: the two mapping launches on a simulator that has no /clock
(1V-fix-b step b3, BRINGUP-1980..1982).

`slam_toolbox_mapping.launch.py` and `mola_mapping.launch.py` ran only on the
gym, where `use_sim_time` is True. On AutoDRIVE (free-running, ADR 0006) that
waits on a /clock nobody publishes. Both now take the value from the
simulator's `capability.clock` through `racing_bringup.simulators.sim_time_for`,
the same rule `sim_pure_pursuit.launch.py` applies to every other node
(BRINGUP-1630). The defaults keep the gym exactly as it was.
"""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from test_bringup_contract import REPOSITORY_ROOT, _merged

LAUNCH_DIR = Path(__file__).parents[1] / "launch"
SURVEY = "config/localization/slam_toolbox_survey.yaml"
ONLINE = "config/localization/slam_toolbox_online.yaml"


@pytest.fixture(autouse=True)
def _repository_root(monkeypatch):
    monkeypatch.chdir(REPOSITORY_ROOT)


def _module(name: str):
    spec = importlib.util.spec_from_file_location(
        f"racing_{name}_launch", LAUNCH_DIR / f"{name}.launch.py"
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _context(module, **arguments: str) -> LaunchContext:
    """A context holding every declared argument's default, then
    `arguments`: what `ros2 launch` hands the OpaqueFunction."""
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if getattr(entity, "default_value", None) is not None:
            context.launch_configurations[entity.name] = "".join(
                part.text for part in entity.default_value
            )
    context.launch_configurations.update(arguments)
    return context


def _slam_nodes(**arguments: str) -> tuple[LaunchContext, list]:
    module = _module("slam_toolbox_mapping")
    context = _context(module, **arguments)
    actions: list = []
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, OpaqueFunction):
            actions.extend(entity.execute(context) or [])
    nodes = [a for a in actions if hasattr(a, "_Node__package")]
    assert [n._Node__package for n in nodes] == ["slam_toolbox"]
    return context, nodes


def _parameter_files(node, context) -> list[str]:
    return [
        "".join(part.text for part in entry.param_file)
        for entry in node._Node__parameters
        if not isinstance(entry, dict)
    ]


def test_bringup_1980_sim_time_for_and_both_mapping_launches() -> None:
    """BRINGUP-1980: `sim_time_for` is True for the gym and False for
    AutoDRIVE and raises ValueError naming an unknown simulator; the
    slam_toolbox mapping launch and MOLA's mapping launch both read
    `simulator` (default the gym) and hand that value to their node:
    True by default, False under `simulator:=autodrive`."""
    from racing_bringup.simulators import sim_time_for

    assert sim_time_for("f1tenth_gym_jax") is True
    assert sim_time_for("autodrive") is False
    with pytest.raises(ValueError, match="nonesuch"):
        sim_time_for("nonesuch")

    for simulator, expected in (
        (None, True),
        ("f1tenth_gym_jax", True),
        ("autodrive", False),
    ):
        arguments = {} if simulator is None else {"simulator": simulator}
        context, nodes = _slam_nodes(**arguments)
        assert _merged(context, nodes[0])["use_sim_time"] is expected, (
            "slam_toolbox",
            simulator,
        )

        module = _module("mola_mapping")
        context = _context(
            module, simplemap_output="/x/out.simplemap", **arguments
        )
        actions = []
        for entity in module.generate_launch_description().entities:
            if isinstance(entity, OpaqueFunction):
                actions.extend(entity.execute(context) or [])
        include = next(
            a for a in actions if isinstance(a, IncludeLaunchDescription)
        )
        value = dict(include.launch_arguments)["use_sim_time"]
        assert value == str(expected), ("mola", simulator, value)


def test_bringup_1981_slam_toolbox_config_argument() -> None:
    """BRINGUP-1981: `slam_toolbox_mapping.launch.py` declares
    `slam_toolbox_config`, defaulting to the survey yaml (today's file); an
    override is the file the node gets."""
    module = _module("slam_toolbox_mapping")
    declared = {
        e.name: "".join(part.text for part in e.default_value)
        for e in module.generate_launch_description().entities
        if hasattr(e, "default_value")
    }
    assert declared.get("slam_toolbox_config") == SURVEY, declared
    context, nodes = _slam_nodes()
    assert [Path(f).name for f in _parameter_files(nodes[0], context)] == [
        Path(SURVEY).name
    ]
    context, nodes = _slam_nodes(slam_toolbox_config=ONLINE)
    assert [Path(f).name for f in _parameter_files(nodes[0], context)] == [
        Path(ONLINE).name
    ]
