"""Tier-1 checks of the capability vocabulary (no ROS graph)."""

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
