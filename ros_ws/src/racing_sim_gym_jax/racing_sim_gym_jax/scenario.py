"""ROS-free scenario parsing and gym environment-id assembly."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

VALID_LONGITUDINAL = frozenset({"acceleration", "velocity"})
VALID_STEERING = frozenset({"steeringangle", "steeringvelocity"})
VALID_REWARDS = frozenset({"alive", "progress", "time"})


@dataclass(frozen=True)
class Scenario:
    """Immutable inputs that define one compiled gym artifact."""

    scenario_id: str
    map_name: str
    num_agents: int
    scans: bool
    collisions: bool
    rewards: tuple[str, ...]
    longitudinal: str
    steering: str
    timestep_ratio_value: int | None
    max_steps: int | None
    version: str
    parameters: dict[str, Any]
    track_path: Path
    map_directory: Path

    @property
    def timestep_ratio(self) -> int:
        """Return the effective ratio, including the legacy arity default."""
        return self.timestep_ratio_value or 1

    @property
    def control_period(self) -> float:
        return (
            float(self.parameters.get("timestep", 0.01)) * self.timestep_ratio
        )

    @property
    def env_id(self) -> str:
        """Assemble one of the three upstream-supported v0 arities."""
        scan = "scan" if self.scans else "noscan"
        collision = "collision" if self.collisions else "nocollision"
        fields = [
            self.map_name,
            str(self.num_agents),
            scan,
            collision,
            "+".join(self.rewards),
            f"{self.longitudinal}+{self.steering}",
        ]
        if self.timestep_ratio_value is not None:
            fields.append(str(self.timestep_ratio_value))
        if self.max_steps is not None:
            if self.timestep_ratio_value is None:
                raise ValueError(
                    "max_steps requires an explicit timestep_ratio"
                )
            fields.append(str(self.max_steps))
        fields.append(self.version)
        return "_".join(fields)


def _required(mapping: dict[str, Any], name: str) -> Any:
    if name not in mapping:
        raise ValueError(f"scenario is missing {name}")
    return mapping[name]


def load_scenario(path: Path) -> Scenario:
    """Load and validate a scenario without importing ROS or JAX."""
    path = path.resolve()
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or document.get("format_version") != 1:
        raise ValueError("scenario format_version must be 1")
    environment = _required(document, "environment")
    if not isinstance(environment, dict):
        raise ValueError("environment must be a mapping")
    action = _required(environment, "action")
    if not isinstance(action, dict):
        raise ValueError("environment.action must be a mapping")

    rewards_value = _required(environment, "rewards")
    rewards = (
        tuple(rewards_value)
        if isinstance(rewards_value, list)
        else tuple(str(rewards_value).split("+"))
    )
    if not rewards or not set(rewards).issubset(VALID_REWARDS):
        raise ValueError(
            "rewards must be a non-empty subset of alive/progress/time"
        )
    longitudinal = str(_required(action, "longitudinal"))
    steering = str(_required(action, "steering"))
    if longitudinal not in VALID_LONGITUDINAL:
        raise ValueError(f"unsupported longitudinal action: {longitudinal}")
    if steering not in VALID_STEERING:
        raise ValueError(f"unsupported steering action: {steering}")

    version = str(_required(environment, "version"))
    if version != "v0":
        raise ValueError("environment version must be v0")
    num_agents = int(_required(environment, "num_agents"))
    if num_agents < 1:
        raise ValueError("num_agents must be positive")
    ratio_raw = environment.get("timestep_ratio")
    ratio = None if ratio_raw is None else int(ratio_raw)
    if ratio is not None and ratio < 1:
        raise ValueError("timestep_ratio must be positive")
    max_steps_raw = environment.get("max_steps")
    max_steps = None if max_steps_raw is None else int(max_steps_raw)
    if max_steps is not None and max_steps < 1:
        raise ValueError("max_steps must be positive")

    parameters = document.get("parameters", {})
    if not isinstance(parameters, dict):
        raise ValueError("parameters must be a mapping")
    track_path = (path.parent / str(_required(document, "track"))).resolve()
    map_directory = (
        path.parent / str(_required(document, "map_directory"))
    ).resolve()
    return Scenario(
        scenario_id=str(_required(document, "scenario_id")),
        map_name=str(_required(environment, "map")),
        num_agents=num_agents,
        scans=bool(_required(environment, "scans")),
        collisions=bool(_required(environment, "collisions")),
        rewards=rewards,
        longitudinal=longitudinal,
        steering=steering,
        timestep_ratio_value=ratio,
        max_steps=max_steps,
        version=version,
        parameters=dict(parameters),
        track_path=track_path,
        map_directory=map_directory,
    )
