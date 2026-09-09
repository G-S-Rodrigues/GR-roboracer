"""Seeded, ROS-free f1tenth_gym_jax rollout harness."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform as host_platform
import subprocess
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

import numpy as np

from sim.track_importer import import_track

ActionProvider = Callable[[Mapping[str, Any], Any], tuple[float, float]]


def pure_pursuit_action_provider(
    canonical_track: Path,
    *,
    lookahead_distance: float,
    wheelbase: float,
    minimum_speed: float,
    maximum_speed: float,
    curvature_speed_gain: float,
) -> ActionProvider:
    """Drive with the authoritative C++ Pure Pursuit implementation."""
    import racing_controller_baseline
    import yaml

    document = yaml.safe_load(canonical_track.read_text(encoding="utf-8"))
    trajectory = [
        racing_controller_baseline.TrajectoryPoint(float(row[0]), float(row[1]))
        for row in document["centerline"]
    ]
    params = racing_controller_baseline.ControllerParams()
    params.lookahead_distance = lookahead_distance
    params.wheelbase = wheelbase
    params.minimum_speed = minimum_speed
    params.maximum_speed = maximum_speed
    params.curvature_speed_gain = curvature_speed_gain
    params.closed_trajectory = bool(document["metadata"]["closed"])

    def provide(
        _observation: Mapping[str, Any], state: Any
    ) -> tuple[float, float]:
        cartesian = np.asarray(state.cartesian_states)[0]
        vehicle = racing_controller_baseline.VehicleState(
            float(cartesian[0]), float(cartesian[1]), float(cartesian[4])
        )
        command = racing_controller_baseline.PurePursuit.compute(
            vehicle, trajectory, params
        )
        return float(command.steering_angle), float(command.speed)

    return provide


def _revision(repo_root: Path) -> str:
    return subprocess.run(
        [
            "git",
            "-c",
            f"safe.directory={repo_root}",
            "rev-parse",
            "HEAD",
        ],
        cwd=repo_root,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()


def _file_version(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _percentile_95(values: list[float]) -> float:
    return float(np.percentile(np.asarray(values), 95)) if values else 0.0


def run_seeded_scenario(
    canonical_track: Path,
    maps_root: Path,
    *,
    scenario_id: str,
    seed: int,
    timestep_ratio: int,
    max_steps: int,
    image_digest: str,
    vehicle_parameter_version: str,
    action_provider: ActionProvider,
) -> dict[str, Any]:
    """Run one unbatched episode and return ScenarioMetrics-compatible JSON."""
    if not image_digest:
        raise ValueError("container image digest is required for attribution")
    if timestep_ratio < 1 or max_steps < 1:
        raise ValueError("timestep_ratio and max_steps must be positive")

    import f1tenth_gym_jax
    import jax
    import jax.numpy as jnp
    import racing_common

    if jax.default_backend() != "gpu":
        raise RuntimeError("headless rollout requires the fast-sim GPU backend")

    track_dir = import_track(canonical_track, maps_root)
    os.environ["F1TENTH_GYM_JAX_MAP_DIR"] = str(maps_root)
    map_name = track_dir.name
    env_id = (
        f"{map_name}_1_scan_collision_progress_"
        f"velocity+steeringangle_{timestep_ratio}_{max_steps}_v0"
    )
    env = f1tenth_gym_jax.make(env_id)
    shared_track = racing_common.Track.from_yaml(canonical_track)

    key = jax.random.PRNGKey(seed)
    observation, state = env.reset(key)
    tracking_errors: list[float] = []
    wall_clearances: list[float] = []
    collision_count = 0
    saturation_events = 0
    was_colliding = False

    for _ in range(max_steps):
        steering, speed = action_provider(observation, state)
        requested = np.asarray([steering, speed], dtype=float)
        action_space = env.action_spaces["agent_0"]
        clipped = np.clip(requested, action_space.low, action_space.high)
        saturation_events += int(
            np.any(requested < action_space.low)
            or np.any(requested > action_space.high)
        )

        key, step_key = jax.random.split(key)
        observation, state, _, dones, _ = env.step_env(
            step_key,
            state,
            {"agent_0": jnp.asarray(clipped)},
        )
        cartesian = np.asarray(state.cartesian_states)[0]
        frenet = shared_track.to_frenet(
            racing_common.CartesianPose(
                float(cartesian[0]), float(cartesian[1]), float(cartesian[4])
            )
        )
        tracking_errors.append(abs(float(frenet.d)))
        wall_clearances.append(float(np.min(np.asarray(state.scans)[0])))

        is_colliding = bool(np.asarray(state.collisions)[0])
        collision_count += int(is_colliding and not was_colliding)
        was_colliding = is_colliding
        if bool(dones["__all__"]):
            break

    elapsed_steps = int(np.asarray(state.step))
    simulated_time = elapsed_steps * float(env.params.timestep) * timestep_ratio
    lap_completed = bool(np.asarray(state.num_laps)[0] >= 1)
    repo_root = Path(__file__).resolve().parents[1]
    return {
        "header": {
            "stamp": {
                "sec": int(simulated_time),
                "nanosec": int((simulated_time % 1.0) * 1_000_000_000),
            },
            "frame_id": "map",
        },
        "source": "headless_sim",
        "scenario_id": scenario_id,
        "seed": seed,
        "timestep_ratio": timestep_ratio,
        "lap_completed": lap_completed,
        "lap_time": simulated_time if lap_completed else 0.0,
        "collision_count": collision_count,
        "minimum_wall_clearance": min(wall_clearances, default=0.0),
        "maximum_tracking_error": max(tracking_errors, default=0.0),
        "p95_tracking_error": _percentile_95(tracking_errors),
        "control_saturation_events": saturation_events,
        "code_revision": _revision(repo_root),
        "container_image_digest": image_digest,
        "track_version": _file_version(canonical_track),
        "vehicle_parameter_version": vehicle_parameter_version,
        "platform": {
            "system": host_platform.system(),
            "machine": host_platform.machine(),
            "python": host_platform.python_version(),
            "jax_backend": jax.default_backend(),
            "jax_version": jax.__version__,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--track", type=Path, required=True)
    parser.add_argument("--maps-root", type=Path, required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--timestep-ratio", type=int, default=1)
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--image-digest", required=True)
    parser.add_argument("--vehicle-parameter-version", required=True)
    parser.add_argument("--lookahead-distance", type=float, default=1.5)
    parser.add_argument("--wheelbase", type=float, default=0.33)
    parser.add_argument("--minimum-speed", type=float, default=1.0)
    parser.add_argument("--maximum-speed", type=float, default=3.0)
    parser.add_argument("--curvature-speed-gain", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    metrics = run_seeded_scenario(
        args.track,
        args.maps_root,
        scenario_id=args.scenario,
        seed=args.seed,
        timestep_ratio=args.timestep_ratio,
        max_steps=args.max_steps,
        image_digest=args.image_digest,
        vehicle_parameter_version=args.vehicle_parameter_version,
        action_provider=pure_pursuit_action_provider(
            args.track,
            lookahead_distance=args.lookahead_distance,
            wheelbase=args.wheelbase,
            minimum_speed=args.minimum_speed,
            maximum_speed=args.maximum_speed,
            curvature_speed_gain=args.curvature_speed_gain,
        ),
    )
    rendered = json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
