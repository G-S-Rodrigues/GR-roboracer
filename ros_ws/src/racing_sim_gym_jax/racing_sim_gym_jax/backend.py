"""ROS-free owner of one warmed f1tenth_gym_jax environment."""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from functools import partial

import jax
import jax.numpy as jnp
import numpy as np
import racing_common
from f1tenth_gym_jax import make

from .laser import LaserMount, laser_poses
from .scenario import Scenario

# A laser at base_link: the scan is the gym's own, cast once.
BASE_LINK = LaserMount()


@dataclass(frozen=True)
class DriveCommand:
    steering_angle: float = 0.0
    steering_angle_velocity: float = 0.0
    speed: float = 0.0
    acceleration: float = 0.0


@dataclass(frozen=True)
class Snapshot:
    x: float
    y: float
    steering_angle: float
    speed: float
    yaw: float
    yaw_rate: float
    # Ray-marched from base_link: /ground_truth/scan, what racing_metrics'
    # clearance and collisions read.
    scan: np.ndarray
    s: float
    d: float
    heading_error: float
    v_s: float
    v_d: float
    collision: bool
    done: bool
    # Ray-marched from the laser mount: /scan, what an estimator reads in
    # frame `laser`. The same array as `scan` when the mount is base_link.
    laser_scan: np.ndarray


class GymBackend:
    """Own one env; reset never reconstructs or silently auto-resets it."""

    def __init__(
        self,
        scenario: Scenario,
        seed: int = 0,
        laser_mount: LaserMount = BASE_LINK,
    ):
        self.scenario = scenario
        self._laser_mount = laser_mount
        os.environ["F1TENTH_GYM_JAX_MAP_DIR"] = str(scenario.map_directory)
        self._env = make(scenario.env_id, **scenario.parameters)
        self._track = racing_common.Track.from_yaml(scenario.track_path)
        self._command = DriveCommand()
        self._action_spec = racing_common.EnvActionSpec(
            getattr(
                racing_common.LongitudinalAction,
                scenario.longitudinal.upper(),
            ),
            getattr(
                racing_common.SteeringAction,
                "STEERING_ANGLE"
                if scenario.steering == "steeringangle"
                else "STEERING_VELOCITY",
            ),
        )
        self.compile_latency_seconds = self._warm(seed)

    def _action(self) -> dict[str, jax.Array]:
        command = racing_common.DriveCommand()
        command.steering_angle = self._command.steering_angle
        command.steering_angle_velocity = self._command.steering_angle_velocity
        command.speed = self._command.speed
        command.acceleration = self._command.acceleration
        values = racing_common.apply_drive_command(command, self._action_spec)
        vector = jnp.asarray(values, dtype=jnp.float32)
        return {agent: vector for agent in self._env.agents}

    def _warm(self, seed: int) -> float:
        """Compile every step signature the run will hit, before it starts.

        Two steps, not one: stepping a *reset* state and stepping an *already
        stepped* state are separate signatures to JAX, so warming only the
        first leaves the second to compile at run time. That showed up as a
        single ~0.5s (CPU) / ~1.5s (GPU) stall on step 2 of a live launch --
        long enough to exceed the safety supervisor's stale-input timeout and
        latch an unrecoverable emergency stop a second into every tier-3 run.
        """
        self.reset(seed)
        warm_key, _ = jax.random.split(self._key)
        started = time.perf_counter()
        _, warm_state, _, _, _ = self._env.step_env(
            warm_key, self._state, self._action()
        )
        warm_state.cartesian_states.block_until_ready()
        self._cast_laser_scan(warm_state, warm_key)
        _, warm_state, _, _, _ = self._env.step_env(
            warm_key, warm_state, self._action()
        )
        warm_state.cartesian_states.block_until_ready()
        self._cast_laser_scan(warm_state, warm_key)
        latency = time.perf_counter() - started
        self.reset(seed)
        return latency

    def set_command(self, command: DriveCommand) -> None:
        self._command = command

    def reset(self, seed: int) -> Snapshot:
        self._key = jax.random.PRNGKey(seed)
        self._observation, self._state = self._env.reset(self._key)
        self._done = False
        self._state.cartesian_states.block_until_ready()
        self._laser_scan = self._cast_laser_scan(self._state, self._key)
        return self.snapshot()

    def step(self) -> Snapshot:
        self._key, step_key = jax.random.split(self._key)
        (
            self._observation,
            self._state,
            _rewards,
            dones,
            _infos,
        ) = self._env.step_env(step_key, self._state, self._action())
        self._state.cartesian_states.block_until_ready()
        self._done = bool(np.asarray(dones["__all__"]))
        self._laser_scan = self._cast_laser_scan(self._state, step_key)
        return self.snapshot()

    def _cast_laser_scan(self, state, key: jax.Array) -> np.ndarray:
        """Ray-march the scan again, from the laser mount."""
        if self._laser_mount.is_origin:
            return np.asarray(state.scans[0], dtype=float)
        return np.asarray(self._laser_scans(state, key)[0], dtype=float)

    @partial(jax.jit, static_argnums=0)
    def _laser_scans(self, state, key: jax.Array) -> jax.Array:
        """One jitted call: eagerly, the pose shift alone cost ~1 ms a step.

        Through the env's own `_scan`, on a state whose pose is the laser's:
        the signature is the one `_scan` was already traced with, so
        jax_pf's `get_scan` - which asserts it is traced at most once - is
        not traced again, and a retrace would fail loudly, not slowly. The
        key is folded, not split, so the run's own key chain (and with it
        every ground-truth value and golden) is untouched.
        """
        cartesian = state.cartesian_states
        laser = laser_poses(cartesian[:, [0, 1, 4]], self._laser_mount)
        cartesian = (
            cartesian.at[:, 0]
            .set(laser[:, 0])
            .at[:, 1]
            .set(laser[:, 1])
            .at[:, 4]
            .set(laser[:, 2])
        )
        return self._env._scan(
            state.replace(cartesian_states=cartesian),
            jax.random.fold_in(key, 1),
        ).scans

    def snapshot(self) -> Snapshot:
        """Return ROS-neutral state from authoritative Cartesian state.

        The observation vector intentionally omits world x, y, and yaw.
        """
        cartesian = np.asarray(self._state.cartesian_states[0])
        scan = np.asarray(self._state.scans[0], dtype=float)
        pose = racing_common.CartesianPose(
            float(cartesian[0]), float(cartesian[1]), float(cartesian[4])
        )
        frenet = self._track.to_frenet(pose)
        speed = float(cartesian[3])
        return Snapshot(
            x=pose.x,
            y=pose.y,
            steering_angle=float(cartesian[2]),
            speed=speed,
            yaw=pose.psi,
            yaw_rate=float(cartesian[5]) if len(cartesian) > 5 else 0.0,
            scan=scan,
            s=frenet.s,
            d=frenet.d,
            heading_error=frenet.heading_error,
            v_s=speed * float(np.cos(frenet.heading_error)),
            v_d=speed * float(np.sin(frenet.heading_error)),
            collision=bool(np.asarray(self._state.collisions[0])),
            done=bool(np.asarray(self._state.done[0])),
            laser_scan=self._laser_scan,
        )
