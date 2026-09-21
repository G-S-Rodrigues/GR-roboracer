# Running the stack

## Host setup

Requirements: WSL2 (Ubuntu 24.04) or plain Linux, Docker with Compose v2, and — for the headless GPU
simulator only — an NVIDIA GPU with the container toolkit. The ROS side needs no GPU.

**The working tree must live on ext4 inside the distro** (`~/gitroot/GR-roboracer`), never under
`/mnt/c/`. See `docs/agents/repo-gotchas.md` #1 for what happens otherwise.

```bash
git clone git@github.com:G-S-Rodrigues/GR-roboracer.git ~/gitroot/GR-roboracer
cd ~/gitroot/GR-roboracer
docker compose -f docker/docker-compose.yaml up -d dev
docker exec -it gr-roboracer-dev bash
```

The entrypoint sources the ROS distro, the third-party underlay and (once built) the workspace
overlay for the container's main process only. A shell opened with `docker exec` inherits none of
it, so source them first with `source /ws/setup.sh`. The devcontainer's VS Code terminal does
this for you.

```bash
colcon build --base-paths ros_ws/src --symlink-install
./scripts/check.sh --full
```

Optional, and only if you want the headless GPU simulator:

```bash
docker compose -f docker/docker-compose.yaml run --rm fast-sim \
  python -c "import jax; print(jax.devices())"     # must NOT print CpuDevice
```

## Driving from a Windows shell

Commands wrap as `wsl.exe -d Ubuntu-24.04 -e bash -lc '…'`. Two traps:

- **Do not put `$PATH` inside the inner string.** WSL interop injects the Windows `PATH`, which
  contains `Program Files (x86)`; the unquoted parentheses are a bash syntax error. Use absolute
  paths instead.
- For editing files from Windows tools, the tree is at
  `\\wsl.localhost\Ubuntu-24.04\home\<user>\gitroot\GR-roboracer`.

## Running a scenario

The full graph, with RViz:

```bash
ros2 launch racing_bringup sim_pure_pursuit.launch.py seed:=42
```

Headless, printing the run's metrics — this is what the nightly sweep drives:

```bash
./scripts/run_scenario.py --seed 42
```

Compare any two metrics records field by field, with the tolerances the tests use:

```bash
./scripts/compare_metrics.py tests/golden/baseline.json log/scenario-seed-42.json
```

Regenerating the golden baseline is a **reviewed diff**, run in `fast-sim`, and never a response to a
failing run:

```bash
# tests/golden/baseline.json (analytic_circle, SIM-3040)
docker compose -f docker/docker-compose.yaml run --rm -T fast-sim python -m sim.rollout \
  --track config/tracks/analytic_circle.yaml --maps-root config/scenarios/maps \
  --scenario baseline --seed 42 --max-steps 9000 \
  --image-digest "$(docker image inspect --format '{{.Id}}' gr-roboracer:fast-sim)" \
  --vehicle-parameter-version f1tenth-default-v1 --lookahead-distance 1.0 \
  --minimum-speed 0.5 --maximum-speed 3.0 --curvature-speed-gain 1.0 \
  --output tests/golden/baseline.json

# tests/golden/spielberg_baseline.json (SIM-3090): the same flags with
#   --track config/tracks/spielberg.yaml --scenario spielberg --max-steps 30000
```

The controller flags are `config/vehicles/f1tenth_default.yaml`'s values; `python sim/rollout.py`
fails with `ModuleNotFoundError: sim`, so run it as a module. It regenerates the gym map assets
under `config/scenarios/maps` from the canonical track, byte-identically. The live scenario must
not set physics parameters the rollout does not (SIMJAX-1060).

## When a run misbehaves

- **The vehicle sits still with `active_clamps=64`.** That is a latched emergency stop: the command
  stream stalled for longer than `stale_input_timeout_ms`. Find the stall; do not widen the timeout
  (`docs/agents/repo-gotchas.md` #13 is the last thing that caused one).
- **A topic "isn't publishing".** Compare QoS on both ends before reading any code (#6).
- **Impossible message rates, or a clamp latched before your run started.** A previous launch's nodes
  are still alive on the same `ROS_DOMAIN_ID` (#10):

  ```bash
  pkill -9 -f 'install/racing_'; pkill -9 -f 'ros2 launch'
  ```
