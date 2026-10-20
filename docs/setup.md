# Setup

Secondary setup and troubleshooting. The first run itself is in [Quick Start](quick-start.md); this
page does not repeat its build and launch sequence.

## Host requirements

- WSL2 with Ubuntu 24.04, and Docker with Compose v2: Docker Desktop with its WSL integration
  enabled for the distro, or Docker Engine installed inside the distro. Check with `docker compose version` from the Ubuntu shell.
- SSH access to GitHub (`ssh -T git@github.com`). Clone with the SSH URL; an HTTPS clone prompts for
  credentials forever.
- The working tree on ext4 inside the distro, such as `~/gitroot/GR-roboracer`. Under `/mnt/c/` a
  colcon build is roughly ten times slower and file watchers stop firing without any error.
- Only the optional GPU simulator needs an NVIDIA GPU with the container toolkit.

## Display

The `dev` container mounts `/tmp/.X11-unix` and `/mnt/wslg` and sets `XDG_RUNTIME_DIR` to WSLg's
runtime directory. If RViz does not open:

- On the host, confirm `ls /mnt/wslg` and `echo $DISPLAY $WAYLAND_DISPLAY` are not empty. Run
  `wsl --shutdown` from Windows and reopen Ubuntu if WSLg is stuck.
- Start the container from the same shell that has those variables, then recreate it:
  `docker compose -f docker/docker-compose.yaml up -d --force-recreate dev`.

### Native Linux GUI (unvalidated)

The RViz window has only been validated on WSL2/WSLg. Native Linux with X11 or Wayland may need
different mounts and variables, and no recipe is published here because none has been tested. Run
the headless commands in [Running](running.md) instead, or raise it in the repository's issues.

## Driving from a Windows shell

Commands wrap as `wsl.exe -d Ubuntu-24.04 -e bash -lc '...'`. Two traps:

- Do not put `$PATH` inside the inner string. WSL interop injects the Windows `PATH`, which
  contains `Program Files (x86)`; the unquoted parentheses are a bash syntax error. Use absolute
  paths.
- To edit files from Windows tools, the tree is at
  `\\wsl.localhost\Ubuntu-24.04\home\<user>\gitroot\GR-roboracer`.

## Optional: the GPU simulator

The headless simulator runs in a separate `fast-sim` container with JAX on CUDA, with no ROS in it.
Build the `dev` workspace first (see [Quick Start](quick-start.md)), because `fast-sim` reads its
build artifacts read-only. Then build the image and check the device:

```bash
docker compose -f docker/docker-compose.yaml build fast-sim
docker compose -f docker/docker-compose.yaml run --rm fast-sim \
  python -c "import jax; print(jax.devices())"
```

The output must not be `CpuDevice`. That means the GPU setup failed and JAX fell back to the CPU,
which gives correct results far more slowly. It is not a successful GPU run.

## Troubleshooting

- **The vehicle sits still with `active_clamps=64`.** A latched emergency stop: the command stream
  stalled for longer than `stale_input_timeout_ms`. Find the stall; do not widen the timeout.
- **A topic "isn't publishing".** Compare the QoS of both ends before reading code: a reliable
  subscriber never hears a best-effort publisher, and nothing says so.
- **`vehicle file not found: config/vehicles/f1tenth_default.yaml`.** The launch was started
  outside `/ws`. `cd /ws` and launch again.
- **Impossible message rates, or a stop latched before your run began.** A previous launch is still
  alive on the same `ROS_DOMAIN_ID`. List what is running with `ps aux | grep -E 'install/racing_|ros2 launch'`,
  then stop only those processes, for example `pkill -f 'install/racing_'` and then
  `pkill -f 'ros2 launch'` from inside the container. Nodes can show in `ros2 node list` for a few
  seconds after they exit.
- **`colcon build` fails with `No rule to make target '/opt/ros/jazzy/lib/...so'` after an image
  rebuild.** The build volumes hold artifacts linked against the old image. Inside the container,
  clear this workspace's generated trees and build again: `rm -rf /ws/build/* /ws/install/*`.
