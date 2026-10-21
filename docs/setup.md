# Setup

Secondary setup and troubleshooting. The first run itself is in [Quick Start](quick-start.md); this
page does not repeat its build and launch sequence.

## Host requirements

- Linux (Ubuntu 24.04 is the one in use) with Docker and Compose v2, usable by your user without
  `sudo`. Check with `docker compose version`.
- SSH access to GitHub (`ssh -T git@github.com`). Clone with the SSH URL; an HTTPS clone prompts for
  credentials forever.
- The working tree on a local Linux filesystem, such as `~/gitroot/GR-roboracer`. On a network or
  FUSE-mounted filesystem a colcon build is far slower and file watchers can stop firing without any
  error.
- Only the optional GPU simulator needs an NVIDIA GPU with the container toolkit.

## Display

RViz opens in a window on your desktop, through the X11 socket the `dev` container mounts. The RViz
display path has not yet been validated on a native Linux desktop. If the window does not open, run
the headless commands in [Running](running.md) instead, or raise it in the repository's issues.

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
