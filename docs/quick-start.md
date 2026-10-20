# Quick Start

This page takes a new machine to a running simulation: the reference stack driving the
analytic-circle track, with RViz open. It is the one complete first-run recipe; [Setup](setup.md)
holds the optional and diagnostic material.

The validated host is **WSL2 (Ubuntu 24.04) with WSLg**. Native Linux with a desktop session has not
been validated for the RViz window; see [Native Linux](setup.md#native-linux-gui-unvalidated).

## Before you start

- A GitHub account with SSH access, and Git.
- Docker with Compose v2, usable by your user without `sudo`: Docker Desktop with its WSL
  integration enabled for Ubuntu 24.04, or Docker Engine installed inside the distro.
- A graphical session: WSLg on Windows 11 (or a current Windows 10 build with WSLg). Check that
  `/mnt/wslg` and `/tmp/.X11-unix` exist in your Ubuntu shell.
- The repository on ext4 inside the distro (`~/gitroot/...`), never under `/mnt/c/`.

You need no host ROS install and no NVIDIA GPU. The first image build downloads and compiles a lot
and takes a while; later runs reuse it.

## 1. Build and start the container

Run these in an Ubuntu 24.04 shell on the host:

```bash
mkdir -p ~/gitroot
git clone git@github.com:G-S-Rodrigues/GR-roboracer.git ~/gitroot/GR-roboracer
cd ~/gitroot/GR-roboracer
docker compose -f docker/docker-compose.yaml build dev
docker compose -f docker/docker-compose.yaml up -d dev
docker exec -it gr-roboracer-dev bash
```

The last command opens a shell inside the `gr-roboracer-dev` container. Everything below runs
there, in `/ws`, which is your checkout.

## 2. Build the workspace and launch

Inside the container. A shell from `docker exec` has sourced nothing, so source `setup.sh` first,
and again after the build so the new overlay is picked up. The launch must start from `/ws`: the
default vehicle file path is relative.

```bash
cd /ws
source /ws/setup.sh
colcon build --base-paths ros_ws/src --symlink-install
source /ws/setup.sh
ros2 launch racing_bringup sim_pure_pursuit.launch.py seed:=42
```

## What you should see

RViz opens in a Windows window with a Displays panel on the left. In the 3D view you see a small
car model on a ring-shaped track drawn as polylines (the left boundary in green, the right in red, and the planned
trajectory in blue), with the lidar scan drawn as points around the car. The car drives around the ring at
real time. The `Map` display shows a warning, because this track has no saved map; that is expected.

This is the reference stack with the simulator's exact pose (`ground_truth`) on the
`analytic_circle` track. It is a short baseline, not the saved-map AMCL demonstration in the
[README](../README.md). The circle is rotationally symmetric, so it says nothing about localization
quality.

## Stop

Press `Ctrl+C` in the launching terminal, then `exit` to return to the host. The container keeps
running, which is fine; do not remove it with `docker compose down` unless you mean to.

## Next

If the window does not appear, or you want the GPU simulator, go to [Setup](setup.md). To choose a
workflow and see which parts of the stack are selectable, use [Configuration](configuration.md).
For headless runs and metrics, see [Running](running.md).
