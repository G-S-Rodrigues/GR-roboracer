# Running on WSL2

The validated host for the RViz window is WSL2 (Ubuntu 24.04) with WSLg. User-facing setup is in
`docs/setup.md` and `docs/quick-start.md` and says nothing about WSL; this file holds what is
specific to it. Like `repo-gotchas.md`, the wrong thing here does not raise an error.

## Host

- Docker is Docker Desktop with its WSL integration enabled for the distro, or Docker Engine
  inside the distro. Check `docker compose version` from the Ubuntu shell.
- The tree must be on ext4 inside the distro (`~/gitroot/GR-roboracer`), never under `/mnt/c/`:
  the 9p bridge makes `colcon build` roughly ten times slower and file watchers stop firing
  (`repo-gotchas.md` #1).
- Windows 11, or a Windows 10 build with WSLg. Check that `/mnt/wslg` and `/tmp/.X11-unix` exist in
  the Ubuntu shell.
- To edit from Windows tools the tree is at
  `\\wsl.localhost\Ubuntu-24.04\home\<user>\gitroot\GR-roboracer`.

## Display

`docker/docker-compose.yaml` mounts `/tmp/.X11-unix` and `/mnt/wslg` into `dev` (lines 25-27) and
sets `DISPLAY` (default `:0`), `WAYLAND_DISPLAY` and `PULSE_SERVER` from the host (lines 38-39, 44).
`XDG_RUNTIME_DIR` is fixed to `/mnt/wslg/runtime-dir` (line 43): a WSL host exports its own
`/run/user/<uid>`, which does not exist in the container, so a `${XDG_RUNTIME_DIR:-...}` fallback
never applied and RViz found no runtime dir (comment at lines 40-42).

If RViz does not open:

- On the host, `ls /mnt/wslg` and `echo $DISPLAY $WAYLAND_DISPLAY` must not be empty. If WSLg is
  stuck, run `wsl --shutdown` from Windows and reopen Ubuntu.
- Compose reads the variables from the shell that starts the container, so start it from a shell
  that has them, then recreate:
  `docker compose -f docker/docker-compose.yaml up -d --force-recreate dev`.

The checkout is bind-mounted at `/ws` (line 18) and owned by UID 1000 while the container runs as
root; the `safe.directory` entry in the same environment block (lines 32-37) is why git works there.

## Driving from a Windows shell

Commands wrap as `wsl.exe -d Ubuntu-24.04 -e bash -lc '...'`. One trap:

- Do not put `$PATH` inside the inner string. WSL interop injects the Windows `PATH`, which contains
  `Program Files (x86)`; the unquoted parentheses are a bash syntax error. Use absolute paths.

## AutoDRIVE GUI

The competition simulator's window runs on **Windows**, not in WSL, and talks to the devkit bridge in
Docker. Measured 2026-10-30 (WSL2 NAT networking, Docker Desktop); the owner drove it by hand.

**The route that works (W1).** `AUTODRIVE_ROS_DOMAIN_ID=66 scripts/autodrive_gui.sh` starts only
`autodrive-devkit`, published on `127.0.0.1:4567`, and prints the Windows commands. Windows reaches
Docker Desktop's published port on `127.0.0.1` (`Test-NetConnection` True). In PowerShell:

1. Download `autodrive_simulator_practice_windows.zip` from the `2026-iros` release of
   AutoDRIVE-RoboRacer-Sim-Racing; its sha256 must be
   `2D3E5CC7073009D8F28F41B2266C129DD64210688C70228A6811B844EE48A751`. The owner unzipped it to
   `C:\dev\Autodrive`.
2. `Set-Culture en-US` (see the trap below).
3. Start the bridge (script above), then `& "C:\dev\Autodrive\autodrive_simulator\AutoDRIVE Simulator.exe" -ip 127.0.0.1 -port 4567`.
   The flags connect on their own; clicking Connect opens a second socket (the devkit logs
   "Connected!" twice), which did no harm.
4. Manual mode drives the car from the window. Autonomous mode does not move until something sends
   commands; `ros2 run autodrive_roboracer teleop_keyboard` in the devkit container (domain 66) does.

**Measured through the GUI build, 3 minutes, domain 66:** `/autodrive/roboracer_1/lidar` 48 Hz
(gaps 12-49 ms), `imu` 34 Hz (max gap 70 ms), no `KeyError` in the devkit log. The official bridge
stalls only headless; with the GUI build it keeps publishing. Render rate on the owner's screen
was 96 Hz.

**GUI build against headless (offline, 2026-10-30).** The W1a recording (652 messages) against part 1's
headless fixture (`porto_stream.jsonl.gz`, 617):
- Same 18 keys. The GUI build's first message already carries the lidar (1081 ranges); the headless
  first message has none, so a client must not assume either.
- 1081 beams in both, 4.3 % / 4.2 % `inf`/`NaN`.
- The camera is real and changing in the GUI build (626 distinct frames, ~6.7 kB) and one constant
  placeholder headless.
- Period: median 24.5 ms (40.8 Hz), 10-90 % 18.0-30.3 ms, max 43.5 ms, against headless 18.2 ms,
  11.2-54.5 ms, max 144.5 ms. Neither is 20 Hz; the GUI stream has no gap near the 100 ms stale
  timeout.
- Throttle-to-speed gain on the 0.04 straight: 25.02 m/s per unit in both, so the physics agree.

**Traps.**
- **The sim writes and parses numbers in the Windows culture.** Under pt-PT it sent
  `'V1 Throttle': '0,0000'` and stalled on a reply of `0.0`. Nothing errors; the car just never
  starts. `Set-Culture en-US` fixed it.
- **Windows cannot reach a listener on the WSL host through `127.0.0.1`.** Only the WSL IP
  (`ip -4 addr show eth0`, it changes on every WSL restart) works for our stand-in recorder
  (`tools/autodrive_record.py`), which is why that route used `172.24.67.156` and the devkit route
  uses the published port. The recorder run (W1a) recorded 652 messages, median 40.8 Hz, 1081
  ranges per frame.
- The sim connects once and never reconnects: start the bridge first.
- One listener on 4567 at a time; the script refuses when a container already publishes it.

**Not attempted.** The Linux binary under Mesa's dzn (W2) and a native-Linux window (W3): W1
worked, so the spec's "stop at the first that works" applies (it supersedes the plan's D1, which
would have run W3 anyway; `docs/running.md`'s Linux section is therefore unverified). Why W2 would have been hard: the
build ships only Vulkan, WSL gives containers no NVIDIA Vulkan, and dzn is absent from the image
and from host Mesa 25.2.
