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
