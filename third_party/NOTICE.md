# third_party — what is referenced

Two kinds of thing are listed. A git submodule is **referenced, not vendored**:
this repository records only its URL and the commit, and none of its files is
copied into this tree, into the Docker images (the Dockerfile `COPY`s named
paths only) or into `install/`. The apt packages in the last section are
installed into the image unmodified.

## `f1tenth_racetracks`

- Upstream: `https://github.com/f1tenth/f1tenth_racetracks`, pinned at
  `866fe657f0bbe245f90a5042bc6eec0b0489d4c3`.
- Licence: **GPL-3.0**, under its own terms (see `LICENSE` in the submodule).
- Use: read at tier 5 only, by SIM-5050, as data (the upstream raceline's
  geometry for the report's reference row). Nothing else reads it, and the
  report renders "upstream reference: not available" when the submodule is not
  initialised, which is the default in a clone.
- The pin matches the revision `config/scenarios/maps/Spielberg/NOTICE.md`
  derives the Spielberg track from.
- Initialise with `git submodule update --init third_party/f1tenth_racetracks`.

## MOLA, mp2p-icp and MRPT (Jazzy apt)

- Source: the `ros-jazzy-mola-*`, `ros-jazzy-mp2p-icp*` and `ros-jazzy-mrpt-*`
  packages of the ROS Jazzy apt index, pinned by name in
  `docker/apt-preferences/mola.pref` (the closure is `mola.closure`).
- Used unmodified from Jazzy apt, as separate processes; not vendored, not
  forked.
- Licences: `mola_lidar_odometry`, `mola_kernel` and `mola_launcher` are
  **GPL-3.0**; `mola_bridge_ros2` and `mp2p_icp` are **BSD-3-Clause**; MRPT is
  BSD.
- The committed MOLA map (`.mm`) is **data output by** GPL-3.0 software. It is
  data, not a copy of the program.
