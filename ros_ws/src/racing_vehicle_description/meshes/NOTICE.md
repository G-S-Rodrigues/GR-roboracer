# Meshes in this package

## Imported from upstream

`hokuyo.stl` (the LIDAR), `left_wheel.stl` and `right_wheel.stl` are copied
unmodified from [f1tenth-dev/simulator](https://github.com/f1tenth-dev/simulator)
(`urdf/meshes/`, `development` branch), licensed Apache License 2.0 — see
`LICENSE-f1tenth-dev-simulator` in this directory. `left_wheel.stl`/`right_wheel.stl`
were briefly deleted and replaced with primitive `<cylinder>` geometry, then
restored from this file's own git history once a licensed alternative turned out
not to exist elsewhere — they're the real wheel meshes again.

**These are the only third-party meshes.** Everything else below is original.

## Originally generated (no license concern)

All other meshes are produced from scratch by
`../scripts/generate_f1tenth_meshes.py` (geometry assembled from boxes and
cylinders in Python, written out as binary STL). None is traced from any CAD
source, so nothing is copied. They are sized to fit this repo's fixed URDF
dimensions (wheelbase 0.3302 m, track 0.2032 m, wheel radius 0.0508 m) and are
authored directly in `base_link` coordinates, so the URDF attaches each one with
an identity joint and there is no per-link placement math in the URDF.

- `chassis.stl` — main plate with raised side rails, cross-members, front/rear
  bars, a steering servo pocket, and rear motor-mount pads. Replaces the earlier
  flat tapered plate (which was itself self-generated). Not copied from any
  upstream CAD file.
- `jetson_board.stl` — Jetson compute module: PCB, heatsink base + fins, fan,
  GPIO header, connector block.
- `battery_pack.stl` — LiPo battery pack with strap and cable exit.
- `camera_box.stl` — front-facing camera body with lens cylinder and mount
  bracket.
- `rear_susp_{left,right}.stl`, `front_susp_{left,right}.stl` — wishbone A-arms
  connecting the chassis rails to each wheel's hub/knuckle location.
- `rear_shock_{left,right}.stl`, `front_shock_{left,right}.stl` — telescoping
  shock-absorber stand-ins between a chassis mount point and each wheel.
- `rear_hub_{left,right}.stl`, `front_hub_{left,right}.stl` — wheel hubs/spacers
  at each wheel center (colocated with the wheel link origins).
- `motor_{left,right}.stl` — rear drive motor can, gearbox housing, drive-shaft
  stub.

## What was removed compared to earlier revisions

- The original `chassis.stl` and `hinge.stl` vendored from f1tenth-dev/simulator
  were removed.
- `camera_mount.stl`, `lidar_mount.stl`, `wifi_mount.stl` (copied unmodified from
  mlab-upenn/f1tenthpublic — a repo with **no LICENSE**) were removed per user
  direction to keep only the LIDAR from the third-party set. Their function is
  now served by the generated `camera_box.stl` and the LIDAR's direct mount on
  the chassis. (Note: keep the LIDAR `hokuyo.stl` — that stays.)

## Licensing research recap

Neither [pranavk-2003/F1tenth](https://github.com/pranavk-2003/F1tenth),
[mit-racecar/racecar_simulator](https://github.com/mit-racecar/racecar_simulator)
nor [Tinker-Twins/AutoDRIVE-F1TENTH](https://github.com/Tinker-Twins/AutoDRIVE-F1TENTH)
have real chassis/wheel mesh files — all three use primitive box/cylinder
geometry, and AutoDRIVE-F1TENTH has no 3D assets at all. The wheelbase/track/
wheel-radius joint offsets that survived from pranavk-2003/F1tenth's
`urdf/f1tenth.xacro` are numbers only, not copied files — that repo has no
`LICENSE`, so nothing beyond non-copyrightable measurements was reused.

Commercial RC buggy chassis STLs (Traxxas Slash and similar) were researched as a
possible swap-in and **not adopted**: they reproduce a specific manufacturer's
patented/copyrighted part shape under hobbyist personal-use terms, a worse
license bet than the self-generated geometry. The official
[f1tenth/f1tenth_simulator](https://github.com/f1tenth/f1tenth_simulator) repo
also has no `LICENSE` and no mesh/STL references.

`urdf/f1tenth.urdf` in this package is **not** a copy of any upstream
macro/xacro file. It is a from-scratch, RViz-only URDF written for this repo
(fixed joints only, no Gazebo/sensor/plugin/transmission elements — a spec
non-goal here).
