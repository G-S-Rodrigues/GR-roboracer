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
dimensions (wheelbase 0.33 m, track 0.236 m, wheel radius 0.058 m) and are
authored directly in `rear_axle` coordinates (the origin is the rear axle's
ground point; base_link is the centre of gravity, 0.17145 m ahead and 0.074 m
above it), so the URDF attaches each one with an identity joint and there is no per-link placement math in the URDF.

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
geometry, and AutoDRIVE-F1TENTH has no 3D assets at all (the simulator repo
`AutoDRIVE-Ecosystem/AutoDRIVE` does; see "Imported from AutoDRIVE" below). The wheelbase/track/
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

## Imported from AutoDRIVE (`autodrive_roboracer/`)

The 3D model of the AutoDRIVE RoboRacer (F1TENTH) car, used by
`urdf/autodrive_roboracer.urdf`.

- Repository: `AutoDRIVE-Ecosystem/AutoDRIVE`, branch `AutoDRIVE-Simulator`,
  commit `75b7df5216524a3c6b3150d698139190c6c44f8f`.
- Path: `Assets/Models/Vehicle/F1TENTH/F1TENTH.fbx` (colours from the `.mat` files in
  its `Materials/` and from `Assets/Prefabs/F1TENTH/F1TENTH.prefab`).
- Licence: BSD 2-Clause, Copyright (c) 2021, Tinker Twins; the verbatim text is
  `LICENSE-autodrive`. Nothing was extracted from a compiled simulator build.
- Modified: re-framed into `rear_axle` (a proper rotation of the Unity frame), split per link
  and per colour into binary STL, colours flattened to the dominant `.mat` colour for the wheels and
  the lidar. **Not simplified**: all 937,506 triangles are kept.
- Vendor sub-parts: the FBX's `Jetson Xavier NX-1/SBC (1).stp` and `Hokuyo UST-10LX`
  parts are probably derived from vendor CAD and have no recorded provenance in the AutoDRIVE
  repository. They are imported under the owner's ruling (V1 D1), and their
  provenance is left to the owner to establish with the AutoDRIVE maintainers.
- Command (network; deterministic):
  `env -u PYTHONPATH uv run --locked --script scripts/import_autodrive_meshes.py meshes/autodrive_roboracer`
  from this package.

Input sha256:

- `F1TENTH.fbx`: `fe21bb814e8a4bf9572f4b95f0de159ca780a1ce31b0907d1a1d22d4eda7f283`
- `F1TENTH.fbx.meta`: `ae84f8bc07623fa6896ab92b5db71e9f6c2d35d2f812d9d636dcfca838b5c583`
- `F1TENTH.prefab`: `a869a1b3d25d567759db052a40c9d87a26ca187bcbe7a64c10ef1ab0bdbd9550`
- `LICENSE`: `b9e7a136ae12a2b6e920435d3a532841ac1df6b2beb0339698b5baab2a68e560`
- `Materials/` (59 `.mat` and their `.meta`): digest `34230b3b42e9728fd1244ebaa8eaf8282d9a5bd7e65d0cf32fec324ecd66a182` (sha256 of the sorted `<name> <sha256>` lines)

Output sha256:

- `chassis_000000.stl`: `1bdb0b267ed97eb9838294bf362bfab4951ba15793322a090d54ba4a15287184`
- `chassis_1a1a1a.stl`: `9f336b567847b86054660d70656640dde03eec1d66e71efd8f5d51ccd5d0d6ea`
- `chassis_1b1b1b.stl`: `260c379ec6499ce8316a986a618d237639868f43deec12d8b5e4d9f5a2e947de`
- `chassis_333333.stl`: `21bfad82a34ca4c050b2c0949ff323f3b54ff4c409f1f0af1804df110e0cc5bd`
- `chassis_3b3b3b.stl`: `fd6e737284fad7f7085dad8dc1a2d092e70ad8a3796ac1dbf74a32cafc99cc64`
- `chassis_4b4b4b.stl`: `b8a10392b6791697f2de1ed9c8dd3d280b8bff4e18532581aeaad6c21ad3e78a`
- `chassis_555555.stl`: `bd8b59aa161adc541a60f5ff5b034318d20e56df49275c545e3ce4b49ebea183`
- `chassis_636363.stl`: `90df59a3a8e1fe3c45f3aa98b733dd95ed9939b1fee2c664acad69ab55983361`
- `chassis_7d7d7d.stl`: `1c7b2b2727894d57a4ff0b5bb284692819e0ccb02df834c83ff48aeedd765094`
- `chassis_afafaf.stl`: `76aebc7121bff17c6de727f11d82dee2aea68c30cecbeba24bcbdfb19ada1c58`
- `chassis_c5c5c5.stl`: `2928a4a5f6485496a6aa868464e03e74f2f3f176d0a9478364e2e8136af5f1e0`
- `chassis_c8c8c8.stl`: `bee933359953e017e545d57c464791368bc0bcb835134fc17f54f05fedad97d5`
- `chassis_d2cec9.stl`: `6280d68ea2dfb7836a16861054386eff93a904b032911461832d46500f4acdf8`
- `chassis_fae37e.stl`: `9410c9d41d7e1a92b05548a5d9412c66e97fca1bab920f647aa8f44b505a98e2`
- `chassis_ffba00.stl`: `14c9a38477c714576fa6a4fc4a34f5d8b87fc9eb7629cd8ee9832630451517eb`
- `chassis_ffdd75.stl`: `1d2b02804de94dfba0e4e5a1b41d8fe94e81095690327f27ff70a24f6929353e`
- `chassis_ffffff.stl`: `df521f27dd1d99a644bcc3749f421f2c77f81d4ed370fdd08b6e6a736590ca93`
- `lidar.stl`: `af2ae6c7bef369c8e4d9dead9cd7016e2b12f58700dba14935c16742a8c6cf84`
- `wheel_fl.stl`: `eb3522b0104454bc92a76ebeb29e1ab32e8dcc92aff166d2cff5d1aaac7a1890`
- `wheel_fr.stl`: `ef33f9e90bb7f5a3fda97c9959445d0171df00e5e2c1e242b2ee6a19a1de0ee7`
- `wheel_rl.stl`: `3fe9e5ac25f3c7c3f460798ee50af36fe0bf8d10b2e3ff1def7ae9bb71b10988`
- `wheel_rr.stl`: `d4a5522d7d6583e50e316c99352f06f27141f8eaf3a41f7542050d9084065e85`
