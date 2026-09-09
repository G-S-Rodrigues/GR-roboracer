"""Generate an original, license-free chassis plate mesh as a binary STL.

Purely a visual stand-in: a thin tapered plate extruded in Z, sized to fit
within this repo's already-fixed URDF dimensions (wheelbase 0.3302 m, track
0.2032 m). Does not affect any joint offset or vehicle behavior -- meshes here
are visualization only. Run from this directory:

    python3 generate_chassis_stl.py
"""

import struct
from pathlib import Path

L = 0.3302  # length, rear (x=0) to front (x=L) -- matches the wheelbase
W = 0.22  # plate width -- track is 0.2413 m center-to-center
# (0.12065*2) with ~0.05 m wheel radius each side, so wheel
# outer faces sit near y=+-0.17; the plate stays inboard of
# the tires instead of the previous 0.16 m, which read as a
# toy-block chassis floating between four wheels
T = 0.008  # plate thickness
Z_CENTER = 0.09  # matches the old chassis box's vertical center
NOSE_START = 0.75 * L  # where the front taper begins

z_bot = Z_CENTER - T / 2
z_top = Z_CENTER + T / 2

# Top-down outline, rear to front, counter-clockwise: rectangle, tapered nose.
outline = [
    (0.0, -W / 2),
    (0.0, W / 2),
    (NOSE_START, W / 2),
    (L, 0.0),
    (NOSE_START, -W / 2),
]

triangles = []


def add_tri(a, b, c):
    triangles.append((a, b, c))


n = len(outline)

# Top face (fan from first vertex), normal +Z.
for i in range(1, n - 1):
    a = (*outline[0], z_top)
    b = (*outline[i], z_top)
    c = (*outline[i + 1], z_top)
    add_tri(a, b, c)

# Bottom face (reversed winding), normal -Z.
for i in range(1, n - 1):
    a = (*outline[0], z_bot)
    b = (*outline[i + 1], z_bot)
    c = (*outline[i], z_bot)
    add_tri(a, b, c)

# Side walls.
for i in range(n):
    x1, y1 = outline[i]
    x2, y2 = outline[(i + 1) % n]
    top1 = (x1, y1, z_top)
    top2 = (x2, y2, z_top)
    bot1 = (x1, y1, z_bot)
    bot2 = (x2, y2, z_bot)
    add_tri(top1, top2, bot2)
    add_tri(top1, bot2, bot1)


def normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    mag = (nx**2 + ny**2 + nz**2) ** 0.5 or 1.0
    return (nx / mag, ny / mag, nz / mag)


out_path = Path(__file__).resolve().parent.parent / "meshes" / "chassis.stl"
header = b"chassis plate, generated, not from any upstream CAD file"
with open(out_path, "wb") as f:
    f.write(header.ljust(80, b"\x00"))
    f.write(struct.pack("<I", len(triangles)))
    for a, b, c in triangles:
        nx, ny, nz = normal(a, b, c)
        f.write(struct.pack("<fff", nx, ny, nz))
        for v in (a, b, c):
            f.write(struct.pack("<fff", *v))
        f.write(struct.pack("<H", 0))

print(f"wrote {len(triangles)} triangles to {out_path}")
