"""Offline URDF mesh renderer -- no ROS, no display, no RViz.

Parses fixed-joint transforms straight out of the URDF, loads each link's mesh
(STL, ASCII or binary, with URDF <mesh scale="..."> applied), places the meshes
in the base_link frame, and renders an oblique 3D view with matplotlib. Lets us
iterate on mesh/mount placement against a reference photo without a live RViz
session -- run it after any urdf/mesh edit.

Usage (from repo root, via uv so numpy/matplotlib are not system-wide):
    uv run --with numpy --with matplotlib python3 \\
        ros_ws/src/racing_vehicle_description/scripts/render_urdf.py \\
        ros_ws/src/racing_vehicle_description/urdf/f1tenth.urdf \\
        ros_ws/src/racing_vehicle_description \\
        /tmp/render.png [--elev E] [--azim A]

Useful views: top-down proportions are --elev 90 --azim -90; a three-quarter
is the default (elev=22, azim=-60).
"""

import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def load_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        expected_size = 84 + n * 50
        actual_size = path.stat().st_size
    if actual_size == expected_size:
        tris = []
        with open(path, "rb") as f:
            f.read(84)
            for _ in range(n):
                f.read(12)
                v = [struct.unpack("<fff", f.read(12)) for _ in range(3)]
                f.read(2)
                tris.append(v)
        return np.array(tris, dtype=float)
    # Fall back to ASCII STL.
    tris = []
    cur = []
    for line in path.read_text(errors="ignore").splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            _, x, y, z = line.split()
            cur.append((float(x), float(y), float(z)))
            if len(cur) == 3:
                tris.append(cur)
                cur = []
    return np.array(tris, dtype=float)


def rpy_to_matrix(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def parse_xyz_rpy(elem):
    if elem is None:
        return np.zeros(3), np.zeros(3)
    xyz = [float(v) for v in elem.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in elem.get("rpy", "0 0 0").split()]
    return np.array(xyz), np.array(rpy)


def main():
    urdf_path = Path(sys.argv[1])
    mesh_root = Path(sys.argv[2])
    out_png = Path(sys.argv[3])
    elev = 22
    azim = -60
    args = sys.argv[4:]
    for i, a in enumerate(args):
        if a == "--elev":
            elev = float(args[i + 1])
        if a == "--azim":
            azim = float(args[i + 1])

    root = ET.parse(urdf_path).getroot()

    # Build parent->child joint map (fixed joints only -- none other exist).
    children = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        xyz, rpy = parse_xyz_rpy(origin)
        T = np.eye(4)
        T[:3, :3] = rpy_to_matrix(*rpy)
        T[:3, 3] = xyz
        children.setdefault(parent, []).append((child, T))

    # BFS from base_link to compute each link's transform relative to base_link.
    link_transform = {"base_link": np.eye(4)}
    queue = ["base_link"]
    while queue:
        cur = queue.pop(0)
        for child, T in children.get(cur, []):
            link_transform[child] = link_transform[cur] @ T
            queue.append(child)

    # Collect material definitions from the URDF root.
    mat_colors = {}
    for m in root.findall("material"):
        c = m.find("color")
        if c is not None:
            rgba = [
                float(v)
                for v in c.attrib.get("rgba", "0.5 0.5 0.5 1.0").split()
            ]
            mat_colors[m.attrib["name"]] = rgba[:3]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    all_pts = []
    link_by_name = {el.attrib["name"]: el for el in root.findall("link")}
    for name, T in link_transform.items():
        link = link_by_name.get(name)
        if link is None:
            continue
        visual = link.find("visual")
        if visual is None:
            continue
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            continue
        mesh_path = mesh.get("filename").replace(
            "package://racing_vehicle_description/", ""
        )
        scale = [float(v) for v in mesh.get("scale", "1 1 1").split()]
        full_path = mesh_root / mesh_path
        if not full_path.exists():
            print(f"WARN: missing mesh {full_path}")
            continue
        tris = load_stl(full_path)
        tris = tris * np.array(scale)
        vis_origin = visual.find("origin")
        vxyz, vrpy = parse_xyz_rpy(vis_origin)
        Tv = np.eye(4)
        Tv[:3, :3] = rpy_to_matrix(*vrpy)
        Tv[:3, 3] = vxyz
        Tfull = T @ Tv
        pts = tris.reshape(-1, 3)
        pts_h = np.hstack([pts, np.ones((len(pts), 1))])
        pts_world = (Tfull @ pts_h.T).T[:, :3]
        tris_world = pts_world.reshape(-1, 3, 3)
        all_pts.append(pts_world)

        # Use the URDF's own material color if available.
        mat_ref = visual.find("material")
        mat_name = mat_ref.attrib["name"] if mat_ref is not None else None
        rgba = mat_colors.get(mat_name, [0.5, 0.5, 0.5])
        # Lighten edges a touch relative to the face for contrast.
        edge = [min(c + 0.18, 1.0) for c in rgba]
        poly = Poly3DCollection(
            tris_world,
            facecolor=rgba,
            edgecolor=edge,
            linewidths=0.2,
            alpha=1.0,
        )
        ax.add_collection3d(poly)

    all_pts = np.vstack(all_pts)
    mins = all_pts.min(axis=0)
    maxs = all_pts.max(axis=0)
    center = (mins + maxs) / 2
    span = (maxs - mins).max() / 2 * 1.2

    # Ground reference plane at z=0 (bottom of wheels).
    gp = span * 1.1
    ground = [
        [center[0] - gp, center[1] - gp, 0],
        [center[0] + gp, center[1] - gp, 0],
        [center[0] + gp, center[1] + gp, 0],
        [center[0] - gp, center[1] + gp, 0],
    ]
    ground_poly = Poly3DCollection(
        [ground], facecolor=(0.88, 0.88, 0.88), edgecolor="none", alpha=0.5
    )
    ax.add_collection3d(ground_poly)

    ax.set_xlim(center[0] - span, center[0] + span)
    ax.set_ylim(center[1] - span, center[1] + span)
    ax.set_zlim(0 - span * 0.15, center[2] + span)
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    fig.tight_layout()
    fig.savefig(out_png, dpi=150, facecolor="white")
    print(f"wrote {out_png}")


if __name__ == "__main__":
    main()
