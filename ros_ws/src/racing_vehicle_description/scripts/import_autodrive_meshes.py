#!/usr/bin/env -S uv run --locked --script
# /// script
# requires-python = ">=3.12"
# dependencies = ["numpy==2.2.6"]
# ///
"""Import AutoDRIVE's RoboRacer (F1TENTH) model as per-link binary STL.

Source: AutoDRIVE-Ecosystem/AutoDRIVE, branch AutoDRIVE-Simulator, pinned
commit below, ``Assets/Models/Vehicle/F1TENTH/F1TENTH.fbx`` (BSD 2-Clause,
Copyright (c) 2021, Tinker Twins). Nothing is extracted from a compiled
simulator build.

The FBX is re-framed into ``rear_axle`` (the ground point under the rear axle
midpoint, gotcha 26) with a proper rotation of the Unity frame:

    x = X/1000 + 0.243,  y = -(Z/1000 + 0.040),  z = Y/1000 - 0.050 + 0.0592

(X, Y, Z are the FBX world coordinates in mm, after the node chain). Then it is
split, without dropping or simplifying a single triangle:

- ``wheel_{fl,fr,rl,rr}.stl``: the ``Wheel-N-solid*`` parts, re-expressed in
  their own wheel frame (the URDF joint origin is subtracted);
- ``lidar.stl``: the ``Hokuyo UST*`` parts, re-expressed in the ``lidar`` frame;
- ``chassis_<rrggbb>.stl``: everything else, one file per resolved colour.

Colours: the prefab MeshRenderer's material for the part's GameObject (slot
= the FBX polygon's material index), else the FBX material via the ``.meta``
externalObjects; the ``.mat`` file's ``_BaseColor``, as 8-bit sRGB.

Usage (network; the output is deterministic and hash-checked)::

    env -u PYTHONPATH uv run --locked --script \\
        scripts/import_autodrive_meshes.py \\
        ros_ws/src/racing_vehicle_description/meshes/autodrive_roboracer

``--inputs DIR`` reads a directory laid out as the downloader writes it
(``F1TENTH.fbx``, ``F1TENTH.fbx.meta``, ``F1TENTH.prefab``, ``LICENSE``,
``mats/*``) instead of downloading. A hash mismatch is an error.
"""

import argparse
import collections
import hashlib
import json
import math
import re
import struct
import sys
import tempfile
import urllib.parse
import urllib.request
import zlib
from pathlib import Path

import numpy as np

REPO = "AutoDRIVE-Ecosystem/AutoDRIVE"
SHA = "75b7df5216524a3c6b3150d698139190c6c44f8f"
VEHICLE_DIR = "Assets/Models/Vehicle/F1TENTH"
INPUTS = {
    "F1TENTH.fbx": (
        f"{VEHICLE_DIR}/F1TENTH.fbx",
        "fe21bb814e8a4bf9572f4b95f0de159ca780a1ce31b0907d1a1d22d4eda7f283",
    ),
    "F1TENTH.fbx.meta": (
        f"{VEHICLE_DIR}/F1TENTH.fbx.meta",
        "ae84f8bc07623fa6896ab92b5db71e9f6c2d35d2f812d9d636dcfca838b5c583",
    ),
    "F1TENTH.prefab": (
        "Assets/Prefabs/F1TENTH/F1TENTH.prefab",
        "a869a1b3d25d567759db052a40c9d87a26ca187bcbe7a64c10ef1ab0bdbd9550",
    ),
    "LICENSE": (
        "LICENSE",
        "b9e7a136ae12a2b6e920435d3a532841ac1df6b2beb0339698b5baab2a68e560",
    ),
}
# sha256 over the sorted "<file name> <sha256>\n" lines of the 118 material
# files (59 .mat and their .meta) under VEHICLE_DIR/Materials.
MATERIALS_DIGEST = (
    "34230b3b42e9728fd1244ebaa8eaf8282d9a5bd7e65d0cf32fec324ecd66a182"
)
EXPECTED_TRIANGLES = 937506

# Joint origins in rear_axle (see urdf/autodrive_roboracer.urdf).
REST_Z = 0.0592
WHEEL_ORIGIN = {
    "rl": (0.0, 0.118, REST_Z),
    "rr": (0.0, -0.118, REST_Z),
    "fl": (0.33, 0.118, REST_Z + 0.010),
    "fr": (0.33, -0.118, REST_Z + 0.010),
}
LIDAR_ORIGIN = (0.2733, 0.0, 0.096 + REST_Z)
HEADER = (
    b"AutoDRIVE F1TENTH.fbx 75b7df5, BSD-2 Tinker Twins; GR-roboracer importer"
)
assert len(HEADER) <= 80 and not HEADER.startswith(b"solid")


def sha256(data):
    return hashlib.sha256(data).hexdigest()


# --------------------------------------------------------------- downloading
def _get(path):
    url = (
        f"https://raw.githubusercontent.com/{REPO}/{SHA}/"
        + urllib.parse.quote(path)
    )
    with urllib.request.urlopen(url, timeout=120) as resp:
        return resp.read()


def download(dest):
    """Fetch every input at the pinned commit into dest, checking hashes."""
    dest = Path(dest)
    (dest / "mats").mkdir(parents=True, exist_ok=True)
    for name, (path, _) in INPUTS.items():
        (dest / name).write_bytes(_get(path))
    tree = json.loads(
        urllib.request.urlopen(
            f"https://api.github.com/repos/{REPO}/git/trees/{SHA}?recursive=1",
            timeout=120,
        ).read()
    )["tree"]
    prefix = f"{VEHICLE_DIR}/Materials/"
    for item in tree:
        p = item["path"]
        if item["type"] == "blob" and p.startswith(prefix):
            (dest / "mats" / p[len(prefix) :]).write_bytes(_get(p))


def check_inputs(root):
    root = Path(root)
    for name, (_, want) in INPUTS.items():
        got = sha256((root / name).read_bytes())
        if got != want:
            raise SystemExit(f"{name}: sha256 {got} != pinned {want}")
    lines = "".join(
        f"{p.name} {sha256(p.read_bytes())}\n"
        for p in sorted((root / "mats").iterdir())
    )
    got = sha256(lines.encode())
    if got != MATERIALS_DIGEST:
        raise SystemExit(f"mats: digest {got} != pinned {MATERIALS_DIGEST}")


# ------------------------------------------------------------- FBX 7400 reader
class Fbx:
    """Binary FBX (7.x) node tree. Only what this importer reads."""

    def __init__(self, data):
        self.data = data
        self.ver = struct.unpack_from("<I", data, 23)[0]
        self.wide = self.ver >= 7500
        o, top = 27, []
        while o < len(data) - 200:
            node, o = self._node(o)
            if node is None:
                break
            top.append(node)
        self.top = {n[0]: n for n in top}

    def _prop(self, o):
        d = self.data
        t = chr(d[o])
        o += 1
        scalar = {
            "Y": ("<h", 2),
            "C": ("<?", 1),
            "I": ("<i", 4),
            "F": ("<f", 4),
            "D": ("<d", 8),
            "L": ("<q", 8),
        }
        if t in scalar:
            fmt, n = scalar[t]
            return struct.unpack_from(fmt, d, o)[0], o + n
        if t in "fdlib":
            _, enc, clen = struct.unpack_from("<III", d, o)
            o += 12
            raw = d[o : o + clen]
            o += clen
            if enc:
                raw = zlib.decompress(raw)
            dt = {"f": "<f4", "d": "<f8", "l": "<i8", "i": "<i4", "b": "?"}[t]
            return np.frombuffer(raw, dtype=dt), o
        if t in "SR":
            n = struct.unpack_from("<I", d, o)[0]
            o += 4
            v = d[o : o + n]
            o += n
            return (v.decode("utf8", "replace") if t == "S" else v), o
        raise ValueError(f"unknown FBX property type {t!r}")

    def _node(self, o):
        d = self.data
        if self.wide:
            end, nprops, _ = struct.unpack_from("<QQQ", d, o)
            o += 24
        else:
            end, nprops, _ = struct.unpack_from("<III", d, o)
            o += 12
        nlen = d[o]
        o += 1
        if end == 0:
            return None, o
        name = d[o : o + nlen].decode()
        o += nlen
        props = []
        for _ in range(nprops):
            v, o = self._prop(o)
            props.append(v)
        kids = []
        while o < end:
            k, o = self._node(o)
            if k is None:
                break
            kids.append(k)
        return (name, props, kids), end


def _p70(node):
    out = {}
    for kid in node[2]:
        if kid[0] == "Properties70":
            for p in kid[2]:
                out[p[1][0]] = p[1][4:]
    return out


def _rot(euler_deg):
    x, y, z = (math.radians(a) for a in euler_deg)
    rx = np.array(
        [
            [1, 0, 0],
            [0, math.cos(x), -math.sin(x)],
            [0, math.sin(x), math.cos(x)],
        ]
    )
    ry = np.array(
        [
            [math.cos(y), 0, math.sin(y)],
            [0, 1, 0],
            [-math.sin(y), 0, math.cos(y)],
        ]
    )
    rz = np.array(
        [
            [math.cos(z), -math.sin(z), 0],
            [math.sin(z), math.cos(z), 0],
            [0, 0, 1],
        ]
    )
    return rz @ ry @ rx


def read_parts(fbx):
    """Yield (object id, name, world-space vertices (mm), tris, slots).

    tris is (n, 3) vertex indices (fan-triangulated polygons) and slots the
    FBX material slot of each triangle.
    """
    objs = fbx.top["Objects"][2]
    geo, models, mats = {}, {}, {}
    for n in objs:
        oid = n[1][0]
        if n[0] == "Geometry":
            geo[oid] = n
        elif n[0] == "Model":
            models[oid] = (n[1][1].split("\x00")[0], _p70(n))
        elif n[0] == "Material":
            mats[oid] = n[1][1].split("\x00")[0]
    parent, geo_model, model_mats = {}, {}, collections.defaultdict(list)
    for c in fbx.top["Connections"][2]:
        if c[1][0] != "OO":
            continue
        child, par = c[1][1], c[1][2]
        if child in models:
            parent[child] = par
        if child in geo:
            geo_model[child] = par
        if child in mats:
            model_mats[par].append(mats[child])

    def local(m):
        p = models[m][1]
        mat = np.eye(4)
        t = p.get("Lcl Translation", [0, 0, 0])
        r = p.get("Lcl Rotation", [0, 0, 0])
        s = p.get("Lcl Scaling", [1, 1, 1])
        pre = p.get("PreRotation", [0, 0, 0])
        mat[:3, :3] = _rot(pre) @ _rot(r) @ np.diag(s)
        mat[:3, 3] = t
        return mat

    def world(m):
        mat = local(m)
        while parent.get(m) in models:
            m = parent[m]
            mat = local(m) @ mat
        return mat

    for gid in sorted(geo):
        node = geo[gid]
        kids = {k[0]: k for k in node[2]}
        verts = kids["Vertices"][1][0].reshape(-1, 3)
        pvi = kids["PolygonVertexIndex"][1][0].astype(np.int64)
        m = geo_model[gid]
        mat = world(m)
        wv = (mat[:3, :3] @ verts.T).T + mat[:3, 3]
        # polygon id of each polygon-vertex, then fan triangulation
        ends = np.flatnonzero(pvi < 0)
        starts = np.concatenate([[0], ends[:-1] + 1])
        idx = np.where(pvi < 0, ~pvi, pvi)
        slot_by_poly = np.zeros(len(ends), dtype=np.int64)
        lem = kids.get("LayerElementMaterial")
        if lem is not None:
            sub = {k[0]: k for k in lem[2]}
            if sub["MappingInformationType"][1][0] == "ByPolygon":
                slot_by_poly = sub["Materials"][1][0].astype(np.int64)
        tris, slots = [], []
        for s, e, slot in zip(starts, ends, slot_by_poly, strict=True):
            for k in range(s + 1, e):
                tris.append((idx[s], idx[k], idx[k + 1]))
                slots.append(slot)
        yield (
            gid,
            models[m][0],
            wv,
            np.array(tris, dtype=np.int64).reshape(-1, 3),
            np.array(slots, dtype=np.int64),
            model_mats[m],
        )


# ---------------------------------------------------------------- Unity files
def _unity_docs(text):
    docs = re.split(r"^--- !u!(\d+) &(\d+).*$", text, flags=re.M)
    return [
        (int(docs[i]), docs[i + 1], docs[i + 2]) for i in range(1, len(docs), 3)
    ]


def mat_colours(root):
    """guid -> (name, rgb 0..1) and name -> rgb for every .mat."""
    by_guid, by_name = {}, {}
    for meta in sorted((Path(root) / "mats").glob("*.mat.meta")):
        guid = re.search(r"guid: (\w+)", meta.read_text()).group(1)
        mat = meta.with_name(meta.name[: -len(".meta")])
        text = mat.read_text()
        name = re.search(r"m_Name: (.*)", text).group(1).strip()
        m = re.search(
            r"- _BaseColor: \{r: ([\d.e+-]+), g: ([\d.e+-]+), b: ([\d.e+-]+)",
            text,
        ) or re.search(
            r"- _Color: \{r: ([\d.e+-]+), g: ([\d.e+-]+), b: ([\d.e+-]+)", text
        )
        rgb = tuple(float(v) for v in m.groups())
        by_guid[guid] = (name, rgb)
        by_name[name] = rgb
    return by_guid, by_name


def prefab_materials(root, by_guid):
    """GameObject name -> [material name per slot], from MeshRenderers."""
    docs = _unity_docs((Path(root) / "F1TENTH.prefab").read_text())
    names = {}
    for cls, fid, body in docs:
        if cls == 1:
            names[fid] = re.search(r"m_Name: (.*)", body).group(1).strip()
    out = {}
    for cls, _, body in docs:
        if cls != 23 or "m_Materials:" not in body:
            continue
        go = re.search(r"m_GameObject: \{fileID: (\d+)", body).group(1)
        block = body.split("m_Materials:")[1].split("m_Static")[0]
        guids = re.findall(r"guid: (\w+)", block)
        out.setdefault(
            names.get(go, "?"), [by_guid[g][0] for g in guids if g in by_guid]
        )
    return out


def srgb8(rgb):
    """Unity stores these linear-looking values as authored; use as-is."""
    return "".join(
        f"{int(round(max(0.0, min(1.0, c)) * 255)):02x}" for c in rgb
    )


# ----------------------------------------------------------------- transform
def to_rear_axle(pts_mm):
    out = np.empty_like(pts_mm, dtype=np.float64)
    out[:, 0] = pts_mm[:, 0] / 1000.0 + 0.243
    out[:, 1] = -(pts_mm[:, 2] / 1000.0 + 0.040)
    out[:, 2] = pts_mm[:, 1] / 1000.0 - 0.050 + REST_Z
    return out


def write_stl(path, tri_xyz):
    """Binary STL; tri_xyz is (n, 3, 3). Deterministic: normals from winding."""
    n = len(tri_xyz)
    t = tri_xyz.astype("<f4")
    a, b, c = (t[:, i, :].astype(np.float64) for i in range(3))
    nrm = np.cross(b - a, c - a)
    mag = np.linalg.norm(nrm, axis=1)
    mag[mag == 0] = 1.0
    nrm = (nrm / mag[:, None]).astype("<f4")
    rec = np.zeros(
        n, dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]
    )
    rec["n"], rec["v"] = nrm, t
    with open(path, "wb") as f:
        f.write(HEADER.ljust(80, b"\x00"))
        f.write(struct.pack("<I", n))
        f.write(rec.tobytes())


# --------------------------------------------------------------------- main
def classify(name, centroid):
    m = re.match(r"Wheel-(\d+)-solid\d+$", name)
    if m:
        return (
            "wheel",
            ("f" if centroid[0] > 0.165 else "r")
            + ("l" if centroid[1] > 0 else "r"),
        )
    if name.startswith("Hokuyo UST"):
        return ("lidar", None)
    return ("chassis", None)


def build(root, outdir):
    root = Path(root)
    by_guid, by_name = mat_colours(root)
    pref = prefab_materials(root, by_guid)
    fbx = Fbx((root / "F1TENTH.fbx").read_bytes())
    groups = collections.defaultdict(list)  # key -> [(colour hex, tris xyz)]
    total = 0
    for _, name, wv, tris, slots, fbx_mats in read_parts(fbx):
        if len(tris) == 0:
            continue
        pts = to_rear_axle(wv)
        kind, corner = classify(name, pts.mean(axis=0))
        slot_mats = pref.get(name) or fbx_mats
        for slot in np.unique(slots):
            sel = tris[slots == slot]
            mname = (
                slot_mats[min(slot, len(slot_mats) - 1)] if slot_mats else None
            )
            rgb = by_name.get(mname, (0.5, 0.5, 0.5))
            groups[(kind, corner)].append((srgb8(rgb), pts[sel]))
            total += len(sel)
    if total != EXPECTED_TRIANGLES:
        raise SystemExit(f"{total} triangles, expected {EXPECTED_TRIANGLES}")
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    report = {}
    for (kind, corner), items in sorted(
        groups.items(), key=lambda kv: str(kv[0])
    ):
        if kind == "chassis":
            by_colour = collections.defaultdict(list)
            for hexc, xyz in items:
                by_colour[hexc].append(xyz)
            for hexc, arrs in sorted(by_colour.items()):
                p = outdir / f"chassis_{hexc}.stl"
                write_stl(p, np.concatenate(arrs))
                report[p.name] = sum(len(a) for a in arrs)
        else:
            origin = np.array(
                WHEEL_ORIGIN[corner] if kind == "wheel" else LIDAR_ORIGIN
            )
            xyz = np.concatenate([a for _, a in items]) - origin
            p = outdir / (
                f"wheel_{corner}.stl" if kind == "wheel" else "lidar.stl"
            )
            write_stl(p, xyz)
            report[p.name] = len(xyz)
            weight = collections.Counter()
            for hexc, a in items:
                weight[hexc] += len(a)
            report[p.name + ":colour"] = weight.most_common(1)[0][0]
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("outdir", help="meshes/autodrive_roboracer")
    ap.add_argument("--inputs", help="a directory of already downloaded inputs")
    args = ap.parse_args(argv)
    with tempfile.TemporaryDirectory() as tmp:
        root = args.inputs or tmp
        if not args.inputs:
            download(root)
        check_inputs(root)
        report = build(root, args.outdir)
    for k, v in report.items():
        print(k, v)
    return 0


if __name__ == "__main__":
    sys.exit(main())
