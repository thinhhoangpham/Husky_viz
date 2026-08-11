"""Compute a mesh's ground footprint (extents + center offset) in metres from a
COLLADA .dae.

Unions ALL <float_array id="...positions...">, so multi-submesh models (a bench
is 5 sub-meshes: legs, slats, frame) get the true extent, not one piece.

Applies the <visual_scene> <node> <matrix> transforms (rotation + translation)
that COLLADA authoring tools attach per-node, BEFORE computing bounds. Without
this, meshes whose sub-parts are authored off the mesh origin (e.g. the bench,
whose nodes translate the geometry ~1.4 m) get a footprint centered on the
mesh origin instead of on the actual geometry -- the extractor then stamps the
static-map box ~1.2 m away from the physical object. Trees (tronco4.dae) have
no node matrices (identity), so they are unaffected.

Returns extents and a center offset, both scaled by the world <scale> (0.15
for park furniture). z is ignored (yaw-only box footprint, per the plan).
"""
import re
import xml.etree.ElementTree as ET

_NS_RE = re.compile(r'\sxmlns="[^"]*"')


def _strip_namespace(txt):
    return _NS_RE.sub("", txt)


def _parse_floats(text):
    return [float(v) for v in text.split()]


def _identity():
    # R (row-major 3x3), T (3,)
    return [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0]


def footprint(dae_path, scale=0.15):
    txt = _strip_namespace(open(dae_path).read())
    root = ET.fromstring(txt)

    # geometry-id -> list of raw (x, y, z) vertices, unioned across all
    # <float_array id="...positions..."> inside that <geometry>.
    geom_positions = {}
    for geom in root.findall(".//geometry"):
        gid = geom.get("id")
        verts = []
        for arr in geom.findall(".//float_array"):
            arr_id = arr.get("id") or ""
            if "positions" not in arr_id.lower():
                continue
            vals = _parse_floats(arr.text or "")
            verts.extend(zip(vals[0::3], vals[1::3], vals[2::3]))
        if verts:
            geom_positions.setdefault(gid, []).extend(verts)

    xs, ys = [], []
    for node in root.findall(".//visual_scene//node"):
        ig = node.find(".//instance_geometry")
        if ig is None:
            continue
        url = ig.get("url", "")
        gid = url.lstrip("#")
        verts = geom_positions.get(gid)
        if not verts:
            continue

        mat_el = node.find("matrix")
        if mat_el is not None:
            m = _parse_floats(mat_el.text)
            R = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]]
            T = [m[3], m[7], m[11]]
        else:
            R, T = _identity()

        for vx, vy, vz in verts:
            wx = R[0] * vx + R[1] * vy + R[2] * vz + T[0]
            wy = R[3] * vx + R[4] * vy + R[5] * vz + T[1]
            xs.append(wx)
            ys.append(wy)

    if not xs:
        raise ValueError("no positions arrays in %s" % dae_path)

    half_dx = (max(xs) - min(xs)) / 2.0 * scale
    half_dy = (max(ys) - min(ys)) / 2.0 * scale
    cx = (min(xs) + max(xs)) / 2.0 * scale
    cy = (min(ys) + max(ys)) / 2.0 * scale
    return half_dx, half_dy, cx, cy


def footprint_dxdy(dae_path, scale=0.15):
    """Backward-compat wrapper: full extents only, no center offset."""
    half_dx, half_dy, _cx, _cy = footprint(dae_path, scale)
    return 2.0 * half_dx, 2.0 * half_dy
