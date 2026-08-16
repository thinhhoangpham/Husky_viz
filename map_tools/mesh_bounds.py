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

import numpy as np

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


def bounds3d(dae_path, scale=0.15):
    """Like footprint() but also returns z half-extent and z center.

    Applies each visual_scene node's <matrix> to its geometry vertices before
    bounding (same as footprint), so translated/rotated geometry is handled.
    Returns (half_dx, half_dy, half_dz, cx, cy, cz), all scaled to metres.
    """
    txt = _strip_namespace(open(dae_path).read())
    root = ET.fromstring(txt)

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

    xs, ys, zs = [], [], []
    for node in root.findall(".//visual_scene//node"):
        ig = node.find(".//instance_geometry")
        if ig is None:
            continue
        gid = ig.get("url", "").lstrip("#")
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
            xs.append(R[0] * vx + R[1] * vy + R[2] * vz + T[0])
            ys.append(R[3] * vx + R[4] * vy + R[5] * vz + T[1])
            zs.append(R[6] * vx + R[7] * vy + R[8] * vz + T[2])

    if not xs:
        raise ValueError("no positions arrays in %s" % dae_path)

    half_dx = (max(xs) - min(xs)) / 2.0 * scale
    half_dy = (max(ys) - min(ys)) / 2.0 * scale
    half_dz = (max(zs) - min(zs)) / 2.0 * scale
    cx = (min(xs) + max(xs)) / 2.0 * scale
    cy = (min(ys) + max(ys)) / 2.0 * scale
    cz = (min(zs) + max(zs)) / 2.0 * scale
    return half_dx, half_dy, half_dz, cx, cy, cz


def _node_transforms(root):
    """Yield (geometry_id, R, T) for every <visual_scene> node that instances a
    geometry, R row-major 3x3 and T a 3-vector, exactly as footprint()/bounds3d()
    derive them from the node's <matrix>.

    Applying these is NOT optional -- see the module docstring. Ignoring node
    matrices is what once put the bench footprint ~1.2 m off its true position.
    """
    for node in root.findall(".//visual_scene//node"):
        ig = node.find(".//instance_geometry")
        if ig is None:
            continue
        gid = ig.get("url", "").lstrip("#")
        mat_el = node.find("matrix")
        if mat_el is not None:
            m = _parse_floats(mat_el.text)
            R = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]]
            T = [m[3], m[7], m[11]]
        else:
            R, T = _identity()
        yield gid, R, T


def _triangles(dae_path, scale=0.15):
    """Return this mesh's faces as an (M, 3, 3) array: M triangles, 3 vertices
    each, 3 coords each, in metres at `scale`, with node transforms applied.

    bounds3d() only needs vertex POSITIONS, so it can ignore face indices
    entirely. A surface sampler cannot: it needs to know which vertices form a
    face in order to weight by triangle area. Hence this separate walk, which
    reuses the same node-matrix transform (_node_transforms) so geometry lands
    in exactly the same place bounds3d puts it.

    Two COLLADA details that will silently produce garbage if you skip them:

    1. The <p> index array is INTERLEAVED across all the <triangles> inputs.
       postes.dae declares VERTEX at offset 0 and NORMAL at offset 1, so <p>
       holds 2 indices per vertex and its length is count*3*2. Consuming every
       integer would read normal indices as if they were positions. The stride
       is the number of DISTINCT offsets, read from the inputs -- never
       hardcoded, because other meshes here may declare a different input set
       (adding TEXCOORD would make the stride 3).

    2. A VERTEX input does not point at a positions source directly; it points
       at a <vertices> element that redirects to the POSITION source. That
       indirection has to be resolved, otherwise the source lookup fails.
    """
    txt = _strip_namespace(open(dae_path).read())
    root = ET.fromstring(txt)

    # geometry-id -> (M_i, 3, 3) faces in the geometry's own (untransformed) frame
    geom_faces = {}
    for geom in root.findall(".//geometry"):
        gid = geom.get("id")

        # source-id -> flat float list, so a VERTEX input can be resolved to
        # whichever source its <vertices> element names as POSITION.
        sources = {}
        for src in geom.findall(".//source"):
            arr = src.find("float_array")
            if arr is None:
                continue
            sources[src.get("id")] = _parse_floats(arr.text or "")

        # <vertices id> -> POSITION source id (the indirection from detail 2)
        vertices_pos = {}
        for vtx in geom.findall(".//vertices"):
            for inp in vtx.findall("input"):
                if inp.get("semantic") == "POSITION":
                    vertices_pos[vtx.get("id")] = inp.get("source", "").lstrip("#")

        faces = []
        for tris in geom.findall(".//triangles"):
            inputs = tris.findall("input")
            offsets = set()
            vert_offset = None
            vert_source = None
            for inp in inputs:
                off = int(inp.get("offset", "0"))
                offsets.add(off)
                if inp.get("semantic") == "VERTEX":
                    vert_offset = off
                    vert_source = inp.get("source", "").lstrip("#")
            if vert_offset is None:
                continue
            stride = len(offsets)

            pos_id = vertices_pos.get(vert_source, vert_source)
            coords = sources.get(pos_id)
            if not coords:
                continue

            p_el = tris.find("p")
            if p_el is None or not (p_el.text or "").strip():
                continue
            idx = [int(v) for v in p_el.text.split()]
            # Take only the VERTEX index out of each interleaved tuple.
            vidx = idx[vert_offset::stride]
            # Trailing partial triangle would be malformed input; drop it rather
            # than index out of range.
            n_tri = len(vidx) // 3
            for t in range(n_tri):
                tri = []
                for k in range(3):
                    i = vidx[3 * t + k] * 3
                    tri.append((coords[i], coords[i + 1], coords[i + 2]))
                faces.append(tri)

        if faces:
            geom_faces.setdefault(gid, []).extend(faces)

    out = []
    for gid, R, T in _node_transforms(root):
        faces = geom_faces.get(gid)
        if not faces:
            continue
        for tri in faces:
            wtri = []
            for vx, vy, vz in tri:
                wtri.append((
                    (R[0] * vx + R[1] * vy + R[2] * vz + T[0]) * scale,
                    (R[3] * vx + R[4] * vy + R[5] * vz + T[1]) * scale,
                    (R[6] * vx + R[7] * vy + R[8] * vz + T[2]) * scale,
                ))
            out.append(wtri)

    if not out:
        raise ValueError("no triangles in %s" % dae_path)
    return np.asarray(out, dtype=float)


def footprint_dxdy(dae_path, scale=0.15):
    """Backward-compat wrapper: full extents only, no center offset."""
    half_dx, half_dy, _cx, _cy = footprint(dae_path, scale)
    return 2.0 * half_dx, 2.0 * half_dy
