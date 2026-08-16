"""Minimal Wavefront .obj triangle reader.

Reads only `v` (vertex) and `f` (face) lines; everything else (`vn`, `vt`,
`usemtl`, `o`, `g`, comments) is ignored. Polygon faces are triangulated by
fan. No third-party mesh libraries -- stdlib + numpy only.
"""
import numpy as np


def read_obj_triangles(obj_path, scale=1.0):
    """Return an (M,3,3) array of triangles (in metres) from a .obj file.

    Face vertex tokens may be `v`, `v/vt`, or `v/vt/vn` -- only the vertex
    index (before the first `/`) is used. Indices are 1-based per the OBJ
    spec; negative (relative-to-end) indices are also supported.
    """
    vertices = []
    faces = []  # list of lists of 0-based vertex indices

    with open(obj_path, "r") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            tokens = line.split()
            tag = tokens[0]
            if tag == "v":
                vertices.append([float(x) for x in tokens[1:4]])
            elif tag == "f":
                face = []
                for tok in tokens[1:]:
                    vidx_str = tok.split("/")[0]
                    vidx = int(vidx_str)
                    if vidx < 0:
                        vidx = len(vertices) + vidx  # relative to current count
                    else:
                        vidx = vidx - 1  # 1-based -> 0-based
                    face.append(vidx)
                faces.append(face)

    verts = np.asarray(vertices, dtype=float)

    triangles = []
    for face in faces:
        v0 = face[0]
        for i in range(1, len(face) - 1):
            triangles.append([verts[v0], verts[face[i]], verts[face[i + 1]]])

    if not triangles:
        return np.zeros((0, 3, 3), dtype=float)

    tris = np.asarray(triangles, dtype=float) * scale
    return tris
