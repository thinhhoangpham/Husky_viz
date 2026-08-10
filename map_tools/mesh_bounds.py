"""Compute a mesh's ground footprint (dx, dy) in metres from a COLLADA .dae.

Unions ALL <float_array id="...positions...">, so multi-submesh models (a bench
is 5 sub-meshes: legs, slats, frame) get the true extent, not one piece. Returns
the local x and y extents scaled by the world <scale> (0.15 for park furniture).
z is ignored (yaw-only box footprint, per the plan).
"""
import re

_POS_RE = re.compile(
    r'<float_array id="[^"]*positions[^"]*"[^>]*>([^<]*)</float_array>', re.I)


def footprint_dxdy(dae_path, scale=0.15):
    txt = open(dae_path).read()
    xs, ys = [], []
    for data in _POS_RE.findall(txt):
        vals = [float(v) for v in data.split()]
        xs.extend(vals[0::3])
        ys.extend(vals[1::3])
    if not xs:
        raise ValueError("no positions arrays in %s" % dae_path)
    return (max(xs) - min(xs)) * scale, (max(ys) - min(ys)) * scale
