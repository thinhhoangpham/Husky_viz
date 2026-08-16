"""Sample points across mesh SURFACES (area-weighted), not raw vertices.

The descriptor must see a laser-like point set. Raw COLLADA vertices cluster
wherever the modeller happened to add detail -- a pole's bolt head can carry
more vertices than a whole metre of smooth mast -- so a descriptor built from
them would not correspond to anything a lidar returns. Sampling uniformly over
surface AREA fixes that: every square metre of the structure contributes points
in proportion to how much of it a beam could actually hit.

Reuses mesh_bounds._triangles so scale and the COLLADA node transforms are
applied exactly as the rest of map_tools applies them (see that module's
docstring for why skipping node matrices is a real, previously-shipped bug).

Determinism matters here: the descriptor map is built offline and must be
reproducible, so the RNG is seeded per call and nothing depends on dict order.
"""
import numpy as np

from map_tools.mesh_bounds import _triangles


def sample_surface(dae_path, scale, n=4000, seed=0):
    """Return (n, 3) points sampled uniformly by triangle area over the mesh
    surface of `dae_path`, in mesh-local metres at `scale`.

    Deterministic given `seed`.
    """
    tris = np.asarray(_triangles(dae_path, scale), dtype=float)  # (M,3,3)
    if len(tris) == 0:
        return np.zeros((0, 3))

    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    total = areas.sum()
    if total <= 0:
        # Every face is degenerate (zero area) -- there is no surface to sample.
        return np.zeros((0, 3))

    # RandomState rather than default_rng: its stream is guaranteed stable
    # across numpy versions, so a descriptor map rebuilt later still matches.
    rng = np.random.RandomState(seed)
    tri_idx = rng.choice(len(tris), size=n, p=areas / total)

    # Uniform point in a triangle: draw (u, w) in the unit square and reflect
    # the half that falls outside u + w <= 1 back into the triangle.
    u = rng.rand(n, 1)
    w = rng.rand(n, 1)
    over = (u + w > 1).ravel()
    u[over] = 1.0 - u[over]
    w[over] = 1.0 - w[over]

    a = v0[tri_idx]
    return a + u * (v1[tri_idx] - a) + w * (v2[tri_idx] - a)
