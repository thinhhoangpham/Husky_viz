"""Analytic surface sampling for objects built from SDF primitives.

The water tower is built from SDF <cylinder> primitives rather than a mesh
file, so map_tools.mesh_sample.sample_surface -- which samples triangles out of
a loaded mesh -- has nothing to read. Sampling the primitives' surfaces
analytically produces the map-side point set in exactly the same form
landmark_loc.descriptor.describe() consumes, so the map and the live
observation stay in one space.

Sampling the SURFACE, not the volume, is the point: a lidar returns points off
an object's outer skin, so a surface sample is what a scan of the object can
be compared against. Faces are drawn in proportion to their true areas, so the
resulting density matches a uniform sampling of the whole outer skin rather
than over-weighting small faces.

numpy only, and deterministic for a fixed seed -- the committed map artifact
must be stable.
"""
import numpy as np


def sample_cylinder_stack(stack, n=4000, seed=0):
    """Sample `n` points over the surfaces of a vertical stack of cylinders.

    `stack` is a list of `(radius, length, z_centre)` tuples in metres, each an
    axis-aligned cylinder centred on the model's vertical axis, with the origin
    at the model's base. For the water tower that is
    `[(1.0, 4.0, 2.0), (2.5, 5.0, 8.5)]` -- a 4 m pedestal carrying a 5 m tank,
    ~11 m tall overall.

    Each cylinder contributes three faces: its lateral wall and its two caps.
    All faces across the whole stack are pooled and sampled in proportion to
    their areas, so a point is equally likely to land anywhere on the object's
    outer skin.

    Interior surfaces where one cylinder meets another are NOT removed. They
    are a small fraction of the total area, and excluding them would mean
    modelling the intersection of arbitrary stacks for no measurable gain in
    the descriptor.

    Returns an (n, 3) float array. Deterministic for a fixed `seed`.
    """
    if n <= 0:
        raise ValueError("n must be positive, got %r" % (n,))
    if not stack:
        raise ValueError("stack must contain at least one cylinder")

    # Build the face table: (kind, radius, length, z_centre, area).
    # kind 0 = lateral wall, 1 = bottom cap, 2 = top cap.
    faces = []
    for radius, length, z_centre in stack:
        if radius <= 0 or length <= 0:
            raise ValueError(
                "cylinder radius and length must be positive, got "
                "radius=%r length=%r" % (radius, length))
        cap_area = np.pi * radius ** 2
        faces.append((0, radius, length, z_centre, 2 * np.pi * radius * length))
        faces.append((1, radius, length, z_centre, cap_area))
        faces.append((2, radius, length, z_centre, cap_area))

    areas = np.array([f[4] for f in faces], dtype=float)
    probs = areas / areas.sum()

    rng = np.random.RandomState(seed)
    # Draw every face assignment and every uniform up front, so the output
    # depends only on the seed and not on per-face call ordering.
    face_idx = rng.choice(len(faces), size=n, p=probs)
    theta = rng.uniform(0.0, 2.0 * np.pi, size=n)
    u = rng.uniform(0.0, 1.0, size=n)

    out = np.empty((n, 3), dtype=float)
    for i, (kind, radius, length, z_centre, _area) in enumerate(faces):
        sel = face_idx == i
        if not sel.any():
            continue
        th = theta[sel]
        uu = u[sel]
        if kind == 0:
            # Lateral wall: fixed radius, height uniform along the axis.
            r = radius
            z = z_centre + (uu - 0.5) * length
        else:
            # Cap disc: sqrt keeps the radial density uniform over the area
            # (a plain uniform radius would crowd points at the centre).
            r = radius * np.sqrt(uu)
            z = z_centre + (length / 2.0 if kind == 2 else -length / 2.0)
        out[sel, 0] = r * np.cos(th)
        out[sel, 1] = r * np.sin(th)
        out[sel, 2] = z
    return out
