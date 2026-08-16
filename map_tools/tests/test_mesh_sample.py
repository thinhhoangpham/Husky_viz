import os

import numpy as np

from map_tools.mesh_sample import sample_surface

LINEA1 = os.path.join(os.path.dirname(__file__), "..", "..",
                      "models_lake_opt", "linea1", "postes.dae")


def test_sample_count_and_determinism():
    a = sample_surface(LINEA1, 0.03, n=2000, seed=0)
    b = sample_surface(LINEA1, 0.03, n=2000, seed=0)
    assert a.shape == (2000, 3)
    assert np.array_equal(a, b)


def test_different_seeds_differ():
    a = sample_surface(LINEA1, 0.03, n=2000, seed=0)
    b = sample_surface(LINEA1, 0.03, n=2000, seed=1)
    assert not np.array_equal(a, b)


def test_sample_within_mesh_bounds():
    from map_tools.mesh_bounds import bounds3d

    pts = sample_surface(LINEA1, 0.03, n=2000, seed=1)
    # bounds3d returns SIX floats: half-extents then centres, already scaled.
    half_dx, half_dy, half_dz, cx, cy, cz = bounds3d(LINEA1, 0.03)

    assert cx - half_dx - 0.1 <= pts[:, 0].min()
    assert pts[:, 0].max() <= cx + half_dx + 0.1
    assert cz - half_dz - 0.1 <= pts[:, 2].min()
    assert pts[:, 2].max() <= cz + half_dz + 0.1

    # the pole mesh is ~16.5 m tall at this scale
    assert 2 * half_dz > 12.0
    assert pts[:, 2].max() - pts[:, 2].min() > 12.0


def test_samples_spread_not_bunched():
    """Guards the interleaved-<p> trap: if the VERTEX index stride were read
    wrong the geometry collapses into a few degenerate blobs, so require the
    points to be spread across the mesh's full height rather than clustered.
    """
    pts = sample_surface(LINEA1, 0.03, n=4000, seed=0)
    z = pts[:, 2]
    span = z.max() - z.min()
    # Every decile of height should contain some points.
    hist, _ = np.histogram(z, bins=10, range=(z.min(), z.max()))
    assert (hist > 0).all()
    # And the sample should not be concentrated in a thin slab.
    assert (np.percentile(z, 95) - np.percentile(z, 5)) > 0.5 * span
