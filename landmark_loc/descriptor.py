"""NDT-style shape descriptor: points -> per-height-band voxel-shape statistics.

Pure numpy, no ROS. Shared by map extraction (mesh-sampled points) and the
runtime detector (lidar points) so the two sides cannot drift. See the design
doc's "Descriptor" section: local shape from covariance eigenvalue RATIOS is a
LOCAL property, so a near-face voxel classifies correctly without the far side.
"""
import numpy as np


def voxel_shape(points):
    """Return (linearity, planarity, sphericity) for an (N,3) point set.

    From the covariance eigenvalues l1>=l2>=l3 (Demantke et al.):
        linearity   = (l1 - l2) / l1
        planarity   = (l2 - l3) / l1
        sphericity  =  l3       / l1
    Normalised to sum to 1. Degenerate input -> (0, 0, 1).
    """
    p = np.asarray(points, dtype=float)
    if p.shape[0] < 3:
        return (0.0, 0.0, 1.0)
    cov = np.cov((p - p.mean(axis=0)).T)
    evals = np.linalg.eigvalsh(cov)  # ascending
    l3, l2, l1 = float(evals[0]), float(evals[1]), float(evals[2])
    if l1 <= 1e-12:
        return (0.0, 0.0, 1.0)
    lin = (l1 - l2) / l1
    pla = (l2 - l3) / l1
    sph = l3 / l1
    s = lin + pla + sph
    if s <= 1e-12:
        return (0.0, 0.0, 1.0)
    return (lin / s, pla / s, sph / s)
