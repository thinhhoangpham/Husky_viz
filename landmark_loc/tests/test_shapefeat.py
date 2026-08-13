import numpy as np
from landmark_loc import shapefeat as sf


def _lamp_post(height=2.5, w=0.14, n=40):
    """Thin near-round vertical post from z=0 to height."""
    pts = []
    for z in np.linspace(0.0, height, n):
        for a in np.linspace(0, 2 * np.pi, 6, endpoint=False):
            pts.append((0.5 * w * np.cos(a), 0.5 * w * np.sin(a), z))
    return np.array(pts, float)


def _low_box(major=1.78, minor=0.80, height=0.9, n=8):
    """Low wide rectangular box (bench-like)."""
    pts = []
    for z in np.linspace(0.0, height, 4):
        for x in np.linspace(-major / 2, major / 2, n):
            for y in (-minor / 2, minor / 2):
                pts.append((x, y, z))
    return np.array(pts, float)


def test_post_width_isolates_thin_post():
    p = _lamp_post()
    assert sf.post_width(p) < 0.25          # thin post, not the head/full box


def test_has_thin_high_band_true_for_lamp():
    assert sf.has_thin_high_band(_lamp_post(height=2.5)) is True


def test_has_thin_high_band_false_for_low_box():
    assert sf.has_thin_high_band(_low_box()) is False


def test_has_thin_high_band_false_for_short_post():
    # a post that stops below HIGH_Z has no thin band up high
    assert sf.has_thin_high_band(_lamp_post(height=1.0)) is False


def test_foot_extents_recovers_box_major():
    mj, mn = sf.foot_extents(_low_box(major=1.78, minor=0.80))
    assert abs(mj - 1.78) < 0.2 and abs(mn - 0.80) < 0.2


def test_circle_roundness_round_post_low_ratio():
    r, ratio = sf.circle_roundness(_lamp_post()[:, :2])
    assert ratio < 0.15                      # round


def test_circle_roundness_oblong_box_high_ratio():
    r, ratio = sf.circle_roundness(_low_box(major=0.68, minor=0.38)[:, :2])
    assert ratio > 0.15                      # not round


def test_too_few_points_safe():
    p = np.zeros((3, 3))
    assert sf.post_width(p) == 0.0
    assert sf.has_thin_high_band(p) is False
