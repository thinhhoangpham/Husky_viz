"""The water_tower type registration.

Registered ahead of the world edit on purpose: sdf_parse.parse_models drops any
model whose name classifies as "skip" (sdf_parse.py:78), so the registry must
know the "water_tower" prefix before a model of that name is visible to the
parser at all.
"""
from map_tools.park_types import classify_prefix, BY_PREFIX


def test_water_tower_registered():
    assert classify_prefix("water_tower") == "water_tower"
    t = BY_PREFIX["water_tower"]
    assert t.is_object is True
    # NOT classified: every is_catalog reader is classifier machinery
    # (classify.KNOWN_RADIUS, the matcher catalog, localizer_node._LABEL_COLOR)
    # and the tower is matched by shape descriptor instead. Keeping this False
    # is also what upholds test_score.py's invariant that every catalog identity
    # is scoreable -- the tower deliberately has no score_family.
    assert t.is_catalog is False
    assert t.score_family is None
    assert t.mesh is None
    assert t.disc_radius == 2.5
