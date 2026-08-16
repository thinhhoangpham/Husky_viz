from map_tools.park_types import classify_prefix, BY_PREFIX


def test_postescable_classifies():
    assert classify_prefix("postescable_pole0") == "postescable"
    assert classify_prefix("postescable") == "postescable"


def test_postescable_is_object_and_catalog():
    t = BY_PREFIX["postescable"]
    assert t.is_object and t.is_catalog
    assert t.identity == "postescable"
