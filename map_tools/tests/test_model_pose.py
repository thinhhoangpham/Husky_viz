"""Pose-precedence tests: <state> must override the <model> definition.

Getting this backwards moves terrain by metres -- see model_pose.py for the
measured errors in this repo's own worlds.
"""
import os

import pytest

from map_tools.model_pose import resolve_pose, resolve_pose_from_file

WORLDS = os.path.join(os.path.dirname(__file__), "..", "..",
                      "natural_environments_ros_opt", "natural_enviroment",
                      "worlds")
PARK = os.path.join(WORLDS, "park.world")
LAKE = os.path.join(WORLDS, "lake.world")


def _world(models_xml, state_xml=""):
    state = ("<state world_name='default'>%s</state>" % state_xml
             if state_xml else "")
    return "<sdf><world name='default'>%s%s</world></sdf>" % (models_xml,
                                                              state)


# ---------------------------------------------------------------- synthetic

def test_state_pose_overrides_definition_pose():
    txt = _world(
        "<model name='ground'><link name='link'>"
        "<pose>9 9 9 0 0 0</pose></link>"
        "<pose>1 2 3 0 0 0</pose></model>",
        "<model name='ground'><pose>10 20 30 0 -0 0</pose>"
        "<link name='link'><pose>0 0 0 0 0 0</pose></link></model>")
    p = resolve_pose(txt, "ground")
    assert p.source == "state"
    assert (p.x, p.y, p.z) == (10.0, 20.0, 30.0)


def test_definition_pose_used_when_model_absent_from_state():
    """A state block exists but says nothing about this model -- the
    definition pose is then the live one (park's terrain was believed to be
    this case)."""
    txt = _world(
        "<model name='ground'><link name='link'>"
        "<pose>9 9 9 0 0 0</pose></link>"
        "<pose>1 2 3 0 0 0</pose></model>",
        "<model name='other'><pose>10 20 30 0 -0 0</pose></model>")
    p = resolve_pose(txt, "ground")
    assert p.source == "definition"
    assert (p.x, p.y, p.z) == (1.0, 2.0, 3.0)


def test_definition_pose_used_when_no_state_block_at_all():
    txt = _world("<model name='ground'><pose>1 2 3 0 0 0</pose></model>")
    p = resolve_pose(txt, "ground")
    assert p.source == "definition"
    assert (p.x, p.y, p.z) == (1.0, 2.0, 3.0)


def test_link_pose_is_not_mistaken_for_the_model_pose():
    """The model's own <pose> can come AFTER its <link>, so 'first pose in the
    block' would return the link's. Nested pose-bearing elements are stripped
    precisely to avoid that."""
    txt = _world(
        "<model name='ground'>"
        "<link name='link'><pose>7 7 7 0 0 0</pose>"
        "<collision name='c'><pose>8 8 8 0 0 0</pose></collision></link>"
        "<pose>1 2 3 0 0 0</pose></model>")
    p = resolve_pose(txt, "ground")
    assert (p.x, p.y, p.z) == (1.0, 2.0, 3.0)


def test_missing_model_returns_none():
    txt = _world("<model name='ground'><pose>1 2 3 0 0 0</pose></model>")
    assert resolve_pose(txt, "nope") is None


def test_exponent_and_signed_floats_parse():
    """Gazebo writes poses like '2.6e-05 2e-06 -2e-06'."""
    txt = _world(
        "<model name='g'><pose>2.6e-05 2e-06 -2e-06 0 -0 0</pose></model>")
    p = resolve_pose(txt, "g")
    assert p.x == pytest.approx(2.6e-05)
    assert p.z == pytest.approx(-2e-06)


def test_nested_child_model_does_not_truncate_the_parent():
    """A model containing a child model must close on its OWN </model>."""
    txt = _world(
        "<model name='outer'>"
        "<model name='inner'><pose>5 5 5 0 0 0</pose></model>"
        "<pose>1 2 3 0 0 0</pose></model>")
    p = resolve_pose(txt, "outer")
    assert (p.x, p.y, p.z) == (1.0, 2.0, 3.0)


# ------------------------------------------------------- the real worlds
# These pin the actual measured values. If a world file is re-saved from
# Gazebo and a pose shifts, this is the test that should fail loudly rather
# than the DTM silently moving.

def test_lake_terrain_uses_state_pose():
    p = resolve_pose_from_file(LAKE, "terreno_lago")
    assert p.source == "state"
    # Definition says (-5.97581, 12.5754, 0); the state override is what runs.
    assert (p.x, p.y, p.z) == pytest.approx((0.0, 0.0, 5.0))


def test_lake_water_uses_state_pose():
    p = resolve_pose_from_file(LAKE, "lago")
    assert p.source == "state"
    assert (p.x, p.y, p.z) == pytest.approx(
        (-12.6254, 5.66516, -0.828242), abs=1e-6)


def test_park_terrain_model_is_named_parque_and_uses_state_pose():
    """The park terrain MODEL is 'parque'. 'terreno_parque' is only the mesh
    directory in the model:// URI -- no model has that name."""
    assert resolve_pose_from_file(PARK, "terreno_parque") is None
    p = resolve_pose_from_file(PARK, "parque")
    assert p.source == "state"
    assert (p.x, p.y, p.z) == pytest.approx(
        (0.0, -1.55121, 2.98891), abs=1e-6)


def test_park_terrain_state_pose_actually_differs_from_definition():
    """Guards the precedence itself: if these were equal the override would be
    untested against the real file."""
    p = resolve_pose_from_file(PARK, "parque")
    assert (p.x, p.y, p.z) != (0.0, 0.0, 0.0)
