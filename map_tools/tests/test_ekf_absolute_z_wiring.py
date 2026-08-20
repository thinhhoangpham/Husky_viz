"""Guards the absolute-z wiring across the dual-EKF setup.

This wiring has now been broken in BOTH directions, so it is pinned here:

  * fused in the map filter while stamped "odom"  -> the filter applied its own
    map->odom output to its own input (robot_localization transforms a pose
    measurement into world_frame before fusing) and settled 1.228 m high.
  * removed from the map filter entirely         -> nothing there observed z, so
    it estimated 0 and emitted map->odom z = -3.839, cancelling the odom filter's
    correct height and putting the robot underground.

The fix is to stamp the measurement in the frame it is actually expressed in
(map, absolute) and fuse it in both filters. These tests assert the three pieces
that make that true and stay true.
"""
import os
import re

import yaml

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_MAP_YAML = os.path.join(_ROOT, "natural_environments_ros_opt", "husky",
                         "husky_control", "config", "localization_map.yaml")
_ODOM_YAML = os.path.join(_ROOT, "natural_environments_ros_opt", "husky",
                          "husky_control", "config", "localization.yaml")
_SCRIPT = os.path.join(_ROOT, "scripts", "publish_ground_height_odom.py")
_RUNBOOK = os.path.join(_ROOT, "RUN-MAP-NAV.md")

_Z_INDEX = 2  # position of the z flag in robot_localization's 15-element config


def _load(path):
    with open(path) as fh:
        return yaml.safe_load(fh)


def _ground_height_input(cfg):
    """(prefix, config-list) for whichever odomN input is odometry/ground_height."""
    for key, value in cfg.items():
        if re.fullmatch(r"odom\d+", key) and value == "odometry/ground_height":
            return key, cfg[key + "_config"]
    return None, None


def test_map_filter_observes_absolute_z():
    """The map filter must have a z input. Without one it does not inherit the
    odom filter's height -- it estimates z at 0 and negates it into map->odom."""
    cfg = _load(_MAP_YAML)
    key, z_cfg = _ground_height_input(cfg)
    assert key is not None, "map EKF has no odometry/ground_height input"
    assert z_cfg[_Z_INDEX] is True, "%s must fuse z" % key
    assert cfg[key + "_differential"] is False, "absolute elevation, not a delta"


def test_odom_filter_still_observes_absolute_z():
    cfg = _load(_ODOM_YAML)
    key, z_cfg = _ground_height_input(cfg)
    assert key is not None, "odom EKF has no odometry/ground_height input"
    assert z_cfg[_Z_INDEX] is True


def test_no_other_map_input_claims_z():
    """Only the ground-height input may assert z in the map filter. abs_fix in
    particular carries z = 0.0, so fusing its z would drag the robot to zero."""
    cfg = _load(_MAP_YAML)
    gh_key, _ = _ground_height_input(cfg)
    for key, value in cfg.items():
        if re.fullmatch(r"(odom|pose)\d+", key) and key != gh_key:
            assert cfg[key + "_config"][_Z_INDEX] is False, \
                "%s (%s) must not assert z" % (key, value)


def test_both_filters_keep_two_d_mode_off():
    """two_d_mode forces z to zero. Either filter having it on flattens the
    composed map->base_link regardless of what the other one estimates."""
    assert _load(_MAP_YAML)["two_d_mode"] is False
    assert _load(_ODOM_YAML)["two_d_mode"] is False


def test_publisher_frame_id_is_configurable_and_defaults_to_odom():
    """The param exists (so the runbook can ask for "map") and its default is
    unchanged, so any pre-existing caller keeps its old behaviour."""
    src = open(_SCRIPT).read()
    assert 'rospy.get_param("~frame_id", "odom")' in src
    # and the published stamp must use it, not a literal
    assert "od.header.frame_id = frame_id" in src
    assert 'od.header.frame_id = "odom"' not in src


def test_runbook_starts_the_publisher_in_the_map_frame():
    """The map filter's z is only feedback-free because the message is stamped
    "map"; the node's own default is "odom", so the runbook must pass it."""
    text = open(_RUNBOOK).read()
    line = [l for l in text.splitlines()
            if "publish_ground_height_odom.py" in l and l.strip().startswith(("PYTHON", "python"))]
    assert line, "runbook no longer launches the ground-height publisher"
    for l in line:
        assert "_frame_id:=map" in l, "runbook must pass _frame_id:=map: %s" % l
        assert "_dtm_path:=" in l
