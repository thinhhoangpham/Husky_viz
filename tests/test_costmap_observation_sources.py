"""Pins the local costmap's TWO-SOURCE marking/clearing split.

Commit 7d4f7d7 gave the local ObstacleLayer a single observation source
pointing at /os0_cloud_node/points_above_terrain, the terrain-relative filtered
cloud. That cloud has the ground deliberately stripped out.

costmap_2d clears an obstacle layer by RAYTRACING along sensor returns, and the
ground returns are most of the rays. With them gone the layer got marks but
almost no clearing, so lethal cells accumulated forever and the robot was boxed
in by phantom obstacles: measured ~4600 lethal cells with the nearest at 0.73 m
while the nearest real tree was 15.21 m away, growing monotonically 12199 ->
12609 over 8 s with nothing real in range.

The fix splits the two decisions across two sources of the same sensor:
marking off the filtered cloud (WHAT IS AN OBSTACLE), clearing off the raw
cloud (WHAT IS FREE). These tests assert that structure survives, including the
height gates -- a height gate on the clearing source would discard exactly the
low ground rays that do the useful clearing and bring the bug straight back.
"""
import os

import yaml

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_COMMON = os.path.join(_ROOT, "config", "costmap_common_gps.yaml")
_LOCAL = os.path.join(_ROOT, "config", "costmap_local_gps.yaml")

_MARK_TOPIC = "/os0_cloud_node/points_above_terrain"
_CLEAR_TOPIC = "/os0_cloud_node/points"


def _load(path):
    with open(path) as fh:
        return yaml.safe_load(fh) or {}


def _merge(base, override):
    """Mimic rosparam's dict merge: same-named dicts merge, scalars replace."""
    out = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge(out[key], value)
        else:
            out[key] = value
    return out


def merged_local_obstacles():
    """The `obstacles` dict as move_base sees it in the local_costmap namespace.

    launch/move_base_gps*.launch loads costmap_common_gps.yaml into
    local_costmap first, then costmap_local_gps.yaml on top of it.
    """
    return _merge(_load(_COMMON), _load(_LOCAL))["obstacles"]


def test_two_observation_sources_named():
    sources = merged_local_obstacles()["observation_sources"].split()
    assert sources == ["os0_mark", "os0_clear"]


def test_marking_source_reads_the_filtered_cloud():
    mark = merged_local_obstacles()["os0_mark"]
    assert mark["topic"] == _MARK_TOPIC
    assert mark["marking"] is True
    assert mark["clearing"] is False


def test_clearing_source_reads_the_raw_cloud():
    """The raw cloud keeps its ground returns, which are the clearing rays."""
    clear = merged_local_obstacles()["os0_clear"]
    assert clear["topic"] == _CLEAR_TOPIC
    assert clear["marking"] is False
    assert clear["clearing"] is True


def test_no_stale_single_source_survives_the_merge():
    """rosparam merges dicts, so a leftover `os0` key would silently persist."""
    assert "os0" not in merged_local_obstacles()


def test_both_sources_keep_their_height_gates_wide_open():
    obstacles = merged_local_obstacles()
    for name in obstacles["observation_sources"].split():
        source = obstacles[name]
        assert source["min_obstacle_height"] <= -1000.0, name
        assert source["max_obstacle_height"] >= 1000.0, name


def test_layer_level_height_gate_stays_at_the_declared_maximum():
    assert merged_local_obstacles()["max_obstacle_height"] == 50.0


def test_both_sources_declare_the_full_per_source_parameter_set():
    obstacles = merged_local_obstacles()
    required = {"sensor_frame", "data_type", "obstacle_range", "raytrace_range",
                "min_obstacle_height", "max_obstacle_height", "topic",
                "marking", "clearing"}
    for name in obstacles["observation_sources"].split():
        assert required <= set(obstacles[name]), name
        assert obstacles[name]["data_type"] == "PointCloud2"
        assert obstacles[name]["sensor_frame"] == "os0_lidar"


def test_local_file_declares_no_obstacles_override():
    """A partial override can only resurrect a source, never remove one."""
    assert "obstacles" not in _load(_LOCAL)
