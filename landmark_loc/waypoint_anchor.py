"""Pure decision logic for waypoint re-anchoring. No ROS.

Arrival is confirmed by DESCRIPTOR SIGHTING, never by move_base status or fused
pose (design doc: the fused pose is exactly what a navsat attack controls). A
pole sighting is a stronger fix than a waypoint assertion, so it wins.
"""
import math


def confirm_arrival(waypoint_xy, expected_names, sightings, radius):
    wx, wy = waypoint_xy
    for name, x, y in sightings:
        if name in expected_names and math.hypot(x - wx, y - wy) <= radius:
            return True
    return False


def choose_anchor(prev_anchor, pole_sighting, confirmed_waypoint):
    if pole_sighting is not None:
        return (pole_sighting, "pole")
    if confirmed_waypoint is not None:
        return (confirmed_waypoint, "waypoint")
    return (prev_anchor, "hold")


def fault_offset(predicted_xy, sighting_xy):
    return math.hypot(predicted_xy[0] - sighting_xy[0],
                      predicted_xy[1] - sighting_xy[1])
