"""Load the named-objects table (maps/park_objects.yaml) and resolve a name to its
map-frame center point. Clearance from the object's inflated footprint is NOT
handled here -- the caller (operate.py) snaps the resolved center to the
nearest free cell in the live global costmap before sending it as a goal.

Deliberately NOT using pyyaml: the file is a flat 'name: {x: .., y: ..}' format
this parses directly, so operator containers need no extra dependency.
"""
import re

_LINE = re.compile(r"^([^:#\s]+):\s*\{x:\s*([-\d.]+),\s*y:\s*([-\d.]+)[^}]*\}")


def load_objects(path):
    objects = {}
    with open(path, "r") as fh:
        for line in fh:
            m = _LINE.match(line.strip())
            if m:
                objects[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    return objects


def resolve(name, objects, offset=0.0):
    if name not in objects:
        raise KeyError("unknown object '%s'; known: %s"
                       % (name, ", ".join(sorted(objects))))
    x, y = objects[name]
    # No offset by default: returns the object center. operate.py snaps this
    # to the nearest free costmap cell before sending it as a move_base goal.
    return (x + offset, y)
