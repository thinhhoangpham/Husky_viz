"""Load the named-places table (maps/park_places.yaml) and resolve a name to a
map-frame goal point offset just outside the object (so the goal is a free cell
beside it, not inside the obstacle the object also is in the costmap).

Deliberately NOT using pyyaml: the file is a flat 'name: {x: .., y: ..}' format
this parses directly, so operator containers need no extra dependency.
"""
import re

_LINE = re.compile(r"^([^:#\s]+):\s*\{x:\s*([-\d.]+),\s*y:\s*([-\d.]+)\}")


def load_places(path):
    places = {}
    with open(path, "r") as fh:
        for line in fh:
            m = _LINE.match(line.strip())
            if m:
                places[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    return places


def resolve(name, places, offset=1.2):
    if name not in places:
        raise KeyError("unknown place '%s'; known: %s"
                       % (name, ", ".join(sorted(places))))
    x, y = places[name]
    # Offset along +x so the goal sits beside the object, not on it. The planner
    # + inflation handle final approach; this only needs to land in a free cell.
    return (x + offset, y)
