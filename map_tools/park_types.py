"""Single source of truth for park object types.

Both the map-building side (map_tools/*) and the lidar-detection side
(landmark_loc/*) previously kept SEPARATE hardcoded tables describing the same
six park object types (bench, garden_table, lamp, trash_bin_1, tree_8,
arbolpartes4). Those tables could silently drift. This module unifies them into
ONE frozen dataclass registry that both sides import.

Placement: this lives in map_tools/ (not landmark_loc/) on purpose. The
established import direction is landmark_loc -> map_tools (e.g.
landmark_loc/signatures.py imports map_tools.mesh_bounds). The registry needs
the map-mesh machinery (map_tools.mesh_bounds.bounds3d) to derive signatures at
import, and must be importable by BOTH trees. Placing it in map_tools/ lets both
map_tools/* and landmark_loc/* import DOWN into it, with no import cycle.

Semantics preserved verbatim from the tables this replaces:
  - world_prefix   : map_tools/sdf_parse.py::_FAMILY_PREFIXES
  - identity       : landmark_loc/catalog.py::_FAMILY_TO_IDENTITY (tree_8->tree)
  - is_object      : map_tools/extract_park_map.py::OBJECT_FAMILIES
  - is_catalog     : landmark_loc/catalog.py::_IDENTITY_FAMILIES
  - disc_radius    : map_tools/extract_park_map.py::RADII
  - mesh           : landmark_loc/signatures.py::_MESHES
  - box_stamped    : map_tools/extract_park_map.py::BOX_MESHES
  - detect_radius  : landmark_loc/classify.py::KNOWN_RADIUS (derived; see below)
  - rect_footprint : landmark_loc/classify.py::_RECT_FOOTPRINT
  - marker_color   : landmark_loc/localizer_node.py::_LABEL_COLOR

Preserved asymmetries:
  - arbolpartes4 = obstacle-only: has disc_radius + world_prefix, but
    is_object=False, is_catalog=False, mesh=None (never becomes an object or a
    catalog landmark; only stamped as a disc obstacle).
  - tree_8 -> identity 'tree': is_object + is_catalog, but mesh=None and its
    detect_radius is hardcoded 0.45 (trunk radius), NOT mesh-derived, because
    trees are identified by vertical profile, not a size band.
  - lamp / trash_bin_1: have a mesh signature + is_object, but box_stamped=False
    (stamped as discs -- their box footprint is sub-cell at 0.15 m).
  - bench / garden_table: box_stamped=True (yaw-oriented box footprints).
"""
import os
from dataclasses import dataclass
from typing import Optional, Tuple

from map_tools import mesh_bounds

_MODELS_ROOT = os.path.join(os.path.dirname(__file__), "..", "models_opt")


@dataclass(frozen=True)
class ParkType:
    world_prefix: str                      # model-name prefix in park.world
    identity: str                          # matcher identity (tree_8 -> 'tree')
    is_object: bool                         # becomes a named goal destination
    is_catalog: bool                       # participates in matcher catalog
    disc_radius: float                     # footprint disc radius (m)
    mesh: Optional[Tuple[Tuple[str, ...], float]]  # (rel-path-parts, scale)|None
    box_stamped: bool                      # stamp as a yaw-oriented box, not disc
    marker_color: Tuple[float, float, float, float]  # RGBA label color

    # --- derived at import, exactly as signatures.py / classify.py do today ---
    def _mesh_path(self):
        if self.mesh is None:
            return None
        parts, scale = self.mesh
        return os.path.join(_MODELS_ROOT, *parts), scale

    @property
    def signature(self):
        """Mesh-derived {major, minor, height} full-extents, or None.

        Same computation as landmark_loc/signatures.py: bounds3d gives half
        extents; major/minor are the max/min horizontal full-extents.
        """
        if self.mesh is None:
            return None
        path, scale = self._mesh_path()
        hx, hy, hz, _, _, _ = mesh_bounds.bounds3d(path, scale)
        dx, dy = 2 * hx, 2 * hy
        return {"major": max(dx, dy), "minor": min(dx, dy), "height": 2 * hz}

    @property
    def detect_radius(self):
        """Near-face pushout radius by identity, metres (classify.KNOWN_RADIUS).

        Mesh types: half the minor horizontal extent from the signature.
        Tree: hardcoded 0.45 (trunk radius; not in a mesh signature). None for
        obstacle-only types (arbolpartes4).
        """
        if self.identity == "tree":
            return 0.45
        sig = self.signature
        if sig is None:
            return None
        return sig["minor"] / 2.0

    @property
    def rect_footprint(self):
        """(length, width) footprint for ICP shape-fit, or None.

        Only box_stamped types get a rect footprint. NOTE: the committed values
        are the ROUNDED LITERALS bench (1.78, 0.80) / garden_table (3.00, 1.32),
        kept verbatim to guarantee ZERO behavior change. They differ slightly
        from the mesh-derived signature values -- bench mesh (1.7821, 0.7989),
        garden_table mesh (3.0000, 1.3188) -- so this is NOT derived from the
        signature; the literals are authoritative here.
        """
        return _RECT_FOOTPRINT_LITERALS.get(self.world_prefix)


# Rounded literals from classify.py::_RECT_FOOTPRINT. See rect_footprint above
# for why these are kept as literals rather than derived from the signature.
_RECT_FOOTPRINT_LITERALS = {
    "bench": (1.78, 0.80),
    "garden_table": (3.00, 1.32),
}


# The registry. One entry per world_prefix. Order here is not semantically
# meaningful -- prefix classification sorts by descending length (see classify).
PARK_TYPES = (
    ParkType(
        world_prefix="bench", identity="bench",
        is_object=True, is_catalog=True, disc_radius=0.90,
        mesh=(("bench", "Bench_1.dae"), 0.15),
        box_stamped=True, marker_color=(0.0, 1.0, 0.0, 1.0),
    ),
    ParkType(
        world_prefix="garden_table", identity="garden_table",
        is_object=True, is_catalog=True, disc_radius=0.60,
        mesh=(("garden_table", "garden_table.dae"), 1.0),
        box_stamped=True, marker_color=(0.0, 1.0, 1.0, 1.0),
    ),
    ParkType(
        world_prefix="lamp", identity="lamp",
        is_object=True, is_catalog=True, disc_radius=0.20,
        mesh=(("lamp", "street_lamp.dae"), 1.0),
        box_stamped=False, marker_color=(1.0, 1.0, 0.0, 1.0),
    ),
    ParkType(
        world_prefix="trash_bin_1", identity="trash_bin_1",
        is_object=True, is_catalog=True, disc_radius=0.25,
        mesh=(("trash_bin_1", "trash_bin.dae"), 1.0),
        box_stamped=False, marker_color=(1.0, 0.5, 0.0, 1.0),
    ),
    ParkType(
        world_prefix="tree_8", identity="tree",
        is_object=True, is_catalog=True, disc_radius=0.45,
        mesh=None,
        box_stamped=False, marker_color=(0.0, 0.4, 0.0, 1.0),
    ),
    ParkType(
        world_prefix="arbolpartes4", identity="arbolpartes4",
        is_object=False, is_catalog=False, disc_radius=0.30,
        mesh=None,
        box_stamped=False, marker_color=(1.0, 0.0, 0.0, 1.0),
    ),
)

# world_prefix -> ParkType.
BY_PREFIX = {t.world_prefix: t for t in PARK_TYPES}

# Prefixes ordered LONGEST-FIRST, so "trash_bin_1"/"tree_8" are not shadowed by
# a shorter prefix. Replicates sdf_parse._FAMILY_PREFIXES' longest-first intent
# without depending on registry declaration order.
PREFIXES_LONGEST_FIRST = tuple(
    sorted((t.world_prefix for t in PARK_TYPES), key=len, reverse=True)
)


def classify_prefix(name):
    """Map a world model name to its world_prefix, or 'skip'.

    Longest-prefix-first so "trash_bin_1"/"tree_8" win over shorter prefixes.
    Matches sdf_parse.classify's semantics exactly.
    """
    for prefix in PREFIXES_LONGEST_FIRST:
        if name == prefix or name.startswith(prefix + "_"):
            return prefix
    return "skip"
