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
    marker_color: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    # RGBA label color. Default is for types NOT in the catalog
    # (is_catalog=False), which never enter _LABEL_COLOR.

    # --- scoring data for the best-score detector (landmark_loc.score) ------
    # Appended LAST with defaults on purpose: every existing PARK_TYPES entry
    # constructs by keyword, and any positional construction elsewhere keeps
    # working unchanged.
    #
    # `score_family` selects WHICH scoring formula applies (the machinery
    # lives in landmark_loc/score.py; only the per-type NUMBERS live here):
    #   "band"    - trash_bin_1 / bench / garden_table. Scored by how close
    #               the measured foot_major / height / aspect sit to a band
    #               centre, normalised by that band's half-width.
    #   "profile" - lamp / tree. These are PREDICATES today, not size bands
    #               (a lamp is "a thin post rising high", a tree is "a wide
    #               canopy over a trunk"), so they are scored by the MARGIN on
    #               their discriminating quantity, not distance to a centre.
    #   None      - not scoreable (obstacle-only types).
    score_family: Optional[str] = None
    #: band type only: {quantity: (centre, half_width)} over foot_major /
    #: height / aspect. See score_bands for which numbers are mesh-derived
    #: (honest) and which are empirical/PROVISIONAL.
    score_band_spec: Optional[dict] = None
    #: profile types only: the margin scale on the discriminating quantity
    #: (metres). See landmark_loc/score.py for how each profile type uses it.
    score_margin: Optional[float] = None
    #: best score below this -> "unknown" (dropped). DELIBERATELY PERMISSIVE
    #: for now: this plugin is opt-in and must not silently start dropping
    #: detections the cascade accepts. Tighten with in-sim evidence.
    score_floor: float = 0.0

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
    def score_bands(self):
        """Resolved {quantity: (centre, half_width)} for a "band" type, or None.

        WHICH NUMBERS ARE HONEST, AND WHICH ARE EMPIRICAL
        -------------------------------------------------
        A centre given as the string "mesh" is resolved from `signature`, i.e.
        from the actual object mesh -- that is the honest source for a bench's
        true major (1.782) or a bin's true major (0.682), and it is the value
        the whole point of the registry is to stop duplicating.

        A NUMERIC centre/half_width is EMPIRICAL and PROVISIONAL: it encodes a
        measurement of live captured clusters rather than the mesh. Two places
        need this, and both are lidar facts the mesh cannot express:

          * height half-widths are wide because a lidar sees a PARTIAL vertical
            slice. Captured bench [12] measures height 0.44 m against a mesh
            height of 0.942 m; captured lamps measure 2.09-2.53 m against a
            mesh height of 3.148 m. A half-width narrow enough to "respect the
            mesh" would reject every real capture.
          * aspect centres/half-widths carry no mesh analogue at all for a
            partial view: captured bench [12] measures aspect 2.55 where the
            mesh aspect is 2.23, because only part of the seat is seen.

        Same provenance discipline as the cascade thresholds in classify.py:
        mesh-derived where a mesh is the truth, measured-and-flagged elsewhere.
        """
        if self.score_band_spec is None:
            return None
        sig = self.signature
        out = {}
        for quantity, (centre, half) in self.score_band_spec.items():
            if centre == "mesh":
                if sig is None:
                    raise ValueError(
                        "%s: score band %r asks for a mesh centre but the type "
                        "has no mesh" % (self.identity, quantity))
                centre = sig[quantity]
            out[quantity] = (float(centre), float(half))
        return out

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
        score_family="band",
        score_band_spec={
            # major centre from the mesh (1.782). Half 0.55 reproduces the
            # cascade's bench window [1.20, 2.30) almost exactly (1.23-2.33).
            "major": ("mesh", 0.55),
            # EMPIRICAL: mesh height is 0.942 but captured bench [12] is 0.44
            # (partial vertical view). Centre 0.60 sits between them; half
            # 0.65 spans 0.0-1.25, matching the cascade's height < 1.20 gate.
            "height": (0.60, 0.65),
            # EMPIRICAL: mesh aspect 2.23, captured bench [12] 2.55. Wide half
            # because a partial seat view swings the ratio a long way.
            "aspect": (2.40, 1.40),
        },
        score_floor=0.05,
    ),
    ParkType(
        world_prefix="garden_table", identity="garden_table",
        is_object=True, is_catalog=True, disc_radius=0.60,
        mesh=(("garden_table", "garden_table.dae"), 1.0),
        box_stamped=True, marker_color=(0.0, 1.0, 1.0, 1.0),
        score_family="band",
        score_band_spec={
            # major centre from the mesh (3.00). Half 0.75 puts the lower edge
            # at 2.25, matching the cascade's table/bench split at 2.30.
            "major": ("mesh", 0.75),
            # EMPIRICAL, same reasoning as bench: mesh height 1.085, but a
            # partial view of a tabletop reads far lower.
            "height": (0.60, 0.65),
            # EMPIRICAL: mesh aspect 2.27. No captured table exists to pin
            # this against -- PROVISIONAL, deliberately wide.
            "aspect": (2.30, 1.40),
        },
        score_floor=0.05,
    ),
    ParkType(
        world_prefix="lamp", identity="lamp",
        is_object=True, is_catalog=True, disc_radius=0.20,
        mesh=(("lamp", "street_lamp.dae"), 1.0),
        box_stamped=False, marker_color=(1.0, 1.0, 0.0, 1.0),
        # PROFILE type: a lamp is not a size band, it is "a thin post rising
        # above HIGH_Z". Scored by how far post_width sits BELOW the
        # _LAMP_POST_MAX cap (0.35 m); score_margin is that cap's full span,
        # so a hairline post (captured lamps measure 0.02-0.15 m) scores near
        # 1.0 and a post right at the cap scores ~0.
        score_family="profile", score_margin=0.35,
        score_floor=0.05,
    ),
    ParkType(
        world_prefix="trash_bin_1", identity="trash_bin_1",
        is_object=True, is_catalog=True, disc_radius=0.25,
        mesh=(("trash_bin_1", "trash_bin.dae"), 1.0),
        box_stamped=False, marker_color=(1.0, 0.5, 0.0, 1.0),
        score_family="band",
        score_band_spec={
            # major centre from the mesh (0.682). Half 0.55 spans 0.13-1.23,
            # matching the cascade's [0.30, 1.20) window at the top; the low
            # end is held by the min-points veto, not by this band.
            "major": ("mesh", 0.55),
            # EMPIRICAL: mesh height 1.041; half 0.60 puts the ceiling at 1.30,
            # close to the cascade's _BIN_MAX_H = 1.20.
            "height": (0.70, 0.60),
            # EMPIRICAL and the real discriminator: mesh aspect 1.79, captured
            # bin [13] 1.33, while the elongated ground fragments [1]/[3] sit
            # at 2.38-2.41. Centre 1.55, half 0.75 -> upper edge 2.30, which
            # keeps those fragments scoring at/below zero here.
            "aspect": (1.55, 0.75),
        },
        score_floor=0.05,
    ),
    ParkType(
        world_prefix="tree_8", identity="tree",
        is_object=True, is_catalog=True, disc_radius=0.45,
        mesh=None,
        box_stamped=False, marker_color=(0.0, 0.4, 0.0, 1.0),
        # PROFILE type: a tree is "a canopy band at least _TREE_CANOPY_MIN_WIDTH
        # (2.0 m) wide at z >= 2.5 m". Scored by how far the widest such band
        # EXCEEDS that 2.0 m floor. score_margin 2.0 m is the observed spread:
        # in-sim canopies measure 2.9-4.75 m wide, i.e. up to ~2.75 m over the
        # floor, so a typical real canopy saturates near 1.0 while a marginal
        # 2.0-2.5 m band scores low. No mesh exists for tree_8 (see above), so
        # this is measured, not derived.
        score_family="profile", score_margin=2.0,
        score_floor=0.05,
    ),
    ParkType(
        world_prefix="arbolpartes4", identity="arbolpartes4",
        is_object=False, is_catalog=False, disc_radius=0.30,
        mesh=None,
        box_stamped=False, marker_color=(1.0, 0.0, 0.0, 1.0),
    ),
    ParkType(
        # Power-line pole added to the park as a UNIQUE-shape landmark. Same
        # asset as lake's postescable (linea1/postes.dae). disc_radius 0.25 =
        # one pole; profile type, 0.35 m post-width margin like the lamp.
        world_prefix="postescable", identity="postescable",
        is_object=True, is_catalog=True, disc_radius=0.25,
        mesh=None,
        box_stamped=False, marker_color=(0.6, 0.3, 0.0, 1.0),
        score_family="profile", score_margin=0.35,
        score_floor=0.05,
    ),
    ParkType(
        # The ONE distinctive object in the park, built from SDF primitives
        # rather than a mesh: a 5 m-wide tank on a narrow 2 m-wide pedestal,
        # ~11 m tall overall. Nothing else in this world is both that tall and
        # that wide -- trees are tall but narrow, poles are tall but a thin open
        # lattice, and every other family is under 3.2 m.
        #
        # NOT CLASSIFIED. The tower is recognised by matching its shape
        # descriptor against maps/park_landmarks.yaml, not by the classifier
        # stack: no score detector, no matcher catalog, no size-band radius, no
        # label colour. is_catalog=False is precisely what expresses that --
        # every is_catalog reader is classifier machinery (classify.KNOWN_RADIUS,
        # catalog's identity set, localizer_node._LABEL_COLOR), and the tower
        # belongs to none of it. The scoring fields stay at their defaults for
        # the same reason.
        #
        # is_object=True is a separate, map-side matter: it is a real physical
        # object and must be stamped into the costmap.
        #
        # EXACTLY ONE instance exists in park.world (asserted by
        # map_tools/tests/test_park_world_tower.py). The whole design depends on
        # it having no twin: descriptor distance between identical instances is
        # exactly 0, which is what sank the earlier six-identical-poles attempt.
        #
        # disc_radius 2.5 = the tank radius, its true footprint.
        # mesh=None because it is built from SDF primitives, not a mesh file.
        #
        # This entry exists ONLY so the map extractor stamps the tower's
        # footprint into the costmap as the obstacle it is. Nothing describes
        # or identifies it: the robot finds landmarks by measuring which blob
        # in the CURRENT scan is unlike its neighbours, and never learns what
        # any blob is. There is deliberately no stored descriptor for this
        # object -- matching a live blob against a prebuilt reference would be
        # identity assignment, i.e. classification, which this design rejects.
        world_prefix="water_tower", identity="water_tower",
        is_object=True, is_catalog=False, disc_radius=2.5,
        mesh=None,
        box_stamped=False,
    ),
)

# world_prefix -> ParkType.
# ---------------------------------------------------------------------------
# Lake world types.
#
# Deliberately a SEPARATE registry, not extra PARK_TYPES entries: PARK_TYPES is
# imported by the whole detection stack (catalog, score, signatures, classify,
# localizer_node), so appending lake species there would put them in the park
# detector's catalog too. Keeping them apart means each world declares only its
# own types, and `classify_prefix(..., types=LAKE_TYPES)` selects which.
#
# Model-instance names in lake.world differ from the model FOLDER names on disk
# (world 'bush' -> folder dry_bush, 'tree' -> tree_8_v, 'postescable' -> linea1).
# world_prefix must match the WORLD name, since that is what gets classified.
#
# What is deliberately ABSENT, per the mapping decision for this world:
#   altaniv_seca_d (68), altaniv (31), bush (30), arbusto3 (34) -- all low
#   vegetation. NOT in the registry at all, so classify_prefix returns 'skip'
#   and they never reach the occupancy grid. The robot decides whether to dodge
#   them from live lidar + the local costmap, not from the static map. Stamping
#   163 vegetation footprints would carve the map into something unnavigable
#   while hiding genuinely passable ground.
# ---------------------------------------------------------------------------
LAKE_TYPES = (
    ParkType(
        # Same asset as park's tree_8 (bark8.obj, 165,380 faces, byte-identical),
        # so the park numbers carry over verbatim: disc_radius 0.45 is the trunk
        # radius, mesh=None because trees are identified by vertical profile
        # rather than a size band, and score_margin 2.0 is the canopy-width
        # spread measured in-sim. Named 'tree' in lake.world, 'tree_8' in park.
        world_prefix="tree", identity="tree",
        is_object=True, is_catalog=True, disc_radius=0.45,
        mesh=None,
        box_stamped=False, marker_color=(0.0, 0.4, 0.0, 1.0),
        score_family="profile", score_margin=2.0,
        score_floor=0.05,
    ),
    ParkType(
        # The power line: ONE model holding BOTH poles plus the span between
        # them. Measured from linea1/postes.dae at its world scale 0.03: the
        # mesh is 58.97 x 3.60 m overall, but near-base geometry clusters into
        # exactly two poles, each 0.50 m wide, centred ~29 m apart.
        #
        # disc_radius 0.25 is therefore ONE pole's radius, not the model's.
        # A single disc at the model origin cannot cover both poles -- see the
        # note in extract_lake_map.py; the cables between them are 8 m up and
        # are not an obstacle at robot height anyway.
        world_prefix="postescable", identity="postescable",
        is_object=True, is_catalog=True, disc_radius=0.25,
        mesh=None,
        box_stamped=False, marker_color=(0.6, 0.3, 0.0, 1.0),
        # PROFILE type, same shape of argument as park's lamp: a pole is "a
        # thin post rising high", not a size band. 0.35 m is the same post-width
        # cap the lamp uses, and a 0.50 m-wide pole sits just above it.
        score_family="profile", score_margin=0.35,
        score_floor=0.05,
    ),
    ParkType(
        # The lake itself. LANDMARK ONLY -- is_object/is_catalog True so it
        # lands in lake_objects.yaml and the matcher catalog, but it is NEVER
        # stamped into the occupancy grid (see extract_lake_map.py: the water
        # family is excluded from build_grid).
        #
        # Rationale: `lago` is a visual-only <box> with NO <collision>, so the
        # lidar returns nothing from it -- live obstacle avoidance is blind to
        # water. It is not an obstacle to bump into; it is a region to plan
        # away from, using prior map knowledge.
        #
        # disc_radius 0.0: never stamped, so no footprint is meaningful.
        world_prefix="lago", identity="lake",
        is_object=True, is_catalog=True, disc_radius=0.0,
        mesh=None,
        box_stamped=False, marker_color=(0.13, 0.44, 0.70, 1.0),
    ),
)

BY_PREFIX = {t.world_prefix: t for t in PARK_TYPES}
LAKE_BY_PREFIX = {t.world_prefix: t for t in LAKE_TYPES}

# Prefixes ordered LONGEST-FIRST, so "trash_bin_1"/"tree_8" are not shadowed by
# a shorter prefix. Replicates sdf_parse._FAMILY_PREFIXES' longest-first intent
# without depending on registry declaration order.
PREFIXES_LONGEST_FIRST = tuple(
    sorted((t.world_prefix for t in PARK_TYPES), key=len, reverse=True)
)
LAKE_PREFIXES_LONGEST_FIRST = tuple(
    sorted((t.world_prefix for t in LAKE_TYPES), key=len, reverse=True)
)


def classify_prefix(name, prefixes=None):
    """Map a world model name to its world_prefix, or 'skip'.

    Longest-prefix-first so "trash_bin_1"/"tree_8" win over shorter prefixes.
    Matches sdf_parse.classify's semantics exactly.

    `prefixes` defaults to the park set, so every existing caller is unchanged.
    Pass LAKE_PREFIXES_LONGEST_FIRST to classify against the lake world instead.
    """
    if prefixes is None:
        prefixes = PREFIXES_LONGEST_FIRST
    for prefix in prefixes:
        if name == prefix or name.startswith(prefix + "_"):
            return prefix
    return "skip"
