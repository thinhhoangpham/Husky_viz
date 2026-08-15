"""The pluggable detector seam: percepts in, typed Observations out.

WHY THIS EXISTS
---------------
`landmark_loc.classify` implements ONE way to decide what a landmark is: a
hand-tuned, first-match-wins cascade of shape rules over a lidar cluster.
Future experiments may want other ways -- a learned best-score classifier, a
CAMERA doing image classification, or a fusion of several sensors.

The seam is therefore placed at the OUTPUT, not the input. The stable,
shared contract is:

    a detector consumes whatever PERCEPTS it needs, and produces
    `classify.Observation` records (identity, x, y, yaw, confidence)
    stamped with the frame and time they are expressed in.

Deliberately, nothing in this module's contract names "cluster", "point
cloud", or "lidar". `observe()` takes an opaque `percepts` sequence whose
element type is defined by the implementation: today's cascade defines it as
`segment.Cluster`; a camera detector would define it as image regions and
would be droppable in without the matcher or solver changing at all.

`unknown` IS PART OF THE CONTRACT (required)
--------------------------------------------
A detector MUST be able to REJECT a percept -- to look at it and emit
nothing. It must not be a pick-the-best-of-N classifier. If a detector
always assigns some class, every ground-scatter fragment and floating tree
canopy becomes phantom furniture, those phantoms get associated to real
catalog landmarks, and the pose fix is corrupted (this is exactly what the
captured-regression tests _NO_PHANTOM / unknown-rate guard against: 9 of 15
captured clusters are correctly rejected). Any new implementation must keep
an explicit `unknown` path that drops the percept.

FRAME AND TIME ARE PART OF THE CONTRACT
---------------------------------------
Every emitted Observation carries `frame_id` and `stamp` (float seconds --
see `classify.Observation`). They are not decoration: x/y mean nothing
without knowing which sensor mount they are relative to and at what instant.
A detector DECLARES the frame/time of its output; a consumer that mixes
detectors MUST transform into a common frame and time before matching. A
future camera detector (different mount => different frame) or a fusing
detector (percepts from different ticks => different stamps) is REQUIRED to
set these correctly rather than inheriting the lidar's.

WHY THE CALLER SUPPLIES FRAME/STAMP (design decision)
-----------------------------------------------------
Two options were available: put frame/stamp on the percept, or pass them to
the detector call. We pass them to the call --
`detect(percepts, frame_id=..., stamp=...)`.

Reasons: (1) `segment.Cluster` is a purely geometric record with no frame or
stamp field, and adding one would push this contract change down into
segmentation, which is out of scope; (2) frame and time are properties of
the SENSOR MESSAGE, and the node owns the message header -- making the node
state them keeps one authority instead of copying the header onto every
percept; (3) a fusing detector that genuinely needs per-percept stamps is
still free to define its own percept type carrying them, because the percept
type is the implementation's business -- the arguments are a DEFAULT the
detector may override, not a ceiling.

ONE PASS PER TICK
-----------------
`detect()` is the entry point callers should use: it returns
`(labels, observations)` in a SINGLE pass -- labels for EVERY percept
(diagnostics/RViz want a label for rejected ones too) and Observations only
for the accepted ones. `label()`/`observe()` remain for tests and simple
callers, but using both means classifying twice per tick; today's cascade is
cheap enough not to care, a future ML or camera detector would pay real
inference cost twice.

SELECTING AN IMPLEMENTATION
---------------------------
`get_detector(name)` maps a mode string to an instance, mirroring the
`~matcher` typed/typeless pattern in localizer_node. Step 1 has exactly one
mode, "cascade", which is a thin indirection over the existing
`classify.classify_cluster` / `classify.to_observations` logic -- identical
labels, identical positions, no behavior change.
"""
from landmark_loc import classify


class Detector(object):
    """Interface a detector implementation must satisfy.

    Implementations are plain objects (no registry magic, no base-class
    machinery required -- duck typing is enough). Subclassing this is
    optional and only documents intent.
    """

    #: mode string this implementation is selected by (the ~classifier param)
    name = None

    def label(self, percept):
        """Return the identity string for one percept.

        MUST return the string "unknown" when the percept matches no known
        landmark type; see the module docstring on why rejection is
        mandatory. Used for diagnostics/RViz labels, which want a label for
        every percept including rejected ones.
        """
        raise NotImplementedError

    def observe(self, percepts, frame_id=None, stamp=None):
        """Return a list of `classify.Observation` for accepted percepts.

        Percepts labelled "unknown" are DROPPED (not emitted with a low
        confidence) -- the matcher must never see them.

        `frame_id` / `stamp` (float seconds) declare where and when the
        percepts were taken; the implementation stamps its Observations with
        them unless it knows better (see the module docstring).
        """
        raise NotImplementedError

    def detect(self, percepts, frame_id=None, stamp=None):
        """Single pass: return `(labels, observations)`.

        `labels` has one entry per percept in input order, INCLUDING
        "unknown"; `observations` holds only the accepted ones. This is the
        entry point callers should use -- calling `label()` then `observe()`
        classifies every percept twice.
        """
        raise NotImplementedError


class CascadeDetector(Detector):
    """The legacy rule cascade, unchanged, behind the detector interface.

    Percept type: `segment.Cluster` (a lidar point cluster). That assumption
    lives HERE, inside the implementation, not in the shared contract.

    Confidence is trivially 1.0: the cascade has no score, it either commits
    to a type or rejects. Rejected percepts are dropped by `observe`, so
    every Observation it emits is one it is fully committed to.
    """

    name = "cascade"

    def __init__(self, margins=classify.DEFAULT_MARGINS):
        self.margins = margins

    def label(self, percept):
        return classify.classify_cluster(percept, self.margins)

    def observe(self, percepts, frame_id=None, stamp=None):
        return classify.to_observations(percepts, self.margins,
                                        frame_id=frame_id, stamp=stamp)

    def detect(self, percepts, frame_id=None, stamp=None):
        # Classify once, then reduce reusing those labels -- exactly the same
        # results as label()-then-observe(), at half the classification cost.
        labels = [classify.classify_cluster(c, self.margins) for c in percepts]
        obs = classify.to_observations(percepts, self.margins,
                                       frame_id=frame_id, stamp=stamp,
                                       labels=labels)
        return labels, obs


#: mode string -> factory. One entry in step 1; add the next implementation
#: here and it becomes selectable via ~classifier with no other edits.
DETECTORS = {
    CascadeDetector.name: CascadeDetector,
}

DEFAULT_DETECTOR = CascadeDetector.name


def get_detector(name=DEFAULT_DETECTOR, **kwargs):
    """Instantiate the detector selected by `name`.

    Raises KeyError on an unknown name; callers that need a fallback (the
    ROS node) validate the param first, as localizer_node does for ~matcher.
    """
    return DETECTORS[name](**kwargs)
