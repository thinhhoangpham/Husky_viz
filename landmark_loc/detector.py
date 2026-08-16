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
`~matcher` typed/typeless pattern in localizer_node. Two modes exist:

  "cascade" (DEFAULT) -- a thin indirection over the existing
      `classify.classify_cluster` / `classify.to_observations` logic:
      identical labels, identical positions, no behavior change. It has no
      score, so it reports confidence 1.0 for everything it commits to.
  "score" -- the best-score classifier in `landmark_loc.score`: every type
      scores the percept, highest wins, below-floor becomes "unknown". It
      reports a REAL confidence.

The default stays "cascade" deliberately: "score" is opt-in until it has been
validated in sim. Because confidence is part of the Observation contract,
consumers must cope with BOTH -- a constant 1.0 and a genuine score.
"""
from landmark_loc import classify, score


class Detector(object):
    """Interface a detector implementation must satisfy.

    Implementations are plain objects (no registry magic, no base-class
    machinery required -- duck typing is enough). Subclassing this is
    optional and only documents intent.
    """

    #: mode string this implementation is selected by (the ~classifier param)
    name = None

    #: True if this detector implements the PERCEPT contract
    #: (label/observe/detect over a percept sequence). A detector with a
    #: different entry point sets this False: it can still live in the
    #: DETECTORS registry so get_detector() can select it, but the A/B percept
    #: harness must skip it rather than construct+compare it. All detectors
    #: currently registered are percept-based.
    percept_based = True

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


class ScoreDetector(Detector):
    """Best-score classifier: every type scores, the highest wins.

    Percept type: `segment.Cluster`, same as the cascade.

    The scoring itself lives in `landmark_loc.score` (machinery) and
    `map_tools.park_types` (per-type numbers); this class is only the seam
    adapter. Unlike the cascade it emits a REAL confidence -- the winning
    type's score -- and it still rejects: a winner that fails its own
    `score_floor`, or any cluster caught by a hard veto, becomes "unknown"
    and is dropped by `observe`.

    Reduction (position/yaw) is deliberately NOT re-implemented here: it is
    type-dispatched inside `classify.to_observations`, which is passed the
    labels this detector chose. So a percept both detectors label the same
    reduces to the SAME position, and the A/B position delta isolates
    labelling differences rather than mixing in reduction differences.
    """

    name = "score"

    def __init__(self, margins=classify.DEFAULT_MARGINS):
        # `margins` is accepted (and passed to to_observations, which uses it
        # only for the reduction path) purely so both detectors share one
        # construction signature and get_detector needs no special-casing.
        self.margins = margins

    def label(self, percept):
        return score.classify_cluster(percept)[0]

    def _label_conf(self, percepts):
        return [score.classify_cluster(c) for c in percepts]

    def observe(self, percepts, frame_id=None, stamp=None):
        return self.detect(percepts, frame_id=frame_id, stamp=stamp)[1]

    def detect(self, percepts, frame_id=None, stamp=None):
        scored = self._label_conf(percepts)
        labels = [ident for ident, _conf in scored]
        obs = classify.to_observations(percepts, self.margins,
                                       frame_id=frame_id, stamp=stamp,
                                       labels=labels)
        # to_observations emits accepted percepts IN INPUT ORDER (the seam
        # contract the A/B harness relies on), so the i-th accepted percept
        # owns the i-th Observation -- that is what lets the real confidence
        # be attached here rather than threaded through the reduction.
        confs = [conf for ident, conf in scored if ident != "unknown"]
        if len(confs) == len(obs):
            for o, conf in zip(obs, confs):
                o.confidence = conf
        return labels, obs


#: mode string -> factory. Add the next implementation here and it becomes
#: selectable via ~classifier with no other edits.
#: NOTE: DEFAULT_DETECTOR stays "cascade" -- "score" is opt-in until it has
#: been validated in sim.
DETECTORS = {
    CascadeDetector.name: CascadeDetector,
    ScoreDetector.name: ScoreDetector,
}

DEFAULT_DETECTOR = CascadeDetector.name


def get_detector(name=DEFAULT_DETECTOR, **kwargs):
    """Instantiate the detector selected by `name`.

    Raises KeyError on an unknown name; callers that need a fallback (the
    ROS node) validate the param first, as localizer_node does for ~matcher.
    """
    return DETECTORS[name](**kwargs)
