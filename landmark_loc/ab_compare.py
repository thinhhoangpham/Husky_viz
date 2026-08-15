"""Offline A/B harness: run two registered detectors over the SAME percepts.

WHY THIS EXISTS
---------------
Correct object identification is the thing that matters, and the current rule
cascade is not the last word on it. More experiments are expected (a best-score
classifier, maybe a camera, maybe a learned model). What makes those
experiments cheap is not the next classifier -- it is being able to ask, of any
two implementations, *where exactly do they disagree, and why*.

So this module answers one question: given the same percepts, which ones do
detector A and detector B label differently, and what shape features drove it.

It is deliberately generic over `detector.DETECTORS`: nothing here names
"cascade" or any other implementation. `--a` / `--b` take any registered name,
so the day a `score` detector is registered it is comparable with no edit here.

WHAT IT REPORTS
---------------
1. A DISAGREEMENT TABLE, one row per percept: index, A's label+confidence,
   B's label+confidence, AGREE/DISAGREE, and -- for disagreements -- the shape
   features that drove the decision (foot_major, foot_minor, height, aspect,
   z_min, n points), i.e. exactly the quantities `classify.classify_cluster`
   gates on, so a disagreement can be judged without re-deriving them.

2. A SUMMARY: totals, per-label counts per detector, and the UNKNOWN RATE for
   each. The unknown rate is a SAFETY metric, not a quality metric. A detector
   that rejects fewer percepts than the incumbent is emitting phantom
   furniture, those phantoms get associated to real catalog landmarks, and the
   pose fix is corrupted. So the summary calls out the unknown-rate delta
   explicitly and flags "B REJECTS FEWER" as a warning line.

3. POSITION DELTAS for percepts where both detectors accepted with the SAME
   label. Reduction is type-dispatched (`classify.to_observations`), so equal
   labels ought to produce equal x/y/yaw; any nonzero delta means the two
   implementations reduced the same percept to different places and is worth
   surfacing. Reported as max and mean over those percepts.

SINGLE PASS
-----------
Each detector is invoked ONCE, via the seam's `detect(percepts, frame_id,
stamp)`, which returns labels for every percept and Observations for the
accepted ones together. Calling `label()` and then `observe()` would classify
twice per detector -- irrelevant for the cascade, real inference cost for a
future ML detector, and this harness is exactly the tool one would run it under.

PERCEPT SOURCE (how a live-captured dataset plugs in later)
-----------------------------------------------------------
`compare()` takes an opaque `percepts` SEQUENCE and never inspects it beyond
handing it to the detectors -- feature extraction for the table is isolated in
`percept_features()`, which degrades to empty context for percept types it does
not understand. Consequently a future live-capture dataset needs to supply only
a loader returning a sequence of percepts; `load_fixture_percepts()` is simply
the first such loader (the 15 captured in-sim clusters checked into
tests/fixtures). Capture TOOLING is deliberately NOT built here -- the capture
modality (bag replay? node-side dump? camera frames?) is still unknown, and
guessing it would bake in the wrong format.
"""
import argparse
import json
import math
import os
from dataclasses import dataclass, field

import numpy as np

from landmark_loc import detector, shapefeat

UNKNOWN = "unknown"

_FIXTURE_DIR = os.path.join(os.path.dirname(__file__), "tests", "fixtures")
_FIXTURE_NPZ = os.path.join(_FIXTURE_DIR, "captured_clusters.npz")


def load_fixture_percepts(npz_path=_FIXTURE_NPZ):
    """Load the captured in-sim clusters as percepts, in index order.

    Mirrors the loader in tests/test_captured_regression.py: the .npz holds one
    (N,3) point array per cluster under key "c<i>", and the derived geometry
    (centroid, PCA extents, height) is recomputed rather than stored.
    """
    from landmark_loc.segment import Cluster, _pca_extents

    arrs = np.load(npz_path)
    out = []
    for key in sorted(arrs.files, key=lambda k: int(k[1:])):
        pts = arrs[key].astype(float)
        major, minor = _pca_extents(pts[:, :2])
        out.append(Cluster(
            points=pts,
            centroid_xy=(float(pts[:, 0].mean()), float(pts[:, 1].mean())),
            major=float(major), minor=float(minor),
            height=float(pts[:, 2].max() - pts[:, 2].min())))
    return out


def percept_features(percept):
    """Shape-feature context for one percept, for judging a disagreement.

    These are the SAME quantities classify.classify_cluster gates on:
    footprint PCA extents, their ratio, cluster height, the base height off the
    ground (the ground-anchoring gate), and the raw point count (the
    measurability gate).

    Returns {} for a percept this harness cannot measure -- a percept type is
    the detector implementation's business (see detector.py), so an
    unrecognised one must degrade to "no context", never raise.
    """
    pts = getattr(percept, "points", None)
    if pts is None or len(pts) == 0:
        return {}
    foot_major, foot_minor = shapefeat.foot_extents(pts)
    height = getattr(percept, "height", None)
    if height is None:
        height = float(pts[:, 2].max() - pts[:, 2].min())
    return {
        "n": int(len(pts)),
        "foot_major": float(foot_major),
        "foot_minor": float(foot_minor),
        "aspect": float(foot_major / max(foot_minor, 1e-6)),
        "height": float(height),
        "z_min": float(pts[:, 2].min()),
    }


@dataclass
class Row:
    """One percept, as seen by both detectors."""
    index: int
    label_a: str
    label_b: str
    conf_a: float = None   # None when that detector rejected the percept
    conf_b: float = None
    #: euclidean x/y distance between the two emitted Observations, and the
    #: yaw difference -- only defined when BOTH accepted with the SAME label.
    pos_delta: float = None
    yaw_delta: float = None
    features: dict = field(default_factory=dict)

    @property
    def agree(self):
        return self.label_a == self.label_b


@dataclass
class Comparison:
    name_a: str
    name_b: str
    rows: list

    @property
    def total(self):
        return len(self.rows)

    @property
    def disagreements(self):
        return [r for r in self.rows if not r.agree]

    @property
    def agreements(self):
        return self.total - len(self.disagreements)

    def label_counts(self, which):
        counts = {}
        for r in self.rows:
            lab = r.label_a if which == "a" else r.label_b
            counts[lab] = counts.get(lab, 0) + 1
        return counts

    def unknown_count(self, which):
        return self.label_counts(which).get(UNKNOWN, 0)

    @property
    def unknown_delta(self):
        """B's unknown count minus A's. NEGATIVE means B rejects FEWER."""
        return self.unknown_count("b") - self.unknown_count("a")

    @property
    def pos_deltas(self):
        return [r.pos_delta for r in self.rows if r.pos_delta is not None]

    @property
    def max_pos_delta(self):
        d = self.pos_deltas
        return max(d) if d else 0.0

    @property
    def mean_pos_delta(self):
        d = self.pos_deltas
        return sum(d) / len(d) if d else 0.0


def _by_percept(labels, observations):
    """Map percept index -> Observation, by walking labels and accepted output.

    The seam guarantees `observations` are the accepted percepts IN INPUT
    ORDER, one per non-unknown label (test_detector_seam asserts exactly this),
    so the i-th non-unknown label owns the i-th observation. Guarded rather
    than assumed: a detector that violates it gets no position comparison
    instead of a silently misaligned one.
    """
    accepted = [i for i, l in enumerate(labels) if l != UNKNOWN]
    if len(accepted) != len(observations):
        return {}
    return dict(zip(accepted, observations))


def compare(percepts, det_a, det_b, frame_id=None, stamp=None):
    """Run both detectors over the same percepts and pair up the results.

    `det_a` / `det_b` are detector INSTANCES (not names), so a caller may pass
    a one-off implementation that is not in the registry -- which is what makes
    this testable against a stub.
    """
    labels_a, obs_a = det_a.detect(percepts, frame_id=frame_id, stamp=stamp)
    labels_b, obs_b = det_b.detect(percepts, frame_id=frame_id, stamp=stamp)
    if len(labels_a) != len(percepts) or len(labels_b) != len(percepts):
        raise ValueError(
            "a detector returned %d/%d labels for %d percepts; detect() must "
            "label every percept" % (len(labels_a), len(labels_b), len(percepts)))

    map_a = _by_percept(labels_a, obs_a)
    map_b = _by_percept(labels_b, obs_b)

    rows = []
    for i, percept in enumerate(percepts):
        oa, ob = map_a.get(i), map_b.get(i)
        row = Row(index=i,
                  label_a=labels_a[i], label_b=labels_b[i],
                  conf_a=None if oa is None else oa.confidence,
                  conf_b=None if ob is None else ob.confidence)
        if oa is not None and ob is not None and row.agree:
            row.pos_delta = math.hypot(oa.x - ob.x, oa.y - ob.y)
            if oa.yaw is not None and ob.yaw is not None:
                row.yaw_delta = abs(_wrap(oa.yaw - ob.yaw))
        if not row.agree:
            row.features = percept_features(percept)
        rows.append(row)

    return Comparison(name_a=getattr(det_a, "name", "A") or "A",
                      name_b=getattr(det_b, "name", "B") or "B",
                      rows=rows)


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# --- reporting ---------------------------------------------------------

def _conf(c):
    return "  -  " if c is None else "%5.2f" % c


def format_table(cmp):
    lines = []
    head = "%4s | %-14s %5s | %-14s %5s | %s" % (
        "idx", cmp.name_a, "conf", cmp.name_b, "conf", "verdict")
    lines.append(head)
    lines.append("-" * len(head))
    for r in cmp.rows:
        lines.append("%4d | %-14s %s | %-14s %s | %s" % (
            r.index, r.label_a, _conf(r.conf_a),
            r.label_b, _conf(r.conf_b),
            "AGREE" if r.agree else "DISAGREE"))
        if not r.agree and r.features:
            f = r.features
            lines.append("     `-> n=%d foot_major=%.3f foot_minor=%.3f "
                         "aspect=%.2f height=%.3f z_min=%.3f"
                         % (f["n"], f["foot_major"], f["foot_minor"],
                            f["aspect"], f["height"], f["z_min"]))
        elif not r.agree:
            lines.append("     `-> (no shape features available for this percept type)")
    return "\n".join(lines)


def _counts_str(counts):
    # ordered by count descending -- magnitude, not alphabet
    return ", ".join("%s=%d" % (k, v) for k, v in
                     sorted(counts.items(), key=lambda kv: (-kv[1], kv[0])))


def format_summary(cmp):
    n = cmp.total
    ua, ub = cmp.unknown_count("a"), cmp.unknown_count("b")
    pct = lambda c: (100.0 * c / n) if n else 0.0
    lines = [
        "SUMMARY  %s (A) vs %s (B)" % (cmp.name_a, cmp.name_b),
        "  percepts:      %d" % n,
        "  agreements:    %d" % cmp.agreements,
        "  disagreements: %d" % len(cmp.disagreements),
        "  labels A: %s" % (_counts_str(cmp.label_counts("a")) or "(none)"),
        "  labels B: %s" % (_counts_str(cmp.label_counts("b")) or "(none)"),
        "",
        "UNKNOWN RATE (safety: rejecting is how phantom landmarks are avoided)",
        "  A %-14s %d/%d (%.1f%%)" % (cmp.name_a, ua, n, pct(ua)),
        "  B %-14s %d/%d (%.1f%%)" % (cmp.name_b, ub, n, pct(ub)),
        "  delta (B-A):   %+d" % cmp.unknown_delta,
    ]
    if cmp.unknown_delta < 0:
        lines.append("  *** WARNING: B REJECTS FEWER PERCEPTS THAN A (%d fewer). "
                     "Every extra acceptance is a candidate phantom landmark; "
                     "check the disagreement rows above before trusting B. ***"
                     % (-cmp.unknown_delta))
    elif cmp.unknown_delta > 0:
        lines.append("  (B rejects %d more than A -- more conservative; check "
                     "it is not dropping real landmarks.)" % cmp.unknown_delta)
    lines += [
        "",
        "POSITION DELTA (percepts both accepted with the SAME label: %d)"
        % len(cmp.pos_deltas),
        "  max  %.6f m" % cmp.max_pos_delta,
        "  mean %.6f m" % cmp.mean_pos_delta,
    ]
    yaws = [r.yaw_delta for r in cmp.rows if r.yaw_delta is not None]
    if yaws:
        lines.append("  max yaw delta %.6f rad" % max(yaws))
    if cmp.max_pos_delta > 0:
        lines.append("  NOTE: reduction is type-dispatched, so identical labels "
                     "should reduce to identical positions -- a nonzero delta "
                     "means the two detectors placed the same percept "
                     "differently.")
    return "\n".join(lines)


def format_report(cmp):
    return format_table(cmp) + "\n\n" + format_summary(cmp)


def to_dict(cmp):
    """JSON-serialisable form of the comparison (for --json)."""
    return {
        "a": cmp.name_a, "b": cmp.name_b,
        "total": cmp.total,
        "agreements": cmp.agreements,
        "disagreements": len(cmp.disagreements),
        "unknown_a": cmp.unknown_count("a"),
        "unknown_b": cmp.unknown_count("b"),
        "unknown_delta": cmp.unknown_delta,
        "labels_a": cmp.label_counts("a"),
        "labels_b": cmp.label_counts("b"),
        "max_pos_delta": cmp.max_pos_delta,
        "mean_pos_delta": cmp.mean_pos_delta,
        "rows": [{"index": r.index,
                  "label_a": r.label_a, "conf_a": r.conf_a,
                  "label_b": r.label_b, "conf_b": r.conf_b,
                  "agree": r.agree,
                  "pos_delta": r.pos_delta, "yaw_delta": r.yaw_delta,
                  "features": r.features} for r in cmp.rows],
    }


# --- CLI ---------------------------------------------------------------

def build_parser():
    names = ", ".join(sorted(detector.DETECTORS))
    p = argparse.ArgumentParser(
        prog="python3 -m landmark_loc.ab_compare",
        description="Compare two registered detectors on the same percepts.",
        epilog="registered detectors: " + names)
    p.add_argument("--a", default=detector.DEFAULT_DETECTOR,
                   help="detector A (baseline). default: %(default)s")
    p.add_argument("--b", default=detector.DEFAULT_DETECTOR,
                   help="detector B (candidate). default: %(default)s")
    p.add_argument("--fixture", default=_FIXTURE_NPZ,
                   help="captured-cluster .npz to compare over. "
                        "default: the checked-in in-sim capture")
    p.add_argument("--json", action="store_true",
                   help="emit machine-readable JSON instead of the table")
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    for which, name in (("--a", args.a), ("--b", args.b)):
        if name not in detector.DETECTORS:
            build_parser().error(
                "%s: unknown detector %r; registered: %s"
                % (which, name, ", ".join(sorted(detector.DETECTORS))))
    percepts = load_fixture_percepts(args.fixture)
    cmp = compare(percepts,
                  detector.get_detector(args.a),
                  detector.get_detector(args.b))
    print(json.dumps(to_dict(cmp), indent=2) if args.json else format_report(cmp))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
