"""Lightweight multi-hypothesis position tracker: hold a few candidate (x, y)
guesses, carry them across scans, and commit the one that stays consistent.

This is the cure for identical-object aliasing (23 identical trees etc.): a
single scan cannot tell a correct match from a plausible-but-wrong one, but only
the correct hypothesis stays reinforced as the robot moves. It is a particle
filter with a handful of particles and no resampling (design spec section 7).
"""
import math


class Hypothesis(object):
    __slots__ = ("x", "y", "support")

    def __init__(self, x, y, support=1):
        self.x = x
        self.y = y
        self.support = support


class HypothesisTracker(object):
    def __init__(self, k=5, merge_dist=2.0, commit_support=3):
        self.k = k
        self.merge_dist = merge_dist
        self.commit_support = commit_support
        self.hypotheses = []

    def predict(self, dx, dy):
        for h in self.hypotheses:
            h.x += dx
            h.y += dy

    def update(self, candidates):
        reinforced = set()
        for (cx, cy) in candidates:
            best, bestd = None, self.merge_dist
            for h in self.hypotheses:
                d = math.hypot(h.x - cx, h.y - cy)
                if d <= bestd:
                    best, bestd = h, d
            if best is not None:
                # move toward the candidate a little; bump support
                best.x = 0.5 * (best.x + cx)
                best.y = 0.5 * (best.y + cy)
                best.support += 1
                reinforced.add(id(best))
            else:
                self.hypotheses.append(Hypothesis(cx, cy, 1))
                reinforced.add(id(self.hypotheses[-1]))
        # decay the unreinforced
        for h in self.hypotheses:
            if id(h) not in reinforced:
                h.support -= 1
        self.hypotheses = [h for h in self.hypotheses if h.support > 0]
        # keep top-k by support
        self.hypotheses.sort(key=lambda h: h.support, reverse=True)
        self.hypotheses = self.hypotheses[:self.k]

    def committed(self):
        best = None
        for h in self.hypotheses:
            if h.support >= self.commit_support:
                if best is None or h.support > best.support:
                    best = h
        return best
