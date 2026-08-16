"""Offline: which map objects have a shape unlike any other object.

An object whose nearest-OTHER descriptor is far away is a unique anchor. This is
a measurement over the extracted descriptor map, not an assumption: two identical
structures correctly score as non-unique and are excluded.
"""
from landmark_loc.descriptor import descriptor_distance


def nearest_distances(descriptors):
    names = list(descriptors)
    out = {}
    for a in names:
        best = float("inf")
        for b in names:
            if a is b or a == b:
                continue
            d = descriptor_distance(descriptors[a], descriptors[b])
            if d < best:
                best = d
        out[a] = best
    return out


def unique_names(descriptors, threshold):
    nd = nearest_distances(descriptors)
    return {name for name, d in nd.items() if d >= threshold}
