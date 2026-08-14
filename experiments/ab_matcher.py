"""A/B harness for the identity-vs-geometry experiment: run the SAME inputs
through the typed and typeless constellation matchers and report a comparison.
Offline/synthetic here; sim wiring is a later step (see the spec's Out of scope).
"""
from landmark_loc import constellation, constellation_typeless


def _summary(pairs):
    names = sorted(lm.name for _, lm in pairs)
    return {"n_inliers": len(pairs), "names": names}


def compare(observations, gated, prior, tol):
    typed = constellation.match(observations, gated, prior, tol)
    typeless = constellation_typeless.match(observations, gated, prior, tol)
    ts, tls = _summary(typed), _summary(typeless)
    return {
        "typed": ts,
        "typeless": tls,
        "agree": ts["names"] == tls["names"],
    }


if __name__ == "__main__":
    import math
    from landmark_loc.classify import Observation
    from landmark_loc.catalog import MapLandmark

    def scene(true_x, true_y, true_yaw, cat):
        c, s = math.cos(-true_yaw), math.sin(-true_yaw)
        return [Observation(lm.identity, c*(lm.x-true_x)-s*(lm.y-true_y),
                            s*(lm.x-true_x)+c*(lm.y-true_y)) for lm in cat]

    # (1) unambiguous scene -> expect agreement
    cat1 = [MapLandmark("lampA", "lamp", 10.0, 0.0),
            MapLandmark("benchB", "bench", 13.0, 4.0),
            MapLandmark("binC", "trash_bin_1", 8.0, 5.0)]
    print("unambiguous:", compare(scene(0, 0, 0, cat1), cat1, (0, 0, 0), 1.0))

    # (2) self-similar scene: a row of identical-type landmarks at equal spacing
    #     -> the typeless matcher can lock onto a period-shifted arrangement.
    row = [MapLandmark(f"lamp{i}", "lamp", 10.0 + 3.0*i, 0.0) for i in range(4)]
    extra = [MapLandmark("benchX", "bench", 11.5, 4.0)]
    cat2 = row + extra
    # robot sees 3 of the lamps + the bench, from the true pose
    seen = [row[0], row[1], row[2], extra[0]]
    print("self-similar:", compare(scene(0, 0, 0, seen), cat2, (0, 0, 0), 1.0))
