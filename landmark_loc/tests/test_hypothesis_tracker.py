from landmark_loc.hypothesis_tracker import HypothesisTracker


def test_single_consistent_candidate_commits_after_n():
    t = HypothesisTracker(commit_support=3, merge_dist=1.0)
    assert t.committed() is None
    t.update([(10.0, 5.0)])
    assert t.committed() is None            # support 1
    t.update([(10.1, 5.0)])
    assert t.committed() is None            # support 2
    t.update([(9.9, 5.1)])
    c = t.committed()
    assert c is not None and abs(c.x - 10.0) < 0.5


def test_impostor_dies_when_it_stops_being_seen():
    t = HypothesisTracker(commit_support=3, merge_dist=1.0)
    # both seen once
    t.update([(10.0, 5.0), (40.0, 30.0)])
    # only the true one keeps being seen; impostor decays
    t.update([(10.0, 5.0)])
    t.update([(10.0, 5.0)])
    c = t.committed()
    assert c is not None and abs(c.x - 10.0) < 0.5
    # impostor should be gone (support decayed to 0)
    assert all(abs(h.x - 40.0) > 1.0 for h in t.hypotheses)


def test_predict_shifts_hypotheses():
    t = HypothesisTracker(merge_dist=0.5)
    t.update([(1.0, 1.0)])
    t.predict(2.0, 0.0)
    # after predict, a candidate at (3,1) reinforces the SAME hypothesis
    t.update([(3.0, 1.0)])
    assert len(t.hypotheses) == 1
    assert t.hypotheses[0].support == 2


def test_keeps_only_top_k():
    t = HypothesisTracker(k=2, merge_dist=0.5)
    t.update([(0, 0), (10, 0), (20, 0), (30, 0)])
    assert len(t.hypotheses) <= 2
