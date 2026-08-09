from gcs_intervene import Intervene

class FakePub:
    def __init__(self): self.sent = []
    def publish(self, m): self.sent.append(m)

class FakeTwist:
    def __init__(self):
        self.linear = type("L", (), {"x":0.0,"y":0.0,"z":0.0})()
        self.angular = type("A", (), {"x":0.0,"y":0.0,"z":0.0})()

class FakeBool:
    def __init__(self, data=False): self.data = data

def test_drive_sets_twist_fields():
    tp, ep = FakePub(), FakePub()
    iv = Intervene(tp, ep, FakeTwist, FakeBool)
    iv.drive(0.5, -0.2)
    assert tp.sent[-1].linear.x == 0.5 and tp.sent[-1].angular.z == -0.2

def test_stop_is_zero_twist():
    tp, ep = FakePub(), FakePub()
    iv = Intervene(tp, ep, FakeTwist, FakeBool)
    iv.stop()
    assert tp.sent[-1].linear.x == 0.0 and tp.sent[-1].angular.z == 0.0

def test_estop_engage_release_bool():
    tp, ep = FakePub(), FakePub()
    iv = Intervene(tp, ep, FakeTwist, FakeBool)
    iv.engage_estop(); iv.release_estop()
    assert ep.sent[0].data is True and ep.sent[1].data is False
