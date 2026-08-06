class GcsState(object):
    MODES = ("AUTO", "MANUAL", "STOPPED", "ESTOP")

    def __init__(self):
        self.mode = "AUTO"
        self.sent_goal = None      # (x, y) map frame
        self.active_goal = None    # (x, y) map frame
        self.nav_status = "NONE"
        self.estop_engaged = False

    def set_mode(self, m):
        if m not in self.MODES:
            raise ValueError("unknown mode: %s" % m)
        self.mode = m

    def engage_estop(self):
        self.estop_engaged = True
        self.mode = "ESTOP"

    def release_estop(self):
        self.estop_engaged = False
        self.mode = "AUTO"
