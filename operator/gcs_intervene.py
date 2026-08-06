class Intervene(object):
    def __init__(self, teleop_pub, estop_pub, twist_cls, bool_cls):
        self._teleop = teleop_pub
        self._estop = estop_pub
        self._Twist = twist_cls
        self._Bool = bool_cls

    def drive(self, linx, angz):
        t = self._Twist()
        t.linear.x = linx
        t.angular.z = angz
        self._teleop.publish(t)

    def stop(self):
        self._teleop.publish(self._Twist())

    def engage_estop(self):
        self._estop.publish(self._Bool(data=True))

    def release_estop(self):
        self._estop.publish(self._Bool(data=False))
