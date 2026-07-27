#!/usr/bin/env python3
"""PyQt5 WASD teleop window for the Gazebo Husky (ROS 1 Noetic, Python 3).

The terminal sibling (husky_teleop.py) cannot see key *release* - a TTY delivers
characters only - so it infers release from a timeout and the robot always
coasts for up to KEY_TIMEOUT_S. Qt delivers real KeyPress/KeyRelease events, so
this version stops the instant the key comes up. Run it on the X session:

    source /opt/ros/noetic/setup.bash && DISPLAY=:1 python3 /workspace/husky_teleop_gui.py

PyQt5 is already present in the container (it arrives with rqt); no rebuild.
"""

import signal
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget

# --------------------------------------------------------------------------
# KEY MAP - edit this dict alone to remap the controls.
#
# Keyed by Qt key code rather than by literal character (as husky_teleop.py is),
# because Qt reports the *physical* key independently of Shift: 'w' and 'W' are
# one entry here, and boost is read off the Shift modifier instead. That also
# means a release event still matches even if the user let go of Shift first.
# --------------------------------------------------------------------------
KEY_MAP = {
    # qt key:        (linear, angular)
    Qt.Key_W:     (1.0, 0.0),    # forward
    Qt.Key_S:     (-1.0, 0.0),   # backward
    Qt.Key_A:     (0.0, 1.0),    # turn left
    Qt.Key_D:     (0.0, -1.0),   # turn right
    Qt.Key_Space: (0.0, 0.0),    # stop
}

KEY_LABELS = {
    Qt.Key_W: 'w', Qt.Key_S: 's', Qt.Key_A: 'a', Qt.Key_D: 'd',
    Qt.Key_Space: 'space',
}

BOOST_FACTOR = 2.0

# Speed adjustment keys. '=' and '_' are accepted as the unshifted/shifted
# twins of '+' and '-' so the user never has to think about Shift here.
SPEED_UP_KEYS = (Qt.Key_Plus, Qt.Key_Equal)
SPEED_DOWN_KEYS = (Qt.Key_Minus, Qt.Key_Underscore)
SPEED_STEP = 0.10          # 10% per press
SPEED_MIN = 0.05           # m/s and rad/s floor, so speed can't reach zero
SPEED_MAX = 2.0            # m/s ceiling for linear (angular scales with it)

QUIT_KEYS = (Qt.Key_Q,)

# ros_control expects a steady stream, not one message per key event, so we
# republish the current command at this rate regardless of input activity.
PUBLISH_RATE_HZ = 10.0

# Backstop only. Real key release is what stops the robot; this catches a
# KeyRelease genuinely lost in transit over a laggy VNC link. It is deliberately
# far longer than the X auto-repeat period (25/s = 40ms) so it can never fire
# while a key is actually held down during normal driving.
KEY_WATCHDOG_S = 1.0

# Starting speeds.
SPEED_LINEAR = 0.5    # m/s
SPEED_ANGULAR = 1.0   # rad/s

# Husky runs twist_mux, which arbitrates cmd_vel inputs by priority.
# /kb_teleop/cmd_vel is the keyboard slot; bare /cmd_vel can be overridden.
CMD_VEL_TOPIC = '/kb_teleop/cmd_vel'

HELP_TEXT = """\
<b>Husky Teleop</b> &rarr; %s<br><br>
&nbsp;<b>w</b> forward &nbsp; <b>s</b> backward &nbsp;
<b>a</b> turn left &nbsp; <b>d</b> turn right<br>
&nbsp;<b>space</b> stop &nbsp;&nbsp; <b>Shift</b>+wasd boost (x%.1f)<br>
&nbsp;<b>+</b> / <b>-</b> speed %d%% steps &nbsp;&nbsp; <b>q</b> quit<br><br>
&nbsp;Motion stops the moment you release the key.<br>
&nbsp;This window must have focus.\
""" % (CMD_VEL_TOPIC, BOOST_FACTOR, int(SPEED_STEP * 100))


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def _describe(linear, angular, held):
    """Name a *blended* command honestly: both axes, or 'hold' if they cancel.

    Picking one axis to display would lie about w+a, and would show 'forward'
    for w+s when the robot is in fact stationary.
    """
    parts = []
    if linear > 0:
        parts.append('forward')
    elif linear < 0:
        parts.append('backward')
    if angular > 0:
        parts.append('left')
    elif angular < 0:
        parts.append('right')
    what = '-'.join(parts) if parts else 'hold'

    keys = '+'.join(KEY_LABELS[k] for k in held if k in KEY_LABELS)
    return '%s (%s)' % (what, keys) if keys else what


class TeleopWindow(QWidget):
    def __init__(self, pub):
        super().__init__()
        self._pub = pub

        self._linear_speed = SPEED_LINEAR
        self._angular_speed = SPEED_ANGULAR

        # Every currently-held direction key. All of them contribute to one
        # Twist: w+a drives forward *while* turning left (an arc) rather than
        # the newest key winning outright. Insertion order is kept only so the
        # status line can list the keys the way they were pressed.
        self._held = []
        self._boost = False
        self._last_key_activity = 0.0

        self._build_ui()

        # A plain QWidget has Qt.NoFocus by default and would then never receive
        # a single key event - the window would look right and do nothing.
        self.setFocusPolicy(Qt.StrongFocus)

        self._publish_timer = QTimer(self)
        self._publish_timer.timeout.connect(self._on_publish_tick)
        self._publish_timer.start(int(1000.0 / PUBLISH_RATE_HZ))

    # -- UI ----------------------------------------------------------------

    def _build_ui(self):
        self.setWindowTitle('Husky Teleop')

        layout = QVBoxLayout(self)
        help_label = QLabel(HELP_TEXT, self)
        help_label.setTextFormat(Qt.RichText)
        layout.addWidget(help_label)

        mono = QFont('Monospace')
        mono.setStyleHint(QFont.TypeWriter)

        self._speed_label = QLabel(self)
        self._speed_label.setFont(mono)
        layout.addWidget(self._speed_label)

        # The user spent a whole session unable to tell whether keypresses were
        # registering at all, so the live command is shown verbatim rather than
        # just the key map.
        self._status_label = QLabel(self)
        self._status_label.setFont(mono)
        layout.addWidget(self._status_label)

        self._refresh_labels(0.0, 0.0, 'idle')

    def _refresh_labels(self, linear, angular, what):
        self._speed_label.setText(
            'speed setting:  %.2f m/s   %.2f rad/s%s'
            % (self._linear_speed, self._angular_speed,
               '   [BOOST x%.1f]' % BOOST_FACTOR if self._boost else ''))
        self._status_label.setText(
            'commanding:     %-22s linear %+.2f  angular %+.2f'
            % (what, linear, angular))

    # -- key handling ------------------------------------------------------

    def keyPressEvent(self, event):
        # X auto-repeat synthesises a KeyRelease/KeyPress pair for every repeat
        # while a key is held. Without this filter the release half looks like a
        # real key-up, so the robot would stutter between driving and stopping
        # ~25 times a second. Do not remove.
        if event.isAutoRepeat():
            self._last_key_activity = time.monotonic()
            return

        key = event.key()

        if key in QUIT_KEYS:
            self.close()
            return

        if key in SPEED_UP_KEYS or key in SPEED_DOWN_KEYS:
            factor = (1.0 + SPEED_STEP) if key in SPEED_UP_KEYS \
                else (1.0 - SPEED_STEP)
            self._linear_speed = min(
                SPEED_MAX, max(SPEED_MIN, self._linear_speed * factor))
            self._angular_speed = max(SPEED_MIN, self._angular_speed * factor)
            self._last_key_activity = time.monotonic()
            return

        if key in KEY_MAP:
            if key == Qt.Key_Space:
                # Space is an explicit "everything off", not a held direction.
                self._held = []
            elif key not in self._held:
                self._held.append(key)
            self._boost = bool(event.modifiers() & Qt.ShiftModifier)
            self._last_key_activity = time.monotonic()
            return

        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        # Same auto-repeat trap as above - this is the half that actually causes
        # the stutter, because it looks exactly like the user letting go.
        if event.isAutoRepeat():
            self._last_key_activity = time.monotonic()
            return

        key = event.key()
        if key in self._held:
            self._held.remove(key)
        self._last_key_activity = time.monotonic()

        if not self._held:
            self._boost = False

        if key not in KEY_MAP:
            super().keyReleaseEvent(event)

    def focusOutEvent(self, event):
        # Alt-tabbing (or clicking Gazebo) means we stop receiving key events -
        # including the KeyRelease for whatever is currently held. Without this
        # the robot would keep driving with no window able to stop it.
        self._held = []
        self._boost = False
        super().focusOutEvent(event)

    # -- publishing --------------------------------------------------------

    def _current_command(self):
        if not self._held:
            return 0.0, 0.0, 'idle'

        # Sum every held key's contribution. Opposing keys cancelling (w+s ->
        # no linear, a+d -> no angular) is the arithmetic falling out of this
        # and is intended; there is deliberately no tie-break.
        lin_scale = sum(KEY_MAP[k][0] for k in self._held)
        ang_scale = sum(KEY_MAP[k][1] for k in self._held)

        # Boost multiplies the blended result as a whole, so a boosted arc
        # keeps the same shape as the unboosted one - only faster.
        boost = BOOST_FACTOR if self._boost else 1.0
        linear = lin_scale * self._linear_speed * boost
        angular = ang_scale * self._angular_speed * boost

        # Blending must not be able to outrun the configured limits. The scales
        # only ever sum to -1..1 per axis, so this bites only on the boost.
        linear = _clamp(linear, SPEED_MAX)
        angular = _clamp(angular, self._angular_speed * BOOST_FACTOR)

        return linear, angular, _describe(linear, angular, self._held)

    def _on_publish_tick(self):
        if rospy.is_shutdown():
            self.close()
            return

        if self._held and \
                time.monotonic() - self._last_key_activity > KEY_WATCHDOG_S:
            # Only reachable if a KeyRelease was lost; auto-repeat would
            # otherwise have refreshed _last_key_activity every 40ms.
            self._held = []
            self._boost = False

        linear, angular, what = self._current_command()

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        try:
            self._pub.publish(twist)
        except rospy.ROSException:
            # Publisher torn down under us during shutdown; nothing useful to do.
            return

        self._refresh_labels(linear, angular, what)

    # -- shutdown ----------------------------------------------------------

    def closeEvent(self, event):
        # Reached both by 'q' and by the window manager's close button, so the
        # final stop lives here rather than next to the quit key.
        self._publish_timer.stop()
        try:
            self._pub.publish(Twist())
            time.sleep(0.1)   # let the publisher flush before the node dies
        except Exception as exc:  # noqa: BLE001 - shutdown path, report and move on
            print('Warning: could not publish the final stop command: %s' % exc,
                  file=sys.stderr)
        print('Teleop stopped; zero velocity sent on %s.' % CMD_VEL_TOPIC)
        super().closeEvent(event)


def main():
    # Distinct node name from husky_teleop.py: both use anonymous=False, so
    # sharing a name would make the master kill whichever registered first -
    # running the terminal fallback alongside this window must stay possible.
    rospy.init_node('husky_teleop_gui', anonymous=False, disable_signals=True)
    pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

    app = QApplication(sys.argv)

    # Python-level signal handlers only run between bytecodes, and Qt's event
    # loop sits in C waiting on X. SIG_DFL makes Ctrl-C kill the process
    # outright; the timer below gives the interpreter a chance to notice it.
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    idle = QTimer()
    idle.start(200)
    idle.timeout.connect(lambda: None)

    window = TeleopWindow(pub)
    window.show()

    # Anchor bottom-left of the 1280x720 Xvfb screen so it overlaps Gazebo as
    # little as possible. Done after show() because the frame size - and hence
    # the y needed to sit on the bottom edge - is not known until then.
    screen = app.primaryScreen().availableGeometry()
    window.move(screen.left(), screen.bottom() - window.frameGeometry().height() + 1)
    window.activateWindow()
    window.raise_()

    sys.exit(app.exec_())


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
