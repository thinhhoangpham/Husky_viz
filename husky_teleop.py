#!/usr/bin/env python3
"""Standalone WASD teleop node for the Gazebo Husky (ROS 1 Noetic, Python 3).

Reads single keypresses from the terminal in cbreak mode and publishes
geometry_msgs/Twist at a fixed rate. Run it from a terminal with a TTY:

    source /opt/ros/noetic/setup.bash && python3 /workspace/husky_teleop.py
"""

import sys
import select
import termios
import tty
import time

import rospy
from geometry_msgs.msg import Twist

# --------------------------------------------------------------------------
# KEY MAP - edit this dict alone to remap the controls.
#
# Each entry maps a single literal keystroke to (linear_scale, angular_scale).
# Those scales are multiplied by the current speed settings (SPEED_LINEAR /
# SPEED_ANGULAR, adjustable at runtime with + / -), so use +/-1.0 for normal
# motion and a larger magnitude for a "boost" variant.
#
# Keys are matched exactly and are case sensitive: 'w' and 'W' are distinct
# entries, which is how Shift is supported (uppercase = boost).
# --------------------------------------------------------------------------
KEY_MAP = {
    # key:  (linear, angular)
    'w': (1.0, 0.0),    # forward
    's': (-1.0, 0.0),   # backward
    'a': (0.0, 1.0),    # turn left
    'd': (0.0, -1.0),   # turn right
    ' ': (0.0, 0.0),    # stop

    # Shift-held variants (uppercase) - same directions at BOOST_FACTOR speed.
    'W': (1.0, 0.0),
    'S': (-1.0, 0.0),
    'A': (0.0, 1.0),
    'D': (0.0, -1.0),
}

# Keys that count as "boost" (Shift held). Kept alongside KEY_MAP so remapping
# stays a single-dict edit for the common case.
BOOST_KEYS = frozenset('WASD')
BOOST_FACTOR = 2.0

# Speed adjustment keys. NOTE (deviation from the original request): Ctrl
# combinations are deliberately NOT bound. In a raw/cbreak terminal they arrive
# as control bytes, and Ctrl-C (0x03) must stay reserved for quitting. '+' / '-'
# are used instead, with '=' accepted as an unshifted synonym for '+'.
SPEED_UP_KEYS = ('+', '=')
SPEED_DOWN_KEYS = ('-', '_')
SPEED_STEP = 0.10          # 10% per press
SPEED_MIN = 0.05           # m/s and rad/s floor, so speed can't reach zero
SPEED_MAX = 2.0            # m/s ceiling for linear (angular scales with it)

QUIT_KEYS = ('q', 'Q', '\x03')  # 'q' or Ctrl-C

# Publish rate and the safety timeout: a terminal cannot detect key *release*,
# so if nothing has been pressed for this long we publish zero velocity rather
# than letting the robot drive away. Must stay comfortably above the X server's
# 250ms auto-repeat delay (`xset r rate 250 30` in entrypoint.sh), or motion
# stutters in the gap before the first repeat arrives; 0.35 leaves ~100ms
# jitter margin. Lower this only after lowering that xset delay first.
PUBLISH_RATE_HZ = 10.0
KEY_TIMEOUT_S = 0.35

# Starting speeds.
SPEED_LINEAR = 0.5    # m/s
SPEED_ANGULAR = 1.0   # rad/s

# Husky runs twist_mux, which arbitrates cmd_vel inputs by priority.
# /kb_teleop/cmd_vel is the keyboard slot; bare /cmd_vel can be overridden.
CMD_VEL_TOPIC = '/kb_teleop/cmd_vel'


def build_help(linear, angular):
    lines = ['', 'Husky WASD teleop -> %s' % CMD_VEL_TOPIC, '']
    for key, (lin, ang) in KEY_MAP.items():
        if key in BOOST_KEYS:
            continue
        label = 'space' if key == ' ' else key
        if lin > 0:
            what = 'forward'
        elif lin < 0:
            what = 'backward'
        elif ang > 0:
            what = 'turn left'
        elif ang < 0:
            what = 'turn right'
        else:
            what = 'stop'
        lines.append('  %-7s %s' % (label, what))
    lines += [
        '  Shift+  %s   boost (x%.1f)' % (
            '/'.join(sorted(BOOST_KEYS)), BOOST_FACTOR),
        '  + / -   speed up / down (%d%% steps)' % int(SPEED_STEP * 100),
        '          Ctrl combinations are NOT bound - in a raw terminal they',
        '          arrive as control bytes and Ctrl-C must stay as quit.',
        '          "=" works as an unshifted "+".',
        '  q       quit (Ctrl-C also works)',
        '',
        '  speed: %.2f m/s  %.2f rad/s' % (linear, angular),
        '',
        '  Keys only register while this terminal is focused.',
        '  Motion stops automatically after %.1fs with no keypress.' % KEY_TIMEOUT_S,
        '',
    ]
    return '\n'.join(lines)


def read_key(timeout):
    """Return one character if available within `timeout` seconds, else None."""
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return None
    ch = sys.stdin.read(1)
    return ch if ch else None


def teleop_loop(pub):
    linear_speed = SPEED_LINEAR
    angular_speed = SPEED_ANGULAR

    print(build_help(linear_speed, angular_speed))

    lin_scale = 0.0
    ang_scale = 0.0
    boost = 1.0
    last_key_time = 0.0

    rate = rospy.Rate(PUBLISH_RATE_HZ)
    period = 1.0 / PUBLISH_RATE_HZ

    while not rospy.is_shutdown():
        key = read_key(period)

        if key is not None:
            if key in QUIT_KEYS:
                break

            if key in SPEED_UP_KEYS or key in SPEED_DOWN_KEYS:
                factor = (1.0 + SPEED_STEP) if key in SPEED_UP_KEYS \
                    else (1.0 - SPEED_STEP)
                linear_speed = min(SPEED_MAX, max(SPEED_MIN, linear_speed * factor))
                angular_speed = max(SPEED_MIN, angular_speed * factor)
                print('speed: %.2f m/s  %.2f rad/s\r' % (linear_speed, angular_speed))
            elif key in KEY_MAP:
                lin_scale, ang_scale = KEY_MAP[key]
                boost = BOOST_FACTOR if key in BOOST_KEYS else 1.0
                last_key_time = time.time()
            # Unmapped keys are ignored; they do not reset the safety timeout.

        # Safety timeout: no keypress recently -> command zero velocity.
        if time.time() - last_key_time > KEY_TIMEOUT_S:
            lin_scale = 0.0
            ang_scale = 0.0
            boost = 1.0

        twist = Twist()
        twist.linear.x = lin_scale * linear_speed * boost
        twist.angular.z = ang_scale * angular_speed * boost
        pub.publish(twist)

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


def main():
    stdin_fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(stdin_fd)

    rospy.init_node('husky_teleop', anonymous=False, disable_signals=True)
    pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=1)

    try:
        tty.setcbreak(stdin_fd)
        teleop_loop(pub)
    except KeyboardInterrupt:
        pass
    finally:
        # Restore the terminal FIRST so the shell is usable no matter what
        # happened above, then make sure the robot is not left coasting.
        termios.tcsetattr(stdin_fd, termios.TCSADRAIN, old_settings)
        try:
            pub.publish(Twist())
            # Give the publisher a moment to flush before the node dies.
            time.sleep(0.1)
        except Exception as exc:  # noqa: BLE001 - shutdown path, report and move on
            print('Warning: could not publish the final stop command: %s' % exc,
                  file=sys.stderr)
        print('\nTeleop stopped; zero velocity sent on %s.' % CMD_VEL_TOPIC)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except termios.error as exc:
        print('Error: this node needs an interactive terminal (TTY): %s' % exc,
              file=sys.stderr)
        sys.exit(1)
