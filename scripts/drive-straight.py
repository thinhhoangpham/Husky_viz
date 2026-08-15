#!/usr/bin/env python3
"""drive-straight.py - spawn the STOCK park Husky itself, then drive it straight.

Publishes a CONSTANT forward geometry_msgs/Twist (linear.x only, angular.z=0)
at a steady rate to the stock Husky's cmd_vel input. No autonomous nav, no
sensing, no steering - just an open-loop "go straight" for the vulnerable
dead-reckoning topology.

WHAT THIS SCRIPT NOW OWNS
-------------------------
This script is SELF-CONTAINED: it no longer assumes some other spawner already
placed a robot in the world. Exactly like send_mapless_goal.py, it performs the
whole STOCK robot bring-up itself before it drives anything, into an
already-running park world:
  1. roslaunch husky_control control.launch (STOCK - odom-frame EKF, no
     GPS/compass), which sets /robot_description and starts the controllers,
  2. /gazebo/delete_model on "husky", which removes any leftover robot left in
     the world by a previous run so the spawn below can actually take effect
     (Gazebo refuses to spawn over an existing model name; see
     delete_existing_robot()),
  3. rosrun gazebo_ros spawn_model, which PLACES that description into the live
     world at the fixed on-path pose below,
  4. a controller-running check, which is the authoritative "did it spawn?"
     signal (see spawn_robot() for why spawn_model's exit code is not),
  5. THEN the constant-forward drive loop.
Everything it starts is torn down in reverse order on exit.

WHICH TOPIC ACTUATES THE ROBOT
------------------------------
The stock husky_control/control.launch runs twist_mux with cmd_vel_out remapped
to husky_velocity_controller/cmd_vel. twist_mux's input slots
(/opt/ros/noetic/share/husky_control/config/twist_mux.yaml) are, by priority:
    joy                 joy_teleop/cmd_vel           priority 10
    kb                  kb_teleop/cmd_vel            priority 9
    interactive_marker  twist_marker_server/cmd_vel  priority 8
    external            cmd_vel                      priority 1
This node publishes to the `external` slot, plain /cmd_vel. We are the only
publisher (no teleop/joystick is started), so the lowest priority is fine -
nothing pre-empts us. That is the topic that ultimately turns the wheels here.

NOTE: this is the STOCK twist_mux, which DOES have a kb_teleop slot (unlike the
natural_environments_ros_opt overlay's twist_mux.yaml, whose only slots are
joy_teleop/cmd_vel, twist_marker_server/cmd_vel and cmd_vel). We stick with
/cmd_vel regardless - it exists in both and is the intended external-command
input.

GROUND-TRUTH RULE: this node reads NO simulator STATE - it neither imports a
gazebo_msgs pose type nor subscribes to /gazebo/model_states or calls
/gazebo/get_model_state. The only Gazebo services it touches are
/gazebo/delete_model and gazebo_ros spawn_model, which reposition/instantiate
the robot; they are not pose sources (send_mapless_goal.py uses exactly these).
It otherwise only publishes a fixed Twist.

USAGE (two terminals, each having sourced /opt/ros/noetic/setup.bash, against
the same ROS master):
    (1) ./load-park-stock-husky.sh      # park WORLD only - no robot
    (2) ./drive-straight.py             # spawns the robot ITSELF, then drives
                                        # 0.5 m/s at 20 Hz to /cmd_vel
    ./drive-straight.py --speed 0.8     # 0.8 m/s
    ./drive-straight.py --rate 10       # 10 Hz publish rate
    ./drive-straight.py --topic /cmd_vel
No separate spawner is needed any more.

Ctrl-C (or any shutdown) publishes a clean zero Twist so the robot stops
rather than coasting on the last command, then tears the spawned robot down.
"""

import argparse
import csv
import os
import signal
import subprocess
import sys
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import DeleteModel
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Defaults. 0.5 m/s matches the starting linear speed in husky_teleop.py and is
# a gentle, controllable straight-line pace. 20 Hz is well inside twist_mux's
# 0.5 s per-slot timeout, so a single dropped message never lets the input lapse
# and stall the robot.
DEFAULT_SPEED = 0.5     # m/s, linear.x
DEFAULT_RATE = 20.0     # Hz
DEFAULT_TOPIC = "/cmd_vel"   # stock twist_mux `external` slot - see module docstring

# On-path spawn: x,y = first bag waypoint (38.26, 1.25); yaw = atan2 of the
# WP1->WP2 direction = -3.1281 rad, i.e. straight down the trail. Identical to
# send_mapless_goal.py; see that file for the full derivation. z=3.3 spawns a
# few cm above the settled terrain so the robot drops and settles.
SPAWN_X = 38.26
SPAWN_Y = 1.25
SPAWN_Z = 3.3
SPAWN_YAW = -3.1281

# Bounds for the two bring-up waits. robot_description is set by control.launch
# almost immediately, so 60s is generous; the controllers have to wait for
# gzserver to actually instantiate the model under a heavy world, hence 120s.
ROBOT_DESCRIPTION_TIMEOUT_S = 60.0
CONTROLLER_TIMEOUT_S = 120.0

# Bounds for the pre-spawn cleanup. The delete service is advertised by gzserver
# as soon as gazebo_ros is up, so if it is not there within 10s it is not coming
# and we must not block the run on it. The settle is how long we give gzserver to
# actually retire the entity after it acknowledges the delete.
DELETE_SERVICE_TIMEOUT_S = 10.0
DELETE_SETTLE_S = 1.5
ROBOT_MODEL_NAME = "husky"


def _stop_proc_group(proc, label):
    """Tear down a roslaunch process group started with start_new_session=True:
    SIGINT the whole group (lets roslaunch shut its nodes down gracefully),
    escalate to SIGTERM then SIGKILL if it lingers, so nothing is left orphaned.
    Faithful to send_mapless_goal.py's _stop_proc_group()."""
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
    except OSError:
        return

    rospy.loginfo("Shutting down %s (pgid=%d) ...", label, pgid)
    for sig, wait_s in ((signal.SIGINT, 15.0), (signal.SIGTERM, 5.0),
                        (signal.SIGKILL, 2.0)):
        try:
            os.killpg(pgid, sig)
        except OSError:
            return  # group already gone
        try:
            proc.wait(timeout=wait_s)
            return
        except subprocess.TimeoutExpired:
            continue


def start_robot():
    """Bring up the STOCK husky_control/control.launch in its own process group
    (odom-frame EKF, no GPS/compass). This loads the STOCK robot_description, the
    controllers, twist_mux and robot_state_publisher. It does NOT itself place
    the model into Gazebo - that is spawn_robot()'s job. Long-lived until
    teardown. Returns the Popen handle. Faithful to send_mapless_goal.py."""
    rospy.loginfo("Robot bring-up: roslaunch husky_control control.launch "
                  "(STOCK - odom EKF, no GPS/compass)")
    return subprocess.Popen(
        ["roslaunch", "husky_control", "control.launch"],
        start_new_session=True,  # own process group (setsid) -> group signalling
    )


def stop_robot(proc):
    """Tear down the control.launch process group started by start_robot()."""
    _stop_proc_group(proc, "robot (husky_control/control.launch)")


def wait_for_robot_description(proc):
    """Block until control.launch has published /robot_description, so that
    spawn_model has something to place. Bounded so a wedged launch cannot hang
    the script forever, and bails out early if the roslaunch process itself
    died. Returns True on success. Faithful to send_mapless_goal.py."""
    rospy.loginfo("Waiting for /robot_description (up to %.0fs) ...",
                  ROBOT_DESCRIPTION_TIMEOUT_S)
    deadline = time.monotonic() + ROBOT_DESCRIPTION_TIMEOUT_S
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            rospy.logerr("husky_control/control.launch exited (rc=%s) before "
                         "setting /robot_description.", proc.returncode)
            return False
        # rospy talks to the param server directly - no need to shell out.
        if rospy.has_param("/robot_description"):
            # Touch the value so a half-written param is not mistaken for ready.
            if rospy.get_param("/robot_description", ""):
                rospy.loginfo("/robot_description is up.")
                return True
        time.sleep(1.0)
    rospy.logerr("/robot_description never appeared within %.0fs.",
                 ROBOT_DESCRIPTION_TIMEOUT_S)
    return False


def delete_existing_robot():
    """CLEANUP step, run immediately before spawn_robot(): remove any model named
    "husky" still in the live world so the spawn below can place a FRESH robot at
    the fixed pose (Gazebo refuses to spawn over a taken model name). Every
    failure mode is non-fatal and logged at info; this function must never be the
    reason a run does not happen. Returns True only if an existing husky was
    genuinely removed, so the caller knows whether the settle sleep is needed.
    Faithful to send_mapless_goal.py; see that file for the full rationale."""
    rospy.loginfo("Pre-spawn cleanup: deleting any existing '%s' model (waiting "
                  "up to %.0fs for %s) ...",
                  ROBOT_MODEL_NAME, DELETE_SERVICE_TIMEOUT_S,
                  "/gazebo/delete_model")
    try:
        rospy.wait_for_service("/gazebo/delete_model",
                               timeout=DELETE_SERVICE_TIMEOUT_S)
    except rospy.ROSException as exc:
        rospy.loginfo("/gazebo/delete_model unavailable within %.0fs (%s) - "
                      "skipping cleanup and going straight to the spawn.",
                      DELETE_SERVICE_TIMEOUT_S, exc)
        return False

    try:
        delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        response = delete_model(model_name=ROBOT_MODEL_NAME)
    except rospy.ServiceException as exc:
        rospy.loginfo("/gazebo/delete_model call failed (%s) - carrying on to "
                      "the spawn; this is not fatal.", exc)
        return False

    if not response.success:
        # THE NORMAL FIRST-RUN PATH: no husky in the world yet, so there was
        # nothing to remove. Deliberately NOT logwarn/logerr.
        rospy.loginfo("No existing '%s' to delete (expected on a first run): %s",
                      ROBOT_MODEL_NAME, response.status_message)
        return False

    rospy.loginfo("Deleted an existing '%s' left over from a previous run: %s",
                  ROBOT_MODEL_NAME, response.status_message)
    # /gazebo/delete_model returns as soon as the removal is QUEUED. A short
    # bounded settle closes the gap before the spawn. Only sleep when we really
    # deleted something.
    rospy.sleep(DELETE_SETTLE_S)
    return True


def spawn_robot():
    """PLACEMENT step: instantiate the robot_description control.launch just set
    into the live world at our fixed pose, via /gazebo/spawn_urdf_model.

    A NON-ZERO EXIT HERE IS EXPECTED AND MUST BE TOLERATED: gazebo_ros
    spawn_model has an internal ~10s wait for the entity to APPEAR in sim; under
    the heavy park world the gzserver update loop stalls and that client-side
    wait elapses, so the client prints an error and exits non-zero even though
    THE ENTITY STILL SPAWNS. So we log the rc and carry on. The AUTHORITATIVE
    success signal is wait_for_controllers() below - never the spawn client's
    return code, and never any Gazebo ground-truth pose service. Faithful to
    send_mapless_goal.py; see that file for the full rationale."""
    rospy.loginfo("Spawning STOCK Husky at x=%.2f y=%.2f z=%.2f yaw=%.4f ...",
                  SPAWN_X, SPAWN_Y, SPAWN_Z, SPAWN_YAW)
    rc = subprocess.call([
        "rosrun", "gazebo_ros", "spawn_model",
        "-x", str(SPAWN_X), "-y", str(SPAWN_Y),
        "-z", str(SPAWN_Z), "-Y", str(SPAWN_YAW),
        "-unpause", "-urdf", "-param", "robot_description", "-model", "husky",
    ])
    if rc != 0:
        rospy.loginfo("spawn_model exited rc=%d - EXPECTED under the heavy park "
                      "world (entity-appear timeout); the controller check is "
                      "the real success signal.", rc)


def wait_for_controllers():
    """Poll controller_manager until BOTH husky_joint_publisher and
    husky_velocity_controller are ( running ). A racing/duplicated spawner leaves
    husky_velocity_controller in `initialized`, which SILENTLY drops every
    cmd_vel with no error anywhere. Warn loudly if that happens but do NOT abort.
    Returns True if both reached ( running ). Faithful to send_mapless_goal.py."""
    rospy.loginfo("Waiting for the Husky controllers (up to %.0fs) ...",
                  CONTROLLER_TIMEOUT_S)
    deadline = time.monotonic() + CONTROLLER_TIMEOUT_S
    listing = ""
    while time.monotonic() < deadline:
        try:
            listing = subprocess.run(
                ["rosrun", "controller_manager", "controller_manager", "list"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                universal_newlines=True, timeout=15.0,
            ).stdout or ""
        except (subprocess.SubprocessError, OSError) as exc:
            rospy.logdebug("controller_manager list failed: %s", exc)
            listing = ""
        running = [line for line in listing.splitlines() if "( running )" in line]
        if (any("husky_joint_publisher" in line for line in running)
                and any("husky_velocity_controller" in line for line in running)):
            rospy.loginfo("Husky controllers are running.")
            return True
        time.sleep(1.0)

    rospy.logwarn(
        "The Husky controllers are NOT both ( running ). Last seen:\n%s\n"
        "husky_velocity_controller stuck in `initialized` means the spawner died "
        "part-way - the robot will look fine but SILENTLY IGNORE every cmd_vel, "
        "so this node will publish and nothing will move. Usual cause: a leftover "
        "roslaunch/gzserver from an earlier run. Stop it and re-run. Re-check "
        "with: rosrun controller_manager controller_manager list. "
        "Continuing anyway.",
        listing.strip() or "  <controller_manager did not answer at all>")
    return False


def bring_up_robot():
    """Full robot bring-up sequence: control.launch -> robot_description ->
    delete any stale husky -> spawn_model -> controller check. Returns the
    control.launch Popen so the caller can tear it down, even if a later step
    failed (the process may still be alive and must not be leaked). Faithful to
    send_mapless_goal.py."""
    proc = start_robot()
    if wait_for_robot_description(proc):
        # The delete goes as late as possible - immediately before the spawn - so
        # the only entity that exists from here on is the fresh one and the
        # controllers unambiguously attach to it. See send_mapless_goal.py.
        delete_existing_robot()
        spawn_robot()
        wait_for_controllers()
    return proc


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="Spawn the stock Husky itself, then publish a constant "
                    "forward Twist to drive it straight.",
    )
    p.add_argument(
        "--speed", type=float, default=DEFAULT_SPEED,
        help="forward speed in m/s (linear.x). Default: %(default)s",
    )
    p.add_argument(
        "--rate", type=float, default=DEFAULT_RATE,
        help="publish rate in Hz. Default: %(default)s",
    )
    p.add_argument(
        "--topic", type=str, default=DEFAULT_TOPIC,
        help="cmd_vel topic to publish to. Default: %(default)s",
    )
    p.add_argument(
        "--csv", type=str, default="drive_straight.csv",
        help="telemetry output CSV path. Default: %(default)s",
    )
    # rospy injects __name/__log when launched via roslaunch; ignore unknowns so
    # a stray remap arg does not crash argparse. Run standalone this is a no-op.
    args, _ = p.parse_known_args(argv)
    return args


def drive(args):
    """The existing open-loop drive: publish a constant forward Twist at a steady
    rate until shutdown, and a zero Twist on shutdown so the robot stops cleanly.
    This is the long-lived part (equivalent to send_mapless_goal.py's run()).

    ALSO records telemetry: it subscribes to /odometry/filtered (the stock EKF's
    fused output - the ONLY pose source, never a Gazebo ground-truth topic) and
    logs one CSV row per second matching the attack scripts' format, so a
    "normal vs attack" heading comparison is possible. The CSV columns are
    elapsed_time,fused_x,fused_y,fused_yaw,cmd_linear_x,cmd_angular_z.
    Telemetry is best-effort and NEVER blocks the drive: if no odom sample has
    arrived yet we log a waiting line and skip the row, exactly as
    attack_imu_faithful._log_row does when cur is None."""
    # queue_size=1: this is a latest-command stream, not a log. If publishing
    # ever falls behind, the freshest Twist is the only one worth sending.
    pub = rospy.Publisher(args.topic, Twist, queue_size=1)

    forward = Twist()
    forward.linear.x = args.speed
    # everything else stays 0 - straight line, no turn.

    stop = Twist()  # all-zero

    # --- telemetry state ----------------------------------------------------
    # The subscriber callback runs in a rospy transport thread while the publish
    # loop below runs in the main thread; the shared odom sample is guarded by a
    # lock (same pattern as attack_imu_faithful).
    lock = threading.Lock()
    fused = {"xy_yaw": None}   # latest (x, y, yaw) from /odometry/filtered

    def on_fused(msg):
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        with lock:
            fused["xy_yaw"] = (msg.pose.pose.position.x,
                               msg.pose.pose.position.y, yaw)

    rospy.Subscriber("/odometry/filtered", Odometry, on_fused, queue_size=1)

    # CSV: open once, write header, flush every row so a mid-run Ctrl-C still
    # leaves a complete, readable file. Closed on shutdown alongside the
    # zero-Twist publish (see below).
    csv_file = open(args.csv, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["elapsed_time", "fused_x", "fused_y", "fused_yaw",
                         "cmd_linear_x", "cmd_angular_z"])
    csv_file.flush()

    # Publish a zero Twist on shutdown so the robot stops cleanly, and close the
    # CSV. Registered before the loop so it fires on Ctrl-C, rosnode kill, or
    # master shutdown.
    def on_shutdown():
        try:
            pub.publish(stop)
        except Exception:
            # During shutdown the transport may already be torn down; a failed
            # final publish must not mask the shutdown itself.
            pass
        try:
            if not csv_file.closed:
                csv_file.flush()
                csv_file.close()
        except Exception as exc:  # noqa: BLE001 -- log, never hide
            rospy.logwarn("Error closing CSV: %s", exc)
    rospy.on_shutdown(on_shutdown)

    rospy.loginfo(
        "drive-straight: publishing linear.x=%.3f m/s to %s at %.1f Hz "
        "(angular.z=0), logging /odometry/filtered telemetry to %s at 1 Hz. "
        "Ctrl-C to stop.",
        args.speed, args.topic, args.rate, args.csv,
    )

    def log_row():
        with lock:
            cur = fused["xy_yaw"]
        elapsed = time.time() - start_wall
        if cur is None:
            rospy.loginfo("[t=%6.1fs] waiting for /odometry/filtered ...", elapsed)
            return
        rospy.loginfo("[t=%6.1fs] fused=(%.3f,%.3f,yaw=%.3f) cmd=(%.3f,%.3f)",
                      elapsed, cur[0], cur[1], cur[2], args.speed, 0.0)
        csv_writer.writerow(["%.3f" % elapsed, "%.4f" % cur[0], "%.4f" % cur[1],
                             "%.4f" % cur[2], "%.4f" % args.speed, "%.4f" % 0.0])
        csv_file.flush()

    start_wall = time.time()
    next_log = start_wall + 1.0  # first telemetry row after 1 s
    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        pub.publish(forward)

        now = time.time()
        if now >= next_log:
            log_row()
            next_log += 1.0

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break


def main(argv):
    args = parse_args(argv)

    if args.rate <= 0:
        print("Error: --rate must be > 0.", file=sys.stderr)
        return 2

    # init_node BEFORE the bring-up: it needs rospy for logging and for reading
    # /robot_description off the param server, both of which happen before the
    # drive loop. Keep anonymous=True (unchanged from the original node).
    rospy.init_node("drive_straight", anonymous=True)

    robot = None
    try:
        robot = bring_up_robot()
        drive(args)
        return 0
    finally:
        # Reverse order of startup. The zero-Twist stop is handled by the
        # on_shutdown hook registered in drive(); here we tear down the spawned
        # robot's control.launch process group so it is never left orphaned.
        stop_robot(robot)


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except rospy.ROSInterruptException:
        sys.exit(0)
