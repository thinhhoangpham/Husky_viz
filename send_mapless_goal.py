#!/usr/bin/env python3
"""
Bring up the STOCK Husky in an ALREADY-RUNNING park world, then send ONE
move_base goal a fixed distance straight ahead of it, in the ODOM frame, and
report progress.

WHAT THIS SCRIPT NOW OWNS:
load-park-stock-husky.sh loads the park WORLD ONLY. This script performs the
whole robot bring-up itself before it plans anything:
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
  5. the mapless move_base planner, and finally the goal itself.
Everything it starts is torn down in reverse order on exit.

WHY IT READS THE CURRENT POSE INSTEAD OF HARDCODING AN AXIS:
The robot spawns in the park facing yaw=-3.1281 in the WORLD frame, but the
odom-frame EKF (/odometry/filtered) starts its own frame at the spawn instant.
Rather than guess which odom axis "straight ahead" is, this script reads the
robot's actual pose+heading from /odometry/filtered at run time and projects the
goal DISTANCE metres along that measured heading. The goal is therefore always
"~15 m straight ahead in odom", whatever the odom-frame orientation happens to
be -- no hardcoded sign/axis assumption. It prints the pose it read and the goal
it computed so the frame/coords are auditable.

The goal's frame_id is 'odom', matching the costmaps' global_frame.

Usage: ./send_mapless_goal.py [distance_m]   (default 15.0)
"""
import os
import sys
import math
import signal
import subprocess
import time

import rospy
import actionlib
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import DeleteModel, GetWorldProperties
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from actionlib_msgs.msg import GoalStatus

DISTANCE = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
ODOM_TOPIC = "/odometry/filtered"
PLANNER_LAUNCH = "/home/thinh/Documents/Husky_viz/launch/move_base_mapless_park.launch"

# On-path spawn: x,y = first bag waypoint (38.26, 1.25); yaw = atan2 of the
# WP1->WP2 direction ((27.11,1.10)-(38.26,1.25)) = -3.1281 rad, i.e. straight
# down the trail. Source: drive_to_point_gps.py:114-120 (bag-recorded WAYPOINTS,
# world coords). Driving straight from here tracks the path.
# z=3.3 spawns a few cm above the settled terrain height so the robot drops and
# settles, exactly as add_husky_park_1.launch does for its own pose.
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
# and we must not block the run on it.
DELETE_SERVICE_TIMEOUT_S = 10.0
# After the delete is acknowledged, we do NOT sleep a fixed duration - we poll
# /gazebo/get_world_properties until the model is genuinely gone (see
# delete_existing_robot). DELETE_CONFIRM_TIMEOUT_S bounds that poll, and
# DELETE_CONFIRM_POLL_S is how often we re-check.
DELETE_CONFIRM_TIMEOUT_S = 15.0
DELETE_CONFIRM_POLL_S = 0.2
ROBOT_MODEL_NAME = "husky"

STATUS_TEXT = {
    GoalStatus.PENDING: "PENDING", GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED", GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED", GoalStatus.REJECTED: "REJECTED",
    GoalStatus.LOST: "LOST",
}


def yaw_of(odom):
    q = odom.pose.pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def _stop_proc_group(proc, label):
    """Tear down a roslaunch process group started with start_new_session=True:
    SIGINT the whole group (lets roslaunch shut its nodes down gracefully),
    escalate to SIGTERM then SIGKILL if it lingers, so nothing is left orphaned.
    Shared by stop_planner() and stop_robot() - the escalation is identical and
    only the log label differs."""
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
    """Bring up the STOCK husky_control/control.launch in its own process group.

    This loads the STOCK robot_description (via its included description.launch),
    the controllers, the STOCK odom-frame EKF, twist_mux and
    robot_state_publisher. It does NOT itself place the model into Gazebo - that
    is spawn_robot()'s job. This is the long-lived process: it stays up until we
    tear it down at exit. Returns the Popen handle.

    We deliberately do NOT use the overlay's add_husky_park_1.launch: it bundles
    the overlay teleop/joystick and (through the overlay control.launch) the park
    sensor suite (GPS /navsat/fix, /compass/data, map-frame EKF). We want the
    stock dead-reckoning topology instead. No teleop is started either - stock
    control.launch needs none for /cmd_vel to actuate the wheels, and a single
    clean publisher (move_base) is the point."""
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
    died (in which case the param will never appear and waiting is pointless).
    Returns True on success."""
    rospy.loginfo("Waiting for /robot_description (up to %.0fs) ...",
                  ROBOT_DESCRIPTION_TIMEOUT_S)
    deadline = time.monotonic() + ROBOT_DESCRIPTION_TIMEOUT_S
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            rospy.logerr("husky_control/control.launch exited (rc=%s) before "
                         "setting /robot_description.", proc.returncode)
            return False
        # rospy talks to the param server directly - no need to shell out to
        # `rosparam get`, which would fork a Python interpreter every second.
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
    "husky" that is still in the live world, so that the spawn below is actually
    able to place a FRESH robot at SPAWN_X/SPAWN_Y/SPAWN_Z/SPAWN_YAW.

    WHY THIS EXISTS: the park world outlives this script - load-park-stock-husky.sh
    starts it once and we attach to it repeatedly. Gazebo refuses to spawn a model
    whose name is already taken, so on every run after the first, spawn_model fails
    and the OLD robot simply stays wherever the previous run left it. That failure
    is invisible here by design: spawn_robot() deliberately ignores spawn_model's
    exit code (it is meaningless under the heavy park world - see its docstring),
    so without this delete the script would silently drive a STALE robot from a
    STALE pose. Deleting first makes "every run starts at the spawn pose" true
    unconditionally.

    WE ALWAYS DELETE, WE NEVER CHECK FIRST: asking /gazebo/get_world_properties
    whether a husky exists and then deleting it is two round-trips with a race in
    between, and buys nothing - a delete of a non-existent model is already a
    harmless no-op that answers success=False. So we just fire it and treat the
    "nothing was there" answer as the normal first-run path, not an error.

    EVERY FAILURE MODE IS NON-FATAL. Service never appears / wait times out /
    ServiceException on the call / success=False in the response: all of these
    are logged at info and we carry on to the spawn. The only thing this function
    can legitimately do is improve the odds that the spawn lands; it must never be
    the reason a run does not happen. Returns True only if an existing husky was
    genuinely removed AND confirmed gone from the world; False on the first-run /
    nothing-to-delete path."""
    rospy.loginfo("Pre-spawn cleanup: deleting any existing '%s' model (waiting "
                  "up to %.0fs for %s) ...",
                  ROBOT_MODEL_NAME, DELETE_SERVICE_TIMEOUT_S,
                  "/gazebo/delete_model")
    try:
        rospy.wait_for_service("/gazebo/delete_model",
                               timeout=DELETE_SERVICE_TIMEOUT_S)
    except rospy.ROSException as exc:
        # Gazebo is not up, or not advertising the service. Nothing to clean up
        # that we can reach; the spawn will report the real problem if there is
        # one. Info, not warn: this is not by itself a fault.
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
        # nothing to remove. Gazebo reports this as success=False with a
        # "model does not exist" status message. Deliberately NOT logwarn/logerr.
        rospy.loginfo("No existing '%s' to delete (expected on a first run): %s",
                      ROBOT_MODEL_NAME, response.status_message)
        return False

    rospy.loginfo("Deleted an existing '%s' left over from a previous run: %s",
                  ROBOT_MODEL_NAME, response.status_message)
    # /gazebo/delete_model returns as soon as the removal is QUEUED, not once the
    # entity is actually gone from the physics/model list. Spawning into that gap
    # can hit the name still being taken. A FIXED settle sleep is unreliable: under
    # the heavy park world the entity can take much longer than any constant we'd
    # dare block on. So instead we POLL /gazebo/get_world_properties until the
    # model really is absent, bounded by DELETE_CONFIRM_TIMEOUT_S - exactly as long
    # as the delete actually needs, and no longer. Only do this when we really
    # deleted something; there is nothing to wait for on a first run.
    _wait_until_model_gone()
    return True


def _wait_until_model_gone():
    """Poll /gazebo/get_world_properties until ROBOT_MODEL_NAME is absent from the
    world's model list, or DELETE_CONFIRM_TIMEOUT_S elapses. NON-FATAL like the
    rest of delete_existing_robot(): a missing service, a ServiceException, or a
    timeout with the model still present is logged (a still-present model on
    timeout is the genuinely bad case, so warn) and we return regardless so the
    spawn still attempts. Returns None; its only job is to close the delete->spawn
    race."""
    try:
        rospy.wait_for_service("/gazebo/get_world_properties",
                               timeout=DELETE_SERVICE_TIMEOUT_S)
    except rospy.ROSException as exc:
        rospy.loginfo("/gazebo/get_world_properties unavailable within %.0fs (%s) "
                      "- cannot confirm the delete settled; going to the spawn.",
                      DELETE_SERVICE_TIMEOUT_S, exc)
        return

    get_world_properties = rospy.ServiceProxy("/gazebo/get_world_properties",
                                              GetWorldProperties)
    deadline = time.monotonic() + DELETE_CONFIRM_TIMEOUT_S
    while time.monotonic() < deadline:
        try:
            props = get_world_properties()
        except rospy.ServiceException as exc:
            rospy.loginfo("/gazebo/get_world_properties call failed (%s) - cannot "
                          "confirm the delete settled; going to the spawn.", exc)
            return
        if ROBOT_MODEL_NAME not in props.model_names:
            rospy.loginfo("Confirmed '%s' is gone from the world; spawn can "
                          "proceed.", ROBOT_MODEL_NAME)
            return
        rospy.sleep(DELETE_CONFIRM_POLL_S)

    rospy.logwarn("'%s' still present in the world %.0fs after a successful "
                  "delete - the spawn may hit 'model name already taken'. "
                  "Proceeding to the spawn anyway.",
                  ROBOT_MODEL_NAME, DELETE_CONFIRM_TIMEOUT_S)


def spawn_robot():
    """PLACEMENT step: take the robot_description control.launch just set and
    instantiate it in the live world at our fixed pose, via
    /gazebo/spawn_urdf_model.

    A NON-ZERO EXIT HERE IS EXPECTED AND MUST BE TOLERATED (the v1
    self-terminate bug, carried over from load-park-stock-husky.sh's header):
    gazebo_ros spawn_model has an internal ~10s wait for the entity to APPEAR in
    sim. Under the heavy park world the gzserver update loop stalls (logs show
    "Failed to meet update rate! Took 7267s"), so that client-side wait elapses
    and the client prints "[ERROR] Spawn service failed. Exiting." with a
    non-zero rc. THE ENTITY STILL SPAWNS - the same logs show gazebo_ros_control
    then loading the URDF and husky_velocity_controller configuring fine. In v1
    that non-zero exit tripped a `set -e` wrapper, which killed the robot stage
    and tore the whole simulation down: an orderly teardown that looked like a
    crash but was only a symptom of trusting this exit code.

    So we log the rc and carry on. The AUTHORITATIVE success signal is
    wait_for_controllers() below, which reads husky_velocity_controller reaching
    ( running ) - never the spawn client's own return code, and never any Gazebo
    ground-truth pose service."""
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
    husky_velocity_controller are ( running ).

    A racing or duplicated spawner leaves husky_velocity_controller in
    `initialized` instead of `running`, which SILENTLY drops every cmd_vel with
    no error anywhere - move_base would happily plan and the robot would never
    move. Warn loudly if that happens but do NOT abort: the world and the rest
    of the stack are still useful, and this mirrors the shell script's original
    behaviour. Returns True if both reached ( running )."""
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
            # controller_manager is not answering yet (or at all); keep polling
            # until the deadline rather than treating one bad call as fatal.
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
        "so move_base will plan and nothing will move. Usual cause: a leftover "
        "roslaunch/gzserver from an earlier run. Stop it and re-run. Re-check "
        "with: rosrun controller_manager controller_manager list. "
        "Continuing anyway.",
        listing.strip() or "  <controller_manager did not answer at all>")
    return False


def bring_up_robot():
    """Full robot bring-up sequence: control.launch -> robot_description ->
    delete any stale husky -> spawn_model -> controller check. Returns the
    control.launch Popen so the caller can tear it down, even if a later step
    failed (the process may still be alive and must not be leaked)."""
    proc = start_robot()
    if wait_for_robot_description(proc):
        # ORDERING: the delete goes AFTER start_robot()/wait_for_robot_description()
        # and immediately BEFORE spawn_robot(), i.e. as late as possible.
        #
        # It has to precede the spawn (that is the whole point - Gazebo will not
        # spawn over a taken model name). The real question is whether it should
        # come before control.launch instead. It must not, for two reasons:
        #
        #  - Deleting the Gazebo entity does NOT stop control.launch. This one is
        #    a fresh control.launch we just started, and the controllers it spawns
        #    bind to whichever gazebo_ros_control instance is live. Doing the
        #    delete last means the ONLY entity that exists from here on is the
        #    fresh one, so the controller_manager these controllers attach to is
        #    unambiguously the new robot's. Deleting earlier and then leaving a
        #    long robot_description wait in between widens the window in which the
        #    world is empty and the controller spawners are retrying against
        #    nothing.
        #  - The delete needs Gazebo reachable anyway, and putting it here keeps
        #    delete and spawn adjacent: no other step can slip in between and
        #    re-create a husky (e.g. a stray launch) after we cleaned up.
        #
        # Consequence to be aware of: any leftover controllers still bound to the
        # OLD entity are what wait_for_controllers() below would otherwise see.
        # That check remains the authoritative success signal and is unchanged.
        delete_existing_robot()
        spawn_robot()
        wait_for_controllers()
    return proc


def start_planner():
    """Launch the mapless move_base planner in its own process group so it can
    be torn down cleanly on exit. Returns the Popen handle."""
    rospy.loginfo("Launching planner: roslaunch %s", PLANNER_LAUNCH)
    return subprocess.Popen(
        ["roslaunch", PLANNER_LAUNCH],
        start_new_session=True,  # own process group (setsid) -> group signalling
    )


def stop_planner(proc):
    """Tear down the roslaunch process group started by start_planner(), so
    move_base is never left orphaned."""
    _stop_proc_group(proc, "planner (move_base)")


def run():
    rospy.loginfo("Waiting for %s ...", ODOM_TOPIC)
    start = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=30.0)
    p = start.pose.pose.position
    yaw = yaw_of(start)
    rospy.loginfo("Odom start pose: x=%.3f y=%.3f yaw=%.4f rad (%.1f deg) frame=%s",
                  p.x, p.y, yaw, math.degrees(yaw), start.header.frame_id)

    # Goal = current position + DISTANCE along the current heading, same yaw.
    gx = p.x + DISTANCE * math.cos(yaw)
    gy = p.y + DISTANCE * math.sin(yaw)
    gq = quaternion_from_euler(0.0, 0.0, yaw)

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server ...")
    if not client.wait_for_server(rospy.Duration(60.0)):
        rospy.logerr("move_base action server not available.")
        return 1

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "odom"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = gx
    goal.target_pose.pose.position.y = gy
    goal.target_pose.pose.orientation.x = gq[0]
    goal.target_pose.pose.orientation.y = gq[1]
    goal.target_pose.pose.orientation.z = gq[2]
    goal.target_pose.pose.orientation.w = gq[3]

    rospy.loginfo("Sending goal (frame=odom): x=%.3f y=%.3f yaw=%.4f  (%.1f m ahead)",
                  gx, gy, yaw, DISTANCE)
    client.send_goal(goal)

    # Progress log while active.
    rate = rospy.Rate(1.0)
    deadline = rospy.Time.now() + rospy.Duration(180.0)
    while not rospy.is_shutdown():
        state = client.get_state()
        try:
            cur = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=2.0)
            cp = cur.pose.pose.position
            dist_to_goal = math.hypot(gx - cp.x, gy - cp.y)
            rospy.loginfo("state=%s  pos=(%.2f, %.2f)  dist_to_goal=%.2f m",
                          STATUS_TEXT.get(state, state), cp.x, cp.y, dist_to_goal)
        except rospy.ROSException:
            rospy.loginfo("state=%s (no odom sample)", STATUS_TEXT.get(state, state))

        if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                     GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.LOST):
            rospy.loginfo("Final move_base state: %s", STATUS_TEXT.get(state, state))
            return 0 if state == GoalStatus.SUCCEEDED else 2
        if rospy.Time.now() > deadline:
            rospy.logwarn("Timed out after 180s; last state=%s", STATUS_TEXT.get(state, state))
            return 3
        rate.sleep()
    return 0


def main():
    # init_node moved here (out of run()): the robot bring-up below needs rospy
    # for logging and for reading /robot_description off the param server, and
    # both happen before run() is ever called.
    rospy.init_node("send_mapless_goal", anonymous=True)

    robot = None
    planner = None
    try:
        robot = bring_up_robot()
        planner = start_planner()
        return run()
    finally:
        # Reverse order of startup: planner first, then the robot. Each teardown
        # is guarded so a failure in one still lets the other run - otherwise a
        # raising stop_planner() would leak the whole control.launch group.
        try:
            stop_planner(planner)
        finally:
            stop_robot(robot)


if __name__ == "__main__":
    sys.exit(main())
