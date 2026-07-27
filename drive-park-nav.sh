#!/usr/bin/env bash
set -euo pipefail

# drive-park-nav.sh - drive the park route with the move_base NAVIGATION STACK
# (obstacle-aware planner) against an ALREADY RUNNING park simulation, on a
# NATIVE Linux + ROS Noetic install.
#
# This is the move_base sibling of drive-park.sh. Where drive-park.sh runs the
# hand-rolled auto_drive_waypoints.py (drives dead-straight to each waypoint, no
# obstacle avoidance), this script runs:
#     launch/move_base.launch   (move_base + costmaps + planner)  +
#     gps_goal_sender.py        (feeds the 5 WORLD waypoints to move_base)
# so the robot PLANS its way to each waypoint instead of beelining. Same 5
# waypoints, same spawn pose - only the thing deciding HOW to get between them
# changes.
#
# It is strictly ADDITIVE to a running sim (like drive-park.sh): it never starts
# or stops roscore / gzserver / the world. If the sim is not up it FAILS FAST and
# tells you to run ./load-park-world.sh. It does, however, roslaunch move_base
# itself if move_base is not already running (move_base is part of THIS nav
# stack, not the sim), and tears that down on exit.
#
# LIDAR / OBSTACLE AVOIDANCE IS INERT FOR NOW - READ THIS
# -------------------------------------------------------
# The costmaps subscribe to the OS0 PointCloud2 on /os0_cloud_node/points. That
# topic does NOT publish on this machine (the Ouster plugin is missing), so the
# costmaps stay EMPTY and move_base plans on open space. This is expected and
# accepted: the robot still DRIVES the route via the global/local planner. When
# the lidar is fixed and /os0_cloud_node/points starts publishing, avoidance
# switches on automatically - no change to this script or the configs needed.
#
# THE /cmd_vel REMAP IS LOAD-BEARING, NOT COSMETIC
# ------------------------------------------------
# move_base publishes to `cmd_vel`, remapped in move_base.launch to /cmd_vel -
# the lowest-priority twist_mux input (priority 1, slot "external") in
# husky/husky_control/config/twist_mux.yaml. Publishing anywhere else reaches
# ZERO subscribers and the robot silently never moves. The remap lives in
# launch/move_base.launch; do not remove it.
#
# USAGE
#   ./drive-park-nav.sh              # launch move_base (if needed), drive, spawn markers
#   ./drive-park-nav.sh --no-markers # skip the 5 red waypoint markers
#   ./drive-park-nav.sh --help       # this help (also -h)
#
# The sim must ALREADY be running: `./load-park-world.sh` in another terminal.
# gps_goal_sender.py runs in the FOREGROUND and streams waypoint progress. Press
# Ctrl-C to stop it; if this script launched move_base, it is stopped too.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_SETUP="/opt/ros/noetic/setup.bash"
PKG_TREE="$SCRIPT_DIR/natural_environments_ros_opt"
MODEL_DIR="$SCRIPT_DIR/models_opt"
GOAL_SENDER="$SCRIPT_DIR/gps_goal_sender.py"
MOVE_BASE_LAUNCH="$SCRIPT_DIR/launch/move_base.launch"

# Node name gps_goal_sender.py registers under (rospy.init_node in the sender).
SENDER_NODE="/gps_goal_sender"
# move_base's node name, used to detect whether it is already running.
MOVE_BASE_NODE="/move_base"

# The 5 waypoints in Gazebo WORLD coords. Used ONLY to place the visual markers -
# gps_goal_sender.py hardcodes the same five pairs, so markers and goals coincide
# by construction. If you change one, change the other.
MARKER_WAYPOINTS=("38.26 1.25" "27.11 1.10" "1.16 -2.40" "-15.95 -3.33" "-30.77 -3.45")

# Park loads slowly and the controller spawner retries; drive-park.sh used 180s
# and that number has held up.
CTRL_TIMEOUT=180
# How long to wait for move_base's action server after launching it.
MOVE_BASE_WAIT=12

SPAWN_MARKERS=1

# PID of a move_base we launched ourselves (0 = we did not launch it, so we must
# not kill it on exit).
MOVE_BASE_PID=0

# Spawn pose the robot is TELEPORTED back to at the start of each run, in the
# world frame. Overridable via env vars; the DEFAULTS are this native park's
# spawn from add_husky_park_1.launch (NOT the reference restart_drive.sh's
# 47.0/1.0/3.15/3.05 - our native spawn differs). SYAW is in radians.
SX="${SX:-45.6396}"
SY="${SY:-0.0208}"
SZ="${SZ:-3.1283}"
SYAW="${SYAW:-2.6132}"

# How long to wait after the EKF reset for /odometry/filtered to publish the reset
# value before we VERIFY it (see step 3 of the respawn block). The bulk of the old
# "just sleep and hope" wait is now replaced by the explicit settle poll in step 2.
RESPAWN_SETTLE=2

# Physical-settle poll (step 2 of the respawn block): the robot is teleported to
# SZ and DROPS onto the terrain, so we watch /gazebo/get_model_state until the
# fall is over before touching the EKF.
SETTLE_POLL_INTERVAL=0.5   # seconds between get_model_state samples
SETTLE_STABLE_SAMPLES=3    # consecutive stable samples required
SETTLE_TIMEOUT=30          # give up (with a warning) after this many seconds
SETTLE_DZ_TOL=0.005        # metres of z change tolerated between samples
SETTLE_DYAW_TOL=0.5        # degrees of yaw change tolerated between samples

# Post-reset verification tolerances (step 3). A latched /set_pose takes a moment
# to propagate, so we retry before declaring the anchor poisoned.
ORIGIN_XY_TOL=1.0          # metres
ORIGIN_YAW_TOL=5.0         # degrees
ORIGIN_CHECK_ATTEMPTS=3
ORIGIN_CHECK_SLEEP=1.5

# ---- helpers ----------------------------------------------------------------
say()  { printf '\n\033[1;36m==>\033[0m %s\n' "$*"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }
err()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; }
fail() { err "$*"; exit 1; }

usage() {
  cat <<EOF
drive-park-nav.sh - drive the park route with the move_base navigation stack
against an already-running park simulation.

Usage: $(basename "${BASH_SOURCE[0]}") [--no-markers] [--help]

  --no-markers   Do not spawn the 5 translucent red waypoint markers.
  -h, --help     Show this message.

THE SIMULATION MUST ALREADY BE RUNNING. Start it first, in another terminal:
  ./load-park-world.sh

This script never starts or stops the sim - it only drives one. It WILL launch
move_base itself if move_base is not already running, and stop that on exit.

Obstacle avoidance is inert until /os0_cloud_node/points (the OS0 lidar
PointCloud2) publishes - the costmaps stay empty and move_base plans on open
space. The robot still drives the route. See the header of this file.
EOF
}

# ---- flags ------------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --no-markers) SPAWN_MARKERS=0 ;;
    -h|--help)    usage; exit 0 ;;
    *)
      err "unknown option '$1'."
      echo "" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

# ---- 1. preflight: ROS environment + overlay --------------------------------
# setup.bash RESETS ROS_PACKAGE_PATH, so the overlay prepend must come AFTER it
# or it is silently wiped. `set +u` around the source because setup.bash
# dereferences unset variables internally.
[ -f "$ROS_SETUP" ]        || fail "$ROS_SETUP not found. This script expects a native ROS Noetic install."
[ -f "$GOAL_SENDER" ]      || fail "goal sender not found: $GOAL_SENDER"
[ -f "$MOVE_BASE_LAUNCH" ] || fail "move_base launch not found: $MOVE_BASE_LAUNCH"

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
export ROS_PACKAGE_PATH="$PKG_TREE${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
export GAZEBO_MODEL_PATH="$MODEL_DIR${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

# ---- 1b. preflight: is there a master to talk to at all? --------------------
say "Checking for a running ROS master..."
if ! timeout 10 rosnode list >/dev/null 2>&1; then
  err "No ROS master is reachable at ${ROS_MASTER_URI:-<ROS_MASTER_URI unset>}."
  err ""
  err "This script drives an ALREADY-RUNNING simulation - it will not start one."
  err "Start the sim first, in another terminal:"
  err "    \"$SCRIPT_DIR/load-park-world.sh\""
  err "and re-run this script once the Husky is up."
  exit 1
fi
info "Master is reachable at ${ROS_MASTER_URI:-<unset>}."

# ---- 1c. respawn the robot to the spawn pose --------------------------------
# ORDER IS LOAD-BEARING, IN TWO SEPARATE WAYS. Read both before reordering.
#
# (a) The whole block MUST run before move_base and the goal sender.
#     gps_goal_sender.py anchors its WORLD->odom transform on the FIRST
#     /odometry/filtered pose it sees, paired with the known spawn pose, and it
#     latches that anchor ONCE (self.calibrated = True) and holds it for the whole
#     run. If the robot were somewhere else (a previous run left it mid-route)
#     that anchor would be wrong and every goal would land in the wrong place.
#
# (b) Within the block the order is TELEPORT -> SETTLE -> RESET EKF -> VERIFY.
#     This is a fix for a real, diagnosed bug; the old order was
#     TELEPORT -> RESET EKF -> sleep 2, and it was WRONG. The teleport puts the
#     robot at z=$SZ, slightly ABOVE the terrain, so it DROPS and lands. Zeroing
#     the EKF while that fall is still in progress means the EKF then integrates
#     the entire landing transient - impact, bounce, settle - into odom. Odom is
#     therefore NOT at the origin (and yaw not zero) by the time the goal sender
#     anchors. Because the anchor is latched once and never revisited, a single
#     poisoned reading rotates ALL FIVE goals for the rest of the run: correct
#     topics, correct transform math, robot drives confidently in the wrong
#     direction, off the mapped terrain, and falls.
#
#     So: teleport, then POLL /gazebo/get_model_state until the physical pose has
#     actually stopped moving, and only THEN zero the EKF. Afterwards we VERIFY
#     /odometry/filtered really reads near the origin and FAIL HARD if it does
#     not - a bad anchor guarantees a bad route, so proceeding is never useful.
#
# Same teleport-then-reset-EKF services as the reference restart_drive.sh
# (/gazebo/set_model_state + /set_pose), with our native spawn pose, plus the
# settle and verify steps it lacks.
say "Respawning the Husky to the spawn pose (world x=$SX y=$SY z=$SZ yaw=$SYAW rad)..."

# 1. Teleport the model. yaw -> quaternion: qz=sin(yaw/2), qw=cos(yaw/2)
#    (a pure rotation about z; qx=qy=0). awk does the trig so bash stays integerless.
QZ="$(awk -v y="$SYAW" 'BEGIN{ printf "%.9f", sin(y/2.0) }')"
QW="$(awk -v y="$SYAW" 'BEGIN{ printf "%.9f", cos(y/2.0) }')"
if timeout 20 rosservice call /gazebo/set_model_state "model_state:
  model_name: 'husky'
  pose:
    position: {x: $SX, y: $SY, z: $SZ}
    orientation: {x: 0.0, y: 0.0, z: $QZ, w: $QW}
  twist:
    linear:  {x: 0.0, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}
  reference_frame: 'world'" >/dev/null 2>&1; then
  info "Teleported husky to the spawn pose."
else
  warn "set_model_state call failed - is the 'husky' model spawned? Continuing;"
  warn "the goal sender's anchor may be off if the robot is not at spawn."
fi

# 2. WAIT FOR THE DROP TO FINISH before touching the EKF. The teleport leaves the
#    robot a few cm above the terrain; it falls and lands. Poll Gazebo's own
#    ground-truth pose and require SETTLE_STABLE_SAMPLES consecutive samples whose
#    z and yaw are unchanged within tolerance. Only then is it safe to zero the
#    EKF - see the "(b)" note above for what happens if we do it mid-fall.
#    python3 parses the get_model_state YAML and derives yaw from the quaternion.
say "Waiting for the Husky to settle on the terrain (up to ${SETTLE_TIMEOUT}s)..."
settle_deadline=$(( $(date +%s) + SETTLE_TIMEOUT ))
prev_z=""
prev_yaw=""
stable_count=0
SETTLED=0
while [ "$(date +%s)" -lt "$settle_deadline" ]; do
  # `|| true` throughout: a failed/timed-out service call must not kill the run
  # under `set -e`; we simply treat that sample as unusable and poll again.
  MS="$(timeout 10 rosservice call /gazebo/get_model_state \
        "model_name: 'husky'
relative_entity_name: 'world'" 2>/dev/null || true)"
  SAMPLE="$(printf '%s' "$MS" | python3 -c '
import math, re, sys
text = sys.stdin.read()

# Pull the position/orientation blocks explicitly: get_model_state also prints a
# twist block with its own x/y/z, so a flat scalar grep would match the wrong one.
o = re.search(r"orientation:\s*\n\s*x:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\n\s*y:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\n\s*z:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\n\s*w:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)", text)
p = re.search(r"position:\s*\n\s*x:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\n\s*y:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\n\s*z:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)", text)
if not o or not p:
    sys.exit(1)
qx, qy, qz, qw = (float(v) for v in o.groups())
z = float(p.group(3))
yaw = math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                              1.0 - 2.0 * (qy * qy + qz * qz)))
print("%.6f %.6f" % (z, yaw))
' 2>/dev/null || true)"

  if [ -z "$SAMPLE" ]; then
    printf '.'
    sleep "$SETTLE_POLL_INTERVAL"
    continue
  fi

  cur_z="$(printf '%s' "$SAMPLE" | awk '{print $1}')"
  cur_yaw="$(printf '%s' "$SAMPLE" | awk '{print $2}')"

  if [ -n "$prev_z" ]; then
    # Yaw difference wrapped into [-180, 180] so a +179 -> -179 crossing does not
    # read as a 358 degree jump.
    steady="$(awk -v z1="$prev_z" -v z2="$cur_z" -v y1="$prev_yaw" -v y2="$cur_yaw" \
                  -v dzt="$SETTLE_DZ_TOL" -v dyt="$SETTLE_DYAW_TOL" 'BEGIN{
        dz = z2 - z1; if (dz < 0) dz = -dz;
        dy = y2 - y1; while (dy > 180) dy -= 360; while (dy < -180) dy += 360;
        if (dy < 0) dy = -dy;
        print (dz < dzt && dy < dyt) ? "yes" : "no";
      }')"
    if [ "$steady" = "yes" ]; then
      stable_count=$((stable_count + 1))
    else
      stable_count=0
    fi
  fi
  prev_z="$cur_z"
  prev_yaw="$cur_yaw"

  if [ "$stable_count" -ge "$SETTLE_STABLE_SAMPLES" ]; then
    SETTLED=1
    break
  fi
  printf '.'
  sleep "$SETTLE_POLL_INTERVAL"
done
printf '\n'

if [ "$SETTLED" -eq 1 ]; then
  info "Husky settled: z=$prev_z  yaw=${prev_yaw} deg (stable over ${SETTLE_STABLE_SAMPLES} samples)."
else
  warn "The Husky pose did not stabilise within ${SETTLE_TIMEOUT}s"
  warn "(last sample: z=${prev_z:-<none>} yaw=${prev_yaw:-<none>} deg)."
  warn "Resetting the EKF anyway, but if the robot is still moving the anchor may"
  warn "absorb that motion. The verification step below will catch it if so."
fi

# 3. Reset the EKF estimate to the odom origin so /odometry/filtered matches the
#    fresh spawn. One latched PoseWithCovarianceStamped on /set_pose, frame 'odom',
#    position (0,0,0), orientation w=1 - exactly as restart_drive.sh does. rostopic
#    pub -1 sends one message and exits (it stays latched for late subscribers).
say "Resetting the EKF (/set_pose) to the odom origin..."
if timeout 20 rostopic pub -1 /set_pose geometry_msgs/PoseWithCovarianceStamped "header:
  frame_id: 'odom'
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  covariance: [0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0]" >/dev/null 2>&1; then
  info "EKF reset to origin."
else
  warn "/set_pose publish failed - the robot_localization node may not expose it."
  warn "Continuing; /odometry/filtered may not read origin at anchor time."
fi

# 4. VERIFY the reset actually landed. This is FATAL on failure, and deliberately
#    so: gps_goal_sender.py latches its WORLD->odom anchor from the first
#    /odometry/filtered pose and never re-derives it, so an odom that is not at
#    the origin rotates every one of the five goals. There is no partial-credit
#    outcome here - the robot would drive the whole route in the wrong direction.
#    The /set_pose message is latched and takes a moment to propagate through the
#    EKF, so we retry a few times before giving up.
info "Waiting ${RESPAWN_SETTLE}s for /odometry/filtered to publish the reset value..."
sleep "$RESPAWN_SETTLE"

say "Verifying /odometry/filtered reads the odom origin..."
ODOM_OK=0
odom_x=""; odom_y=""; odom_yaw=""
attempt=1
while [ "$attempt" -le "$ORIGIN_CHECK_ATTEMPTS" ]; do
  ODOM="$(timeout 15 rostopic echo -n1 /odometry/filtered/pose/pose 2>/dev/null || true)"
  ODOM_PARSED="$(printf '%s' "$ODOM" | python3 -c '
import math, re, sys
text = sys.stdin.read()
p = re.search(r"position:\s*\n\s*x:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\n\s*y:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)", text)
o = re.search(r"orientation:\s*\n\s*x:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\n\s*y:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\n\s*z:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*\n\s*w:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)", text)
if not p or not o:
    sys.exit(1)
qx, qy, qz, qw = (float(v) for v in o.groups())
yaw = math.degrees(math.atan2(2.0 * (qw * qz + qx * qy),
                              1.0 - 2.0 * (qy * qy + qz * qz)))
print("%.6f %.6f %.6f" % (float(p.group(1)), float(p.group(2)), yaw))
' 2>/dev/null || true)"

  if [ -n "$ODOM_PARSED" ]; then
    odom_x="$(printf '%s' "$ODOM_PARSED" | awk '{print $1}')"
    odom_y="$(printf '%s' "$ODOM_PARSED" | awk '{print $2}')"
    odom_yaw="$(printf '%s' "$ODOM_PARSED" | awk '{print $3}')"
    near="$(awk -v x="$odom_x" -v y="$odom_y" -v yw="$odom_yaw" \
                -v xyt="$ORIGIN_XY_TOL" -v yt="$ORIGIN_YAW_TOL" 'BEGIN{
        ax = (x < 0 ? -x : x); ay = (y < 0 ? -y : y);
        ayw = yw; while (ayw > 180) ayw -= 360; while (ayw < -180) ayw += 360;
        if (ayw < 0) ayw = -ayw;
        print (ax < xyt && ay < xyt && ayw < yt) ? "yes" : "no";
      }')"
    if [ "$near" = "yes" ]; then
      ODOM_OK=1
      break
    fi
    warn "attempt $attempt/$ORIGIN_CHECK_ATTEMPTS: odom reads x=$odom_x y=$odom_y yaw=${odom_yaw} deg - not at the origin yet."
  else
    warn "attempt $attempt/$ORIGIN_CHECK_ATTEMPTS: could not read /odometry/filtered within 15s."
  fi
  attempt=$((attempt + 1))
  if [ "$attempt" -le "$ORIGIN_CHECK_ATTEMPTS" ]; then
    sleep "$ORIGIN_CHECK_SLEEP"
  fi
done

if [ "$ODOM_OK" -ne 1 ]; then
  err "/odometry/filtered is NOT at the odom origin after the EKF reset."
  err "Last reading: x=${odom_x:-<unread>}  y=${odom_y:-<unread>}  yaw=${odom_yaw:-<unread>} deg"
  err "(tolerances: |x|,|y| < ${ORIGIN_XY_TOL} m and |yaw| < ${ORIGIN_YAW_TOL} deg)"
  err ""
  err "REFUSING TO DRIVE. gps_goal_sender.py latches its WORLD->odom anchor from"
  err "the first /odometry/filtered pose it sees and never recomputes it, so this"
  err "offset would rotate ALL FIVE waypoints. The robot would plan and drive"
  err "confidently in the wrong direction, off the mapped terrain."
  err ""
  err "Things to check:"
  err "  - Is /ekf_localization running and subscribed to /set_pose?"
  err "      rosnode info /ekf_localization"
  err "  - Did the robot actually settle, or is it still moving/falling?"
  err "      rosservice call /gazebo/get_model_state \"model_name: 'husky'\""
  err "  - Is something else still publishing motion (a stale goal sender,"
  err "    a leftover teleop, an old move_base)?"
  err "  - Restarting the sim with load-park-world.sh clears all of the above."
  exit 1
fi
info "Odometry verified at the origin: x=$odom_x  y=$odom_y  yaw=${odom_yaw} deg."

# 5. Non-fatal sanity log: report the WORLD->odom rotation the goal sender is
#    about to derive, theta = odom_yaw - compass_yaw, so a wrong anchor is
#    visible in the terminal BEFORE the robot moves. For the DEFAULT spawn
#    (SYAW=2.6132 rad = 149.7 deg) this should print roughly -149.8 deg. NOT
#    fatal: SX/SY/SZ/SYAW are overridable via env, so another theta is perfectly
#    legitimate - this is a human-readable cross-check, not a gate.
COMPASS="$(timeout 15 rostopic echo -n1 /compass/data/orientation 2>/dev/null || true)"
COMPASS_YAW="$(printf '%s' "$COMPASS" | python3 -c '
import math, re, sys
text = sys.stdin.read()
vals = {}
for key in ("x", "y", "z", "w"):
    m = re.search(r"^%s:\s*(-?[0-9]+\.?[0-9]*(?:[eE][-+]?[0-9]+)?)\s*$" % key, text, re.M)
    if not m:
        sys.exit(1)
    vals[key] = float(m.group(1))
yaw = math.degrees(math.atan2(2.0 * (vals["w"] * vals["z"] + vals["x"] * vals["y"]),
                              1.0 - 2.0 * (vals["y"] ** 2 + vals["z"] ** 2)))
print("%.4f" % yaw)
' 2>/dev/null || true)"

if [ -n "$COMPASS_YAW" ]; then
  THETA="$(awk -v oy="$odom_yaw" -v cy="$COMPASS_YAW" 'BEGIN{
      t = oy - cy; while (t > 180) t -= 360; while (t < -180) t += 360;
      printf "%.2f", t;
    }')"
  info "Compass yaw = ${COMPASS_YAW} deg; the goal sender will anchor"
  info "theta = odom_yaw - compass_yaw = ${THETA} deg (default spawn: ~ -149.8 deg)."
else
  info "Could not read /compass/data for the theta cross-check (non-fatal)."
fi

# ---- 2. wait for BOTH controllers to report ( running ) ---------------------
# Same load-bearing readiness check as drive-park.sh: a controller stuck in
# `initialized` silently drops every cmd_vel, so move_base would plan beautifully
# while the robot never moves. Fatal on failure.
say "Waiting for the Husky controllers to report ( running ) (up to ${CTRL_TIMEOUT}s)..."
CTRL_LIST=""
CONTROLLERS_READY=0
deadline=$(( $(date +%s) + CTRL_TIMEOUT ))
while [ "$(date +%s)" -lt "$deadline" ]; do
  CTRL_LIST="$(rosrun controller_manager controller_manager list 2>/dev/null || true)"
  if grep -q 'husky_joint_publisher.*( running )'    <<< "$CTRL_LIST" \
  && grep -q 'husky_velocity_controller.*( running )' <<< "$CTRL_LIST"; then
    CONTROLLERS_READY=1
    break
  fi
  printf '.'
  sleep 2
done
printf '\n'

if [ "$CONTROLLERS_READY" -ne 1 ]; then
  err "The Husky controllers did not both reach ( running ) within ${CTRL_TIMEOUT}s."
  err "Last controller_manager output was:"
  printf '%s\n' "${CTRL_LIST:-  <controller_manager did not answer at all>}" >&2
  err ""
  err "Stop any leftover roslaunch/gzserver, restart the sim with"
  err "load-park-world.sh, and try again. Nothing was torn down."
  exit 1
fi
info "Both husky_joint_publisher and husky_velocity_controller are ( running )."

# ---- 3. sanity-check odometry -----------------------------------------------
# CONSOLIDATED: this used to be a second, warn-only "is odom near the origin?"
# check. It duplicated the verification in step 1c (which now runs immediately
# after the EKF reset) and, being warn-only, let a poisoned anchor sail straight
# through into the drive. The single FATAL check in 1c replaces it. Odom cannot
# have moved between there and here - nothing in between commands the base - and
# if something ever does, that check is the place to re-run, not a softer copy.

# ---- 4. spawn the low-opacity red waypoint markers --------------------------
# Visual-only, static, collision-free translucent red poles at each waypoint.
# Server-side models, so this works against a --headless sim too. Each name is
# DELETED first (a leftover would make spawn_model fail), then spawned; a failed
# spawn is reported and skipped, never fatal.
if [ "$SPAWN_MARKERS" -eq 1 ]; then
  say "Spawning the 5 low-opacity red waypoint markers..."
  MARKER_SDF="$(mktemp -t wp_marker.XXXXXX --suffix=.sdf)"
  cat > "$MARKER_SDF" <<'SDF'
<?xml version="1.0"?>
<sdf version="1.6">
  <model name="marker">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <transparency>0.6</transparency>
        <geometry><cylinder><radius>0.5</radius><length>15</length></cylinder></geometry>
        <material>
          <ambient>1 0 0 0.4</ambient><diffuse>1 0 0 0.4</diffuse><emissive>0.6 0 0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
SDF

  i=1
  for wp in "${MARKER_WAYPOINTS[@]}"; do
    # shellcheck disable=SC2086
    set -- $wp
    name="wp_marker_$i"
    timeout 15 rosservice call /gazebo/delete_model "model_name: '$name'" >/dev/null 2>&1 || true
    if timeout 30 rosrun gazebo_ros spawn_model -sdf -file "$MARKER_SDF" \
         -model "$name" -x "$1" -y "$2" -z 4 >/dev/null 2>&1; then
      info "marker $i at ($1, $2)"
    else
      warn "marker $i at ($1, $2) FAILED to spawn - continuing without it."
    fi
    i=$((i+1))
  done

  rm -f "$MARKER_SDF"
else
  info "Skipping waypoint markers (--no-markers)."
fi

# ---- 5. ensure move_base is running -----------------------------------------
# move_base is part of THIS nav stack, not the sim, so unlike the sim we DO
# launch it if it is absent. If it is already up (e.g. from a previous run or a
# separate terminal) we leave it alone and do NOT kill it on exit.
say "Checking whether move_base is already running..."
if timeout 10 rosnode list 2>/dev/null | grep -qx "$MOVE_BASE_NODE"; then
  info "move_base is already running ($MOVE_BASE_NODE); leaving it as-is."
else
  info "move_base not running; launching $MOVE_BASE_LAUNCH ..."
  # Background roslaunch, output to the terminal. We own this PID and stop it on
  # exit via the trap below.
  roslaunch "$MOVE_BASE_LAUNCH" &
  MOVE_BASE_PID=$!
  info "move_base launched (pid $MOVE_BASE_PID); waiting ${MOVE_BASE_WAIT}s for it to come up..."
  sleep "$MOVE_BASE_WAIT"
  if ! kill -0 "$MOVE_BASE_PID" 2>/dev/null; then
    fail "move_base exited during startup. Check the roslaunch output above."
  fi
  if ! timeout 10 rosnode list 2>/dev/null | grep -qx "$MOVE_BASE_NODE"; then
    warn "move_base node not visible yet after ${MOVE_BASE_WAIT}s; the goal"
    warn "sender has its own connect-retry loop, so continuing."
  else
    info "move_base is up."
  fi
fi

# Stop ONLY a move_base we launched ourselves, on any exit (Ctrl-C included).
cleanup() {
  if [ "$MOVE_BASE_PID" -ne 0 ] && kill -0 "$MOVE_BASE_PID" 2>/dev/null; then
    say "Stopping the move_base we launched (pid $MOVE_BASE_PID)..."
    kill "$MOVE_BASE_PID" 2>/dev/null || true
    wait "$MOVE_BASE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# ---- 6. stop a stale goal sender --------------------------------------------
# By NODE NAME via the master, not a broad pkill. "node not found" is normal.
say "Stopping any stale goal sender ($SENDER_NODE)..."
if timeout 15 rosnode kill "$SENDER_NODE" >/dev/null 2>&1; then
  info "Killed a running $SENDER_NODE."
  sleep 2
else
  info "No running $SENDER_NODE (nothing to stop)."
fi

# ---- 7. start the goal sender in the foreground -----------------------------
say "Starting the move_base goal sender."
info "Waypoint progress streams below. move_base plans each leg; on this machine"
info "the costmaps are empty (no /os0_cloud_node/points), so it plans on open"
info "space - the robot still drives the route. Press Ctrl-C to stop."
printf '\n'

# NOT exec here (unlike drive-park.sh): if we launched move_base we must keep the
# shell alive so the EXIT trap can stop it. `-u` is unbuffered stdout so the
# waypoint-progress log streams live instead of in 4 KB gulps.
python3 -u "$GOAL_SENDER"
