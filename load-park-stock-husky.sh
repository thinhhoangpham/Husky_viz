#!/usr/bin/env bash
set -euo pipefail

# load-park-stock-husky.sh - load the natural_environments "park" Gazebo world
# on a NATIVE Linux + ROS Noetic install. WORLD ONLY: this script no longer
# spawns any robot. The companion ./send_mapless_goal.py now brings up stock
# husky_control/control.launch, spawns the STOCK /opt/ros/noetic Husky into this
# already-running world, and then plans to a goal.
#
# The stock-vs-overlay explanation below still applies in full: this script owns
# ROS_PACKAGE_PATH, so the ordering it sets is what send_mapless_goal.py's
# roslaunch inherits when it is run from a shell that sourced the same ROS setup.
#
# WHY THIS EXISTS (and how it differs from load-park-world.sh)
# -----------------------------------------------------------
# load-park-world.sh spawns the PARK-flavoured Husky: its stage 2 runs the
# overlay's add_husky_park_1.launch, which pulls in the overlay
# husky_control/control.launch - i.e. the park SENSOR SUITE (GPS /navsat/fix,
# /compass/data, a map-frame EKF that fuses those). That topology is NOT what
# we want here.
#
# We want the STOCK Husky instead:
#   * stock robot_description (husky_description/urdf/husky.urdf.xacro from
#     /opt/ros/noetic, NO Ouster mast, NO sensor arch), and
#   * stock husky_control/control.launch, whose EKF is configured by
#     /opt/ros/noetic/share/husky_control/config/localization.yaml:
#         world_frame: odom      (dead-reckoning, NOT a map frame)
#         odom0: husky_velocity_controller/odom
#         imu0:  imu/data         (fused; NO GPS, NO compass)
#     (verified in localization.yaml lines: odom_frame/base_link_frame/
#      world_frame all = odom; only odom0 + imu0 as inputs.)
#
# This is the classic vulnerable dead-reckoning topology (wheel odom + IMU,
# no absolute position reference) that a later IMU-spoofing experiment will
# target. That experiment is CONTEXT ONLY - nothing here implements any attack.
#
# THE CRUX: STOCK-vs-OVERLAY PACKAGE RESOLUTION
# --------------------------------------------
# The WORLD needs the overlay: `natural_environments` (spelt
# natural_enviroment on disk) exists ONLY in
# natural_environments_ros_opt/, so create_park.launch cannot resolve without
# that tree on ROS_PACKAGE_PATH.
#
# The ROBOT must come from STOCK /opt/ros/noetic. But the overlay ALSO ships
# husky_control and husky_description under
# natural_environments_ros_opt/husky/. load-park-world.sh PREPENDS the overlay,
# which makes rospack resolve husky_* to the OVERLAY copies (the park sensor
# suite). Verified:
#   ROS_PACKAGE_PATH=<overlay>:<default>
#     rospack find husky_control -> .../natural_environments_ros_opt/husky/husky_control   (WRONG for us)
#
# rospack returns the FIRST match along ROS_PACKAGE_PATH. So we APPEND the
# overlay instead of prepending. Then the default /opt/ros/noetic entries win
# for husky_*, while `natural_environments` (absent from the default path)
# still resolves from the appended overlay. Verified under this exact env:
#   ROS_PACKAGE_PATH=<default>:<overlay>
#     rospack find husky_control        -> /opt/ros/noetic/share/husky_control          (STOCK - correct)
#     rospack find husky_description    -> /opt/ros/noetic/share/husky_description       (STOCK - correct)
#     rospack find natural_environments -> .../natural_environments_ros_opt/natural_enviroment (overlay - correct)
#   roslaunch --files husky_control control.launch resolves its included
#     description.launch to /opt/ros/noetic/share/husky_description too.
# That ordering is the whole trick and is applied below (search for "APPEND").
#
# WHAT DRIVES THE ROBOT (cmd_vel topology)
# ----------------------------------------
# Started by send_mapless_goal.py, not here, but the topology is the reason the
# stock packages matter: stock control.launch starts twist_mux with cmd_vel_out
# remapped to husky_velocity_controller/cmd_vel. Its input slots (stock
# twist_mux.yaml):
#     joy                 joy_teleop/cmd_vel           priority 10
#     kb                  kb_teleop/cmd_vel            priority 9
#     interactive_marker  twist_marker_server/cmd_vel  priority 8
#     external            cmd_vel                      priority 1
# move_base, launched by send_mapless_goal.py, publishes to the `external` slot,
# plain /cmd_vel. That is the topic that ultimately actuates the wheels here. No
# teleop/joystick is started anywhere - stock control.launch does not need one
# for cmd_vel to work, and we want a clean single publisher.
#
# GROUND-TRUTH RULE: this script only POLLS /gazebo/spawn_urdf_model and
# /gazebo/get_world_properties as world-readiness signals. It never reads
# /gazebo/get_model_state, /gazebo/model_states, or any simulator internal pose;
# neither does send_mapless_goal.py, whose spawn pose is a fixed literal.
#
# NATIVE, NOT DOCKER: like load-park-world.sh, this is a bare-metal Linux +
# ROS Noetic script. No DISPLAY=:1, no noVNC, no docker guards - the user's
# real X display is respected as inherited.
#
# USAGE
#   ./load-park-stock-husky.sh            # park world, GUI            (DEFAULT)
#   ./load-park-stock-husky.sh --headless # park world, no gzclient window
#   ./load-park-stock-husky.sh --help     # this help (also -h)
# Then, in a SECOND terminal: source /opt/ros/noetic/setup.bash && ./send_mapless_goal.py
#
# FLAG NOTE: -h is HELP, not headless (same deliberate choice as
# load-park-world.sh - a -h that started a long headless load would be a
# nasty surprise).

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_SETUP="/opt/ros/noetic/setup.bash"
PKG_TREE="$SCRIPT_DIR/natural_environments_ros_opt"
MODEL_DIR="$SCRIPT_DIR/models_opt"
WORLD_FILE="$PKG_TREE/natural_enviroment/worlds/park.world"

MODE="ros"    # ros | headless

# See load-park-world.sh for the reasoning behind this ceiling.
WORLD_TIMEOUT_S=300

usage() {
  cat <<EOF
load-park-stock-husky.sh - load the natural_environments "park" world. WORLD
ONLY: no robot is spawned here. The companion ./send_mapless_goal.py brings up
the STOCK /opt/ros/noetic Husky (dead-reckoning odom EKF, no GPS/compass),
spawns it into this world, and plans to a goal.

Usage: $(basename "${BASH_SOURCE[0]}") [--headless] [--help]

  (no flags)   roslaunch natural_environments create_park.launch, then hold the
               world open until Ctrl-C. GUI on.
  --headless   No gzclient window (SSH / services-only).
  -h, --help   Show this message.

Once the world reports ready, in a SECOND terminal run:

  source /opt/ros/noetic/setup.bash
  ./send_mapless_goal.py
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --headless) MODE="headless" ;;
    -h|--help)  usage; exit 0 ;;
    *)
      echo "Error: unknown option '$1'." >&2
      echo "" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

# ---------------------------------------------------------------------------
# Preflight. Same spirit as load-park-world.sh: fail early and clearly on the
# things that otherwise produce a confusing downstream error.
# ---------------------------------------------------------------------------
fail() { echo "Error: $*" >&2; exit 1; }

[ -f "$ROS_SETUP" ] \
  || fail "$ROS_SETUP not found. This script expects a native ROS Noetic install."

command -v gzserver >/dev/null 2>&1 \
  || fail "gzserver is not on PATH. Is Gazebo installed (apt install gazebo11)?"

[ -d "$MODEL_DIR" ] \
  || fail "models directory not found: $MODEL_DIR
Without it Gazebo cannot resolve model:// URIs and the world loads empty."

[ -d "$PKG_TREE" ] \
  || fail "package tree not found: $PKG_TREE
This is the overlay that provides the 'natural_environments' package."

[ -f "$WORLD_FILE" ] \
  || fail "world file not found: $WORLD_FILE"

# ---------------------------------------------------------------------------
# Environment.
# ---------------------------------------------------------------------------
# GAZEBO_MODEL_PATH: park meshes/heightmaps live under models_opt. ${VAR:-}
# guard is mandatory under `set -u` on a fresh shell.
export GAZEBO_MODEL_PATH="$MODEL_DIR${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

# setup.bash references unset vars and RESETS ROS_PACKAGE_PATH, so relax `set -u`
# around it and do the overlay merge AFTER it.
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u

# APPEND the overlay (not prepend - see the crux note in the header). This is
# the single line that makes `natural_environments` resolvable for the WORLD
# while leaving husky_control/husky_description resolving to STOCK for the ROBOT.
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+$ROS_PACKAGE_PATH:}$PKG_TREE"

# GAZEBO_PLUGIN_PATH: gzserver (started by THIS script's roslaunch below) loads
# every model plugin, including the robot's when send_mapless_goal.py spawns it
# into this world. The Ouster ray plugin libgazebo_ros_ouster_laser.so lives ONLY
# in the overlay workspace's devel/lib (nowhere on the default gazebo plugin
# path), so without this export gzserver cannot load it and /os0_cloud_node/points
# never appears. This is a pure Gazebo env var - it does NOT touch
# ROS_PACKAGE_PATH, so it leaves the stock-vs-overlay package resolution (and the
# guard below) untouched. We deliberately do NOT source the overlay's
# setup.bash here: that PREPENDS the overlay to ROS_PACKAGE_PATH and would shadow
# stock husky_control/husky_description, which is exactly what this script exists
# to prevent (and would trip the assertion guard below).
export GAZEBO_PLUGIN_PATH="$HOME/husky_overlay_ws/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"

# NOT set here, on purpose:
#   DISPLAY=:1   - that was the container's Xvfb; respect the real DISPLAY.
#   HUSKY_URDF_EXTRAS / HUSKY_SENSOR_ARCH - we want the STOCK URDF with no
#                  sensor arch, so these are deliberately left UNSET. A stray
#                  HUSKY_URDF_EXTRAS would graft the Ouster mast back on.

# ---------------------------------------------------------------------------
# Deterministic resolution guard. The append trick above is load-bearing and
# easy to break with a future edit, so ASSERT it rather than trust it. If
# husky_control ever resolves to the overlay we would silently get the park
# sensor suite (GPS/compass/map-EKF) - the exact topology this script exists to
# avoid - so refuse to continue.
# ---------------------------------------------------------------------------
HC_PATH="$(rospack find husky_control 2>/dev/null || true)"
HD_PATH="$(rospack find husky_description 2>/dev/null || true)"
case "$HC_PATH" in
  /opt/ros/noetic/*) : ;;
  "") fail "rospack could not find husky_control at all under this environment." ;;
  *)  fail "husky_control resolved to '$HC_PATH', not the stock /opt/ros/noetic copy.
The overlay is shadowing the stock package. ROS_PACKAGE_PATH must APPEND the
natural_environments_ros_opt tree, never prepend it (see this script's header)." ;;
esac
case "$HD_PATH" in
  /opt/ros/noetic/*) : ;;
  *)  fail "husky_description resolved to '$HD_PATH', not stock /opt/ros/noetic.
Same cause and fix as above - the overlay must be appended, not prepended." ;;
esac

# ---------------------------------------------------------------------------
# Display sanity (GUI mode only) - same helpful early warning as
# load-park-world.sh.
# ---------------------------------------------------------------------------
if [ "$MODE" != "headless" ] && [ -z "${DISPLAY:-}" ]; then
  echo "Warning: DISPLAY is not set, so the Gazebo GUI cannot open." >&2
  echo "         On SSH, re-run with --headless, or use 'ssh -X' / a desktop session." >&2
  echo "" >&2
fi

# ---------------------------------------------------------------------------
# Shutdown discipline - same strategy as load-park-world.sh, now with a single
# process group (the world), started with setsid and torn down with an
# INT -> TERM -> KILL escalation. The robot lives in send_mapless_goal.py's own
# process groups and is torn down by that script. No pkill anywhere (it matches
# this shell and unrelated Gazebos). See load-park-world.sh's cleanup() comments
# for the full reasoning.
# ---------------------------------------------------------------------------
WORLD_PGID=""
CLEANED_UP=0

kill_pgid() {
  local pgid="$1"
  [ -n "$pgid" ] || return 0
  kill -INT -- "-$pgid" 2>/dev/null || true
  local _
  for _ in $(seq 1 15); do
    kill -0 -- "-$pgid" 2>/dev/null || return 0
    sleep 1
  done
  kill -TERM -- "-$pgid" 2>/dev/null || true
  sleep 3
  kill -KILL -- "-$pgid" 2>/dev/null || true
  return 0
}

cleanup() {
  [ "$CLEANED_UP" -eq 1 ] && return 0
  CLEANED_UP=1
  kill_pgid "$WORLD_PGID"
  return 0
}
trap cleanup INT TERM EXIT

pgid_of() {
  local pid="$1" pgid
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  [ -n "$pgid" ] || pgid="$pid"
  printf '%s' "$pgid"
}

# ---------------------------------------------------------------------------
# Heads-up before the long silence (same known-harmless notes as
# load-park-world.sh - see that script for the Untitled2 / slow-first-load
# explanations).
# ---------------------------------------------------------------------------
cat <<'EOF'
--------------------------------------------------------------------------
Loading the park world. WORLD ONLY - no robot is spawned by this script.
Once the world is genuinely up, run ./send_mapless_goal.py in a second
terminal; that script brings up and spawns the STOCK Husky itself.

EXPECTED, HARMLESS: a mesh-not-found error for a model named `Untitled2`
(a dead absolute path baked into park.world; unused junk far below the map).
FIRST LOAD IS SLOW: the terrain assets are hundreds of MB. A long silent
startup is normal, not a hang.

Ctrl-C to stop.
--------------------------------------------------------------------------
EOF

echo "ROS_PACKAGE_PATH  -> ${ROS_PACKAGE_PATH%%:*}  (+overlay appended)"
echo "GAZEBO_MODEL_PATH -> ${GAZEBO_MODEL_PATH%%:*}"
echo "husky_control     -> $HC_PATH"
echo "husky_description -> $HD_PATH"
echo "DISPLAY           -> ${DISPLAY:-<unset>}"
echo "Robot             -> none here; spawned by ./send_mapless_goal.py"
echo ""

# ---------------------------------------------------------------------------
# Stage 1: the world. setsid -> own process group so cleanup() can signal the
# whole tree (gzserver/gzclient included). Headless flips gui/headless through
# roslaunch so use_sim_time and /gazebo/* services stay identical to GUI mode.
# ---------------------------------------------------------------------------
case "$MODE" in
  ros)
    echo "Stage 1: roslaunch natural_environments create_park.launch"
    setsid roslaunch natural_environments create_park.launch &
    ;;
  headless)
    echo "Stage 1: roslaunch natural_environments create_park.launch gui:=false headless:=true"
    setsid roslaunch natural_environments create_park.launch gui:=false headless:=true &
    ;;
esac
WORLD_PID=$!
WORLD_PGID="$(pgid_of "$WORLD_PID")"

# ---------------------------------------------------------------------------
# Readiness poll (unchanged from load-park-world.sh): a fixed sleep is not
# usable, and we must not tell the user to start the robot before the world can
# actually accept it. Both signals required, cheapest first:
#   1. /gazebo/spawn_urdf_model advertised (the service send_mapless_goal.py
#      will call), and
#   2. /gazebo/get_world_properties lists model `parque` (the SDF is actually
#      instantiated, not merely that services are up).
# ---------------------------------------------------------------------------
echo ""
echo -n "Waiting for the park world to finish loading (up to ${WORLD_TIMEOUT_S}s) "
WORLD_READY=0
for _ in $(seq 1 "$WORLD_TIMEOUT_S"); do
  if ! kill -0 "$WORLD_PID" 2>/dev/null; then
    echo ""
    echo "Error: the world roslaunch exited before the world came up." >&2
    echo "Scroll up for its output - a missing model or a bad ROS_PACKAGE_PATH" >&2
    echo "is the usual cause." >&2
    exit 1
  fi
  if rosservice list 2>/dev/null | grep -q '^/gazebo/spawn_urdf_model$' \
     && rosservice call /gazebo/get_world_properties 2>/dev/null | grep -q 'parque'; then
    WORLD_READY=1
    echo " ready."
    break
  fi
  echo -n "."
  sleep 1
done

if [ "$WORLD_READY" -ne 1 ]; then
  echo ""
  echo "Error: the park world was not ready within ${WORLD_TIMEOUT_S}s." >&2
  echo "Expected /gazebo/spawn_urdf_model advertised AND" >&2
  echo "/gazebo/get_world_properties to list model 'parque'." >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# World is up. The robot is NOT this script's job any more: send_mapless_goal.py
# owns the whole bring-up (control.launch -> robot_description -> spawn_model ->
# controller check) as well as the planner and the goal, and tears its own
# processes down again. Keeping the two in separate terminals also means a
# failed/finished run can be retried without reloading the (slow) world.
# ---------------------------------------------------------------------------
cat <<'EOF'

--------------------------------------------------------------------------
Park world is up and accepting spawns.

Now open a SECOND terminal, source ROS, and run the companion script - it
brings up stock husky_control/control.launch, spawns the STOCK Husky
(odom-frame EKF, /imu/data fused, NO GPS/compass) into this world, starts the
mapless planner and sends the goal:

  source /opt/ros/noetic/setup.bash
  ./send_mapless_goal.py

Leave THIS terminal running - Ctrl-C here tears the world down.
--------------------------------------------------------------------------

EOF

# Hold the world open until Ctrl-C; the trap tears down its process group.
wait "$WORLD_PID" || true
