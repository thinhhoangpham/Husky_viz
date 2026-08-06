#!/usr/bin/env bash
set -euo pipefail

# drive-park.sh - start the autonomous waypoint driver against an ALREADY
# RUNNING park simulation, on a NATIVE Linux + ROS Noetic install.
#
# This is the native port of the Docker-era restart-drive.sh. Every guard below
# is carried over from that script because each one encodes a bug that was
# actually hit. Everything Docker-shaped in the original (docker compose exec,
# /workspace paths, the Xvfb/x11vnc/websockify/noVNC display stack, the gzclient
# babysitter) is GONE on purpose: this box has no Docker and ROS/Gazebo run
# against the user's real X display. Do not "restore" any of it.
#
# SCOPE DIFFERENCE FROM restart-drive.sh - READ THIS BEFORE EXTENDING
# -------------------------------------------------------------------
# restart-drive.sh stopped and fully RELAUNCHED the sim so the world and the
# fused odometry reset to the origin. This script deliberately does NOT.
# In the container there was an entrypoint that owned roscore and the world, so
# a relaunch was cheap. Here the sim is owned by the user's own foreground
# `./load-park-world.sh` session; tearing it down from this script would just
# kill that terminal out from under them. So drive-park.sh is strictly ADDITIVE
# to a running sim:
#   1. preflight  (ROS env + overlay, master reachable, driver file present)
#   2. wait for BOTH husky controllers to read ( running )
#   3. teleport the Husky back to the spawn pose        (warn only, never fatal)
#   4. sanity-check the driver's two sensor inputs      (warn only, never fatal)
#   5. (re)spawn the 5 translucent red waypoint markers
#   6. kill a stale driver node, if one is left over
#   7. exec the driver in the foreground and stream its progress
#
# If the sim is not up, this script FAILS FAST and tells you to run
# ./load-park-world.sh. It never starts roscore, gzserver or the world itself.
#
# WHY THE DRIVER TARGETS /cmd_vel - AND WHY THERE IS NO REMAP
# -----------------------------------------------------------
# drive_to_point_gps.py publishes DIRECTLY to /cmd_vel, so the exec line at the
# bottom carries no remap. That is deliberate, and /cmd_vel is the correct
# target. This run uses the natural_environments_ros_opt overlay, whose
# twist_mux.yaml (husky/husky_control/config/twist_mux.yaml) has only three
# input slots:
#     joy                 joy_teleop/cmd_vel           priority 10
#     interactive_marker  twist_marker_server/cmd_vel  priority 8
#     external            cmd_vel                      priority 1
# `external` -> /cmd_vel is the low-priority slot an autonomous driver belongs
# in: joystick and interactive-marker input outrank it and can take over at any
# time. Note there is NO kb_teleop slot in this overlay. The PREVIOUS driver
# (auto_drive_waypoints.py) defaulted to /kb_teleop/cmd_vel, which is correct
# only for the STOCK husky_control config; here it reaches ZERO subscribers and
# the robot silently never moves, with no error anywhere. That is why this
# script used to carry a `/kb_teleop/cmd_vel:=/cmd_vel` remap. The new driver
# needs no such remap - and applying one now would remap a topic it never
# publishes. Do not "restore" it. Keep publishing to /cmd_vel.
#
# USAGE
#   ./drive-park.sh                # respawn to spawn pose, spawn markers, drive
#   ./drive-park.sh --no-markers   # skip the marker spawn
#   ./drive-park.sh --no-respawn   # skip the teleport back to the spawn pose
#   ./drive-park.sh --help         # this help (also -h)
#
# The sim must ALREADY be running: `./load-park-world.sh` in another terminal.
# The driver drives the 5 waypoints ONCE and then exits - it does not loop and
# does not respawn the robot. This script streams its output until it finishes;
# Ctrl-C stops it early.
#
# NOTE ON HOW LONG A RUN CAN TAKE: the driver's TIMEOUT is PER WAYPOINT LEG, not
# for the whole route, so the worst case before it gives up is 5 legs' worth.
# That bound has nothing to do with CTRL_TIMEOUT below, which only covers the
# controller-readiness wait. See TIMEOUT in drive_to_point_gps.py for the rest.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_SETUP="/opt/ros/noetic/setup.bash"
PKG_TREE="$SCRIPT_DIR/natural_environments_ros_opt"
MODEL_DIR="$SCRIPT_DIR/models_opt"
DRIVER="$SCRIPT_DIR/drive_to_point_gps.py"

# Node name the driver registers under (rospy.init_node in the driver). Used to
# kill a stale instance by NAME rather than by process pattern - see stage 6.
#
# CAVEAT - THIS KILL IS BEST-EFFORT AND OFTEN WILL NOT MATCH:
# drive_to_point_gps.py calls init_node("drive_to_point_gps", anonymous=True).
# Anonymous nodes register under a name with a random pid/timestamp suffix
# (e.g. /drive_to_point_gps_12345_1700000000000), so `rosnode kill
# /drive_to_point_gps` will NOT match a stale instance. Stage 6 therefore
# reports "nothing to stop" even when one is running. It is kept because it is
# harmless and still catches a non-anonymous instance; do not rely on it to
# guarantee a clean slate. Making the driver non-anonymous would fix this, but
# that is the driver's call to make, not this script's.
DRIVER_NODE="/drive_to_point_gps"

# The 5 waypoints from the bag (/navigation/objetive_gps), converted to Gazebo
# WORLD coords via a least-squares fit of /navsat/fix against
# /gazebo/model_states (lat->world X, lon->world -Y; residuals < 5 mm). These are
# ONLY used here to place the visual markers - the driver hardcodes the same five
# pairs itself, as WAYPOINTS in drive_to_point_gps.py, so markers and goals
# coincide by construction. Nothing checks that at runtime: if you change one
# list, change the other.
MARKER_WAYPOINTS=("38.26 1.25" "27.11 1.10" "1.16 -2.40" "-15.95 -3.33" "-30.77 -3.45")

# Park loads slowly and the controller spawner retries; restart-drive.sh used
# 180s and that number has held up. Generous on purpose - it should only ever
# fire on a genuine failure, never on a merely slow-but-healthy bring-up.
CTRL_TIMEOUT=180

SPAWN_MARKERS=1
RESPAWN=1

# Spawn pose the robot is TELEPORTED back to at the start of each run, in the
# world frame. Overridable via env vars; the DEFAULTS are this native park's
# spawn from add_husky_park_1.launch, and they match drive-park-nav.sh so both
# scripts start a route from the same place. SYAW is in radians.
SX="${SX:-45.6396}"
SY="${SY:-0.0208}"
SZ="${SZ:-3.1283}"
SYAW="${SYAW:-2.6132}"

# How long to wait after the teleport for the robot to drop onto the terrain and
# settle, before the driver's own startup gate begins sampling GPS. A FIXED sleep
# is deliberate: the obvious alternative - polling Gazebo until the pose stops
# moving - would READ pose back from the simulator, which CLAUDE.md forbids. Two
# seconds comfortably covers a fall of a few centimetres.
RESPAWN_SETTLE=2

# NO --speed FLAG. The old driver re-read /husky_auto_drive/linear_speed from
# the parameter server every iteration; drive_to_point_gps.py has a hardcoded
# MAX_LIN and reads no rosparams at all, so setting that parameter would do
# nothing. The flag was removed rather than left as a silent no-op. To change
# the speed, edit MAX_LIN in the driver.

# ---- helpers ----------------------------------------------------------------
say()  { printf '\n\033[1;36m==>\033[0m %s\n' "$*"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }
err()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; }
fail() { err "$*"; exit 1; }

usage() {
  cat <<EOF
drive-park.sh - run the autonomous waypoint driver against an already-running
park simulation.

Usage: $(basename "${BASH_SOURCE[0]}") [--no-markers] [--no-respawn] [--help]

  --no-markers   Do not spawn the 5 translucent red waypoint markers.
  --no-respawn   Do not teleport the Husky back to the spawn pose first; drive
                 from wherever it currently is.
  -h, --help     Show this message.

Before driving, the Husky is teleported back to the park spawn pose so every run
starts from the same place. The pose defaults to x=$SX y=$SY z=$SZ yaw=$SYAW rad
and can be overridden with the SX/SY/SZ/SYAW environment variables. A failed
teleport is only a warning - the driver reads its own position from GPS, so it
still works, it just starts from wherever the robot already was.

THE SIMULATION MUST ALREADY BE RUNNING. Start it first, in another terminal:
  ./load-park-world.sh

This script never starts or stops the sim - it only drives one. If no ROS
master is reachable it exits immediately rather than guessing.

The driver runs in the FOREGROUND, drives the 5 waypoints ONCE and then exits -
it does not loop and does not respawn the robot. Press Ctrl-C to stop it early;
either way the driver publishes a zero-velocity command on its way out, so the
robot halts.

The driving speed is fixed in the driver (MAX_LIN in drive_to_point_gps.py) and
cannot be set from this script.

POSITION SOURCE - USE_EKF environment variable (passed through to the driver):
  USE_EKF unset / true (DEFAULT): navigate on the fused map-frame EKF
      (/odometry/filtered_map), which fuses GPS + wheel odom + IMU. Spoofing
      husky_velocity_controller/odom drags this fused pose and steers the robot
      off target. Requires the sim to be launched with the EKF stack enabled
      (control.launch enable_ekf:=true, the default).
  USE_EKF=false: raw-GPS fallback - navigate on /navsat/fix directly, odom
      ignored (the original pre-EKF behaviour). e.g.:
          USE_EKF=false ./drive-park.sh
EOF
}

# ---- flags ------------------------------------------------------------------
while [ $# -gt 0 ]; do
  case "$1" in
    --no-markers) SPAWN_MARKERS=0 ;;
    --no-respawn) RESPAWN=0 ;;
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
# Same ordering as load-park-world.sh, and the ordering is load-bearing:
# setup.bash RESETS ROS_PACKAGE_PATH, so the overlay prepend must come AFTER it
# or it is silently wiped. `set +u` around the source because setup.bash
# dereferences unset variables internally.
[ -f "$ROS_SETUP" ] || fail "$ROS_SETUP not found. This script expects a native ROS Noetic install."
[ -f "$DRIVER" ]    || fail "driver not found: $DRIVER
This script does not create it; it only runs it."

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
export ROS_PACKAGE_PATH="$PKG_TREE${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
# `${VAR:-}` guards are mandatory under `set -u`: neither is exported on a fresh
# shell and a bare reference would abort the script with "unbound variable".
export GAZEBO_MODEL_PATH="$MODEL_DIR${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

# ---- 1b. preflight: is there a master to talk to at all? --------------------
# Everything below (controller list, rostopic echo, spawn_model, rosnode kill)
# blocks or times out confusingly against a dead master. One cheap probe up
# front converts that into one clear line. `rosnode list` is used because it
# round-trips to the master itself, unlike some cached-file paths.
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

# ---- 2. wait for BOTH controllers to report ( running ) ---------------------
# THIS IS THE LOAD-BEARING READINESS CHECK, and it catches a real recurring bug
# documented in CLAUDE.md: when launches race or duplicate, the controller
# spawner dies part-way and husky_velocity_controller is left in state
# `initialized` instead of `running`. The sim then looks perfectly healthy - the
# robot is visible, tf publishes - but EVERY cmd_vel message is silently dropped.
# A driver started against that state drives nothing while logging success, which
# is the worst possible failure mode. So this one IS fatal: refuse to start.
#
# The `( running )` spacing matches controller_manager's own output format. One
# call feeds both greps so the two checks observe the same instant.
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
  err "husky_velocity_controller stuck in 'initialized' means the controller"
  err "spawner died part-way - usually because a SECOND simulation is running and"
  err "duplicate node names killed robot_state_publisher. The robot would look"
  err "fine in Gazebo but silently ignore every cmd_vel message."
  err ""
  err "Stop any leftover roslaunch/gzserver, restart the sim with"
  err "load-park-world.sh, and try again. The driver was NOT started - running it"
  err "now would just spin uselessly. Nothing was torn down."
  exit 1
fi
info "Both husky_joint_publisher and husky_velocity_controller are ( running )."

# ---- 3. respawn the robot to the spawn pose ---------------------------------
# WHY THIS LIVES HERE AND NOT IN THE DRIVER: drive_to_point_gps.py must stay free
# of any Gazebo dependency so it remains runnable on real hardware. Teleporting is
# a simulator-only test convenience, so it belongs in this sim wrapper. Do not
# move it into the driver.
#
# ORDER IS LOAD-BEARING, BOTH SIDES:
#   * AFTER stage 2 - the controllers must be up before the robot is
#     repositioned, or the reposition races the controller spawner.
#   * BEFORE the driver starts - so the route always begins from the same place.
#
# WHAT THIS IS AND IS NOT, w.r.t. the CLAUDE.md rule banning Gazebo ground truth:
# /gazebo/set_model_state is used PURELY to PLACE the robot between runs, which
# that rule explicitly permits. Nothing here READS pose back from the simulator -
# no /gazebo/get_model_state, no /gazebo/model_states - not to verify the
# teleport, not to detect settling, not to log. The only permitted pose sources
# remain /navsat/fix and /compass/data, which stage 4 checks and the driver reads.
#
# WARN-ONLY, like the rest of this script: if the teleport fails the robot is
# simply wherever it already was, and the driver still works - it reads its own
# position from GPS rather than assuming a start pose.
if [ "$RESPAWN" -eq 1 ]; then
  say "Respawning the Husky to the spawn pose (world x=$SX y=$SY z=$SZ yaw=$SYAW rad)..."

  # yaw -> quaternion: a pure rotation about z, so qx=qy=0, qz=sin(yaw/2),
  # qw=cos(yaw/2). awk does the trig so bash stays integerless.
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
    warn "set_model_state call failed - is the 'husky' model spawned, and is this"
    warn "really a Gazebo sim? NOT fatal: the robot just stays where it already"
    warn "was, and the driver still works - it reads its position from GPS rather"
    warn "than assuming it starts at the spawn pose. The route will simply begin"
    warn "from the robot's current position."
  fi

  # The teleport puts the robot at z=$SZ, slightly ABOVE the terrain, so it DROPS
  # and lands. Give it a moment to settle before stage 4 and the driver's own
  # startup gate start sampling GPS, so neither reads a pose mid-fall.
  info "Waiting ${RESPAWN_SETTLE}s for the robot to drop onto the terrain and settle..."
  sleep "$RESPAWN_SETTLE"
else
  info "Skipping the respawn (--no-respawn); driving from the robot's current pose."
fi

# ---- 4. sanity-check the driver's sensor inputs (warn only) -----------------
# The position topic the driver reads DEPENDS ON THE MODE (USE_EKF, see below):
#   * fused/default (USE_EKF unset or true): /odometry/filtered_map, the
#     map-frame robot_localization EKF that fuses GPS + wheel odom + IMU. This
#     is the topic whose position an odom spoof corrupts.
#   * raw fallback (USE_EKF=false): /navsat/fix directly, odom ignored.
# Heading is /compass/data in both modes. We check whichever position topic the
# current mode uses, plus the compass. This only confirms each one PUBLISHES; it
# does not judge the values.
#
# DELIBERATELY NON-FATAL, matching the rest of this script's style: the driver
# has its own 30 s startup gate that blocks until both topics have delivered a
# usable message and fails loudly (exit 1) if either does not. Duplicating that
# as a hard failure here would only make the same problem surface twice, with
# less detail. A warning is enough - it just gets the diagnosis in front of you
# ~30 s earlier.
#
# USE_EKF mirrors the driver's own default: anything but false/0/no/off is fused.
case "$(printf '%s' "${USE_EKF:-true}" | tr '[:upper:]' '[:lower:]')" in
  false|0|no|off) POS_TOPIC="/navsat/fix" ;;
  *)              POS_TOPIC="/odometry/filtered_map" ;;
esac
say "Checking the driver's sensor inputs ($POS_TOPIC, /compass/data)..."
for topic in "$POS_TOPIC" /compass/data; do
  if timeout 15 rostopic echo -n1 "$topic" >/dev/null 2>&1; then
    info "$topic is publishing."
  else
    warn "No message on $topic within 15s."
    warn "The driver needs BOTH $POS_TOPIC and /compass/data; its own 30s"
    warn "startup gate will fail loudly if this one really is missing."
    warn "Continuing so you get that fuller diagnosis."
  fi
done

# ---- 5. spawn the low-opacity red waypoint markers --------------------------
# Visual-only, static, collision-free translucent red poles at each waypoint, so
# you can see where the robot is heading. transparency 0.6 => ~40% opaque.
# They are SERVER-side models, so this works fine against a --headless sim too -
# no need to gate on the GUI.
#
# DUPLICATE HANDLING: spawn_model fails outright if a model of that name already
# exists, and markers survive as long as gzserver does. A leftover wp_marker_N
# from a previous drive-park.sh run would therefore make every marker spawn fail.
# So each name is DELETED first via /gazebo/delete_model (errors ignored - "model
# does not exist" is the normal case on a first run), then spawned. A failed
# spawn is reported and skipped, never fatal: markers are decoration, and losing
# them is not a reason to refuse to drive.
if [ "$SPAWN_MARKERS" -eq 1 ]; then
  say "Spawning the 5 low-opacity red waypoint markers..."
  # --suffix rather than putting .sdf in the template: GNU mktemp requires the
  # X's to be the LAST characters of the template. spawn_model does not care
  # about the extension, but a readable /tmp is worth the two extra words.
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
    # Word-splitting "X Y" into $1/$2 is intentional here.
    # shellcheck disable=SC2086
    set -- $wp
    name="wp_marker_$i"
    # Clear a leftover of the same name. Failure is the normal first-run case.
    timeout 15 rosservice call /gazebo/delete_model "model_name: '$name'" >/dev/null 2>&1 || true
    if timeout 30 rosrun gazebo_ros spawn_model -sdf -file "$MARKER_SDF" \
         -model "$name" -x "$1" -y "$2" -z 4 >/dev/null 2>&1; then
      info "marker $i at ($1, $2)"
    else
      warn "marker $i at ($1, $2) FAILED to spawn - continuing without it."
    fi
    i=$((i+1))
  done

  # Clean up the temp SDF NOW, not in an EXIT trap: this script ends in `exec`,
  # which replaces the shell, so no EXIT trap would ever fire.
  rm -f "$MARKER_SDF"
else
  info "Skipping waypoint markers (--no-markers)."
fi

# ---- 6. stop a stale driver -------------------------------------------------
# By NODE NAME, via the master - not `pkill -f auto_drive_waypoints.py`. The
# container gave the old script process isolation; here a broad pkill pattern
# could in principle match an unrelated user process (an editor holding the
# filename in its argv, another shell, this script's own argv). rosnode kill is
# both narrower and cleaner: the node gets a proper shutdown, so its `finally:`
# still publishes the zero-velocity stop. "node not found" is the normal case.
say "Stopping any stale driver ($DRIVER_NODE)..."
if timeout 15 rosnode kill "$DRIVER_NODE" >/dev/null 2>&1; then
  info "Killed a running $DRIVER_NODE."
  # Let it finish unregistering; starting a second node with the same name
  # before the first is gone makes the master evict one of them.
  sleep 2
else
  info "No running $DRIVER_NODE (nothing to stop)."
fi

# NOT done here: `rosnode cleanup`. It prompts interactively and would hang this
# script, and it unregisters ghosts belonging to OTHER nodes too. If a previous
# driver died hard and its registration lingers, rospy's init_node simply evicts
# the ghost when the new node registers under the same name.

# ---- 7. start the driver in the foreground ----------------------------------
# (There is no speed-override stage: see the note next to SPAWN_MARKERS above.)
say "Starting the autonomous waypoint driver."
info "Waypoint progress streams below."
info "The driver visits the 5 waypoints ONCE and then exits - it advances to the"
info "next waypoint on arrival and does not loop or respawn the robot. Press"
info "Ctrl-C to stop it early - either way the robot is halted with a"
info "zero-velocity command on the way out."
info "The simulation itself keeps running; it is owned by load-park-world.sh."
printf '\n'

# `exec` replaces this shell with the python process, so:
#   * Ctrl-C from the terminal is delivered DIRECTLY to the driver, which lets
#     its `finally:` publish the zero Twist that actually stops the robot. No
#     signal-forwarding trap to get wrong.
#   * there is no wrapper shell left holding a PID that outlives the driver.
# `-u` is unbuffered stdout, without which the waypoint-progress log would only
# appear in 4 KB gulps and the run would look frozen.
# NO REMAP: this driver publishes to /cmd_vel itself - see the note at the top.
exec python3 -u "$DRIVER"
