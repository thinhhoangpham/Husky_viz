#!/usr/bin/env bash
#
# restart-drive.sh — one command to "reset to start and drive the route again".
#
# WHAT IT DOES (host macOS -> operates inside the running Docker container):
#   1. Makes sure the Husky container is up, WITHOUT any destructive Docker op.
#   2. Stops the autonomous waypoint driver, if it is still running.
#   3. Cleanly stops the running Gazebo sim (roslaunch + gzserver/gzclient +
#      controllers + robot_localization + twist_mux + spawners) but LEAVES
#      roscore alive (the container entrypoint owns it).
#   4. Relaunches the sim fresh so the world, the robot pose, and the fused
#      odometry all reset to the origin.
#   5. Waits (polls, does not blindly sleep) until BOTH husky controllers report
#      ( running ).
#   6. Sanity-checks that /odometry/filtered is fresh near (0,0).
#   7. Starts the autonomous driver and streams its waypoint-progress log so you
#      can watch the robot work the route. Ctrl-C stops watching gracefully.
#
# SAFETY GUARANTEES (see CLAUDE.md for the reasons):
#   * Only NON-DESTRUCTIVE Docker operations are used: `ps`, `start`, `exec`.
#   * NEVER `docker compose down`, NEVER `up`, NEVER restart Docker Desktop,
#     NEVER calls start-sim.sh / stop-sim.sh. Those either break controllers on a
#     live sim or trigger the stale-mount trap documented in CLAUDE.md.
#   * roscore is never killed.
#   * If the sim fails to come up in time, the script prints a clear error and
#     exits non-zero. It does NOT tear anything down.
#   * It does not modify auto_drive_waypoints.py or any other project file.
#
# Safe to run repeatedly (idempotent): every run stops whatever is there and
# brings up a clean sim + driver.
#
# Run it with:  "/Volumes/Extreme Pro/Husky viz/restart-drive.sh"

set -u

# ---- config -----------------------------------------------------------------
COMPOSE_DIR="$HOME/husky-docker"       # build context / compose file (internal SSD)
SERVICE="husky"                        # docker compose service name

# --- optimized PARK bring-up (mirrors start-park-optimized.sh) ---------------
# The sim is launched EXACTLY as start-park-optimized.sh does it: source the
# park-env-opt.sh overlay (it sources ROS setup.bash itself, then points
# ROS_PACKAGE_PATH/GAZEBO_MODEL_PATH at the _opt trees and exports DISPLAY=:1
# plus the HUSKY_* sensor vars), then a TWO-STAGE launch -- world first, robot
# second -- with a long wait for the world to finish loading in between. This is
# the natural_environments "park" world park_1.bag was recorded in, not the
# Clearpath playpen. Do NOT prefix `source /opt/ros/noetic/setup.bash`: it would
# reset ROS_PACKAGE_PATH and wipe the overlay (roslaunch would then fail with
# "package 'natural_environments' not found").
PARK_ENV="source /workspace/park-env-opt.sh"
WORLD_LAUNCH="roslaunch natural_environments create_park.launch"
ROBOT_LAUNCH="roslaunch natural_environments add_husky_park_1.launch"
WORLD_LOG="/tmp/park_world.log"        # in-container world-stage log
ROBOT_LOG="/tmp/park_robot.log"        # in-container robot-stage log
WORLD_TIMEOUT=300                      # seconds to wait for the park world to load
DRIVER="/workspace/auto_drive_waypoints.py"
# The 5 waypoints from the bag (/navigation/objetive_gps), converted to Gazebo
# WORLD coords via a least-squares fit of /navsat/fix against /gazebo/model_states
# (lat->world X, lon->world -Y; residuals < 5 mm). All 5 verified to lie on the
# park walkway. These are the SAME coordinates the markers are spawned at, and
# the driver steers by ground-truth world pose, so goals and markers coincide.
WORLD_WAYPOINTS='[[38.26, 1.25], [27.11, 1.10], [1.16, -2.40], [-15.95, -3.33], [-30.77, -3.45]]'
CTRL_TIMEOUT=180                       # seconds to wait for controllers (park loads slowly)
NOVNC_URL="http://localhost:6080/vnc.html"

SOURCE_ROS="source /opt/ros/noetic/setup.bash"

# ---- helpers ----------------------------------------------------------------
say()  { printf '\n\033[1;36m==>\033[0m %s\n' "$*"; }
info() { printf '    %s\n' "$*"; }
warn() { printf '\033[1;33m[warn]\033[0m %s\n' "$*"; }
err()  { printf '\033[1;31m[error]\033[0m %s\n' "$*" >&2; }

# Run a command inside the container (non-interactive).
cexec() { docker compose exec -T "$SERVICE" bash -lc "$*"; }

fail() { err "$*"; exit 1; }

# ---- 0. sanity: compose dir + docker reachable ------------------------------
cd "$COMPOSE_DIR" 2>/dev/null || fail "Cannot cd to $COMPOSE_DIR (compose files live there)."

if ! docker compose ps >/dev/null 2>&1; then
    fail "Cannot talk to Docker / this compose project. Is Docker Desktop running?"
fi

# ---- 1. ensure the container is up (non-destructively) ----------------------
say "Checking the Husky container..."
STATUS="$(docker compose ps --status running --services 2>/dev/null | grep -x "$SERVICE" || true)"
if [ -n "$STATUS" ]; then
    info "Container already up. Leaving it as-is."
else
    info "Container not running. Starting it (docker compose start — never up/down)..."
    if ! docker compose start "$SERVICE"; then
        fail "docker compose start failed. If you see a stale-mount error, a Docker Desktop restart is needed (see CLAUDE.md). This script will NOT restart Docker for you."
    fi
    # give the entrypoint a moment to bring roscore up
    for _ in $(seq 1 30); do
        if cexec "pgrep -f 'bin/roscore' >/dev/null" 2>/dev/null; then break; fi
        sleep 1
    done
fi

# Confirm roscore is alive (must never be killed by us).
if ! cexec "pgrep -f 'bin/roscore' >/dev/null" 2>/dev/null; then
    fail "roscore is not running in the container and did not come up. Aborting (not tearing down)."
fi
info "roscore is alive."

# ---- 1.5 ensure the display stack is alive (so noVNC can show the sim) ------
# Xvfb/x11vnc/fluxbox sometimes die between runs while stale websockify
# processes keep listening on 6080. The browser then connects to websockify but
# there is no x11vnc behind it -> "Failed to connect to server". So if ANY piece
# is missing, clear the stale front end and rebuild the whole stack. Done BEFORE
# the sim launches so gzclient has a display to render into.
say "Checking the display stack (Xvfb / fluxbox / x11vnc / websockify)..."
cexec '
  need=0
  for p in Xvfb fluxbox x11vnc websockify; do
    pgrep -x $p >/dev/null || need=1
  done
  if [ "$need" = "0" ]; then echo "    display stack already up"; exit 0; fi
  echo "    display stack incomplete -- rebuilding"
  pkill -x websockify 2>/dev/null; pkill -x x11vnc 2>/dev/null; sleep 1
  if ! pgrep -x Xvfb >/dev/null; then
    setsid Xvfb :1 -screen 0 1280x720x24 >/tmp/xvfb.log 2>&1 </dev/null &
  fi
  sleep 2
  export DISPLAY=:1
  xset q >/dev/null 2>&1 || echo "    WARNING: X display :1 not responding"
  xset r on 2>/dev/null; xset r rate 250 30 2>/dev/null
  pgrep -x fluxbox >/dev/null || { setsid fluxbox >/tmp/fluxbox.log 2>&1 </dev/null & }
  setsid x11vnc -display :1 -forever -shared -nopw -repeat -rfbport 5900 -quiet >/tmp/x11vnc.log 2>&1 </dev/null &
  sleep 1
  setsid websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/websockify.log 2>&1 </dev/null &
  sleep 2
  for p in Xvfb fluxbox x11vnc websockify; do echo "    $p: $(pgrep -c -x $p)"; done
'

# ---- 2. stop the driver node, if any ----------------------------------------
say "Stopping the old autonomous driver (if running)..."
cexec "pkill -f 'auto_drive_waypoints.py' 2>/dev/null; true"
info "Driver stopped (or was not running)."

# ---- 3. stop the running sim cleanly (but NOT roscore) ----------------------
say "Stopping the old simulation cleanly..."
# SIGINT the sim's roslaunch process(es) first so they shut children down
# gracefully. Match by EXACT process name (-x roslaunch) rather than a launch-
# file pattern: this cleanly stops a previously-running PARK sim too, which is
# TWO roslaunches (world create_park.launch + robot add_husky_park_1.launch),
# and -x can never match this cleanup shell itself. roscore is a python process,
# not roslaunch, so it is left untouched.
cexec "pkill -INT -x roslaunch 2>/dev/null; true"

# Wait a few seconds for graceful shutdown of gzserver/gzclient.
for _ in $(seq 1 8); do
    if ! cexec "pgrep -x gzserver >/dev/null" 2>/dev/null; then break; fi
    sleep 1
done

# Force-kill any stragglers (a lingering gzserver blocks a clean relaunch).
cexec "pkill -TERM -x roslaunch 2>/dev/null; \
       pkill -TERM -x gzserver 2>/dev/null; \
       pkill -TERM -x gzclient 2>/dev/null; true"
sleep 2
cexec "pkill -KILL -x roslaunch 2>/dev/null; \
       pkill -KILL -x gzserver 2>/dev/null; \
       pkill -KILL -x gzclient 2>/dev/null; true"

# Verify gzserver is actually gone before relaunching.
for _ in $(seq 1 10); do
    if ! cexec "pgrep -x gzserver >/dev/null" 2>/dev/null; then break; fi
    sleep 1
done
if cexec "pgrep -x gzserver >/dev/null" 2>/dev/null; then
    fail "gzserver is still alive after kill attempts; refusing to relaunch on top of it. Inspect the container manually."
fi
info "Old sim is fully stopped (roscore preserved)."

# ---- 4. relaunch the OPTIMIZED PARK sim fresh (two-stage) -------------------
say "Launching a fresh OPTIMIZED PARK simulation (world first, then the robot)..."
info "This is the natural_environments park world park_1.bag was recorded in,"
info "not the Clearpath playpen. The park loads slowly; please wait."

# Stage 1/2: the world only (no robot). Source ONLY park-env-opt.sh (see the
# config note above for why setup.bash must NOT be prefixed).
cexec "$PARK_ENV && nohup $WORLD_LAUNCH > $WORLD_LOG 2>&1 & echo '    world stage launched, pid '\$!"
info "Park world loading in the background (logging to $WORLD_LOG in the container)."

# Wait for the world to genuinely finish loading before spawning the robot.
# /gazebo/get_world_properties only answers once gzserver is up, and its model
# list only contains 'parque' once the world SDF has finished instantiating.
# Spawning into a half-built world puts the robot under the terrain or times the
# spawn service out, so this is a real readiness poll, not a blind sleep.
say "Waiting for the park world to finish loading (up to ${WORLD_TIMEOUT}s)..."
world_deadline=$(( $(date +%s) + WORLD_TIMEOUT ))
world_ready=0
while [ "$(date +%s)" -lt "$world_deadline" ]; do
    if cexec "$PARK_ENV >/dev/null 2>&1 && rosservice call /gazebo/get_world_properties 2>/dev/null" 2>/dev/null | grep -q 'parque'; then
        world_ready=1
        break
    fi
    printf '.'
    sleep 2
done
printf '\n'
if [ "$world_ready" -ne 1 ]; then
    err "The park world did not report model 'parque' within ${WORLD_TIMEOUT}s; the robot was NOT spawned."
    err "World-stage log tail ($WORLD_LOG):"
    cexec "tail -n 20 $WORLD_LOG 2>/dev/null" || true
    fail "Park world did not load. Nothing was torn down; you can inspect the container."
fi
info "Park world is loaded (model 'parque' present)."

# Stage 2/2: spawn the Husky and its controllers into the loaded world.
say "Spawning the Husky into the park..."
cexec "$PARK_ENV && nohup $ROBOT_LAUNCH > $ROBOT_LOG 2>&1 & echo '    robot stage launched, pid '\$!"
info "Robot spawning in the background (logging to $ROBOT_LOG in the container)."

# ---- 5. wait for BOTH controllers to report ( running ) ---------------------
say "Waiting for the robot's controllers to come up (up to ${CTRL_TIMEOUT}s)..."
deadline=$(( $(date +%s) + CTRL_TIMEOUT ))
ready=0
while [ "$(date +%s)" -lt "$deadline" ]; do
    LIST="$(cexec "$SOURCE_ROS && rosrun controller_manager controller_manager list 2>/dev/null" 2>/dev/null || true)"
    joint=$(printf '%s' "$LIST"  | grep -E 'husky_joint_publisher'    | grep -c 'running' || true)
    vel=$(printf '%s'   "$LIST"  | grep -E 'husky_velocity_controller' | grep -c 'running' || true)
    if [ "${joint:-0}" -ge 1 ] && [ "${vel:-0}" -ge 1 ]; then
        ready=1
        break
    fi
    printf '.'
    sleep 3
done
printf '\n'

if [ "$ready" -ne 1 ]; then
    err "Controllers did not reach ( running ) within ${CTRL_TIMEOUT}s."
    err "Last controller_manager list output was:"
    cexec "$SOURCE_ROS && rosrun controller_manager controller_manager list 2>/dev/null" || true
    err "Robot-stage log tail ($ROBOT_LOG):"
    cexec "tail -n 20 $ROBOT_LOG 2>/dev/null" || true
    fail "Simulation did not become ready. Nothing was torn down; you can inspect the container."
fi
info "Both husky_joint_publisher and husky_velocity_controller are ( running )."

# ---- 6. sanity-check odometry is fresh near the origin ----------------------
say "Checking that odometry reset to the start (near 0,0)..."
POS="$(cexec "$SOURCE_ROS && timeout 15 rostopic echo -n1 /odometry/filtered/pose/pose/position 2>/dev/null" 2>/dev/null || true)"
ox="$(printf '%s' "$POS" | awk '/^x:/{print $2; exit}')"
oy="$(printf '%s' "$POS" | awk '/^y:/{print $2; exit}')"
if [ -z "${ox:-}" ] || [ -z "${oy:-}" ]; then
    warn "Could not read /odometry/filtered yet. The driver anchors on the first odom message, so it will still work, but freshness was not confirmed."
else
    fresh="$(awk -v x="$ox" -v y="$oy" 'BEGIN{ ax=(x<0?-x:x); ay=(y<0?-y:y); print (ax<1.0 && ay<1.0)?"yes":"no" }')"
    if [ "$fresh" = "yes" ]; then
        info "Odometry is fresh: x=$ox  y=$oy  (both < 1.0 m of origin)."
    else
        warn "Odometry reads x=$ox  y=$oy — NOT near the origin."
        warn "This can mean the restart did not fully reset state. Continuing, but watch the robot's motion."
    fi
fi

# ---- 6.5 spawn low-opacity red waypoint markers -----------------------------
# Visual-only, static, collision-free translucent red poles at each of the 5
# waypoints (world-frame coords from the bag GPS fit; spawn = 45.64, 0.02, yaw 2.6132).
# They mark where the robot is driving. Re-spawned every run because a world
# reload wipes them. transparency 0.6 => ~40% opaque ("low opacity").
say "Spawning low-opacity red waypoint markers..."
cexec '
  source /opt/ros/noetic/setup.bash
  cat > /tmp/wp_marker.sdf <<"SDF"
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
  for wp in "38.26 1.25" "27.11 1.10" "1.16 -2.40" "-15.95 -3.33" "-30.77 -3.45"; do
    set -- $wp
    rosrun gazebo_ros spawn_model -sdf -file /tmp/wp_marker.sdf -model wp_marker_$i -x $1 -y $2 -z 4 >/dev/null 2>&1 \
      && echo "    marker $i at ($1, $2)" || echo "    marker $i FAILED"
    i=$((i+1))
  done
'

# ---- 6.6 ensure the Gazebo viewer (gzclient) is running --------------------
# gzserver runs the physics headless; gzclient is what actually draws the scene
# onto display :1 for noVNC. Without it you connect to noVNC and see an empty
# desktop. LP_NUM_THREADS caps llvmpipe's render threads (see CLAUDE.md) so the
# software renderer does not starve Qt's event loop.
say "Checking the Gazebo viewer (gzclient)..."
cexec '
  if pgrep -x gzclient >/dev/null; then echo "    gzclient already running"; exit 0; fi
  source /workspace/park-env-opt.sh >/dev/null 2>&1
  export DISPLAY=:1
  export LP_NUM_THREADS=4
  setsid gzclient >/tmp/gzclient.log 2>&1 </dev/null &
  sleep 4
  echo "    gzclient: $(pgrep -c -x gzclient)"
'

# ---- 7. start the driver and stream its progress ----------------------------
say "Starting the autonomous waypoint driver."
info "Watch the robot move in the noVNC view: $NOVNC_URL"
info "Waypoint progress streams below. Press Ctrl-C to stop watching (the sim keeps running)."
printf '\n'

# Foreground stream: the driver runs unbuffered (-u) inside the container and its
# stdout/stderr come straight back to this terminal. Ctrl-C here sends SIGINT to
# the driver, which stops the robot cleanly (its finally: publishes zero velocity).
cexec "$SOURCE_ROS && rosparam set /husky_auto_drive/world_waypoints '$WORLD_WAYPOINTS' && exec python3 -u $DRIVER /kb_teleop/cmd_vel:=/cmd_vel"
