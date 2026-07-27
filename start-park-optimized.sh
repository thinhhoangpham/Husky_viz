#!/usr/bin/env bash
set -euo pipefail

# start-park-optimized.sh - launch the OPTIMIZED natural_environments "park"
# world (the world park_1.bag was recorded in) with a Husky spawned into it.
#
# This is the optimized sibling of start-park.sh. "Optimized" means it sources
# /workspace/park-env-opt.sh instead of /workspace/park-env.sh, which points
# ROS_PACKAGE_PATH at /workspace/natural_environments_ros_opt and
# GAZEBO_MODEL_PATH at /workspace/models_opt. That tree's park.world replaces
# the 2.1M-triangle terreno_parque COLLADA terrain with a native Gazebo
# <heightmap>, so the world loads and renders far more cheaply. The unmodified
# trees are left alone - run start-park.sh for those.
#
# The other deliberate difference: this script ALWAYS pre-cleans the leftover
# ROS/Gazebo processes inside the container first, so every run begins from a
# clean simulation (see the clean block below for why it is done that way and
# NOT with `docker compose restart`).
#
# Everything else here is copied deliberately from start-park.sh because each
# guard encodes a bug that was actually hit; see the comments on each. The
# two-stage launch (world first, robot second) and the long world-load wait are
# unchanged.

# Docker build context / compose files live on the internal SSD, not the project folder.
DOCKER_DIR="$HOME/husky-docker"

NOVNC_URL="http://localhost:6080/vnc.html"

# The Qt teleop window positions itself bottom-left of the 1280x720 Xvfb screen
# on its own, so there is no geometry to pass here.
#
# A leftover teleop window from a previous run would start a second
# husky_teleop_gui node; rospy.init_node(anonymous=False) makes the master kill
# the older one, leaving two identical windows in noVNC of which one is silently
# dead. So this run tracks its own process and kills exactly that PID on the way
# out - never a broad match, which would also hit a teleop.sh --wasd session in
# the same container that the user started deliberately.
#
# Pidfile basenames are distinct from start-park.sh's on purpose: both scripts
# share one container, so identical names would let one run's cleanup kill the
# other run's processes.
TELEOP_PIDFILE="/tmp/husky_park_opt_teleop_gui.$$.pid"

# Both stages are exec'd with -T (mandatory: a backgrounded exec without -T fails
# with "the input device is not a TTY"), and -T means no TTY, which means Ctrl-C
# on the host reaches only the local docker client. roslaunch/gzserver/gzclient
# survive inside the container, orphaned and invisible, and the next run then
# starts a SECOND roslaunch into the same container - two masters' worth of
# nodes, with robot_state_publisher killed as a duplicate name and the
# controller spawner dying, so the robot ignores cmd_vel. Hence these pidfiles:
# the trap has to terminate the in-container processes by hand.
#
# Two separate pidfiles, not one, because this launch is two independent
# roslaunch processes: the world (create_park.launch) and the robot
# (add_husky_park_1.launch). They must be killed separately - killing only the
# world leaves an orphaned robot roslaunch spamming a dead master.
WORLD_PIDFILE="/tmp/husky_park_opt_world_roslaunch.$$.pid"
ROBOT_PIDFILE="/tmp/husky_park_opt_robot_roslaunch.$$.pid"
CLEANED_UP=0

cleanup() {
  # Runs on INT, TERM and normal EXIT, so it must tolerate being called twice
  # and must not abort the script when there is nothing left to kill.
  [ "$CLEANED_UP" -eq 1 ] && return 0
  CLEANED_UP=1

  # The pidfile only exists once the teleop window actually started, so the
  # readiness timeout paths (which exit before that) fall through harmlessly
  # here. A plain SIGTERM is enough: Qt tears the window down and the node's
  # atexit/close path publishes a final zero-velocity Twist.
  docker compose exec -T husky bash -lc \
    "pid=\$(cat '$TELEOP_PIDFILE' 2>/dev/null) || exit 0; \
     [ -n \"\$pid\" ] && kill \"\$pid\" 2>/dev/null; \
     rm -f '$TELEOP_PIDFILE'; exit 0" >/dev/null 2>&1 || true

  # Robot stage before world stage: the robot's controllers want a live
  # gzserver to unload against, and tearing the world out from under them first
  # produces a hang in controller_manager's shutdown.
  #
  # roslaunch needs SIGINT to shut its nodes down cleanly; only after that do we
  # escalate. gzserver/gzclient regularly outlive their parent, so they get a
  # name-based sweep as a backstop.
  #
  # DANGER: NEVER use `pkill -f <pattern>` here. `pkill -f "roslaunch
  # natural_environments"` or `pkill -f gzserver` matches THIS shell, whose own
  # argv contains that string - it kills itself mid-command and the rest of the
  # cleanup silently never runs. That exact bug was hit again while developing
  # this script. Everything below is either PID-based from a pidfile or `-x`
  # (exact process-NAME match, which cannot match `bash`). No exceptions.
  for pf in "$ROBOT_PIDFILE" "$WORLD_PIDFILE"; do
    docker compose exec -T husky bash -lc \
      "pid=\$(cat '$pf' 2>/dev/null) || pid=''; \
       if [ -n \"\$pid\" ]; then \
         kill -INT \"\$pid\" 2>/dev/null || true; \
         for _ in 1 2 3 4 5 6 7 8 9 10; do \
           kill -0 \"\$pid\" 2>/dev/null || break; \
           sleep 1; \
         done; \
         kill -TERM \"\$pid\" 2>/dev/null || true; \
         sleep 2; \
         kill -KILL \"\$pid\" 2>/dev/null || true; \
       fi; \
       rm -f '$pf'; exit 0" >/dev/null 2>&1 || true
  done

  docker compose exec -T husky bash -lc \
    "pkill -x gzclient 2>/dev/null || true; \
     pkill -x gzserver 2>/dev/null || true; \
     sleep 2; \
     pkill -KILL -x gzclient 2>/dev/null || true; \
     pkill -KILL -x gzserver 2>/dev/null || true; \
     exit 0" >/dev/null 2>&1 || true
}

trap cleanup INT TERM EXIT

cd "$DOCKER_DIR"

# Fail early with a clear message if the Docker daemon isn't reachable.
if ! docker info >/dev/null 2>&1; then
  echo "Error: cannot talk to the Docker daemon. Is Docker Desktop running?" >&2
  exit 1
fi

# ===========================================================================
# STARTUP / CLEAN SECTION
#
# NEVER add `docker compose restart` or `docker compose down` to this script.
# BOTH trigger the Docker daemon's stale bind-mount bug:
#
#   error while creating mount source path
#   '/host_mnt/Volumes/Extreme Pro/Husky viz': mkdir /host_mnt/Volumes/Extreme
#   Pro: file exists
#
# Once that happens the container cannot be started at all, and ONLY a full
# quit + relaunch of Docker Desktop clears it (`docker compose rm -f` does
# not - the stale entry lives in the daemon's mount table). CLAUDE.md
# documented this trap for `down`; it was verified this session to apply to
# `restart` as well.
#
# Second, independent reason the old unconditional `docker compose restart
# husky` had to go: /tmp lives in the container's writable layer, so
# /tmp/.X1-lock and /tmp/.X11-unix/X1 SURVIVE a restart. The entrypoint's
# `Xvfb :1` then refuses to start ("server already active for display 1"), and
# because the entrypoint backgrounds Xvfb with `&`, its `set -e` never catches
# the failure - fluxbox and x11vnc silently die too, while websockify (which
# needs no X) keeps serving port 6080. The result is a noVNC page that loads
# but reports "Failed to connect to server".
#
# So instead: create/start the container, clean the ROS/Gazebo processes in
# place, and repair the display stack only if it is actually down.
# ===========================================================================

echo "Starting containers in $DOCKER_DIR ..."
# `up -d` creates the container if it has never existed and starts it if it is
# stopped; it is a no-op if it is already running. NO restart, ever.
docker compose up -d

# ---------------------------------------------------------------------------
# B. In-container pre-clean of leftover ROS/Gazebo processes.
# ---------------------------------------------------------------------------
# Why: start-park.sh and this script share ONE container and ONE ROS master. A
# leftover roslaunch/gzserver from a previous run - of either script - produces
# duplicate nodes: robot_state_publisher is killed as a duplicate name and the
# controller spawner dies, leaving husky_velocity_controller in state
# `initialized` rather than `running`, at which point the robot silently
# ignores every cmd_vel message.
#
# roscore/rosmaster/rosout are deliberately LEFT ALIVE - they are started by
# the container entrypoint and are fine to reuse.
#
# The `self=$$` guard is mandatory. `docker compose exec -T husky bash -lc
# "<string>"` puts the whole pattern text into this shell's own argv, so a
# naive `pkill -f <pattern>` matches and kills the cleaning shell itself,
# silently skipping the rest of the clean. Exact-name matches use `pkill -x`
# (which cannot match `bash`); pattern matches go through pgrep + the $$
# exclusion.
echo "Cleaning up any leftover ROS/Gazebo processes in the container ..."
docker compose exec -T husky bash -lc '
self=$$
for n in roslaunch gzserver gzclient rviz robot_state_publisher spawn_model nodelet; do
  pkill -x "$n" 2>/dev/null || true
done
for pat in husky_teleop_gui controller_spawner ekf_localization twist_mux twist_marker joy_node teleop_twist; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do
    [ "$p" != "$self" ] && kill "$p" 2>/dev/null || true
  done
done
sleep 3
pkill -9 -x gzserver 2>/dev/null || true
pkill -9 -x gzclient 2>/dev/null || true
exit 0
' >/dev/null 2>&1 || true

# ---------------------------------------------------------------------------
# C. Display-stack health check and repair.
# ---------------------------------------------------------------------------
# If Xvfb is running, leave everything alone. If it is not, clear the stale
# X lock (see the /tmp note above) and bring the stack back up.
#
# The flags below are LOAD-BEARING - do not "tidy" them:
#   * `xset r on && xset r rate 250 30` - Xvfb's default auto-repeat delay is
#     660 ms, which exceeds husky_teleop.py's 0.6 s KEY_TIMEOUT_S, so held-key
#     teleop breaks without it.
#   * `x11vnc -repeat` - x11vnc defaults to -norepeat, which would undo that
#     xset setting the moment a VNC client connects.
# They match ~/husky-docker/entrypoint.sh exactly; keep the two in sync.
echo "Checking the container's display stack ..."
if ! docker compose exec -T husky bash -lc '
if pgrep -x Xvfb >/dev/null 2>&1; then echo "display stack already up"; exit 0; fi
rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
export DISPLAY=:1 LIBGL_ALWAYS_SOFTWARE=1 LP_NUM_THREADS=4
nohup Xvfb :1 -screen 0 1280x720x24 >/tmp/xvfb.log 2>&1 &
sleep 2
ok=0
for _ in $(seq 1 20); do xset q >/dev/null 2>&1 && { ok=1; break; }; sleep 0.5; done
[ "$ok" = 1 ] || { echo "Xvfb failed to start; see /tmp/xvfb.log" >&2; cat /tmp/xvfb.log >&2; exit 1; }
xset r on && xset r rate 250 30
pgrep -x fluxbox >/dev/null 2>&1 || nohup fluxbox >/tmp/fluxbox.log 2>&1 &
sleep 1
pgrep -x x11vnc >/dev/null 2>&1 || nohup x11vnc -display :1 -forever -shared -nopw -repeat -rfbport 5900 -quiet >/tmp/x11vnc.log 2>&1 &
sleep 2
echo "display stack restarted"
exit 0
'; then
  echo ""
  echo "Error: the container's X display stack could not be brought up." >&2
  echo "Launching the simulation without an X server would produce a sim you" >&2
  echo "cannot see and a noVNC page that fails to connect, so this run stops here." >&2
  echo "Inspect with: cd \"$DOCKER_DIR\" && docker compose exec husky bash -lc \\" >&2
  echo "  'cat /tmp/xvfb.log /tmp/x11vnc.log'" >&2
  exit 1
fi

# ---------------------------------------------------------------------------
# D. Make sure websockify is alive.
# ---------------------------------------------------------------------------
# websockify is what actually serves port 6080 and bridges it to x11vnc on
# 5900, and it is what the readiness poll below probes. It needs no X server,
# so it can easily still be running while the rest of the display stack is
# dead - hence it is checked separately from step C.
#
# Same `self=$$` discipline as step B: the pattern `websockify` appears in this
# shell's own argv, so a bare `pgrep -f websockify` would always match.
docker compose exec -T husky bash -lc '
self=$$
found=0
for p in $(pgrep -f websockify 2>/dev/null); do
  [ "$p" != "$self" ] && found=1
done
if [ "$found" = 0 ]; then
  nohup websockify --web=/usr/share/novnc 6080 localhost:5900 >/tmp/websockify.log 2>&1 &
  sleep 2
  echo "websockify restarted"
else
  echo "websockify already up"
fi
exit 0
' || true

# ---------------------------------------------------------------------------
# E. Wait for noVNC to answer on port 6080 (~60s max). Deliberately positioned
# AFTER the clean and the display repair, so it waits on the stack this run
# just made healthy rather than passing against a half-dead one.
# ---------------------------------------------------------------------------
echo -n "Waiting for noVNC on http://localhost:6080 "
for _ in $(seq 1 60); do
  if curl -fsS -o /dev/null http://localhost:6080; then
    echo " ready."
    break
  fi
  echo -n "."
  sleep 1
done

if ! curl -fsS -o /dev/null http://localhost:6080; then
  echo ""
  echo "Error: noVNC did not respond on http://localhost:6080 within 60s." >&2
  echo "Check container logs with: cd \"$DOCKER_DIR\" && docker compose logs" >&2
  exit 1
fi

echo ""
echo "noVNC:  $NOVNC_URL"
echo ""
echo "Stage 1/2: loading the OPTIMIZED park world (Ctrl-C to stop)..."
echo "Watch progress in noVNC."

# ---------------------------------------------------------------------------
# Stage 1: the world only, no robot.
# ---------------------------------------------------------------------------
#
# The container shell sources /workspace/park-env-opt.sh and NOTHING ELSE. Do
# not add `source /opt/ros/noetic/setup.bash` in front of it: park-env-opt.sh
# sources setup.bash itself and *then* prepends
# /workspace/natural_environments_ros_opt to ROS_PACKAGE_PATH. Sourcing
# setup.bash afterwards would reset ROS_PACKAGE_PATH and wipe the overlay, and
# roslaunch would fail with "package 'natural_environments' not found".
# park-env-opt.sh also exports DISPLAY=:1, GAZEBO_MODEL_PATH and the HUSKY_*
# sensor vars, so it is sufficient on its own.
#
# The _opt tree ships its own create_park.launch and add_husky_park_1.launch,
# so with park-env-opt.sh sourced these two names resolve to the optimized
# versions - the launch pair is intentionally identical to start-park.sh's.
#
# As with start-park.sh, the container shell records its own $$ and then
# `exec`s roslaunch over itself, so the recorded PID *is* roslaunch - there is
# no other way to learn an in-container PID from the host.
#
# WORLD_EXEC_PID below is the HOST-side docker client, which is NOT the
# roslaunch PID: killing it only tears down the local end of the -T exec and
# leaves roslaunch running in the container. Confusing these two is the exact
# bug the pidfiles exist to fix.
#
# EXPECTED, HARMLESS ERROR ON LOAD: Gazebo prints a mesh-loading error for a
# model named `Untitled2`, which references a dead absolute mesh path baked
# into park.world by whoever authored it. Verified: the _opt park.world still
# references Untitled2, so this error still appears here. The world loads fine
# and the model is scenery. See park_world_notes.md. Do not mistake it for a
# failure.
docker compose exec -T husky bash -lc "source /workspace/park-env-opt.sh && echo \$\$ > '$WORLD_PIDFILE' && exec roslaunch natural_environments create_park.launch" &
WORLD_EXEC_PID=$!

# Wait for the world to be genuinely loaded before spawning the robot. A fixed
# sleep is not usable here: load time varies hugely with host load, and
# spawning the Husky into a half-built world puts it under the terrain or makes
# the spawn service call time out.
#
# The real readiness signal is gzserver's own service layer:
# /gazebo/get_world_properties only answers once gzserver is up, and its
# model_names list only contains `parque` (the park world's top-level model)
# once the world SDF has finished parsing and instantiating. So poll for both
# at once - the call succeeding AND `parque` appearing in its output. The _opt
# world keeps the model name `parque`, so this probe is unchanged.
#
# Timeout stays 300s (5 minutes). natural_environments_ros/readme.txt:49-50
# documents world load times of 2-10 minutes for the original COLLADA terrain;
# the heightmap world here is expected to load much faster, but the ceiling is
# deliberately left at 300 so it is only ever a failure point, never something
# that trips on a merely slow-but-healthy load.
echo -n "Waiting for the park world to finish loading (up to 5 min) "
WORLD_READY=0
for _ in $(seq 1 300); do
  if docker compose exec -T husky bash -lc \
       'source /workspace/park-env-opt.sh >/dev/null 2>&1 && rosservice call /gazebo/get_world_properties' 2>/dev/null \
     | grep -q 'parque'; then
    WORLD_READY=1
    echo " ready."
    break
  fi
  echo -n "."
  sleep 1
done

if [ "$WORLD_READY" -ne 1 ]; then
  echo ""
  echo "Error: the park world did not report model 'parque' via" >&2
  echo "/gazebo/get_world_properties within 300s, so the robot was NOT spawned." >&2
  echo "Inspect with: cd \"$DOCKER_DIR\" && docker compose exec husky bash -lc \\" >&2
  echo "  'source /workspace/park-env-opt.sh && rosservice call /gazebo/get_world_properties'" >&2
  echo "If gzserver vanished rather than being slow, suspect an OOM kill (see below)." >&2
  kill "$WORLD_EXEC_PID" 2>/dev/null || true
  wait "$WORLD_EXEC_PID" 2>/dev/null || true
  exit 1
fi

echo ""
echo "Stage 2/2: spawning the Husky and its controllers..."

# ---------------------------------------------------------------------------
# Stage 2: the robot. Same park-env-opt.sh-only rule as stage 1, same $$/exec
# pidfile idiom, same host-vs-container PID caveat.
# ---------------------------------------------------------------------------
#
# MEMORY NOTE: the container has been observed to be OOM-killed at this stage,
# because add_husky_park_1.launch brings up the robot WITH its sensors (Ouster
# lidar + stereo) and the Docker VM only has 7.65 GiB. If gzserver or the whole
# container dies here for no visible reason, check:
#   docker inspect <container> --format '{{.State.OOMKilled}}'
# This script deliberately does not try to work around that - raise the Docker
# Desktop memory limit instead. (The heightmap terrain frees a good deal of
# memory versus the original world, but it does not remove this risk.)
# Sensor arch ENABLED: Husky spawns with the Ouster OS1-64 lidar (stereo stays
# off; it is commented out in sensor_description.urdf). park-env-opt.sh already
# exports HUSKY_SENSOR_ARCH=true and HUSKY_URDF_EXTRAS pointing at
# sensor_description.urdf, so we simply do NOT override them off here.
docker compose exec -T husky bash -lc "source /workspace/park-env-opt.sh && echo \$\$ > '$ROBOT_PIDFILE' && exec roslaunch natural_environments add_husky_park_1.launch" &
ROBOT_EXEC_PID=$!

# The controller spawner can die mid-way and leave husky_velocity_controller
# merely "initialized", in which case the robot silently ignores cmd_vel.
# Only open teleop once both controllers actually report ( running ).
echo -n "Waiting for the Husky controllers "
CONTROLLERS_READY=0
for _ in $(seq 1 120); do
  if docker compose exec -T husky bash -lc 'source /workspace/park-env-opt.sh >/dev/null 2>&1 && rosrun controller_manager controller_manager list' 2>/dev/null \
     | grep -q 'husky_velocity_controller.*( running )'; then
    if docker compose exec -T husky bash -lc 'source /workspace/park-env-opt.sh >/dev/null 2>&1 && rosrun controller_manager controller_manager list' 2>/dev/null \
       | grep -q 'husky_joint_publisher.*( running )'; then
      CONTROLLERS_READY=1
      echo " ready."
      break
    fi
  fi
  echo -n "."
  sleep 1
done

if [ "$CONTROLLERS_READY" -ne 1 ]; then
  echo ""
  echo "Error: husky_joint_publisher and husky_velocity_controller did not both reach" >&2
  echo "( running ) within 120s, so teleop was NOT opened - the robot would ignore it." >&2
  echo "Inspect with: cd \"$DOCKER_DIR\" && docker compose exec husky bash -lc \\" >&2
  echo "  'source /workspace/park-env-opt.sh && rosrun controller_manager controller_manager list'" >&2
  echo "A common cause is a second simulation already running. The other likely cause" >&2
  echo "here is an OOM kill of gzserver: docker inspect <container> --format '{{.State.OOMKilled}}'" >&2
  kill "$ROBOT_EXEC_PID" 2>/dev/null || true
  wait "$ROBOT_EXEC_PID" 2>/dev/null || true
  kill "$WORLD_EXEC_PID" 2>/dev/null || true
  wait "$WORLD_EXEC_PID" 2>/dev/null || true
  exit 1
fi

# -d so the exec returns immediately. No xterm and no TTY: the Qt window reads
# real X KeyPress/KeyRelease events, which is what lets it stop the robot the
# instant a key is released. husky_teleop.py remains the terminal fallback,
# reachable via `teleop.sh --wasd`.
#
# `docker compose exec -d` gives us no way to learn the in-container PID, so the
# shell records its own $$ and then `exec`s python over itself - after the exec,
# that recorded PID *is* the teleop process.
#
# Remap the teleop GUI's hardcoded topic (husky_teleop_gui.py:74,
# CMD_VEL_TOPIC='/kb_teleop/cmd_vel') onto /cmd_vel. VERIFIED: the twist_mux
# in add_husky_park_1.launch has only three input slots --
#   joy                (joy_teleop/cmd_vel,          priority 10)
#   interactive_marker (twist_marker_server/cmd_vel, priority 8)
#   external           (cmd_vel,                     priority 1)
# There is NO kb_teleop slot here, so the GUI's default topic has zero
# subscribers and the robot never moves. /cmd_vel is the "external" slot.
# rospy applies name:=value remaps from argv, so no edit to the GUI is needed
# -- which keeps start-sim.sh / start-park.sh (playpen) working unchanged.
docker compose exec -d husky bash -lc "source /workspace/park-env-opt.sh && echo \$\$ > '$TELEOP_PIDFILE' && exec python3 '/workspace/husky_teleop_gui.py' /kb_teleop/cmd_vel:=/cmd_vel"

echo ""
echo "noVNC:  $NOVNC_URL"
echo "Teleop is open in the 'Husky Teleop' window there (bottom-left) - click it to"
echo "focus, then drive with WASD. The robot stops as soon as you release the key."
echo ""

# Wait on the robot stage; the world stage is torn down by the trap on exit.
wait "$ROBOT_EXEC_PID"
