#!/usr/bin/env bash
set -euo pipefail

# start-park.sh - launch the natural_environments "park" world (the world
# park_1.bag was recorded in) with a Husky spawned into it.
#
# This is the sibling of start-sim.sh, which launches the Clearpath playpen.
# The differences are only three: the environment overlay (park-env.sh), the
# TWO-STAGE launch (world first, robot second), and the long world-load wait.
# Everything else here is copied deliberately from start-sim.sh because each
# guard encodes a bug that was actually hit; see the comments on each.

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
TELEOP_PIDFILE="/tmp/husky_teleop_gui.$$.pid"

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
WORLD_PIDFILE="/tmp/husky_park_world_roslaunch.$$.pid"
ROBOT_PIDFILE="/tmp/husky_park_robot_roslaunch.$$.pid"
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

echo "Starting containers in $DOCKER_DIR ..."
docker compose up -d

# Wait for noVNC to answer on port 6080 (~60s max).
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
echo "Stage 1/2: loading the park world (Ctrl-C to stop)..."
echo "This is slow - see the timeout note below. Watch progress in noVNC."

# ---------------------------------------------------------------------------
# Stage 1: the world only, no robot.
# ---------------------------------------------------------------------------
#
# The container shell sources /workspace/park-env.sh and NOTHING ELSE. Do not
# add `source /opt/ros/noetic/setup.bash` in front of it: park-env.sh sources
# setup.bash itself and *then* prepends /workspace/natural_environments_ros to
# ROS_PACKAGE_PATH. Sourcing setup.bash afterwards would reset
# ROS_PACKAGE_PATH and wipe the overlay, and roslaunch would fail with
# "package 'natural_environments' not found". park-env.sh also exports
# DISPLAY=:1, GAZEBO_MODEL_PATH and the HUSKY_* sensor vars, so it is
# sufficient on its own.
#
# As with start-sim.sh, the container shell records its own $$ and then `exec`s
# roslaunch over itself, so the recorded PID *is* roslaunch - there is no other
# way to learn an in-container PID from the host.
#
# WORLD_EXEC_PID below is the HOST-side docker client, which is NOT the
# roslaunch PID: killing it only tears down the local end of the -T exec and
# leaves roslaunch running in the container. Confusing these two is the exact
# bug the pidfiles exist to fix.
#
# EXPECTED, HARMLESS ERROR ON LOAD: Gazebo prints a mesh-loading error for a
# model named `Untitled2`, which references a dead absolute mesh path baked
# into park.world by whoever authored it. The world still loads fine and the
# model is scenery. See park_world_notes.md. Do not mistake it for a failure.
docker compose exec -T husky bash -lc "source /workspace/park-env.sh && echo \$\$ > '$WORLD_PIDFILE' && exec roslaunch natural_environments create_park.launch" &
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
# at once - the call succeeding AND `parque` appearing in its output.
#
# Timeout 300s (5 minutes). natural_environments_ros/readme.txt:49-50 documents
# world load times of 2-10 minutes; park is the simplest of the provided worlds,
# so 5 minutes is a generous ceiling for it while still failing fast enough to
# be useful if something is actually wrong.
echo -n "Waiting for the park world to finish loading (up to 5 min) "
WORLD_READY=0
for _ in $(seq 1 300); do
  if docker compose exec -T husky bash -lc \
       'source /workspace/park-env.sh >/dev/null 2>&1 && rosservice call /gazebo/get_world_properties' 2>/dev/null \
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
  echo "  'source /workspace/park-env.sh && rosservice call /gazebo/get_world_properties'" >&2
  echo "If gzserver vanished rather than being slow, suspect an OOM kill (see below)." >&2
  kill "$WORLD_EXEC_PID" 2>/dev/null || true
  wait "$WORLD_EXEC_PID" 2>/dev/null || true
  exit 1
fi

echo ""
echo "Stage 2/2: spawning the Husky and its controllers..."

# ---------------------------------------------------------------------------
# Stage 2: the robot. Same park-env.sh-only rule as stage 1, same $$/exec
# pidfile idiom, same host-vs-container PID caveat.
# ---------------------------------------------------------------------------
#
# MEMORY NOTE: the container has been observed to be OOM-killed at this stage,
# because add_husky_park_1.launch brings up the robot WITH its sensors (Ouster
# lidar + stereo) and the Docker VM only has 7.65 GiB. If gzserver or the whole
# container dies here for no visible reason, check:
#   docker inspect <container> --format '{{.State.OOMKilled}}'
# This script deliberately does not try to work around that - raise the Docker
# Desktop memory limit instead.
docker compose exec -T husky bash -lc "source /workspace/park-env.sh && echo \$\$ > '$ROBOT_PIDFILE' && exec roslaunch natural_environments add_husky_park_1.launch" &
ROBOT_EXEC_PID=$!

# The controller spawner can die mid-way and leave husky_velocity_controller
# merely "initialized", in which case the robot silently ignores cmd_vel.
# Only open teleop once both controllers actually report ( running ).
echo -n "Waiting for the Husky controllers "
CONTROLLERS_READY=0
for _ in $(seq 1 120); do
  if docker compose exec -T husky bash -lc 'source /workspace/park-env.sh >/dev/null 2>&1 && rosrun controller_manager controller_manager list' 2>/dev/null \
     | grep -q 'husky_velocity_controller.*( running )'; then
    if docker compose exec -T husky bash -lc 'source /workspace/park-env.sh >/dev/null 2>&1 && rosrun controller_manager controller_manager list' 2>/dev/null \
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
  echo "  'source /workspace/park-env.sh && rosrun controller_manager controller_manager list'" >&2
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
docker compose exec -d husky bash -lc "source /workspace/park-env.sh && echo \$\$ > '$TELEOP_PIDFILE' && exec python3 '/workspace/husky_teleop_gui.py'"

echo ""
echo "noVNC:  $NOVNC_URL"
echo "Teleop is open in the 'Husky Teleop' window there (bottom-left) - click it to"
echo "focus, then drive with WASD. The robot stops as soon as you release the key."
echo ""

# Wait on the robot stage; the world stage is torn down by the trap on exit.
wait "$ROBOT_EXEC_PID"
