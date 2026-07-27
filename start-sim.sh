#!/usr/bin/env bash
set -euo pipefail

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

# The sim is exec'd with -T (mandatory: a backgrounded exec without -T fails with
# "the input device is not a TTY"), and -T means no TTY, which means Ctrl-C on the
# host reaches only the local docker client. roslaunch/gzserver/gzclient survive
# inside the container, orphaned and invisible, and the next run then starts a
# SECOND roslaunch into the same container - two masters' worth of nodes, with
# robot_state_publisher killed as a duplicate name and the controller spawner
# dying, so the robot ignores cmd_vel. Hence this second pidfile: the trap has to
# terminate the in-container sim by hand. Do not "simplify" this away.
SIM_PIDFILE="/tmp/husky_sim_roslaunch.$$.pid"
CLEANED_UP=0

cleanup() {
  # Runs on INT, TERM and normal EXIT, so it must tolerate being called twice
  # and must not abort the script when there is nothing left to kill.
  [ "$CLEANED_UP" -eq 1 ] && return 0
  CLEANED_UP=1

  # The pidfile only exists once the teleop window actually started, so the
  # readiness timeout path (which exits before that) falls through harmlessly
  # here. A plain SIGTERM is enough: Qt tears the window down and the node's
  # atexit/close path publishes a final zero-velocity Twist.
  docker compose exec -T husky bash -lc \
    "pid=\$(cat '$TELEOP_PIDFILE' 2>/dev/null) || exit 0; \
     [ -n \"\$pid\" ] && kill \"\$pid\" 2>/dev/null; \
     rm -f '$TELEOP_PIDFILE'; exit 0" >/dev/null 2>&1 || true

  # Same story for the sim, except roslaunch needs SIGINT to shut its nodes down
  # cleanly; only after that do we escalate. gzserver/gzclient regularly outlive
  # their parent, so they get a name-based sweep as a backstop.
  #
  # DANGER: `pkill -f gzserver` would match THIS shell, whose own argv contains
  # the string "gzserver" - it would kill itself mid-script and the rest of the
  # cleanup would silently never run. Everything below is either PID-based from
  # the pidfile or `-x` (exact process-name match), which cannot match `bash`.
  docker compose exec -T husky bash -lc \
    "pid=\$(cat '$SIM_PIDFILE' 2>/dev/null) || pid=''; \
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
     pkill -x gzclient 2>/dev/null || true; \
     pkill -x gzserver 2>/dev/null || true; \
     sleep 2; \
     pkill -KILL -x gzclient 2>/dev/null || true; \
     pkill -KILL -x gzserver 2>/dev/null || true; \
     rm -f '$SIM_PIDFILE'; exit 0" >/dev/null 2>&1 || true
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
echo "Launching the simulation (Ctrl-C to stop)..."

# Backgrounded rather than exec'd so we can wait for the controllers and then
# open the teleop xterm.
#
# As with the xterm, the container shell records its own $$ and then `exec`s
# roslaunch over itself, so the recorded PID *is* roslaunch - there is no other
# way to learn an in-container PID from the host.
#
# DOCKER_EXEC_PID below is the HOST-side docker client, which is NOT the
# roslaunch PID: killing it only tears down the local end of the -T exec and
# leaves roslaunch running in the container. Confusing these two is the exact
# bug this pidfile exists to fix.
docker compose exec -T husky bash -lc "source /opt/ros/noetic/setup.bash && export DISPLAY=:1 && echo \$\$ > '$SIM_PIDFILE' && exec roslaunch husky_gazebo husky_playpen.launch" &
DOCKER_EXEC_PID=$!

# The controller spawner can die mid-way and leave husky_velocity_controller
# merely "initialized", in which case the robot silently ignores cmd_vel.
# Only open teleop once both controllers actually report ( running ).
echo -n "Waiting for the Husky controllers "
CONTROLLERS_READY=0
for _ in $(seq 1 120); do
  if docker compose exec -T husky bash -lc 'source /opt/ros/noetic/setup.bash && rosrun controller_manager controller_manager list' 2>/dev/null \
     | grep -q 'husky_velocity_controller.*( running )'; then
    if docker compose exec -T husky bash -lc 'source /opt/ros/noetic/setup.bash && rosrun controller_manager controller_manager list' 2>/dev/null \
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
  echo "  'source /opt/ros/noetic/setup.bash && rosrun controller_manager controller_manager list'" >&2
  echo "A common cause is a second simulation already running." >&2
  kill "$DOCKER_EXEC_PID" 2>/dev/null || true
  wait "$DOCKER_EXEC_PID" 2>/dev/null || true
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
docker compose exec -d husky bash -lc "source /opt/ros/noetic/setup.bash && export DISPLAY=:1 && echo \$\$ > '$TELEOP_PIDFILE' && exec python3 '/workspace/husky_teleop_gui.py'"

echo ""
echo "noVNC:  $NOVNC_URL"
echo "Teleop is open in the 'Husky Teleop' window there (bottom-left) - click it to"
echo "focus, then drive with WASD. The robot stops as soon as you release the key."
echo ""

wait "$DOCKER_EXEC_PID"
