#!/usr/bin/env bash
set -euo pipefail

# Replay park_1.bag inside the Husky container: RViz + costmap_2d + rosbag play,
# all started as ONE roslaunch (see replay_park.launch). Ctrl-C tears it down.
#
# Docker build context / compose files live on the internal SSD, not the project
# folder - Docker's build-context sender fails on the external drive with
# "failed to xattr ./._Dockerfile: operation not permitted".
DOCKER_DIR="$HOME/husky-docker"

NOVNC_URL="http://localhost:6080/vnc.html"

# The launch is exec'd with -T (mandatory: a backgrounded exec without -T fails
# with "the input device is not a TTY"), and -T means no TTY, which means Ctrl-C
# on the host reaches only the local docker client. roslaunch, rviz,
# costmap_2d_node and rosbag play all survive inside the container, orphaned and
# invisible, and the next run then starts a SECOND set of them against the same
# master - duplicate node names get killed off by the master at random and the
# display breaks. Hence this pidfile: the trap has to terminate the
# in-container roslaunch by hand. Do not "simplify" this away.
LAUNCH_PIDFILE="/tmp/husky_replay_roslaunch.$$.pid"
CLEANED_UP=0

cleanup() {
  # Runs on INT, TERM and normal EXIT, so it must tolerate being called twice
  # and must not abort the script when there is nothing left to kill.
  [ "$CLEANED_UP" -eq 1 ] && return 0
  CLEANED_UP=1

  # The pidfile only exists once the launch actually started, so the readiness
  # timeout path (which exits before that) falls through harmlessly here.
  # roslaunch needs SIGINT to shut its nodes down cleanly; only after that do we
  # escalate to TERM and finally KILL.
  docker compose exec -T husky bash -lc \
    "pid=\$(cat '$LAUNCH_PIDFILE' 2>/dev/null) || pid=''; \
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
     rm -f '$LAUNCH_PIDFILE'; exit 0" >/dev/null 2>&1 || true
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

# PRE-CLEAN. A leftover rviz / costmap_2d_node / rosbag play / robot_state_publisher
# from an earlier run fights this one: duplicate node names make the master kill
# one of each pair at random, so the display ends up half-dead, and a surviving
# robot_state_publisher publishes transforms that conflict with the bag's own TF
# tree. rosnode cleanup afterwards purges the registrations of nodes that are now
# gone but that the master still believes in.
#
# DANGER: `pkill -f <pattern>` is BANNED here. The in-container shell is started
# as `bash -lc "...rviz...costmap_2d_node...rosbag play..."`, so its OWN argv
# contains every one of those strings and `pkill -f` would match the shell
# itself, killing it mid-command - the remaining kills and the `rosnode cleanup`
# would then silently never run. That exact bug was hit FOUR times in one
# session (`pkill -f gzserver`, `pkill -f "roslaunch natural_environments"`, and
# `pkill -f robot_state_publisher`). Everything below is either `pkill -x`
# (exact process-NAME match, which cannot match `bash`) or PID-based, where the
# PIDs come from a `ps -eo pid,args` pipeline that explicitly drops `bash -lc`
# lines before any kill happens.
echo "Pre-cleaning stale ROS processes inside the container ..."
docker compose exec -T husky bash -lc \
  "pids=\$(ps -eo pid,args --no-headers \
      | grep -v 'bash -lc' \
      | grep -v ' grep ' \
      | grep -E 'rviz|costmap_2d_node|rosbag[ /]|robot_state_publisher' \
      | awk '{print \$1}'); \
   for p in \$pids; do kill -INT \"\$p\" 2>/dev/null || true; done; \
   sleep 2; \
   for p in \$pids; do kill -KILL \"\$p\" 2>/dev/null || true; done; \
   pkill -x rviz 2>/dev/null || true; \
   pkill -x costmap_2d_node 2>/dev/null || true; \
   pkill -x robot_state_publisher 2>/dev/null || true; \
   sleep 1; \
   source /opt/ros/noetic/setup.bash && rosnode cleanup </dev/null >/dev/null 2>&1 || true; \
   exit 0" >/dev/null 2>&1 || true

echo ""
echo "noVNC:  $NOVNC_URL"
echo ""
echo "Launching the bag replay (Ctrl-C to stop)..."

# park-env.sh is sourced and is deliberately NOT followed by sourcing
# /opt/ros/noetic/setup.bash: that would reset ROS_PACKAGE_PATH and wipe the
# package overlay park-env.sh just put in place, and $(find xacro) / the
# husky_viz rviz config in the launch file would then fail to resolve.
#
# The container shell records its own $$ and then `exec`s roslaunch over itself,
# so the recorded PID *is* roslaunch - there is no other way to learn an
# in-container PID from the host.
#
# DOCKER_EXEC_PID below is the HOST-side docker client, which is NOT the
# roslaunch PID: killing it only tears down the local end of the -T exec and
# leaves roslaunch running in the container. Confusing these two is the exact
# bug the pidfile above exists to fix.
docker compose exec -T husky bash -lc "source /workspace/park-env.sh && export DISPLAY=:1 && echo \$\$ > '$LAUNCH_PIDFILE' && exec roslaunch /workspace/replay_park.launch" &
DOCKER_EXEC_PID=$!

echo ""
echo "noVNC:  $NOVNC_URL"
echo ""

# The bag is roughly 253 s long. rosbag play is NOT required="true" in the launch
# file, so when playback ends RViz and the costmap node stay up holding the final
# state - which is the point, you want to look at the finished costmap. So this
# wait does not return at the end of the bag: it returns when the user hits
# Ctrl-C, or when they close the RViz window (rviz IS required="true", so closing
# it ends the whole launch).
wait "$DOCKER_EXEC_PID"
