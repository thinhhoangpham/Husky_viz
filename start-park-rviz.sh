#!/usr/bin/env bash
set -euo pipefail

# Launch the LIVE park simulation inside the Husky container: gzserver (headless)
# + Husky spawn + costmap_2d + RViz, all as ONE roslaunch (see park_rviz.launch).
# Ctrl-C tears it down.
#
# Docker build context / compose files live on the internal SSD, not the project
# folder - Docker's build-context sender fails on the external drive with
# "failed to xattr ./._Dockerfile: operation not permitted".
DOCKER_DIR="$HOME/husky-docker"

NOVNC_URL="http://localhost:6080/vnc.html"

# WHAT YOU WILL SEE, so you don't misread normal startup as a failure:
#  - Gazebo runs HEADLESS (gui:=false in park_rviz.launch). There is NO Gazebo
#    window in noVNC - only RViz. That is deliberate: gzclient measured 1.5 GiB
#    against a 9.7 GiB Docker VM ceiling, and the park world alone already uses
#    ~7.5 GiB in gzserver. Dropping the GUI is what makes this fit at all.
#  - The park world takes MINUTES to load (304 MB terrain mesh). RViz appears
#    long before the world is ready and will look empty. Expected. Wait.
#  - costmap_node logs TF timeout warnings until the robot has spawned. Expected,
#    it retries on its own.
#  - A Gazebo error about a model named "Untitled2" with a dead absolute mesh
#    path is EXPECTED and harmless; see park_world_notes.md.
#  - Memory is TIGHT. If the container gets OOM-killed,
#      docker inspect husky-docker-husky-1 --format '{{.State.OOMKilled}}'
#    reports true. Do NOT try to work around that here - this script cannot fix
#    a VM memory ceiling.
#  - Do NOT run start-sim.sh or start-park.sh at the same time as this. Duplicate
#    sims kill robot_state_publisher and leave husky_velocity_controller stuck in
#    state `initialized`, after which the robot silently ignores all cmd_vel.

# The launch is exec'd with -T (mandatory: a backgrounded exec without -T fails
# with "the input device is not a TTY"), and -T means no TTY, which means Ctrl-C
# on the host reaches only the local docker client. roslaunch, gzserver, rviz and
# costmap_2d_node all survive inside the container, orphaned and invisible, and
# the next run then starts a SECOND set of them against the same master -
# duplicate node names get killed off by the master at random and the display
# breaks. Hence this pidfile: the trap has to terminate the in-container
# roslaunch by hand. Do not "simplify" this away.
LAUNCH_PIDFILE="/tmp/husky_park_rviz_roslaunch.$$.pid"

# A leftover teleop window from a previous run would start a second
# husky_teleop_gui node; rospy.init_node(anonymous=False) makes the master kill
# the older one, leaving two identical windows in noVNC of which one is silently
# dead. So this run tracks its own process and kills exactly that PID on the way
# out - never a broad match, which would also hit a teleop.sh --wasd session in
# the same container that the user started deliberately.
TELEOP_PIDFILE="/tmp/husky_teleop_gui.$$.pid"

# THIS instance's gzserver PID, recorded after the launch starts. READ THE
# COMMENT IN cleanup() BEFORE TOUCHING ANYTHING THAT KILLS GZSERVER.
GZSERVER_PIDFILE="/tmp/husky_park_rviz_gzserver.$$.pid"
CLEANED_UP=0

cleanup() {
  # Runs on INT, TERM and normal EXIT, so it must tolerate being called twice
  # and must not abort the script when there is nothing left to kill.
  [ "$CLEANED_UP" -eq 1 ] && return 0
  CLEANED_UP=1

  # The pidfile only exists once the teleop window actually started, so the
  # readiness/timeout paths (which exit before that) fall through harmlessly
  # here. A plain SIGTERM is enough: Qt tears the window down and the node's
  # atexit/close path publishes a final zero-velocity Twist.
  docker compose exec -T husky bash -lc \
    "pid=\$(cat '$TELEOP_PIDFILE' 2>/dev/null) || exit 0; \
     [ -n \"\$pid\" ] && kill \"\$pid\" 2>/dev/null; \
     rm -f '$TELEOP_PIDFILE'; exit 0" >/dev/null 2>&1 || true

  # The pidfile only exists once the launch actually started, so the readiness
  # timeout path (which exits before that) falls through harmlessly here.
  # roslaunch needs SIGINT to shut its nodes down cleanly; only after that do we
  # escalate to TERM and finally KILL.
  #
  # gzserver routinely outlives its parent roslaunch, and a surviving gzserver
  # holds ~7.5 GiB, which guarantees the next run is OOM-killed. So it MUST be
  # killed here - but by PID, never by name.
  #
  # !!! DO NOT REINTRODUCE `pkill -x gzserver` IN cleanup(). !!!
  # It was here, and it killed the WRONG process. Concrete, observed sequence:
  # the user pressed Ctrl-C and re-ran this script promptly. This cleanup was
  # still running - it SIGINTs the launch PID, waits up to 10s, escalates to
  # TERM, then KILL, and only THEN swept by name. By the time the sweep ran, the
  # NEW run had already started its own Gazebo, and `pkill -9 -x gzserver`
  # SIGKILLed it. Evidence: roslaunch logs show `[gazebo-1] process has died ...
  # exit code -9` twice (15:22:48 and 15:31:38) while cgroup oom_kill was 0 and
  # memory.peak was 9.30 GiB against a 9.703 GiB limit - so NOT an OOM kill.
  # A name-based match cannot distinguish THIS instance's gzserver from a newer
  # one; only a recorded PID can. This is the SECOND distinct process-killing
  # bug in this project (the first being `pkill -f` matching the killing shell's
  # own argv - see the DANGER comment further down). Name matching remains
  # correct in PRE-CLEAN, where by definition nothing of ours is running yet and
  # killing a stale gzserver is the entire point.
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
     gz=\$(cat '$GZSERVER_PIDFILE' 2>/dev/null) || gz=''; \
     if [ -n \"\$gz\" ]; then \
       kill -INT \"\$gz\" 2>/dev/null || true; \
       for _ in 1 2 3 4 5 6 7 8 9 10; do \
         kill -0 \"\$gz\" 2>/dev/null || break; \
         sleep 1; \
       done; \
       kill -TERM \"\$gz\" 2>/dev/null || true; \
       sleep 2; \
       kill -KILL \"\$gz\" 2>/dev/null || true; \
     fi; \
     rm -f '$LAUNCH_PIDFILE' '$GZSERVER_PIDFILE'; exit 0" >/dev/null 2>&1 || true
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
# robot_state_publisher publishes transforms that conflict with the new TF tree.
# Unlike replay-bag.sh this run also needs gzserver/gzclient swept: a stale
# gzserver still holds the ~7.5 GiB park world, and starting a second one on a
# 9.7 GiB VM gets the container OOM-killed outright. rosnode cleanup afterwards
# purges the registrations of nodes that are now gone but that the master still
# believes in.
#
# DANGER: `pkill -f <pattern>` is BANNED here. The in-container shell is started
# as `bash -lc "...rviz...costmap_2d_node...gzserver..."`, so its OWN argv
# contains every one of those strings and `pkill -f` would match the shell
# itself, killing it mid-command - the remaining kills and the `rosnode cleanup`
# would then silently never run. That exact bug was hit FOUR times in one
# session (`pkill -f gzserver`, `pkill -f "roslaunch natural_environments"`, and
# `pkill -f robot_state_publisher`). Everything below is either `pkill -x`
# (exact process-NAME match, which cannot match `bash`) or PID-based, where the
# PIDs come from a `ps -eo pid,args` pipeline that explicitly drops `bash -lc`
# lines before any kill happens.
#
# rosnode cleanup prompts for a y/n confirmation. Reading from /dev/null does
# NOT decline it - it just blocks forever waiting on stdin. `yes |` answers the
# prompt and `timeout 20` bounds it, so a wedged master cannot hang the whole
# script here. This actually happened: the script sat in this exec indefinitely
# and the sim never launched.
echo "Pre-cleaning stale ROS processes inside the container ..."
docker compose exec -T husky bash -lc \
  "pids=\$(ps -eo pid,args --no-headers \
      | grep -v 'bash -lc' \
      | grep -v ' grep ' \
      | grep -E 'rviz|costmap_2d_node|rosbag[ /]|robot_state_publisher|gzserver|gzclient|roslaunch' \
      | awk '{print \$1}'); \
   for p in \$pids; do kill -INT \"\$p\" 2>/dev/null || true; done; \
   sleep 3; \
   for p in \$pids; do kill -KILL \"\$p\" 2>/dev/null || true; done; \
   pkill -x rviz 2>/dev/null || true; \
   pkill -x costmap_2d_node 2>/dev/null || true; \
   pkill -x robot_state_publisher 2>/dev/null || true; \
   pkill -x gzserver 2>/dev/null || true; \
   pkill -x gzclient 2>/dev/null || true; \
   sleep 2; \
   pkill -9 -x gzserver 2>/dev/null || true; \
   pkill -9 -x gzclient 2>/dev/null || true; \
   sleep 1; \
   source /opt/ros/noetic/setup.bash && yes | timeout 20 rosnode cleanup >/dev/null 2>&1 || true; \
   exit 0" >/dev/null 2>&1 || true

echo ""
echo "Launching the live park simulation (Ctrl-C to stop)..."
echo "Gazebo is HEADLESS - expect RViz only, and expect it to look empty for"
echo "several minutes while the 304 MB park terrain mesh loads."

# park-env.sh is sourced and is deliberately NOT followed by sourcing
# /opt/ros/noetic/setup.bash: that would reset ROS_PACKAGE_PATH and wipe the
# package overlay park-env.sh just put in place, and $(find gazebo_ros) /
# $(find natural_environments) in the launch file would then fail to resolve.
#
# The container shell records its own $$ and then `exec`s roslaunch over itself,
# so the recorded PID *is* roslaunch - there is no other way to learn an
# in-container PID from the host.
#
# DOCKER_EXEC_PID below is the HOST-side docker client, which is NOT the
# roslaunch PID: killing it only tears down the local end of the -T exec and
# leaves roslaunch (and gzserver) running in the container. Confusing these two
# is the exact bug the pidfile above exists to fix.
docker compose exec -T husky bash -lc "source /workspace/park-env.sh && export DISPLAY=:1 && echo \$\$ > '$LAUNCH_PIDFILE' && exec roslaunch /workspace/park_rviz.launch" &
DOCKER_EXEC_PID=$!

# Record THIS instance's gzserver PID so cleanup() can kill exactly it and never
# a newer run's gzserver (see the emphatic comment in cleanup()). Backgrounded on
# the host so the script still reaches its final `wait "$DOCKER_EXEC_PID"`. The
# park world takes minutes to load, hence the 300s bound; if gzserver never
# appears this gives up quietly and cleanup() just finds an empty pidfile.
# The PID comes from a `ps -eo pid,args` pipeline that explicitly drops
# `bash -lc` lines - never `pgrep -f`, for the reason in the DANGER comment above.
docker compose exec -T husky bash -lc \
  "for _ in \$(seq 1 300); do \
     p=\$(ps -eo pid,args --no-headers \
         | grep -v 'bash -lc' \
         | grep -v ' grep ' \
         | awk '\$2 ~ /(^|\\/)gzserver\$/ {print \$1; exit}'); \
     if [ -n \"\$p\" ]; then echo \"\$p\" > '$GZSERVER_PIDFILE'; exit 0; fi; \
     sleep 1; \
   done; exit 0" >/dev/null 2>&1 &

# -d so the exec returns immediately. No xterm and no TTY: the Qt window reads
# real X KeyPress/KeyRelease events, which is what lets it stop the robot the
# instant a key is released. husky_teleop.py remains the terminal fallback,
# reachable via `teleop.sh --wasd`.
#
# `docker compose exec -d` gives us no way to learn the in-container PID, so the
# shell records its own $$ and then `exec`s python over itself - after the exec,
# that recorded PID *is* the teleop process.
#
# TIMING: unlike start-sim.sh there is NO controller-readiness polling here. This
# launch loads a 304 MB terrain mesh and takes minutes, so a readiness loop would
# only add complexity. The teleop window therefore may appear before the robot
# has even spawned, and keys will do nothing until the controllers are up. That
# is expected, not a fault. Verify the controllers with:
#   rosrun controller_manager controller_manager list
# both husky_joint_publisher and husky_velocity_controller must read ( running ).
#
# The trailing `/kb_teleop/cmd_vel:=/cmd_vel` is a ROS command-line remap and is
# REQUIRED. husky_teleop_gui.py publishes to the global topic /kb_teleop/cmd_vel
# (its CMD_VEL_TOPIC constant, line 74), but the researchers' twist_mux config at
# natural_environments_ros/husky/husky_control/config/twist_mux.yaml DELETED the
# `kb` input that the stock Clearpath config provides - verified: their config
# lists only joy (priority 10), interactive_marker (8) and external -> /cmd_vel
# (1). Without the remap the keys do nothing at all and the robot silently
# ignores them. /cmd_vel is accepted at priority 1.
docker compose exec -d husky bash -lc "source /workspace/park-env.sh && export DISPLAY=:1 && echo \$\$ > '$TELEOP_PIDFILE' && exec python3 /workspace/husky_teleop_gui.py /kb_teleop/cmd_vel:=/cmd_vel"

echo ""
echo "noVNC:  $NOVNC_URL"
echo "Teleop is open in the 'Husky Teleop' window there (bottom-left) - click it to"
echo "focus, then drive with WASD. The robot stops as soon as you release the key."
echo "Keys do nothing until the robot has spawned and the controllers are running."
echo ""

# This is a live sim, so it never ends on its own: the wait returns when the user
# hits Ctrl-C, or when they close the RViz window (rviz IS required="true" in the
# launch file, so closing it ends the whole launch). Either way the trap runs and
# sweeps gzserver.
wait "$DOCKER_EXEC_PID"
