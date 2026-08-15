#!/usr/bin/env bash
set -euo pipefail

# load-park-world.sh - load the natural_environments "park" Gazebo world on a
# NATIVE Linux + ROS Noetic install, AND spawn the Husky into it.
#
# SCOPE: the world plus the robot. Spawning the Husky is the DEFAULT - use
# --no-robot for the old world-only behaviour. This script still does NOT
# start rviz and does NOT play a bag; those remain separate concerns.
#
# The launch is TWO STAGES, world first and robot second, because the robot's
# spawn_model call needs a live, fully parsed world to place the Husky into.
# See the readiness poll below - a fixed sleep is not good enough here.
#
# KNOWN LIMITATION - THE OUSTER LIDAR:
#   libgazebo_ros_ouster_laser.so does not exist anywhere on this machine. It
#   used to be built in the old Docker container's /root/catkin_ws, which is
#   gone. Consequence: the robot spawns, but with NO laser, and
#   /os0_cloud_node/points never publishes. The script searches for the plugin
#   at runtime and warns loudly if it is missing - it does NOT abort, because a
#   Husky with no lidar is still a useful Husky. To fix it properly, build
#   natural_environments_ros_opt/ouster_example in a catkin workspace.
#
# RELATIONSHIP TO THE OLDER SCRIPTS IN THIS FOLDER
# ------------------------------------------------
# start-park.sh, start-park-optimized.sh, park-env.sh, park-env-opt.sh,
# start-sim.sh and friends were written for a completely different machine:
# macOS + Docker Desktop, with the project bind-mounted into a Noetic
# container at /workspace and rendered through Xvfb + x11vnc + noVNC on
# DISPLAY=:1. NONE of that applies here - this box is native Linux, docker is
# not even installed, and ROS/Gazebo run directly against the user's real X
# display. So all the Docker guards, the noVNC readiness poll and the
# unconditional `export DISPLAY=:1` from those scripts are intentionally
# absent below. Do not "restore" them.
#
# What IS carried over from park-env-opt.sh, because it is still true:
#   * source /opt/ros/noetic/setup.bash BEFORE the ROS_PACKAGE_PATH prepend -
#     setup.bash *resets* ROS_PACKAGE_PATH, so prepending first would be
#     silently undone and roslaunch would fail with
#     "package 'natural_environments' not found".
#   * the natural_environments_ros_opt overlay must be PREPENDED so its
#     husky_* packages shadow the stock /opt/ros/noetic ones.
#   * GAZEBO_MODEL_PATH must include models_opt - that is where Gazebo
#     resolves model://terreno_parque/terreno_parque_heightmap.png and the
#     rest of the park's meshes.
#
# Note the package directory is spelled `natural_enviroment` (missing an 'n')
# but the ROS package NAME in its package.xml is `natural_environments`.
# rospack crawls ROS_PACKAGE_PATH recursively, so pointing at the tree root is
# enough and the misspelt directory name never has to be typed.
#
# USAGE
#   ./load-park-world.sh              # world + Husky, GUI            (DEFAULT)
#   ./load-park-world.sh --headless   # world + Husky, no gzclient window
#   ./load-park-world.sh --no-robot   # world only (the pre-robot behaviour)
#   ./load-park-world.sh --gzonly     # plain `gazebo park.world`, no ROS, NO ROBOT
#   ./load-park-world.sh --help       # this help (also -h)
#
# --gzonly cannot spawn the robot BY DEFINITION: it starts no roscore and no
# /gazebo/* services, and spawn_model is a ROS service call. It is a "just look
# at the scene" mode. --headless, by contrast, is only about the GUI window, so
# it DOES spawn the robot.
#
# FLAG NOTE: -h is HELP, not headless. The two were deliberately not allowed
# to collide - `--headless` has no short form, because a `-h` that silently
# started a five-minute headless world load instead of printing usage is
# exactly the kind of surprise worth avoiding.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_SETUP="/opt/ros/noetic/setup.bash"
PKG_TREE="$SCRIPT_DIR/natural_environments_ros_opt"
SENSOR_URDF="$PKG_TREE/husky/husky_description/urdf/sensor_description.urdf"

MODE="ros"    # ros | headless | gzonly
SPAWN_ROBOT=1 # default ON; cleared by --no-robot, forced off by --gzonly

# Which world to load. Park is the default so every existing invocation and
# runbook step is unchanged; --world lake switches the world file, the launch
# pair and GAZEBO_MODEL_PATH. Both worlds are self-contained in their own
# models_*_opt tree. See docs/lake-optimization-plan.md.
WORLD_NAME="park"

set_world_paths() {
  case "$WORLD_NAME" in
    park)
      MODEL_DIR="$SCRIPT_DIR/models_opt"
      WORLD_FILE="$PKG_TREE/natural_enviroment/worlds/park.world"
      WORLD_LAUNCH="create_park.launch"
      ROBOT_LAUNCH="add_husky_park_1.launch"
      # A model name that only exists once the world is genuinely parsed. The
      # readiness poll greps get_world_properties for it -- it MUST be a model
      # from THIS world or the poll never succeeds and the robot never spawns.
      WORLD_READY_MODEL="parque"
      ;;
    lake)
      # Self-contained, exactly like park's models_opt: models_lake_opt holds
      # BOTH the low-poly visual meshes and the original collision meshes, plus
      # a tree_8 symlink to park's shared tree. Verified: all 16 mesh URIs in
      # lake.world resolve from this one root. No external drive needed at
      # runtime -- that is only required to REBUILD the low-poly meshes
      # (scripts/optimize_lake_meshes.py).
      MODEL_DIR="$SCRIPT_DIR/models_lake_opt"
      WORLD_FILE="$PKG_TREE/natural_enviroment/worlds/lake.world"
      WORLD_LAUNCH="create_lake.launch"
      ROBOT_LAUNCH="add_husky_lake_1.launch"
      WORLD_READY_MODEL="terreno_lago"
      ;;
    *)
      echo "Error: unknown --world '$WORLD_NAME' (expected park|lake)" >&2
      exit 2
      ;;
  esac
}

# How long to wait for the world before giving up and NOT spawning the robot.
# natural_environments_ros/readme.txt:49-50 documents 2-10 minute load times for
# the original COLLADA terrain. The _opt heightmap world is much faster, but the
# ceiling stays generous so it is only ever a genuine-failure signal, never
# something that trips on a merely slow-but-healthy load.
WORLD_TIMEOUT_S=300
# Controllers come up quickly once the robot is spawned; this only needs to
# outlast the controller_spawner's own retry loop.
CONTROLLER_TIMEOUT_S=120

usage() {
  cat <<EOF
load-park-world.sh - load the natural_environments "park" Gazebo world and
spawn the Husky into it.

Usage: $(basename "${BASH_SOURCE[0]}") [--world park|lake] [--headless] [--no-robot] [--gzonly] [--help]

  --world W    Which world to load: park (default) or lake. Selects the world
               file, the create_/add_husky_ launch pair, and GAZEBO_MODEL_PATH.
               Lake is self-contained in models_lake_opt/ (low-poly visuals
               + original collision meshes); no external drive needed.
  (no flags)   Two stages: roslaunch natural_environments \$WORLD_LAUNCH,
               then, once the world is genuinely up,
               roslaunch natural_environments \$ROBOT_LAUNCH.
               Full ROS stack: gzserver + gzclient GUI, use_sim_time=true.
               SPAWNING THE ROBOT IS THE DEFAULT.
  --headless   No gzclient window. Use over SSH or when you just need the ROS
               services/topics up. The robot is STILL spawned - headless is
               about the GUI, not the robot.
  --no-robot   World only, no Husky. This was the previous behaviour of this
               script and is still handy for inspecting the scene under ROS.
  --gzonly     Bypass ROS entirely: plain \`gazebo <park.world>\` with only
               GAZEBO_MODEL_PATH set. Quickest way to just look at the scene.
               No roscore, no /gazebo/* services - and therefore NO ROBOT,
               since spawn_model is a ROS service call. That is the point of
               this mode, not an oversight.
  -h, --help   Show this message.

No rviz and no bag playback in any mode.
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --headless) MODE="headless" ;;
    --gzonly)   MODE="gzonly" ;;
    --no-robot) SPAWN_ROBOT=0 ;;
    --world)
      shift
      [ $# -gt 0 ] || { echo "Error: --world needs a value (park|lake)." >&2; exit 2; }
      WORLD_NAME="$1"
      ;;
    --world=*)  WORLD_NAME="${1#--world=}" ;;
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

# Resolve MODEL_DIR / WORLD_FILE / launch names from WORLD_NAME. After parsing
# so --world may appear in any position; exits 2 on an unknown world.
set_world_paths

# --gzonly starts no ROS master, so there is nothing to spawn the robot with.
# Rather than erroring out on `--gzonly --no-robot` (which is redundant but not
# wrong) or on a bare `--gzonly` (which never implied a robot), just force it
# off. Order matters: this runs AFTER argument parsing so flag order is
# irrelevant.
if [ "$MODE" = "gzonly" ]; then
  SPAWN_ROBOT=0
fi

# ---------------------------------------------------------------------------
# Preflight. Every one of these is a thing that, if missing, produces a
# confusing downstream failure rather than an obvious one - hence checking up
# front instead of letting roslaunch/Gazebo stumble into it.
# ---------------------------------------------------------------------------
fail() { echo "Error: $*" >&2; exit 1; }

# --gzonly does not touch ROS, so it is exempt from the ROS checks below.
if [ "$MODE" != "gzonly" ] && [ ! -f "$ROS_SETUP" ]; then
  fail "$ROS_SETUP not found. This script expects a native ROS Noetic install."
fi

# gzserver rather than gazebo: `gazebo` is the client+server wrapper and is not
# used in headless mode, whereas gzserver is required by all three modes.
command -v gzserver >/dev/null 2>&1 \
  || fail "gzserver is not on PATH. Is Gazebo installed (apt install gazebo11)?"

if [ "$MODE" = "gzonly" ]; then
  command -v gazebo >/dev/null 2>&1 \
    || fail "--gzonly needs the 'gazebo' wrapper on PATH, but it is missing."
fi

# MODEL_DIR may be a colon-separated LIST (lake needs three roots), so check
# each entry rather than the whole string -- `[ -d a:b ]` is always false.
_check_model_dirs() {
  local IFS=':' d
  for d in $MODEL_DIR; do
    [ -d "$d" ] || fail "models directory not found: $d
Without it Gazebo cannot resolve model:// URIs and the world loads empty.
(GAZEBO_MODEL_PATH for --world $WORLD_NAME is: $MODEL_DIR)"
  done
}
_check_model_dirs

[ -d "$PKG_TREE" ] \
  || fail "package tree not found: $PKG_TREE
This is the overlay that provides the 'natural_environments' package."

[ -f "$WORLD_FILE" ] \
  || fail "world file not found: $WORLD_FILE"

# ---------------------------------------------------------------------------
# Environment.
# ---------------------------------------------------------------------------
# GAZEBO_MODEL_PATH is needed in all three modes. `${VAR:-}` guards are
# mandatory under `set -u`: on a fresh shell neither of these is exported yet,
# and a bare $GAZEBO_MODEL_PATH would abort the script with "unbound variable".
export GAZEBO_MODEL_PATH="$MODEL_DIR${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"

if [ "$MODE" != "gzonly" ]; then
  # setup.bash references unset vars internally, so `set -u` has to be relaxed
  # around it. It also resets ROS_PACKAGE_PATH - which is precisely why the
  # prepend below comes AFTER it, never before.
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  set -u
  export ROS_PACKAGE_PATH="$PKG_TREE${ROS_PACKAGE_PATH:+:$ROS_PACKAGE_PATH}"
fi

# NOT set here, on purpose:
#   DISPLAY=:1 - that was the container's Xvfb. Here the user's real DISPLAY
#     must be respected, so it is left exactly as inherited.

# ---------------------------------------------------------------------------
# Robot environment. Only relevant when a robot is actually spawned, so the
# whole block is skipped otherwise - in particular --gzonly must not inherit
# HUSKY_* vars it has no use for.
#
# These two came from the old park-env-opt.sh and select the sensor arch the
# bag was recorded with (Ouster OS1-64; the stereo pair is commented out in
# sensor_description.urdf). Documented in
# natural_environments_ros_opt/readme.txt:28-29.
# ---------------------------------------------------------------------------
OUSTER_PLUGIN_DIR=""
SENSOR_ARCH_ON=0

if [ "$SPAWN_ROBOT" -eq 1 ]; then
  # The existence check is LOAD-BEARING, not defensive politeness. HUSKY_*
  # vars are consumed by husky_description's xacro via $(optenv ...). If
  # HUSKY_URDF_EXTRAS points at a file that is not there, the xacro expansion
  # fails outright, robot_description never gets set, and spawn_model has
  # nothing to spawn - i.e. NO robot at all. A stock Husky with no sensor arch
  # is strictly better than that, so on a missing file we warn and fall back
  # rather than exporting a dangling path.
  if [ -f "$SENSOR_URDF" ]; then
    export HUSKY_SENSOR_ARCH=true
    export HUSKY_URDF_EXTRAS="$SENSOR_URDF"
    SENSOR_ARCH_ON=1
  else
    echo "Warning: sensor arch URDF not found:" >&2
    echo "           $SENSOR_URDF" >&2
    echo "         HUSKY_SENSOR_ARCH / HUSKY_URDF_EXTRAS will NOT be set, and a" >&2
    echo "         STOCK Husky (no Ouster, no sensor mast) will be spawned instead." >&2
    echo "         Exporting a dangling HUSKY_URDF_EXTRAS would break the xacro" >&2
    echo "         expansion and spawn no robot at all, which is worse." >&2
    echo "" >&2
  fi

  # ---------------------------------------------------------------------
  # Ouster Gazebo plugin hunt.
  #
  # libgazebo_ros_ouster_laser.so is not packaged as a deb; it is built from
  # source (natural_environments_ros_opt/ouster_example) in a catkin
  # workspace. The old container built it at /root/catkin_ws/devel/lib, which
  # no longer exists on this machine. Search the plausible places rather than
  # hardcoding one, and treat "not found" as a warning, never a failure - the
  # robot itself spawns fine without it.
  #
  # GAZEBO_PLUGIN_PATH is searched first so that a user who has already wired
  # it up correctly wins without this script touching anything.
  # ---------------------------------------------------------------------
  OUSTER_SO="libgazebo_ros_ouster_laser.so"
  OUSTER_SEARCH_DIRS=""
  # ${VAR:-} guard: GAZEBO_PLUGIN_PATH is typically unset, and `set -u` would
  # abort on a bare reference.
  if [ -n "${GAZEBO_PLUGIN_PATH:-}" ]; then
    OUSTER_SEARCH_DIRS="${GAZEBO_PLUGIN_PATH//:/$'\n'}"
  fi
  OUSTER_SEARCH_DIRS="$OUSTER_SEARCH_DIRS
$HOME/husky_overlay_ws/devel/lib
/opt/ros/noetic/lib
$HOME/catkin_ws/devel/lib
$SCRIPT_DIR/catkin_ws/devel/lib
$SCRIPT_DIR/../catkin_ws/devel/lib"

  while IFS= read -r d; do
    [ -n "$d" ] || continue
    if [ -f "$d/$OUSTER_SO" ]; then
      OUSTER_PLUGIN_DIR="$d"
      break
    fi
  done <<< "$OUSTER_SEARCH_DIRS"

  if [ -n "$OUSTER_PLUGIN_DIR" ]; then
    export GAZEBO_PLUGIN_PATH="$OUSTER_PLUGIN_DIR${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
    echo "Ouster plugin      -> $OUSTER_PLUGIN_DIR/$OUSTER_SO"
  else
    cat >&2 <<EOF

!! ---------------------------------------------------------------------- !!
!! WARNING: $OUSTER_SO was NOT found.
!!
!! The Husky WILL still spawn and WILL still drive. But it will come up with
!! NO laser sensor: /os0_cloud_node/points will never publish, and anything
!! downstream of the point cloud (mapping, obstacle avoidance, rviz lidar
!! display) will sit there waiting forever on a topic that never appears.
!! Gazebo logs this only as a terse plugin-load error that is easy to miss,
!! which is why it is called out loudly here instead.
!!
!! Searched:
$(while IFS= read -r d; do [ -n "$d" ] && echo "!!   $d"; done <<< "$OUSTER_SEARCH_DIRS")
!!
!! To fix: build $PKG_TREE/ouster_example
!! in a catkin workspace, e.g.
!!   mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
!!   ln -s "$PKG_TREE/ouster_example" .
!!   cd ~/catkin_ws && source /opt/ros/noetic/setup.bash && catkin_make
!! then re-run this script - ~/catkin_ws/devel/lib is already searched above.
!! ---------------------------------------------------------------------- !!

EOF
  fi
fi

# ---------------------------------------------------------------------------
# Display sanity. A GUI mode with no DISPLAY fails deep inside Ogre with an
# unhelpful message, so say something useful first.
# ---------------------------------------------------------------------------
if [ "$MODE" != "headless" ] && [ -z "${DISPLAY:-}" ]; then
  echo "Warning: DISPLAY is not set, so the Gazebo GUI cannot open." >&2
  echo "         If you are on SSH, re-run with --headless (gzserver only)," >&2
  echo "         or connect with 'ssh -X' / run from a desktop session." >&2
  echo "" >&2
fi

# ---------------------------------------------------------------------------
# Shutdown. Everything this script starts is placed in its OWN process group
# via setsid, and the trap signals that group by PID - never a name pattern.
#
# Why so careful: the old scripts' comments record two real bugs.
#   1. Orphaned gzserver/roslaunch from a previous run cause duplicate nodes,
#      a killed robot_state_publisher and a controller that never reaches
#      ( running ) - so leaving orphans behind is not cosmetic.
#   2. A broad `pkill -f gzserver` matches THIS shell (the pattern is in its
#      own argv) and also any unrelated Gazebo the user is running elsewhere.
#      Hence: no pkill here at all, ever.
#
# The process group matters because roslaunch's children (gzserver, gzclient,
# gazebo_gui) are what actually need the signal; killing only roslaunch's PID
# regularly leaves gzserver alive.
#
# TWO GROUPS, NOT ONE. The world (create_park.launch) and the robot
# (add_husky_park_1.launch) are independent roslaunch processes, each with its
# own process group. Tracking only one leaves the other orphaned - an orphaned
# robot roslaunch spams a dead master, and an orphaned world leaves a gzserver
# that poisons the NEXT run with duplicate nodes.
#
# ORDER IS LOAD-BEARING: robot group FIRST, world group SECOND. The old
# start-park-optimized.sh records why - the robot's controllers need a live
# gzserver to unload against, and tearing the world out from under them first
# hangs controller_manager's shutdown.
# ---------------------------------------------------------------------------
WORLD_PGID=""
ROBOT_PGID=""
CLEANED_UP=0

# Tear down ONE process group with the INT -> TERM -> KILL escalation.
# Factored out so both stages get identical treatment and the ordering in
# cleanup() stays readable. Tolerates an empty/already-dead group.
kill_pgid() {
  local pgid="$1"
  [ -n "$pgid" ] || return 0

  # SIGINT first: roslaunch only shuts its nodes down cleanly on INT, and
  # gzserver saves/closes properly on it too. Negative PID = process group.
  kill -INT -- "-$pgid" 2>/dev/null || true

  # Give it up to 15s to go away on its own before escalating.
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
  # Runs on INT, TERM and normal EXIT, so it must tolerate a second call and
  # must never abort the script when there is nothing left to kill.
  [ "$CLEANED_UP" -eq 1 ] && return 0
  CLEANED_UP=1

  kill_pgid "$ROBOT_PGID"
  kill_pgid "$WORLD_PGID"
  return 0
}
trap cleanup INT TERM EXIT

# Read a process group id back from ps rather than assuming PGID == PID.
# With setsid the child IS its own group leader so they are equal in practice,
# but reading it back means a future change to the launch line cannot silently
# make the trap signal the wrong group. Falls back to the PID if ps has nothing
# (the child already exited, in which case there is nothing to signal anyway).
pgid_of() {
  local pid="$1" pgid
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ' || true)"
  [ -n "$pgid" ] || pgid="$pid"
  printf '%s' "$pgid"
}

# ---------------------------------------------------------------------------
# Heads-up before the long silence.
# ---------------------------------------------------------------------------
cat <<EOF
--------------------------------------------------------------------------
Loading the $WORLD_NAME world (stage 1), then the Husky (stage 2) once the world
is genuinely up. Stage 2 is skipped entirely with --no-robot or --gzonly.
EOF
if [ "$WORLD_NAME" = "park" ]; then
  cat <<'EOF'

EXPECTED, HARMLESS ERROR
  Gazebo will log a mesh-not-found error for a model named `Untitled2`
  pointing at /home/a/Desktop/modelos_mundo_dataset/terreno_dataset.dae -
  a dead absolute path baked into park.world by its original author. That
  model is unused junk parked at z ~ -1.76e8 and is NOT the ground. The
  real ground is the `parque` and `camino_parque` models. Documented in
  park_world_notes.md. Do not read it as a failed load.

FIRST LOAD IS SLOW
  The terrain assets are large (terreno_parque.dae alone is 304 MB), so a
  long silent startup with no output is normal, not a hang. Later loads
  are faster once the assets are in the page cache.
EOF
else
  cat <<'EOF'

LAKE USES THE LOW-POLY MESHES
  Visuals are the low-poly meshes; collision stays on the ORIGINAL
  meshes, so the lidar sees full detail. Both live in models_lake_opt/,
  so this world is self-contained. See docs/lake-optimization-plan.md.

THE WATER IS NOT AN OBSTACLE
  `lago` is a visual-only box with no <collision>, so the lidar returns
  nothing from it and live obstacle avoidance cannot see it. It is a
  landmark in maps/lake_objects.yaml, not occupied cells in lake_map.pgm.
EOF
fi
cat <<'EOF'

Ctrl-C to stop.
--------------------------------------------------------------------------
EOF

echo "ROS_PACKAGE_PATH  -> ${ROS_PACKAGE_PATH%%:*}"
echo "GAZEBO_MODEL_PATH -> ${GAZEBO_MODEL_PATH%%:*}"
echo "DISPLAY           -> ${DISPLAY:-<unset>}"
if [ "$SPAWN_ROBOT" -eq 1 ]; then
  if [ "$SENSOR_ARCH_ON" -eq 1 ]; then
    echo "Robot             -> Husky WITH sensor arch (HUSKY_SENSOR_ARCH=true)"
  else
    echo "Robot             -> stock Husky (sensor arch URDF missing)"
  fi
else
  echo "Robot             -> not spawned"
fi
echo ""

# `setsid` puts the child in a fresh process group so the trap above can signal
# the whole tree. A side effect: Ctrl-C is no longer delivered to the child by
# the terminal, only to this script - which is fine, because cleanup() forwards
# it to the group. Do not drop setsid without also rethinking cleanup().
case "$MODE" in
  ros)
    echo "Stage 1: roslaunch natural_environments $WORLD_LAUNCH"
    setsid roslaunch natural_environments "$WORLD_LAUNCH" &
    ;;
  headless)
    # create_park.launch -> simulator_empty_world.launch -> empty_world.launch,
    # which accepts gui/headless args, so the GUI is switched off through
    # roslaunch rather than by starting gzserver by hand. That keeps
    # use_sim_time and the /gazebo/* services identical to the GUI mode.
    echo "Stage 1: roslaunch natural_environments $WORLD_LAUNCH gui:=false headless:=true"
    setsid roslaunch natural_environments "$WORLD_LAUNCH" gui:=false headless:=true &
    ;;
  gzonly)
    echo "Running: gazebo \"$WORLD_FILE\"  (no ROS, and therefore no robot)"
    setsid gazebo "$WORLD_FILE" &
    ;;
esac

WORLD_PID=$!
WORLD_PGID="$(pgid_of "$WORLD_PID")"

# ---------------------------------------------------------------------------
# World-only paths stop here and just wait on stage 1.
# ---------------------------------------------------------------------------
if [ "$SPAWN_ROBOT" -eq 0 ]; then
  # `wait` returns 128+signo if the child is signalled; `|| true` keeps `set -e`
  # from turning an ordinary Ctrl-C shutdown into a scary non-zero abort before
  # the trap has run.
  wait "$WORLD_PID" || true
  exit 0
fi

# ---------------------------------------------------------------------------
# Readiness poll before stage 2.
#
# A fixed sleep is NOT usable here. Load time varies hugely with host load and
# page-cache state (the terrain assets are hundreds of MB), and spawning the
# Husky into a half-built world either drops it through unloaded terrain or
# makes the spawn service call time out.
#
# Two signals, both required, cheapest first:
#   1. /gazebo/spawn_urdf_model appears in `rosservice list`. That is literally
#      the service stage 2 is about to call, so its presence is the most direct
#      possible precondition. It also implies roscore and gazebo_ros are up.
#   2. /gazebo/get_world_properties lists the model `parque`. gzserver
#      advertises its services BEFORE it has finished parsing the world SDF, so
#      signal 1 alone fires too early; `parque` is the park world's top-level
#      ground model and only appears once the SDF is actually instantiated.
#
# /clock was considered as a third probe and rejected: create_park.launch may
# start paused, in which case /clock is silent even though the world is
# perfectly ready. It would produce a false timeout.
# ---------------------------------------------------------------------------
echo ""
echo -n "Waiting for the $WORLD_NAME world to finish loading (up to ${WORLD_TIMEOUT_S}s) "
WORLD_READY=0
for _ in $(seq 1 "$WORLD_TIMEOUT_S"); do
  # If stage 1 died outright there is no point burning the rest of the timeout.
  if ! kill -0 "$WORLD_PID" 2>/dev/null; then
    echo ""
    echo "Error: the world roslaunch exited before the world came up." >&2
    echo "Scroll up for its output - a missing model or a bad ROS_PACKAGE_PATH" >&2
    echo "is the usual cause. The robot was NOT spawned." >&2
    exit 1
  fi
  if rosservice list 2>/dev/null | grep -q '^/gazebo/spawn_urdf_model$' \
     && rosservice call /gazebo/get_world_properties 2>/dev/null | grep -q "$WORLD_READY_MODEL"; then
    WORLD_READY=1
    echo " ready."
    break
  fi
  echo -n "."
  sleep 1
done

if [ "$WORLD_READY" -ne 1 ]; then
  echo ""
  echo "Error: the park world was not ready within ${WORLD_TIMEOUT_S}s, so the robot" >&2
  echo "was NOT spawned. Expected /gazebo/spawn_urdf_model to be advertised AND" >&2
  echo "/gazebo/get_world_properties to list the model 'parque'." >&2
  echo "Inspect by hand in another terminal, with this script's environment:" >&2
  echo "  source /opt/ros/noetic/setup.bash" >&2
  echo "  export ROS_PACKAGE_PATH=\"$PKG_TREE:\$ROS_PACKAGE_PATH\"" >&2
  echo "  rosservice list | grep gazebo" >&2
  echo "  rosservice call /gazebo/get_world_properties" >&2
  echo "If gzserver vanished rather than being merely slow, suspect an OOM kill:" >&2
  echo "  dmesg | tail" >&2
  # The EXIT trap tears the world down; do not duplicate that here.
  exit 1
fi

# ---------------------------------------------------------------------------
# Stage 2: the robot. Its own setsid group so cleanup() can kill it first,
# independently of and before the world.
# ---------------------------------------------------------------------------
echo ""
echo "Stage 2: roslaunch natural_environments $ROBOT_LAUNCH"
echo "         (spawns at the bag's recorded start pose: x=45.64 y=0.02 z=3.3 yaw=2.6132)"
setsid roslaunch natural_environments "$ROBOT_LAUNCH" &
ROBOT_PID=$!
ROBOT_PGID="$(pgid_of "$ROBOT_PID")"

# ---------------------------------------------------------------------------
# Controller verification.
#
# This checks a REAL recurring bug, documented in CLAUDE.md: when launches race
# or duplicate, the controller_spawner dies part-way through and
# husky_velocity_controller is left in state `initialized` instead of
# `running`. The simulation then looks completely healthy - the robot is
# visible, tf is publishing - but every cmd_vel message is silently dropped and
# teleop appears dead with no error anywhere. Catching it here converts a
# baffling twenty-minute debugging session into one line of output.
#
# Deliberately NOT fatal: the GUI and the world are still useful, and the user
# may well want to look around regardless. Warn loudly, then carry on.
#
# The `( running )` spacing matches controller_manager's own output format.
# ---------------------------------------------------------------------------
echo -n "Waiting for the Husky controllers (up to ${CONTROLLER_TIMEOUT_S}s) "
CONTROLLERS_READY=0
for _ in $(seq 1 "$CONTROLLER_TIMEOUT_S"); do
  # One call, both greps - two separate calls would be twice the round trips
  # and could observe two different instants.
  CTRL_LIST="$(rosrun controller_manager controller_manager list 2>/dev/null || true)"
  if grep -q 'husky_joint_publisher.*( running )' <<< "$CTRL_LIST" \
     && grep -q 'husky_velocity_controller.*( running )' <<< "$CTRL_LIST"; then
    CONTROLLERS_READY=1
    echo " ready."
    break
  fi
  echo -n "."
  sleep 1
done

if [ "$CONTROLLERS_READY" -ne 1 ]; then
  cat >&2 <<EOF

!! ---------------------------------------------------------------------- !!
!! WARNING: the Husky controllers are NOT both ( running ).
!!
!! Last seen state:
${CTRL_LIST:-  <controller_manager did not answer at all>}
!!
!! DIAGNOSIS: husky_velocity_controller stuck in \`initialized\` rather than
!! \`running\` means the controller spawner died part-way. The robot will look
!! completely fine in Gazebo but will SILENTLY IGNORE every cmd_vel message -
!! teleop will appear dead with no error printed anywhere.
!!
!! The usual cause is a second simulation already running: duplicate node
!! names make the master kill robot_state_publisher and the spawner dies with
!! it. Check for a leftover roslaunch/gzserver from an earlier run, stop it,
!! and re-run this script.
!!
!! Re-check by hand with:
!!   rosrun controller_manager controller_manager list
!!
!! The simulation is being left up anyway, in case you just want the GUI.
!! ---------------------------------------------------------------------- !!

EOF
else
  # ---------------------------------------------------------------------
  # Teleop hint.
  #
  # CLAUDE.md says to drive via /kb_teleop/cmd_vel, because Husky runs
  # twist_mux (which arbitrates inputs by priority) and /kb_teleop/cmd_vel is
  # the keyboard slot. That is true of the STOCK husky_control config.
  #
  # BUT: this overlay ships its own twist_mux config at
  # natural_environments_ros_opt/husky/husky_control/config/twist_mux.yaml,
  # and it has only THREE input slots -
  #     joy                 joy_teleop/cmd_vel           priority 10
  #     interactive_marker  twist_marker_server/cmd_vel  priority 8
  #     external            cmd_vel                      priority 1
  # There is NO kb_teleop slot. Publishing to /kb_teleop/cmd_vel here reaches
  # zero subscribers and the robot never moves. The overlay's `external` slot,
  # /cmd_vel, is the one to use. Both are printed, most-likely-to-work first,
  # so the discrepancy is visible rather than silently papered over.
  # ---------------------------------------------------------------------
  cat <<EOF

--------------------------------------------------------------------------
Husky is up. To drive it, open a SECOND terminal. It needs this script's ROS
environment, so source it there first:

  source /opt/ros/noetic/setup.bash

then:

  rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/cmd_vel

Keys: i / , forward-back, j / l turn, k stop, q / z speed.

NOTE ON THE TOPIC: CLAUDE.md documents /kb_teleop/cmd_vel as the keyboard slot
of Husky's twist_mux, and that is correct for the STOCK husky_control config:

  rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/kb_teleop/cmd_vel

This run uses the natural_environments_ros_opt overlay's twist_mux.yaml, whose
only slots are joy_teleop/cmd_vel, twist_marker_server/cmd_vel and cmd_vel -
no kb_teleop. So /cmd_vel is the working one here. If the robot does not move,
try the other topic before assuming anything is broken.
--------------------------------------------------------------------------

EOF
fi

# Wait on the robot stage. The world stage is torn down by the trap on exit,
# in the correct order (robot first, then world).
wait "$ROBOT_PID" || true
