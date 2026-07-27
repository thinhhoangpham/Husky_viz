#!/usr/bin/env bash
set -euo pipefail

# Docker build context / compose files live on the internal SSD, not the project folder.
DOCKER_DIR="$HOME/husky-docker"

# Host path bind-mounted into the container at /workspace (note the spaces).
PROJECT_DIR="/Volumes/Extreme Pro/Husky viz"
HUSKY_CONTAINER="husky-docker-husky-1"
PROBE_IMAGE="alpine:latest"

cd "$DOCKER_DIR"

# Fail early with a clear message if the Docker daemon isn't reachable.
if ! docker info >/dev/null 2>&1; then
  echo "Error: cannot talk to the Docker daemon. Is Docker Desktop running?" >&2
  exit 1
fi

# --- 1. Stop the simulation cleanly inside the container (if it is running) ---
if [ "$(docker inspect -f '{{.State.Running}}' "$HUSKY_CONTAINER" 2>/dev/null || echo false)" = "true" ]; then
  echo "Stopping the simulation inside $HUSKY_CONTAINER ..."
  # SIGINT first so Gazebo can flush and exit; ignore failures if nothing matches.
  docker exec "$HUSKY_CONTAINER" bash -lc \
    'pkill -INT -f roslaunch; pkill -INT -f gzclient; pkill -INT -f gzserver; pkill -INT -f spawner; true' \
    >/dev/null 2>&1 || true
  for _ in $(seq 1 15); do
    if ! docker exec "$HUSKY_CONTAINER" bash -lc 'pgrep -f "gzserver|gzclient|roslaunch" >/dev/null' >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
  docker exec "$HUSKY_CONTAINER" bash -lc \
    'pkill -TERM -f gzserver; pkill -TERM -f gzclient; true' >/dev/null 2>&1 || true
  echo "Simulation processes stopped."
else
  echo "Container $HUSKY_CONTAINER is not running - nothing to stop inside it."
fi

# --- 2. Record other running containers, which a Docker restart would take down ---
OTHERS="$(docker ps --format '{{.Names}}' | grep -v "^${HUSKY_CONTAINER}$" || true)"

# --- 3. Tear down ---
echo "Running 'docker compose down' in $DOCKER_DIR ..."
docker compose down

# --- 4. Probe whether the bind mount can still be re-created ---
# A throwaway --rm container mounting the exact same host path reproduces the
# daemon-side mount that 'docker compose up' would make, without starting the sim.
PROBE_STATE="skipped"
if docker image inspect "$PROBE_IMAGE" >/dev/null 2>&1; then
  echo "Probing the bind mount for $PROJECT_DIR ..."
  if PROBE_ERR="$(docker run --rm --pull=never -v "$PROJECT_DIR":/probe "$PROBE_IMAGE" true 2>&1)"; then
    PROBE_STATE="ok"
  else
    PROBE_STATE="stale"
  fi
else
  PROBE_ERR="local image $PROBE_IMAGE not present"
fi

# --- 5. Report ---
echo ""
case "$PROBE_STATE" in
  ok)
    echo "Shutdown clean. The bind mount re-created fine - start-sim.sh will work normally."
    ;;
  stale)
    echo "**********************************************************************"
    echo "WARNING: the bind mount for \"$PROJECT_DIR\" is STALE in the Docker daemon."
    echo "start-sim.sh will FAIL until Docker Desktop is restarted."
    echo ""
    echo "Probe error was:"
    echo "  $PROBE_ERR"
    echo ""
    echo "To fix:"
    echo "  1. Docker Desktop menu bar icon -> Restart  (or: quit and reopen Docker Desktop)"
    echo "  2. Wait for the whale icon to stop animating"
    echo "  3. Re-run \"$PROJECT_DIR/start-sim.sh\""
    echo ""
    echo "Note: 'docker compose rm -f' does NOT clear this - the stale entry lives"
    echo "in the daemon, not in the container definition."
    echo "**********************************************************************"
    ;;
  *)
    echo "Mount probe SKIPPED ($PROBE_ERR); mount health is unknown."
    echo "If start-sim.sh fails with 'error while creating mount source path', restart Docker Desktop."
    ;;
esac

if [ -n "$OTHERS" ]; then
  echo ""
  echo "Other containers that were running (a Docker Desktop restart stops these,"
  echo "and they may not come back automatically):"
  while IFS= read -r c; do
    [ -n "$c" ] && echo "  - $c"
  done <<< "$OTHERS"
fi
