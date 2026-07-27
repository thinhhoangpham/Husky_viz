#!/usr/bin/env bash
set -euo pipefail

# Docker build context / compose files live on the internal SSD, not the project folder.
DOCKER_DIR="$HOME/husky-docker"

# Husky runs twist_mux, which arbitrates cmd_vel inputs by priority.
# /kb_teleop/cmd_vel is the keyboard slot; publishing to bare /cmd_vel can be overridden.
CMD_VEL_TOPIC="/kb_teleop/cmd_vel"

MODE="keyboard"

usage() {
  cat <<EOF
Usage: teleop.sh [--gui|--wasd] [-h|--help]

  (no arguments)  keyboard teleop in this terminal (stock teleop_twist_keyboard)
  --wasd          custom WASD teleop node (husky_teleop.py) in this terminal
  --gui           rqt_robot_steering window in the noVNC session
EOF
}

for arg in "$@"; do
  case "$arg" in
    --gui) MODE="gui" ;;
    --wasd) MODE="wasd" ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Error: unknown argument '$arg'." >&2
      usage >&2
      exit 1
      ;;
  esac
done

cd "$DOCKER_DIR"

if [ "$MODE" = "gui" ]; then
  echo "Launching rqt_robot_steering (Ctrl-C to stop)..."
  echo "The window appears in the noVNC session: http://localhost:6080/vnc.html"
  echo "Set the topic field to $CMD_VEL_TOPIC - the widget does not default to it."
  echo ""
  exec docker compose exec husky bash -lc 'source /opt/ros/noetic/setup.bash && export DISPLAY=:1 && rosrun rqt_robot_steering rqt_robot_steering'
fi

if [ "$MODE" = "wasd" ]; then
  echo "Custom WASD teleop on $CMD_VEL_TOPIC (q or Ctrl-C to stop)."
  echo "The node prints its own key map on startup."
  echo ""
  echo "Keep this terminal focused - keys only register in the focused window."
  echo ""
  # No -T: the node reads single keypresses and needs a TTY.
  exec docker compose exec husky bash -lc 'source /opt/ros/noetic/setup.bash && python3 "/workspace/husky_teleop.py"'
fi

echo "Keyboard teleop on $CMD_VEL_TOPIC (Ctrl-C to stop)."
echo ""
echo "  i / ,   forward / back"
echo "  j / l   turn left / right"
echo "  k       stop"
echo "  q / z   increase / decrease speed"
echo ""
echo "Keep this terminal focused - keys only register in the focused window."
echo ""

exec docker compose exec husky bash -lc 'source /opt/ros/noetic/setup.bash && rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/kb_teleop/cmd_vel'
