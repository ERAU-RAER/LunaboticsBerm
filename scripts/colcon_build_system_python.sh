#!/usr/bin/env bash
# Helper to force colcon to use the system /usr/bin/python3 interpreter.

set -euo pipefail

# Ensure system python3 is found first on PATH.
export PATH="/usr/bin:${PATH}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # Temporarily relax nounset; ROS setup expects unset vars.
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "ROS 2 Humble not found at /opt/ros/humble. Please install/sourcing manually." >&2
  exit 1
fi

export COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
echo "Using python3 at $(command -v python3)"
echo "Using COLCON_PYTHON_EXECUTABLE=${COLCON_PYTHON_EXECUTABLE}"

colcon build "$@"
