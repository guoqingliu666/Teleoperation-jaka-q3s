#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export ROS_LOG_DIR="$ROOT/log/ros" PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
exec ros2 run v3_moveit synthetic_quest
