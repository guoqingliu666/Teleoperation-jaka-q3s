#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export TMPDIR="$ROOT/.tmp" ROS_LOG_DIR="$ROOT/log/ros" XDG_CACHE_HOME="$ROOT/.cache" PYTHONDONTWRITEBYTECODE=1
export ROS_HOME="$ROOT/.ros" COLCON_HOME="$ROOT/.colcon"
mkdir -p "$ROS_HOME" "$COLCON_HOME"
mkdir -p "$TMPDIR" "$ROS_LOG_DIR" "$XDG_CACHE_HOME"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
exec ros2 launch v3_moveit bringup.launch.py mode:=fake "$@"
