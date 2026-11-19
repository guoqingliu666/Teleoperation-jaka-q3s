#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
export PATH=/usr/bin:/bin PYTHONDONTWRITEBYTECODE=1 LD_BIND_NOW=1
export ROS_DOMAIN_ID=90 ROS_LOCALHOST_ONLY=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_STATIC_PEERS=
export TMPDIR="$ROOT/.tmp" ROS_LOG_DIR="$ROOT/log/ros" XDG_CACHE_HOME="$ROOT/.cache"
export ROS_HOME="$ROOT/.ros" COLCON_HOME="$ROOT/.colcon"
export V3_PAYLOAD_CONFIG="$ROOT/src/v3_moveit/config/payload.json"
mkdir -p "$TMPDIR" "$ROS_LOG_DIR" "$XDG_CACHE_HOME" "$ROS_HOME" "$COLCON_HOME"
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
if [[ -f "$ROOT/.cache/operator/hand_ros/install/setup.bash" ]]; then source "$ROOT/.cache/operator/hand_ros/install/setup.bash"; fi
set -u
exec "$@"
