#!/usr/bin/env bash
# Display-only build. No control process or vendor driver is started.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"
export TMPDIR="$ROOT/.tmp" XDG_CACHE_HOME="$ROOT/.cache" PYTHONDONTWRITEBYTECODE=1
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME/operator" artifacts
python3 -B tools/operator_model.py
bash tools/operator_ros.sh cmake -S tools/operator_scene -B .cache/operator/scene_build
bash tools/operator_ros.sh cmake --build .cache/operator/scene_build --parallel 2
HAND_INTERFACES="$ROOT/vendor/dh116/hand_control_interfaces"
if [[ ! -f .cache/operator/hand_ros/install/setup.bash ]]; then
  bash tools/operator_ros.sh colcon --log-base .cache/operator/hand_ros/log build --base-paths "$HAND_INTERFACES" --build-base .cache/operator/hand_ros/build --install-base .cache/operator/hand_ros/install --event-handlers console_direct+
fi
