#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
export TMPDIR="$ROOT/.tmp" ROS_LOG_DIR="$ROOT/log/ros" XDG_CACHE_HOME="$ROOT/.cache" PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=73
export ROS_HOME="$ROOT/.ros" COLCON_HOME="$ROOT/.colcon"
mkdir -p "$ROS_HOME" "$COLCON_HOME"
mkdir -p "$TMPDIR" "$ROS_LOG_DIR" "$XDG_CACHE_HOME" artifacts
set +u
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
set -u
python3 -B - <<'PY_INVALIDATE'
import json
from pathlib import Path
Path('artifacts/ros_smoke_pass.json').write_text(json.dumps(dict(passed=False,reason='full verification started')))
PY_INVALIDATE
# Resolve the installed plugin and all relocations without creating hardware or calling SDK APIs.
PLUGIN="$ROOT/install/v3_moveit/lib/libv3_jaka_system.so"
if ! ldd -r "$PLUGIN" > artifacts/hardware_plugin_dependencies.txt 2>&1; then
  cat artifacts/hardware_plugin_dependencies.txt
  exit 1
fi
if grep -Eq 'not found|undefined symbol:' artifacts/hardware_plugin_dependencies.txt; then
  cat artifacts/hardware_plugin_dependencies.txt
  exit 1
fi
python3 -B - "$PLUGIN" <<'PY_LOAD'
import ctypes
import os
import sys
ctypes.CDLL(sys.argv[1], mode=os.RTLD_NOW | os.RTLD_LOCAL)
print('PASS: hardware plugin loaded with all symbols resolved; no SDK functions called')
PY_LOAD
python3 -B tools/verify_offline.py
g++ -std=c++17 -Wall -Wextra -I src/v3_moveit/include -I vendor/jaka/include tools/guard_tests.cpp -o artifacts/guard_tests
./artifacts/guard_tests
python3 -B tools/supervisor_tests.py
cmake --build build/v3_moveit --target v3_path_retime_tests v3_stop_envelope_tests v3_servo_interval_tests v3_edg_feedback_tests v3_controller_activation_tests --parallel 2
./build/v3_moveit/v3_path_retime_tests
./build/v3_moveit/v3_stop_envelope_tests
./build/v3_moveit/v3_servo_interval_tests
./build/v3_moveit/v3_edg_feedback_tests
./build/v3_moveit/v3_controller_activation_tests src/v3_moveit/description/s5.urdf
python3 -B tools/hardware_lifecycle_tests.py
python3 -B tools/smooth_controller_ros_test.py
python3 -B tools/ros_smoke.py
python3 -B tools/fixed_servo_ros_test.py
python3 -B tools/check_ros_pass.py
