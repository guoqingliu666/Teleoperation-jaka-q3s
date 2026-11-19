#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
mkdir -p "$ROOT/.cache/operator/arm_build" "$ROOT/.tmp"
export TMPDIR="$ROOT/.tmp"
g++ -std=c++17 -O2 -Wall -Wextra -pthread -I "$ROOT/src/v3_moveit/include" -I "$ROOT/vendor/jaka/include" \
  "$ROOT/tools/operator_arm_power.cpp" -L "$ROOT/vendor/jaka/lib" -Wl,-rpath,"$ROOT/vendor/jaka/lib" -ljakaAPI -o "$ROOT/.cache/operator/arm_build/v3_arm_power"
