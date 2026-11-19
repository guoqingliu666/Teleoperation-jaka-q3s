#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$ROOT"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export PYTHONDONTWRITEBYTECODE=1 TMPDIR="$ROOT/.tmp" XDG_CACHE_HOME="$ROOT/.cache"
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME/operator" artifacts/operator_sessions
bash tools/build_dh116.sh > artifacts/operator_hand_build.log 2>&1 || echo 'DH116 组件构建失败，详情见 artifacts/operator_hand_build.log。' >&2
if [[ ! -x .cache/operator/arm_build/v3_arm_power || tools/operator_arm_power.cpp -nt .cache/operator/arm_build/v3_arm_power || vendor/jaka/include/jakaAPI.h -nt .cache/operator/arm_build/v3_arm_power ]]; then
  bash tools/build_operator_arm.sh > artifacts/operator_arm_build.log 2>&1 || echo '机器人电源组件构建失败，详情见 artifacts/operator_arm_build.log。' >&2
fi
if [[ ! -x .cache/operator/scene_build/v3_operator_scene || ! -f .cache/operator/display.urdf || tools/operator_scene/main.cpp -nt .cache/operator/scene_build/v3_operator_scene || tools/operator_scene/target_markers.hpp -nt .cache/operator/scene_build/v3_operator_scene || tools/operator_scene/target_overlay.hpp -nt .cache/operator/scene_build/v3_operator_scene ]]; then
  if ! bash tools/build_operator_display.sh > artifacts/operator_display_setup.log 2>&1; then
    echo '3D 显示组件准备失败，详情见 artifacts/operator_display_setup.log；仍可打开界面。' >&2
  fi
fi
# Rebuild the display URDF from the active control model on every start.
/usr/bin/python3 -B tools/operator_model.py
exec /usr/bin/python3 -B tools/operator_panel.py "$@"
