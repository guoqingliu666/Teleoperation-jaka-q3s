#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"
# Manufacturer source snapshot with the small feedback-only adaptations.
bash tools/operator_ros.sh colcon --log-base .cache/operator/dh116_driver/log build \
  --base-paths vendor/dh116/hand_control_service \
  --build-base .cache/operator/dh116_driver/build \
  --install-base .cache/operator/dh116_driver/install \
  --packages-select hand_control_service --event-handlers console_direct+ \
  --cmake-args -DBUILD_TESTING=OFF -DEC_PRIMARY_MAC=02:01:01:01:01:01 \
  -DHAND_CONTROL_THIRDPARTY_DIR="$ROOT/vendor/dh116/sdk_20261031" \
  -UHAND_CONTROL_SDK_LIB
