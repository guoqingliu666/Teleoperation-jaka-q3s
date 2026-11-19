#!/usr/bin/env bash
# Invoked in the clean project ROS environment, as the user with CAP_NET_RAW.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"
set +u
source "$ROOT/.cache/operator/dh116_driver/install/local_setup.bash"
set -u
export ROS_DOMAIN_ID=0
exec "$ROOT/.cache/operator/dh116_driver/install/hand_control_service/lib/hand_control_service/hand_control_service" \
  --ros-args -r __ns:=/hand_control --params-file "$ROOT/config/dh116_ethercat.yaml"
