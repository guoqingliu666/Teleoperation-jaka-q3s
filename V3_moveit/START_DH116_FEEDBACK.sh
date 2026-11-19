#!/usr/bin/env bash
# Establishes EtherCAT OP and reads feedback. No motor command services.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$ROOT"
if [[ "${1:-}" != --hardware || $# != 1 ]]; then
  echo '用法：bash START_DH116_FEEDBACK.sh --hardware'
  echo '连接 DH116 EtherCAT 反馈；不使能、回零或移动手指。Ctrl+C 断开。'
  exit 2
fi
[[ -x .cache/operator/dh116_driver/install/hand_control_service/lib/hand_control_service/hand_control_service ]] || {
  echo '先运行 bash tools/build_dh116_driver.sh'; exit 1;
}
mkdir -p .cache/operator
exec 9>.cache/operator/dh116.lock
flock -n 9 || { echo 'DH116 反馈连接已在运行。'; exit 1; }
DH116_UID="$(id -u)"
DH116_GID="$(id -g)"
[[ "$DH116_UID" != 0 ]] || { echo '请以桌面用户运行；需要时会弹出系统认证。'; exit 1; }
echo '连接现有 enp11s0 / 交换机。仅临时授予此进程原始网卡访问；不修改 IP 或路由。'
child=''
requested_stop=0
stop_code=0
stop_child() {
  requested_stop=1
  if [[ -n "$child" ]]; then
    kill -INT "$child" 2>/dev/null || true
    wait "$child" || stop_code=$?
  fi
}
trap stop_child INT TERM
pkexec /usr/bin/setpriv --reuid="$DH116_UID" --regid="$DH116_GID" --init-groups \
  --inh-caps=-all,+net_raw --ambient-caps=-all,+net_raw --bounding-set=-all,+net_raw \
  /usr/bin/bash "$ROOT/tools/operator_ros.sh" /usr/bin/bash "$ROOT/tools/dh116_driver_node.sh" &
child=$!
set +e
wait "$child"
code=$?
set -e
if [[ "$requested_stop" == 1 ]]; then code="$stop_code"; fi
trap - INT TERM
exit "$code"
