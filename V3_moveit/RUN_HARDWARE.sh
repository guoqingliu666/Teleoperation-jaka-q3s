#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$ROOT"
if [[ "${1:-}" != --hardware ]]; then
  echo '实机入口：bash RUN_HARDWARE.sh --hardware [runtime_config:=/完整路径/site.yaml] [rviz:=true|false]'; exit 2
fi
shift
/usr/bin/python3 -B - <<'PY_RT'
import resource
if resource.getrlimit(resource.RLIMIT_RTPRIO)[0]<50:
    raise SystemExit('本次会话缺少实时权限。请使用 START_OPERATOR.sh，在设备连接页启用本次实时权限。')
PY_RT
mkdir -p .cache/operator
exec 9>.cache/operator/control.lock
flock -n 9 || { echo '已有本机控制会话运行。' >&2; exit 1; }
python3 -B - <<'PY'
from pathlib import Path
import yaml
root=Path.cwd();cfg=yaml.safe_load((root/'site_ubuntu.yaml').read_text());cfg['auto_replan']=False
(root/'.cache/operator/runtime.yaml').write_text(yaml.safe_dump(cfg))
PY
RUNTIME="$ROOT/.cache/operator/runtime.yaml"
RVIZ=true
for ARG in "$@"; do
  case "$ARG" in
    runtime_config:=*) RUNTIME="${ARG#runtime_config:=}" ;;
    rviz:=true) RVIZ=true ;;
    rviz:=false) RVIZ=false ;;
    *) echo "不支持的参数：$ARG"; exit 2 ;;
  esac
done
python3 -B tools/check_hardware_readiness.py --runtime-config "$RUNTIME"
exec bash tools/operator_ros.sh ros2 launch --noninteractive v3_moveit bringup.launch.py mode:=hardware "runtime_config:=$RUNTIME" "payload_config:=$ROOT/src/v3_moveit/config/payload.json" "rviz:=$RVIZ" sigterm_timeout:=86400 sigkill_timeout:=86400
