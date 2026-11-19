#!/usr/bin/env bash
# One-time, narrowly scoped raw Ethernet access. No stored password or sudoers rule.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
if [[ $EUID -ne 0 ]]; then
  echo '请使用 sudo bash tools/setup_dh116_access.sh 安装 DH116 网口权限。' >&2
  exit 1
fi
for name in v3_dh116_grasp v3_dh116_feedback; do
  binary="$ROOT/.cache/dh116/build/$name"
  [[ -f "$binary" && ! -L "$binary" && -x "$binary" ]] || { echo "先运行 tools/build_dh116.sh：$name 未构建" >&2; exit 1; }
done
for name in v3_dh116_grasp v3_dh116_feedback; do
  /usr/sbin/setcap cap_net_raw=ep "$ROOT/.cache/dh116/build/$name"
  /usr/sbin/getcap "$ROOT/.cache/dh116/build/$name"
done
