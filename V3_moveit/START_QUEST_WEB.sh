#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
if [[ $# -ne 1 || ! "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo '用法：bash START_QUEST_WEB.sh 本机连接Quest所在网络的IPv4地址'; exit 2
fi
mkdir -p .cache/quest-tls
export PYTHONDONTWRITEBYTECODE=1
CERT="$ROOT/.cache/quest-tls/$1.crt"
KEY="$ROOT/.cache/quest-tls/$1.key"
if [[ ! -f "$CERT" ]]; then
  openssl req -x509 -newkey rsa:2048 -nodes -days 30 -keyout "$KEY" -out "$CERT" \
    -subj "/CN=$1" -addext "subjectAltName=IP:$1"
  chmod 600 "$KEY"
fi
echo "在 Quest 浏览器打开 https://$1:8443 ，核对地址后接受本地证书。"
if [[ ! -s .cache/operator/display.urdf ]]; then
  /usr/bin/python3 -B tools/operator_model.py
fi
exec /usr/bin/python3 -B tools/operator_xr/server.py --cert "$CERT" --key "$KEY"
