#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"
export ZED_DIR="$ROOT/.cache/operator/zed_sdk_unpacked"
export ZED_SDK_RESOURCES_DIR="$ZED_DIR"
export LD_LIBRARY_PATH="$ZED_DIR/lib:$ZED_DIR/dependencies/ai_dependencies/lib"
export TMPDIR="$ROOT/.tmp" XDG_CACHE_HOME="$ROOT/.cache" CUDA_CACHE_PATH="$ROOT/.cache/cuda" PYTHONDONTWRITEBYTECODE=1 PYTHONUTF8=1
unset PYTHONPATH
mkdir -p "$ZED_DIR/resources" "$ZED_DIR/settings" "$CUDA_CACHE_PATH"
exec "$ROOT/.venv-zed/bin/python" -B "$@"
