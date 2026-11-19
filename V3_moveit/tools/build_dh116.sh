#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$ROOT"
export TMPDIR="$ROOT/.tmp" XDG_CACHE_HOME="$ROOT/.cache"
cmake -S tools/dh116_native -B .cache/dh116/build
cmake --build .cache/dh116/build --parallel 2
