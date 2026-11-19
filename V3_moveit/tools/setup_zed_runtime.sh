#!/usr/bin/env bash
# Install the ZED Mini camera environment in this project on Ubuntu 24.04 amd64.
set -euo pipefail
root="$(cd -- "$(dirname -- "$0")/.." && pwd)"
cd "$root"
sdk="$root/.cache/operator/zed_sdk_unpacked"
cache="$root/.cache/zed-install"
mkdir -p "$cache" "$root/.cache/operator"
url=https://download.stereolabs.com/zedsdk/5.5.1/cu12_trt10/ubuntu24
if [[ ! -s "$sdk/lib/libsl_zed.so" ]]; then
  installer="${1:-$cache/ZED_SDK_Ubuntu24_cuda12.8_tensorrt10.9_v5.5.1.zstd.run}"
  if [[ ! -f "$installer" && $# == 0 ]]; then
    curl --fail --location --retry 3 --continue-at - "$url" -o "$installer.part"
    mv -- "$installer.part" "$installer"
  fi
  [[ -f "$installer" ]] || { echo "找不到 ZED SDK 安装包：$installer" >&2; exit 1; }
  printf '%s  %s\n' 0c282d59630cb14b2171b8a92c8ed5de09943c3e05d569782126f1adf26ef87f "$installer" | sha256sum -c -
  temp="$(mktemp -d "$cache/extract.XXXXXX")"
  bash "$installer" --nox11 --noexec --target "$temp"
  grep -Fq 'set(PACKAGE_VERSION "5.5.1")' "$temp/zed-config-version.cmake"
  [[ -s "$temp/lib/libsl_zed.so" ]] || { echo 'SDK 安装包缺少 libsl_zed.so' >&2; exit 1; }
  mkdir -p "$sdk"
  cp -a "$temp/." "$sdk/"
  rm -rf -- "$temp"
fi
grep -Fq 'set(PACKAGE_VERSION "5.5.1")' "$sdk/zed-config-version.cmake"
mkdir -p "$sdk/resources" "$sdk/settings"
cp -n "$root"/config/zed/settings/*.conf "$sdk/settings/"
/usr/bin/python3 -m venv "$root/.venv-zed"
"$root/.venv-zed/bin/python" -m pip install -r tools/requirements_zed.lock
bash tools/operator_zed.sh -c 'import cv2, pyzed.sl as sl; print("ZED SDK", sl.Camera.get_sdk_version())'
