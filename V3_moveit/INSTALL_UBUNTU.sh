#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"
export PATH=/usr/bin:/bin
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
export PYTHONDONTWRITEBYTECODE=1
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export MAKEFLAGS="${MAKEFLAGS:--j2}"
export TMPDIR="$ROOT/.tmp" XDG_CACHE_HOME="$ROOT/.cache" PIP_CACHE_DIR="$ROOT/.cache/pip"
export ROS_LOG_DIR="$ROOT/log/ros" PYTHONDONTWRITEBYTECODE=1
export ROS_HOME="$ROOT/.ros" COLCON_HOME="$ROOT/.colcon"
mkdir -p "$ROS_HOME" "$COLCON_HOME"
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME" "$ROS_LOG_DIR" artifacts
. /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || { echo '需要 Ubuntu 24.04。'; exit 1; }
[[ "$(uname -m)" == x86_64 ]] || { echo '当前 SDK 是 x86_64，不能混用 ARM 库。'; exit 1; }
[[ -w "$ROOT" ]] || { echo '工程所在分区不可写；先处理挂载状态。'; exit 1; }
if [[ ! -f /opt/ros/jazzy/setup.bash && ! -f /usr/share/ros-apt-source/ros2.sources && "$(dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null || true)" != *' ok installed' ]]; then
  sudo apt-get update
  sudo apt-get install -y curl gnupg software-properties-common
  sudo add-apt-repository -y universe
  sudo dpkg -i "$ROOT/vendor/ubuntu/ros2-apt-source_1.2.0.noble_all.deb"
fi
packages=(
  build-essential cmake openssl python3-aiohttp python3-colcon-common-extensions
  qtbase5-dev python3-gi python3-pip libcap2-bin pkexec ros-jazzy-rosidl-default-generators
  python3-numpy python3-scipy python3-yaml python3-pytest python3-pyqt5 python3-pyqt5.qtsvg python3-opencv python3-venv zstd curl adb
  ros-jazzy-ros-base ros-jazzy-moveit ros-jazzy-moveit-servo
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ament-cmake-python
  ros-jazzy-robot-state-publisher ros-jazzy-xacro ros-jazzy-rviz2
)
missing_packages=()
for package in "${packages[@]}"; do
  if [[ "$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)" != *' ok installed' ]]; then
    missing_packages+=("$package")
  fi
done
if ((${#missing_packages[@]})); then
  sudo apt-get update
  sudo apt-get install -y "${missing_packages[@]}"
else
  printf '系统依赖已齐全，跳过 apt 安装，继续检查和编译。\n'
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
ldd vendor/jaka/lib/libjakaAPI.so | tee artifacts/sdk_dependencies.txt
if grep -q 'not found' artifacts/sdk_dependencies.txt; then echo 'SDK 系统依赖缺失，详见 artifacts/sdk_dependencies.txt'; exit 1; fi
python3 -B tools/verify_offline.py
# Vendor reference packages are compiled separately under the project's cache.
colcon build --base-paths src --executor sequential --event-handlers console_direct+
python3 -m pip install --disable-pip-version-check --no-index --no-deps --find-links vendor/python-wheels \
  --target "$ROOT/.cache/operator/python" --upgrade -r tools/requirements_operator_visual.lock
bash tools/build_dh116.sh
bash tools/build_operator_arm.sh
bash tools/build_operator_display.sh
bash tools/build_dh116_driver.sh
python3 tools/install_operator_shortcut.py
dpkg-query -W 'ros-jazzy-moveit*' 'ros-jazzy-joint-trajectory-controller' 'ros-jazzy-controller-manager' > artifacts/installed_versions.txt
printf '\n安装和编译完成。运行 bash START_OPERATOR.sh 打开 V3 上位机。\n'
