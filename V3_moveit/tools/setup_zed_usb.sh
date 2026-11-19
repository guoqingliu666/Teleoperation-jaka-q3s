#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd)"
sudo install -m 0644 "$ROOT/tools/70-v3-zed-mini.rules" /etc/udev/rules.d/70-v3-zed-mini.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=usb --attr-match=idVendor=2b03
printf 'ZED Mini USB permissions installed. Reconnect the camera if SDK detection is still unavailable.\n'
