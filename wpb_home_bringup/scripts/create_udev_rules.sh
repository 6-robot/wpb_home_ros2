#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for rule in ftdi.rules rplidar.rules 90-kinect2.rules; do
  sudo install -m 0644 "$script_dir/$rule" "/etc/udev/rules.d/$rule"
done
sudo udevadm control --reload-rules
sudo udevadm trigger
echo '设备规则已安装。请重新插拔底盘、雷达和 Kinect2，再检查 /dev/ftdi、/dev/rplidar 以及 Kinect2 枚举结果。'
