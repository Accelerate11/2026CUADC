#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble not found at /opt/ros/humble/setup.bash" >&2
  exit 2
fi
# ROS_SETUP_NOUNSET_GUARD
set +u
source /opt/ros/humble/setup.bash
set -u
cd "$ROOT/ros2_ws"
colcon build --symlink-install --packages-select vision_servo_calibration
