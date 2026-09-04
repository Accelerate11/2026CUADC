#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PARAMS="$ROOT/config/alignment_generated.yaml"
INSTALL="$ROOT/ros2_ws/install"
source /opt/ros/humble/setup.bash
source "$INSTALL/setup.bash"
exec ros2 run vision_servo_calibration vision_provider_template --ros-args --params-file "$PARAMS"
