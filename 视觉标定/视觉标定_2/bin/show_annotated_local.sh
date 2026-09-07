#!/usr/bin/env bash
set -euo pipefail
TOPIC=${1:-/cuadc/calib/annotated}
set +u
source /opt/ros/humble/setup.bash
set -u
if command -v rqt_image_view >/dev/null 2>&1; then
  echo "Opening rqt_image_view; select topic: $TOPIC"
  exec rqt_image_view "$TOPIC"
fi
echo "rqt_image_view not found. Install: sudo apt install ros-humble-rqt-image-view" >&2
exit 2
