#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PARAMS="$ROOT/config/alignment_generated.yaml"
INSTALL="$ROOT/ros2_ws/install"
if [[ ! -r "$PARAMS" ]]; then
  echo "Missing $PARAMS; run generate_config.py first" >&2
  exit 2
fi
if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$INSTALL/setup.bash" ]]; then
  echo "Viewer workspace not built; run ./bin/build.sh" >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "$INSTALL/setup.bash"
set -u
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="$ROOT/logs/display_alignment_$STAMP"
mkdir -p "$OUT/ros"
export ROS_LOG_DIR="$OUT/ros"
echo "Display alignment log: $OUT"
exec ros2 run vision_servo_calibration alignment_viewer --ros-args \
  --params-file "$PARAMS" \
  -p live_view_enabled:=true \
  -p annotated_image_topic:=/vision_servo/alignment/image \
  -p detection_csv_path:="$OUT/detections.csv" \
  -p video_path:="$OUT/annotated.avi"
