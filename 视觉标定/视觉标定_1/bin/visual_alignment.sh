#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
PARAMS="$ROOT/config/alignment_generated.yaml"
INSTALL="$ROOT/ros2_ws/install"
if [[ ! -r "$PARAMS" ]]; then
  echo "Missing $PARAMS; run: python3 tools/generate_config.py" >&2
  exit 2
fi
if [[ ! -r /opt/ros/humble/setup.bash || ! -r "$INSTALL/setup.bash" ]]; then
  echo "ROS 2 workspace is not built; run ./bin/build.sh" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$INSTALL/setup.bash"
STAMP=$(date +%Y%m%d_%H%M%S)
OUT="$ROOT/logs/static_alignment_$STAMP"
mkdir -p "$OUT/ros"
export ROS_LOG_DIR="$OUT/ros"
echo "Alignment log: $OUT"
echo "NOTE: this script starts only the viewer. Start your camera + vision provider first."
exec ros2 run vision_servo_calibration alignment_viewer --ros-args \
  --params-file "$PARAMS" \
  -p detection_csv_path:="$OUT/detections.csv" \
  -p video_path:="$OUT/annotated.avi"
