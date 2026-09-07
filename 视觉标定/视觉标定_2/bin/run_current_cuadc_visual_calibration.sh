#!/usr/bin/env bash
set -euo pipefail
WS=${1:-$HOME/cuadc_mission}
MODEL="$WS/src/cuadc_visual_drop_flight/models/basket-v3.pt"
PARAMS="$WS/src/cuadc_visual_drop_flight/config/flight_params.yaml"
if [[ ! -r "$PARAMS" || ! -r "$MODEL" ]]; then
  echo "Invalid CUADC workspace: $WS" >&2
  exit 2
fi
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
exec ros2 run cuadc_visual_drop_flight drop_bucket_realsense_node.py --ros-args \
  --params-file "$PARAMS" \
  -p camera_xy_calib_bias_m:="[0.047805,-0.014577]" \
  -p camera_optical_to_body_rotation:="[-0.971358,0.237622,0.0,0.237622,0.971358,0.0,0.0,0.0,-1.0]" \
  -p calibration_release_offsets_body_m:="[0.026,-0.065,-0.320,0.0109,0.0720,-0.320]" \
  -p color_auto_exposure:=true \
  -p calibration_overlay_enabled:=true \
  -p calibration_overlay_width:=960 \
  -p record_annotated_video:=false \
  -p model_path:="$MODEL"
