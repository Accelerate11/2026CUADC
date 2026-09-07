#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
TOPICS=$(ros2 topic list || true)
for t in \
  /perception/drop_buckets_body \
  /cuadc/calib/annotated \
  /cuadc/calib/color/image_raw \
  /cuadc/calib/color/camera_info \
  /vision_servo/targets_body \
  /vision_servo/alignment/image
 do
  if grep -qx "$t" <<<"$TOPICS"; then
    echo "[OK] $t"
  else
    echo "[--] $t"
  fi
done
