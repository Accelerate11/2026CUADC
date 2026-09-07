#!/usr/bin/env bash
set -euo pipefail
PORT=${1:-8080}
if ss -lnt 2>/dev/null | awk '{print $4}' | grep -Eq "(^|:)$PORT$"; then
  echo "Port $PORT is already in use. If an existing web_video_server is running, reuse it."
  pgrep -af web_video_server || true
  echo "Direct overlay URL: http://127.0.0.1:${PORT}/stream?topic=/cuadc/calib/annotated&type=mjpeg"
  echo "Independent viewer URL: http://127.0.0.1:${PORT}/stream?topic=/vision_servo/alignment/image&type=mjpeg"
  exit 0
fi
set +u
source /opt/ros/humble/setup.bash
set -u
echo "Starting web_video_server on port $PORT"
echo "Windows CMD tunnel: ssh -L ${PORT}:127.0.0.1:${PORT} cuadc@NUC_IP"
exec ros2 run web_video_server web_video_server --ros-args -p port:="$PORT"
