#!/usr/bin/env bash
set -euo pipefail
pkill -TERM -f 'vision_servo_calibration.*alignment_viewer' 2>/dev/null || true
pkill -TERM -f web_video_server 2>/dev/null || true
echo "Stopped alignment viewer/web helpers. Formal vision node was NOT killed."
