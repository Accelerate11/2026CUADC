#!/usr/bin/env bash
set -eo pipefail

pkill -f "gz sim .*hazard_recognition_single" || true
pkill -f "ros2 launch cuadc_hazard_recognition_sim hazard_recognition.launch.py" || true
echo "Stopped hazard recognition simulation processes if they were running."
