#!/usr/bin/env bash
set -eo pipefail

pkill -f "gz sim .*cuadc_rescue_single" || true
pkill -f "ros2 launch cuadc_rescue_sim cuadc_sim.launch.py" || true
echo "Stopped CUADC rescue simulation processes if they were running."
