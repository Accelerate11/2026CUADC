#!/usr/bin/env bash
set -eo pipefail

cd /home/accelerate/cuadc_ws
source install/setup.bash

log_file=/tmp/cuadc_hazard_recognition_sim.log
rm -f "$log_file"

setsid -f ros2 launch cuadc_hazard_recognition_sim hazard_recognition.launch.py >"$log_file" 2>&1 </dev/null
echo "Started hazard recognition simulation, log=$log_file"
