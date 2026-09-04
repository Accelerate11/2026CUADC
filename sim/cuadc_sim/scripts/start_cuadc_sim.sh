#!/usr/bin/env bash
set -eo pipefail

cd /home/accelerate/cuadc_ws
source install/setup.bash

log_file=/tmp/cuadc_rescue_sim.log
rm -f "$log_file"

nohup ros2 launch cuadc_rescue_sim cuadc_sim.launch.py >"$log_file" 2>&1 </dev/null &
echo "$!" >/tmp/cuadc_rescue_sim.pid
echo "Started CUADC rescue simulation, pid=$(cat /tmp/cuadc_rescue_sim.pid), log=$log_file"
