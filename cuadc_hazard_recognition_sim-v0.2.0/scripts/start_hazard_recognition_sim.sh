#!/usr/bin/env bash
set -eo pipefail

package_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${CUADC_WS:-$(cd "$package_root/../.." && pwd)}"

source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"

log_file="${CUADC_LOG_FILE:-/tmp/cuadc_hazard_recognition_sim.log}"
rm -f "$log_file"
setsid -f ros2 launch cuadc_hazard_recognition_sim hazard_recognition.launch.py \
  start_state_machine:="${START_STATE_MACHINE:-false}" \
  vision_model:="${CUADC_HAZARD_MODEL:-}" >"$log_file" 2>&1 </dev/null
echo "Started hazard recognition simulation, log=$log_file"
