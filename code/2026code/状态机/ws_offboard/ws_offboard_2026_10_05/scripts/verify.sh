#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
workspace=$(cd -- "$script_dir/.." && pwd)
python3 "$script_dir/validate_source.py"
for script in "$script_dir"/*.sh; do bash -n "$script"; done
if [[ -r /opt/ros/humble/setup.bash ]]; then
  source "$script_dir/_common.sh"
  cuadc_source_ros
fi
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$workspace/src/cuadc_bringup:$workspace/src/cuadc_perception:${PYTHONPATH:-}"
python3 -m unittest discover -s "$workspace/tests" -v
