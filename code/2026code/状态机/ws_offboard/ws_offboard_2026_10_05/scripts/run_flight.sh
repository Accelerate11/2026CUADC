#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$script_dir/_common.sh"
if (( $# < 2 )) || [[ "$1" == --help || "$1" == -h ]]; then
  printf 'Usage: bash scripts/run_flight.sh ROUTE_YAML FCU_URL [launch_arg:=value ...]\n'
  printf 'Example: bash scripts/run_flight.sh routes/round_01.yaml serial:///dev/serial/by-id/YOUR_FCU:115200\n'
  [[ "${1:-}" == --help || "${1:-}" == -h ]] && exit 0
  exit 2
fi
route_file=$1
fcu_url=$2
shift 2
cd -- "$CUADC_WORKSPACE"
if [[ ! -f "$route_file" ]]; then
  printf 'Route YAML not found: %s\n' "$route_file" >&2
  exit 2
fi
cuadc_source_workspace
exec ros2 launch cuadc_bringup flight.launch.py \
  "route_file:=$route_file" "fcu_url:=$fcu_url" "$@"
