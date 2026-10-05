#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$script_dir/_common.sh"
if (( $# == 0 )) || [[ "$1" == --help || "$1" == -h ]]; then
  printf 'Usage: bash scripts/calibrate_route.sh FCU_URL [--output-dir DIRECTORY]\n'
  [[ "${1:-}" == --help || "${1:-}" == -h ]] && exit 0
  exit 2
fi
fcu_url=$1
shift
cuadc_source_workspace
cd -- "$CUADC_WORKSPACE"
exec ros2 run cuadc_tools calibrate_route --fcu-url "$fcu_url" --output-dir "$CUADC_WORKSPACE/routes" "$@"
