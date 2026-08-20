#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=_common.sh
source "$script_dir/_common.sh"

aircraft=${CUADC_AIRCRAFT:-}
config_root=${CUADC_CONFIG_ROOT:-}
mission=${CUADC_MISSION:-competition_2026}

usage() {
  printf '%s\n' \
    'Usage: preflight.sh --aircraft v6x_3E0032|v5nano_410035 --config-root DIR [--mission competition_2026]' \
    '' \
    'Runs the only bringup launch in read-only mode. No flight process or runtime gate is started.'
}

while (( $# > 0 )); do
  case "$1" in
    --aircraft) aircraft=${2:?missing value for --aircraft}; shift 2 ;;
    --config-root) config_root=${2:?missing value for --config-root}; shift 2 ;;
    --mission) mission=${2:?missing value for --mission}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
done

[[ -n "$aircraft" ]] || cuadc_die '--aircraft is required.'
cuadc_validate_aircraft "$aircraft"
cuadc_source_workspace
config_root=$(cuadc_resolve_config_root "$config_root")

exec ros2 launch cuadc_bringup flight.launch.py \
  aircraft:="$aircraft" \
  mission:="$mission" \
  config_root:="$config_root" \
  preflight_only:=true
