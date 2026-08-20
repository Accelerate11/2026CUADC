#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=_common.sh
source "$script_dir/_common.sh"

usage() {
  printf '%s\n' \
    'Usage: check_devices.sh' \
    'Lists exact serial by-id links and attached RealSense identities; sends no command.'
}

if (( $# > 0 )); then
  case "$1" in
    -h|--help) usage; exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; usage >&2; exit 2 ;;
  esac
fi

cuadc_source_workspace
[[ -d /dev/serial/by-id ]] || cuadc_die '/dev/serial/by-id is unavailable.'
mapfile -t serial_links < <(find /dev/serial/by-id -maxdepth 1 -type l -print | LC_ALL=C sort)
(( ${#serial_links[@]} > 0 )) || cuadc_die 'No serial by-id links were found.'

printf '[Serial by-id links]\n'
for link in "${serial_links[@]}"; do
  target=$(readlink -e -- "$link") || cuadc_die "Broken serial link: $link"
  [[ -c "$target" ]] || cuadc_die "Serial link target is not a character device: $link -> $target"
  access=denied
  if [[ -r "$link" && -w "$link" ]]; then
    access=read-write
  fi
  printf '%s -> %s [%s]\n' "$link" "$target" "$access"
done

printf '[RealSense devices]\n'
python3 - <<'PY'
import sys

try:
    import pyrealsense2 as rs
except Exception as exc:
    print(f"pyrealsense2 unavailable: {exc}", file=sys.stderr)
    raise SystemExit(2)

devices = list(rs.context().query_devices())
if not devices:
    print("No RealSense device found.", file=sys.stderr)
    raise SystemExit(2)
for device in devices:
    name = device.get_info(rs.camera_info.name)
    serial = device.get_info(rs.camera_info.serial_number)
    print(f"{name}: serial={serial}")
PY

printf '%s\n' \
  'Compare these identities with the selected external profile; candidate values are not approval evidence.'
