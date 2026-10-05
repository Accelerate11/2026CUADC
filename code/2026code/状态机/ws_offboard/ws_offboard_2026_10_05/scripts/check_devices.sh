#!/usr/bin/env bash
set -euo pipefail
if (( $# > 0 )); then
  printf 'Usage: bash scripts/check_devices.sh\n'
  [[ "$1" == --help || "$1" == -h ]] && exit 0
  exit 2
fi
printf 'Serial devices:\n'
if [[ -d /dev/serial/by-id ]]; then
  ls -l /dev/serial/by-id/
else
  printf '/dev/serial/by-id is unavailable.\n'
fi
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  rs-enumerate-devices
else
  printf 'rs-enumerate-devices is unavailable. Install the RealSense runtime tools on the aircraft computer.\n'
fi
