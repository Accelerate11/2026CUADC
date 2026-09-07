#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
AIRCRAFT="$ROOT/config/aircraft.yaml"
RUNTIME="$ROOT/config/runtime.env"
if [[ ! -r "$AIRCRAFT" ]]; then
  echo "Missing $AIRCRAFT" >&2
  exit 2
fi
python3 "$ROOT/tools/validate_config.py" --config "$AIRCRAFT" --mode servo
python3 "$ROOT/tools/generate_config.py" --aircraft "$AIRCRAFT" >/dev/null
if [[ ! -r "$RUNTIME" ]]; then
  echo "FCU runtime configuration was not generated" >&2
  exit 2
fi
source "$RUNTIME"
source /opt/ros/humble/setup.bash
if pgrep -af '[m]avros_node' >/dev/null; then
  echo "A MAVROS node is already running; stop it before this isolated test" >&2
  exit 2
fi
MAVROS_PID=""
cleanup() {
  if [[ -n "$MAVROS_PID" ]] && kill -0 "$MAVROS_PID" 2>/dev/null; then
    kill -INT "$MAVROS_PID" 2>/dev/null || true
    wait "$MAVROS_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM
ros2 launch mavros apm.launch \
  fcu_url:="serial://${VS_FCU_DEVICE}:${VS_FCU_BAUD}" \
  namespace:=mavros >/tmp/vision_servo_ground_check_mavros.log 2>&1 &
MAVROS_PID=$!
python3 "$ROOT/tools/servo_cycle_test.py" --aircraft "$AIRCRAFT"
