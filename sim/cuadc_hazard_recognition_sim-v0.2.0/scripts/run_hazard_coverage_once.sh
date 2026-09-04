#!/usr/bin/env bash
set -o pipefail
export ROS_DOMAIN_ID="${CUADC_ROS_DOMAIN_ID:-42}"
export GZ_PARTITION="${CUADC_GZ_PARTITION:-cuadc_hazard_i1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export QT_XCB_GL_INTEGRATION="${QT_XCB_GL_INTEGRATION:-xcb_glx}"
SHOW_VISION_WINDOW="${CUADC_SHOW_VISION_WINDOW:-true}"
SHOW_GAZEBO_GUI="${CUADC_SHOW_GAZEBO_GUI:-true}"

SEED="${1:-2026}"
PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE="${CUADC_WS:-$(cd "$PACKAGE_ROOT/../.." && pwd)}"
ARDUPILOT="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}"
MODEL="${CUADC_HAZARD_MODEL:-$WORKSPACE/vision/dangerous_target.onnx}"
export ARDUPILOT_GAZEBO_DIR="$ARDUPILOT_GAZEBO"
RUN_DIR="${CUADC_RUN_DIR:-$WORKSPACE/cuadc_outputs/hazard_runs/seed_${SEED}}"
RAW_RESULT="$RUN_DIR/observations.json"
RESULT="$RUN_DIR/result.json"
if [[ ! -f "$MODEL" ]]; then
  echo "ONNX model not found: $MODEL" >&2
  echo "Set CUADC_HAZARD_MODEL=/absolute/path/to/model.onnx" >&2
  exit 4
fi
if [[ ! -x "$ARDUPILOT/build/sitl/bin/arducopter" ]]; then
  echo "ArduPilot SITL binary not found under $ARDUPILOT" >&2
  exit 5
fi
if [[ ! -d "$ARDUPILOT_GAZEBO/build" ]]; then
  echo "ardupilot_gazebo build directory not found under $ARDUPILOT_GAZEBO" >&2
  exit 6
fi
mkdir -p "$RUN_DIR"
rm -f "$RAW_RESULT" "$RESULT"

children=()
cleanup() {
  for pid in "${children[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${children[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

if ss -lun 2>/dev/null | grep -q ':9012 '; then
  echo "Refusing to start: CUADC hazard instance port 9012 is already in use" >&2
  exit 10
fi

cd "$WORKSPACE/src/cuadc_hazard_recognition_sim"
python3 scripts/generate_scene.py --seed "$SEED" | tee "$RUN_DIR/generate.log"
cp config/generated_scene.yaml "$RUN_DIR/generated_scene.yaml"

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
colcon build --packages-select cuadc_hazard_recognition_sim >"$RUN_DIR/build.log" 2>&1
source install/setup.bash

# Start Gazebo first so the ArduPilot UDP endpoint and IMU subscription are
# ready before SITL emits its first servo frame.
setsid ros2 launch cuadc_hazard_recognition_sim hazard_recognition.launch.py \
  start_state_machine:=true start_vision:=false use_bridge:=false \
  show_gazebo_gui:="$SHOW_GAZEBO_GUI" result_path:="$RAW_RESULT" \
  >"$RUN_DIR/mission.log" 2>&1 &
mission_pid=$!
children+=("$mission_pid")

if [[ "$SHOW_GAZEBO_GUI" == "true" && ( -n "$DISPLAY" || -n "$WAYLAND_DISPLAY" ) ]]; then
  for _ in $(seq 1 30); do
    grep -q "\[GUI\]" "$RUN_DIR/mission.log" 2>/dev/null && break
    kill -0 "$mission_pid" 2>/dev/null || break
    sleep 1
  done
  if ! grep -q "\[GUI\]" "$RUN_DIR/mission.log"; then
    echo "Gazebo GUI requested but no GUI startup was observed" >&2
    tail -100 "$RUN_DIR/mission.log" >&2
    exit 20
  fi
  sleep 3
  if grep -q "Segmentation fault" "$RUN_DIR/mission.log"; then
    echo "Gazebo GUI client crashed during initialization" >&2
    tail -120 "$RUN_DIR/mission.log" >&2
    exit 20
  fi
fi
sleep 6

cd "$ARDUPILOT"
sitl_args=(build/sitl/bin/arducopter -S --model JSON --speedup 1 --slave 0 --defaults Tools/autotest/default_params/copter.parm,Tools/autotest/default_params/gazebo-iris.parm --sim-address=127.0.0.1 -I1)
if [[ "${CUADC_SITL_GDB:-0}" == "1" ]]; then
  setsid gdb -q -batch -ex "handle SIGFPE stop print" -ex run -ex "thread apply all bt full" --args "${sitl_args[@]}" >"$RUN_DIR/sitl.log" 2>&1 &
else
  setsid "${sitl_args[@]}" >"$RUN_DIR/sitl.log" 2>&1 &
fi
sitl_pid=$!
children+=("$sitl_pid")
sleep 2

cd "$WORKSPACE"
setsid ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5770 >"$RUN_DIR/mavros.log" 2>&1 &
mavros_pid=$!
children+=("$mavros_pid")

for _ in $(seq 1 60); do
  grep -q 'JSON received' "$RUN_DIR/sitl.log" 2>/dev/null && break
  sleep 1
done
if ! grep -q 'JSON received' "$RUN_DIR/sitl.log"; then
  echo "SITL-Gazebo JSON handshake failed" >&2
  exit 11
fi
for _ in $(seq 1 60); do
  grep -q 'Got HEARTBEAT' "$RUN_DIR/mavros.log" 2>/dev/null && break
  sleep 1
done
if ! grep -q 'Got HEARTBEAT' "$RUN_DIR/mavros.log"; then
  echo "MAVROS heartbeat failed" >&2
  exit 12
fi

# Request ArduPilot local-position and GPS streams only after MAVROS plugins
# are ready. A heartbeat alone is not sufficient; require a service response.
stream_ready=false
for attempt in 1 2 3; do
  timeout 20 ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate '{stream_id: 0, message_rate: 10, on_off: true}' >"$RUN_DIR/mavros_stream_rate.log" 2>&1 || true
  if grep -q 'response:' "$RUN_DIR/mavros_stream_rate.log"; then
    stream_ready=true
    break
  fi
  sleep 3
done
if [[ "$stream_ready" != "true" ]]; then
  echo "MAVROS stream-rate request received no response" >&2
  cat "$RUN_DIR/mavros_stream_rate.log" >&2
  exit 16
fi

setsid ros2 run cuadc_hazard_recognition_sim static_camera_roi_node.py --ros-args -p generated_scene_path:="$WORKSPACE/install/cuadc_hazard_recognition_sim/share/cuadc_hazard_recognition_sim/config/generated_scene.yaml" >"$RUN_DIR/image_bridge.log" 2>&1 &
bridge_pid=$!
children+=("$bridge_pid")

# Start vision during takeoff so inference and both GUI windows are ready
# before the coverage route reaches the randomized recognition area.
for _ in $(seq 1 150); do
  grep -q 'Enter TAKEOFF' "$RUN_DIR/mission.log" 2>/dev/null && break
  if ! kill -0 "$sitl_pid" 2>/dev/null; then
    echo "SITL exited while waiting for local/RTK position" >&2
    exit 17
  fi
  sleep 1
done
if ! grep -q 'Enter TAKEOFF' "$RUN_DIR/mission.log"; then
  echo "State machine did not acquire local/RTK position" >&2
  tail -100 "$RUN_DIR/mission.log" >&2
  exit 18
fi

mkdir -p "$RUN_DIR/frames"
setsid ros2 run cuadc_hazard_recognition_sim hazard_vision_node.py --ros-args   -p model_path:="$MODEL" -p confidence:=0.05 -p min_consecutive_confirm:=3   -p show_window:="$SHOW_VISION_WINDOW" -p publish_annotated:=true   -p debug_image_dir:="$RUN_DIR/frames" -p debug_save_every_n:=15   -p debug_save_annotated:=true >"$RUN_DIR/vision.log" 2>&1 &
vision_pid=$!
children+=("$vision_pid")

if [[ "$SHOW_VISION_WINDOW" == "true" && ( -n "$DISPLAY" || -n "$WAYLAND_DISPLAY" ) ]]; then
  for _ in $(seq 1 45); do
    grep -q 'Recognition window opened' "$RUN_DIR/vision.log" 2>/dev/null && break
    kill -0 "$vision_pid" 2>/dev/null || break
    sleep 1
  done
  if ! grep -q 'Recognition window opened' "$RUN_DIR/vision.log"; then
    echo "Vision GUI requested but the recognition window did not open" >&2
    tail -80 "$RUN_DIR/vision.log" >&2
    tail -80 "$RUN_DIR/image_bridge.log" >&2
    exit 19
  fi
fi

deadline=$((SECONDS + 360))
next_status=$SECONDS
while (( SECONDS < deadline )); do
  if [[ -s "$RAW_RESULT" ]]; then
    python3 "$WORKSPACE/src/cuadc_hazard_recognition_sim/scripts/evaluate_coverage_result.py" \
      --scene "$RUN_DIR/generated_scene.yaml" \
      --observations "$RAW_RESULT" --output "$RESULT" \
      | tee "$RUN_DIR/evaluation.log"
    echo "RESULT_READY seed=$SEED path=$RESULT"
    cat "$RESULT"
    exit 0
  fi
  if ! kill -0 "$mission_pid" 2>/dev/null; then
    echo "mission launch exited before result" >&2
    tail -100 "$RUN_DIR/mission.log" >&2
    exit 2
  fi
  if ! kill -0 "$sitl_pid" 2>/dev/null; then
    echo "SITL exited before mission completion" >&2
    tail -120 "$RUN_DIR/sitl.log" >&2
    exit 15
  fi
  if (( SECONDS >= next_status )); then
    json_state="no"; heartbeat="no"; flight_state="WAITING"
    grep -q 'JSON received' "$RUN_DIR/sitl.log" 2>/dev/null && json_state="yes"
    grep -q 'Got HEARTBEAT' "$RUN_DIR/mavros.log" 2>/dev/null && heartbeat="yes"
    flight_state=$(grep -o 'state=[A-Z_]*' "$RUN_DIR/mission.log" 2>/dev/null | tail -1 | cut -d= -f2)
    echo "STATUS seed=$SEED json=$json_state heartbeat=$heartbeat state=${flight_state:-UNKNOWN} elapsed=$SECONDS"
    next_status=$((SECONDS + 10))
  fi
  sleep 2
done

echo "mission timeout seed=$SEED" >&2
tail -140 "$RUN_DIR/mission.log" >&2
exit 3




