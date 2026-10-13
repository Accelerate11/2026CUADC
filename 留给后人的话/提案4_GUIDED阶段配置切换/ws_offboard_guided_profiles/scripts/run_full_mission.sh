#!/usr/bin/env bash
set -o pipefail
set +u

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
PKG_NAME="cuadc_full_mission_20260803"
INSTALL_BASE="${CUADC_INSTALL_BASE:-${WS_ROOT}/install}"
APP_ROOT="${CUADC_APP_ROOT:-${WS_ROOT}}"
PARAMS_SOURCE="${CUADC_PARAMS_SOURCE:-${WS_ROOT}/src/${PKG_NAME}/config/mission_real.yaml}"
LOG_ROOT="${CUADC_LOG_ROOT:-${APP_ROOT}/logs}"

FCU_URL="${CUADC_FCU_URL:-}"
ROUTES_DIR="${CUADC_ROUTES_DIR:-${WS_ROOT}/routes}"
SERIAL_LOCK="${CUADC_SERIAL_LOCK:-${WS_ROOT}/.flight_serial.lock}"

PREFLIGHT_ONLY=false
ROUTE_NAME=""
ROUTE_FILE=""
ROUTE_HEADING_DEG=""
COLOR_EXPOSURE=""
CAMERA_SERIAL="${CUADC_CAMERA_SERIAL:-}"
DROP_ONLY=false
FIXED_RELEASE_TEST=false
ERRORS=0
MAVROS_LAUNCH_PID=""
MAVROS_MANAGED_PIDS=()
MAVROS_CLEANED=false
POSTFLIGHT_REPORT_PID=""
MISSION_LAUNCH_PID=""

usage() {
  cat <<'USAGE'
用法:
  bash full_mission/run_full_mission.sh route_name --fcu-url SERIAL_URL [expo=曝光值]
  bash drop_only/run_drop_only.sh route_name --fcu-url SERIAL_URL [expo=曝光值]

说明:
  route_name 对应工作区 routes/route_name.yaml（可用 CUADC_ROUTES_DIR 覆盖），由两点 GPS 计算中轴线航向。
  曝光只能用 expo=数值覆盖；expo=0 表示自动曝光，省略时使用 YAML 默认值。
  示例：bash full_mission/run_full_mission.sh round_01 --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 expo=20
  必须通过 --fcu-url 或 CUADC_FCU_URL 显式指定飞控串口 URL；不自动选择设备。
  D435i 默认自动选择当前连接设备，也可用 --camera-serial 或 CUADC_CAMERA_SERIAL 指定设备。
  数字航向参数已移除；航向只来自路线 YAML。
  --drop-only 在两次投放完成后直接返航，不执行灾情侦察航段。
  原 V16 部署参数检查只报告异常；串口独占、路线/姿态预检仍按原行为阻断。
  --preflight 只执行离线部署检查，不启动任务或 MAVROS。
USAGE
}

report_error() {
  printf '[ERROR] %s\n' "$1" >&2
  ERRORS=$((ERRORS + 1))
}

cleanup_mavros() {
  if [[ "${MAVROS_CLEANED}" == true ]]; then
    return
  fi
  MAVROS_CLEANED=true

  if [[ -n "${MAVROS_LAUNCH_PID}" ]] &&
    kill -0 "${MAVROS_LAUNCH_PID}" 2>/dev/null
  then
    kill -INT "${MAVROS_LAUNCH_PID}" 2>/dev/null || true
    (
      sleep 5
      kill -TERM "${MAVROS_LAUNCH_PID}" 2>/dev/null || true
      sleep 2
      kill -KILL "${MAVROS_LAUNCH_PID}" 2>/dev/null || true
    ) &
    local cleanup_guard_pid=$!
    wait "${MAVROS_LAUNCH_PID}" 2>/dev/null || true
    kill "${cleanup_guard_pid}" 2>/dev/null || true
    wait "${cleanup_guard_pid}" 2>/dev/null || true
  fi
  for pid in "${MAVROS_MANAGED_PIDS[@]}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
    fi
  done
}

cleanup_children() {
  cleanup_mavros
}

start_postflight_report_watcher() {
  if [[ "${CUADC_POSTFLIGHT_REPORT:-false}" != true ]]; then
    return
  fi
  local watcher_script="${WS_ROOT}/tools/postflight_report_watcher.sh"
  local report_script="${WS_ROOT}/tools/flight_summary_report.py"
  if [[ ! -r "${watcher_script}" || ! -r "${report_script}" ]]; then
    echo "[ERROR] 后置报告监视器或报告脚本不可用。" >&2
    return
  fi
  # The watcher has its own session and monitors the mission launch directly.
  # Terminal signals and MAVROS cleanup therefore cannot suppress the report.
  setsid bash "${watcher_script}" "${MISSION_LAUNCH_PID}" "${report_script}" \
    "${FLIGHT_LOG}" </dev/null >/dev/null 2>&1 &
  POSTFLIGHT_REPORT_PID=$!
  echo "[INFO] 独立飞行报告监视器已启动：PID=${POSTFLIGHT_REPORT_PID}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --fcu-url)
      if [[ $# -lt 2 || -z "$2" ]]; then echo "--fcu-url 缺少参数" >&2; exit 2; fi
      FCU_URL="$2"; shift ;;
    --camera-serial)
      if [[ $# -lt 2 || -z "$2" ]]; then echo "--camera-serial 缺少参数" >&2; exit 2; fi
      CAMERA_SERIAL="$2"; shift ;;
    --preflight) PREFLIGHT_ONLY=true ;;
    --drop-only) DROP_ONLY=true ;;
    --fixed-release-test) FIXED_RELEASE_TEST=true ;;
    -h|--help) usage; exit 0 ;;
    expo=*)
      if [[ -n "${COLOR_EXPOSURE}" ]]; then
        echo "曝光值重复: $1" >&2
        usage >&2
        exit 2
      fi
      COLOR_EXPOSURE="${1#expo=}"
      if [[ ! "${COLOR_EXPOSURE}" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
        echo "曝光值无效: $1" >&2
        usage >&2
        exit 2
      fi
      ;;
    *)
      if [[ -z "${ROUTE_NAME}" && "$1" != -* && "$1" != *=* ]]; then
        ROUTE_NAME="$1"
      else
        echo "未知参数或重复路线名（航向角输入已移除）: $1" >&2
        usage >&2
        exit 2
      fi
      ;;
  esac
  shift
done

if [[ -z "${ROUTE_NAME}" ]]; then
  echo '[FATAL] 必须提供路线名，例如 ./run_full_mission.sh north_axis' >&2
  usage >&2
  exit 2
fi
if [[ "${ROUTE_NAME}" == */* || "${ROUTE_NAME}" == *\\* || "${ROUTE_NAME}" == . || "${ROUTE_NAME}" == .. ]]; then
  echo "[FATAL] 路线名不能包含路径；请用 CUADC_ROUTES_DIR 指定目录。" >&2
  exit 2
fi
FCU_PARTS=$(python3 "${WS_ROOT}/tools/deployment_config.py" "${FCU_URL}") || exit 2
IFS=$'\t' read -r FCU_DEVICE FCU_BAUD <<<"${FCU_PARTS}"
ROUTE_FILE="${ROUTES_DIR}/${ROUTE_NAME}.yaml"
if [[ ! -r "${ROUTE_FILE}" ]]; then
  echo "[FATAL] 路线文件不可读：${ROUTE_FILE}" >&2
  exit 2
fi
ROUTE_HEADING_DEG=$(python3 - "${ROUTE_FILE}" <<'PY'
from pathlib import Path
import math, sys, yaml
data = yaml.safe_load(Path(sys.argv[1]).read_text(encoding='utf-8'))
try:
    p1 = (float(data['point_1']['latitude_deg']), float(data['point_1']['longitude_deg']))
    p2 = (float(data['point_2']['latitude_deg']), float(data['point_2']['longitude_deg']))
except (KeyError, TypeError, ValueError) as exc:
    raise SystemExit(f'路线必须包含有效的 point_1/point_2 坐标: {exc}')
lat1, lat2 = map(math.radians, (p1[0], p2[0]))
dlon = math.radians(p2[1] - p1[1])
y = math.sin(dlon) * math.cos(lat2)
x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
heading = math.degrees(math.atan2(y, x)) % 360.0
print(f'{heading:.12g}')
PY
) || exit 2

if [[ "${DROP_ONLY}" == true && "${FIXED_RELEASE_TEST}" == true ]]; then
  echo '[FATAL] --drop-only 与 --fixed-release-test 不能同时使用。' >&2
  exit 2
fi

conda deactivate 2>/dev/null || true

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo '[FATAL] 未找到 /opt/ros/humble/setup.bash，无法执行 ros2 launch。' >&2
  exit 2
fi
source /opt/ros/humble/setup.bash

if [[ ! -r "${INSTALL_BASE}/setup.bash" ]]; then
  echo "[FATAL] 工作区尚未编译：${INSTALL_BASE}/setup.bash" >&2
  echo "先运行：bash ${WS_ROOT}/scripts/build_onboard.sh" >&2
  exit 2
fi
source "${INSTALL_BASE}/setup.bash"

if ! PACKAGE_PREFIX=$(ros2 pkg prefix "${PKG_NAME}" 2>/dev/null); then
  echo "[FATAL] 已编译环境中未找到功能包：${PKG_NAME}" >&2
  exit 2
fi

MODEL_FILE="${CUADC_MODEL_FILE:-${PACKAGE_PREFIX}/share/${PKG_NAME}/models/basket_detect.onnx}"

if [[ ! -e "${FCU_DEVICE}" ]]; then
  report_error "未找到目标飞控：${FCU_DEVICE}"
else
  FCU_RESOLVED=$(readlink -f -- "${FCU_DEVICE}")
  if [[ ! -c "${FCU_RESOLVED}" ]]; then
    report_error "飞控链接未解析为字符设备：${FCU_RESOLVED}"
  else
    echo "[OK] 飞控：${FCU_DEVICE} -> ${FCU_RESOLVED} @ ${FCU_BAUD}"
  fi
fi

if [[ -z "${CAMERA_SERIAL}" ]] && command -v rs-enumerate-devices >/dev/null 2>&1; then
  CAMERA_SERIAL=$(timeout 10s rs-enumerate-devices -s 2>/dev/null |
    awk '$1 == "Intel" && $2 == "RealSense" && toupper($3) == "D435I" {print $4; exit}')
fi
if [[ -z "${CAMERA_SERIAL}" ]]; then
  report_error '室内部署未检测到 D435i；视觉节点将尝试自动选择设备。'
else
  echo "[OK] D435i：serial=${CAMERA_SERIAL}"
fi

if pgrep -af '[b]asket_vision_ros_node.py|[c]uadc_full_mission_node' >/dev/null; then
  report_error '检测到已有视觉或任务节点；继续启动可能产生节点名冲突。'
fi

if [[ ! -r "${PARAMS_SOURCE}" ]]; then
  echo "[FATAL] 参数文件不可读：${PARAMS_SOURCE}" >&2
  exit 2
fi
if [[ ! -r "${MODEL_FILE}" ]]; then
  echo "[FATAL] 模型文件不可读：${MODEL_FILE}" >&2
  exit 2
fi

python3 - "${PARAMS_SOURCE}" "${FIXED_RELEASE_TEST}" \
  "${ROUTE_HEADING_DEG}" "${COLOR_EXPOSURE}" <<'PY' || ERRORS=$((ERRORS + 1))
from pathlib import Path
import math
import sys
import yaml

data = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
mission = data["cuadc_full_mission_node"]["ros__parameters"]
vision = data["basket_vision_ros_node"]["ros__parameters"]
fixed_release_test = sys.argv[2].lower() == "true"
heading = float(sys.argv[3])
exposure = float(sys.argv[4] or vision.get("exposure", -1.0))
checks = [
    (mission.get("flight_enable") is True, "flight_enable=true"),
    (mission.get("auto_arm_on_guided") is True, "auto_arm_on_guided=true"),
    (
        math.isfinite(heading) and
        (0.0 <= heading < 360.0),
        f"geographic_heading_deg={heading:g}",
    ),
    (math.isfinite(exposure) and exposure >= 0.0,
     "exposure=auto" if exposure == 0.0 else f"exposure={exposure:g}"),
    (list(mission.get("servo_channels", [])) == [7, 8], "servo_channels=[7, 8]"),
    (list(mission.get("servo_stowed_pwm", [])) == [1100, 1100], "servo_stowed_pwm=[1100, 1100]"),
    (list(mission.get("servo_release_pwm", [])) == [1900, 1900], "servo_release_pwm=[1900, 1900]"),
]
if fixed_release_test:
    points = list(mission.get("fixed_release_points_field_xy", []))
    checks.extend((
        (mission.get("coarse_release_trial_mode") is True, "coarse_release_trial_mode=true"),
        (points == [32.5, -2.6, 32.5, 2.6], "fixed_release_points=(32.5,-2.6)/(32.5,2.6)"),
        (mission.get("drop_only_mode") is False, "fixed_release_test continues recon"),
    ))
failed = False
for ok, label in checks:
    print(f"[{'OK' if ok else 'ERROR'}] {label}", file=sys.stderr if not ok else sys.stdout)
    failed |= not ok
raise SystemExit(1 if failed else 0)
PY

echo "[INFO] FCU_URL=${FCU_URL}"
echo "[INFO] 路线=${ROUTE_FILE}，规划航向=${ROUTE_HEADING_DEG}°"
if (( ERRORS > 0 )); then
  echo "[ERROR] 预检共报告 ${ERRORS} 项异常；按配置继续由飞手判断。" >&2
else
  echo '[OK] 可在当前环境确认的部署检查均通过。'
fi

if [[ "${PREFLIGHT_ONLY}" == true ]]; then
  echo '[INFO] 仅预检，未启动 MAVROS、视觉或任务节点。'
  exit 0
fi

RUN_STAMP=$(date +%Y%m%d_%H%M%S)
FLIGHT_LOG="${LOG_ROOT}/flight_${RUN_STAMP}"
RUNTIME_PARAMS="${FLIGHT_LOG}/mission_runtime.yaml"
mkdir -p "${FLIGHT_LOG}/ros"
cp -a -- "${PARAMS_SOURCE}" "${RUNTIME_PARAMS}"
chmod u+w "${RUNTIME_PARAMS}"

if [[ -n "${ROUTE_HEADING_DEG}" || -n "${COLOR_EXPOSURE}" ]]; then
  if ! python3 - "${RUNTIME_PARAMS}" "${ROUTE_HEADING_DEG}" "${COLOR_EXPOSURE}" <<'PY'
from pathlib import Path
import math
import re
import sys

path = Path(sys.argv[1])
heading_text = sys.argv[2]
exposure_text = sys.argv[3]
text = path.read_text(encoding="utf-8")

if heading_text:
    heading = float(heading_text)
    if not math.isfinite(heading) or not 0.0 <= heading < 360.0:
        raise SystemExit("地理航向角必须在 [0, 360) 度范围内")
    replacement = f"{heading:.1f}" if heading.is_integer() else f"{heading:.12g}"
    text, count = re.subn(
        r"(?m)^(\s*geographic_heading_deg:\s*).*$",
        rf"\g<1>{replacement}", text, count=1)
    if count != 1:
        raise SystemExit("未能唯一设置 geographic_heading_deg")

if exposure_text:
    exposure = float(exposure_text)
    if not math.isfinite(exposure) or exposure < 0.0:
        raise SystemExit("曝光必须是大于等于 0 的有限数字")
    replacement = f"{exposure:.1f}" if exposure.is_integer() else f"{exposure:.12g}"
    text, count = re.subn(
        r"(?m)^(\s*exposure:\s*).*$",
        rf"\g<1>{replacement}", text, count=1)
    if count != 1:
        raise SystemExit("未能唯一设置 exposure")

path.write_text(text, encoding="utf-8")
PY
  then
    report_error '地理航向或曝光覆盖无效；继续使用 YAML 默认值。'
    cp -a -- "${PARAMS_SOURCE}" "${RUNTIME_PARAMS}"
  fi
fi

VIDEO_PATH="${FLIGHT_LOG}/vision/basket_annotated.avi"
CAMERA_DETECTIONS_PATH="${FLIGHT_LOG}/vision/detections_camera.csv"
PROJECTION_PATH="${FLIGHT_LOG}/bucket_projection.csv"
SNAPSHOT_DIR="${FLIGHT_LOG}/vision/release_snapshots"
TELEMETRY_PATH="${FLIGHT_LOG}/flight_telemetry.csv"
if ! python3 - "${RUNTIME_PARAMS}" "${VIDEO_PATH}" "${CAMERA_DETECTIONS_PATH}" "${PROJECTION_PATH}" "${DROP_ONLY}" "${FIXED_RELEASE_TEST}" "${SNAPSHOT_DIR}" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
video_path = sys.argv[2]
camera_detections_path = sys.argv[3]
projection_path = sys.argv[4]
drop_only = sys.argv[5].lower() == "true"
fixed_release_test = sys.argv[6].lower() == "true"
snapshot_dir = sys.argv[7]
text = path.read_text(encoding="utf-8")

replacements = (
    (
        r"(?m)^(\s*diagnostic_video_enabled:\s*).*$",
        lambda match: f"{match.group(1)}true",
        "diagnostic_video_enabled",
    ),
    (
        r"(?m)^(\s*diagnostic_video_path:\s*).*$",
        lambda match: f'{match.group(1)}"{video_path}"',
        "diagnostic_video_path",
    ),
    (
        r"(?m)^(\s*diagnostic_detection_csv_path:\s*).*$",
        lambda match: f'{match.group(1)}"{camera_detections_path}"',
        "diagnostic_detection_csv_path",
    ),
    (
      r"(?m)^(\s*bucket_projection_log_path:\s*).*$",
        lambda match: f'{match.group(1)}"{projection_path}"',
      "bucket_projection_log_path",
    ),
    (
      r"(?m)^(\s*release_snapshot_dir:\s*).*$",
      lambda match: f'{match.group(1)}"{snapshot_dir}"',
      "release_snapshot_dir",
    ),
    (
        r"(?m)^(\s*drop_only_mode:\s*).*$",
        lambda match: f"{match.group(1)}{'true' if drop_only else 'false'}",
        "drop_only_mode",
    ),
    (
        r"(?m)^(\s*fixed_release_test_mode:\s*).*$",
        lambda match: f"{match.group(1)}{'true' if fixed_release_test else 'false'}",
        "fixed_release_test_mode",
    ),
)
for pattern, replacement, name in replacements:
    text, count = re.subn(pattern, replacement, text, count=1)
    if count != 1:
        raise SystemExit(f"未能唯一设置 {name}")

path.write_text(text, encoding="utf-8")
PY
then
  echo '[FATAL] 运行参数生成失败，未启动任务。' >&2
  exit 2
fi
chmod 0444 "${RUNTIME_PARAMS}"

python3 - "${RUNTIME_PARAMS}" <<'PY'
from pathlib import Path
import sys
import yaml

data = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="utf-8"))
mission = data["cuadc_full_mission_node"]["ros__parameters"]
vision = data["basket_vision_ros_node"]["ros__parameters"]
print(f"[OK] geographic_heading_deg={float(mission['geographic_heading_deg']):g}")
exposure = float(vision["exposure"])
print("[OK] exposure=auto" if exposure <= 0.0 else f"[OK] exposure={exposure:g}")
PY

export ROS_LOG_DIR="${FLIGHT_LOG}/ros"

# Serialize full/drop-only startup before cleaning old processes.
exec 9>"${SERIAL_LOCK}"
if ! flock -n 9; then
  echo '[FATAL] 已有任务持有飞控启动锁，请先结束该任务。' >&2
  exit 3
fi
if ! python3 "${WS_ROOT}/tools/ros_startup_guard.py" clean "${FCU_DEVICE}"; then
  echo '[FATAL] ROS/飞控串口未清理干净，拒绝启动。' >&2
  exit 3
fi

trap cleanup_children EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

{
  MAVROS_LOG="${FLIGHT_LOG}/mavros_stdout.log"
  echo "[INFO] MAVROS 未运行，先启动 MAVROS；日志：${MAVROS_LOG}"
  # 所有入口使用同一 MAVROS 启动文件，保留 V16 的飞控时间同步配置。
  MAVROS_PACKAGE="${PKG_NAME}"
  MAVROS_LAUNCH=mission_mavros.launch.py
  ros2 launch "${MAVROS_PACKAGE}" "${MAVROS_LAUNCH}" \
    fcu_url:="${FCU_URL}" \
    namespace:=mavros >"${MAVROS_LOG}" 2>&1 &
  MAVROS_LAUNCH_PID=$!

  for _ in {1..50}; do
    mapfile -t MAVROS_MANAGED_PIDS < <(
      pgrep -x mavros_node 2>/dev/null || true
    )
    if (( ${#MAVROS_MANAGED_PIDS[@]} > 0 )); then
      break
    fi
    if ! kill -0 "${MAVROS_LAUNCH_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
}

if (( ${#MAVROS_MANAGED_PIDS[@]} != 1 )); then
  echo '[FATAL] MAVROS 未能作为唯一实例启动。' >&2
  exit 3
fi
# Allow the newly spawned process to open the FCU device before validation.
MAVROS_VERIFIED=false
for _ in {1..30}; do
  if python3 "${WS_ROOT}/tools/ros_startup_guard.py" verify "${FCU_DEVICE}" \
      --mavros-pid "${MAVROS_MANAGED_PIDS[0]}" >"${FLIGHT_LOG}/serial_guard.log" 2>&1; then
    MAVROS_VERIFIED=true
    break
  fi
  sleep 0.2
done
if [[ "${MAVROS_VERIFIED}" != true ]]; then
  cat "${FLIGHT_LOG}/serial_guard.log" >&2
  exit 3
fi

if [[ "${FIXED_RELEASE_TEST}" == true ]]; then
  MISSION_LABEL="固定坐标双投放测试 + 灾情侦察"
elif [[ "${DROP_ONLY}" == true ]]; then
  MISSION_LABEL="drop-only"
else
  MISSION_LABEL="全任务"
fi
echo "[INFO] 启动 D435i 视觉 + ${MISSION_LABEL}，日志：${FLIGHT_LOG}"
echo "[INFO] 标注视频：${VIDEO_PATH}"
echo "[INFO] 结构化飞行遥测：${TELEMETRY_PATH}"
echo '[INFO] 切入 GUIDED 后，现有状态机在定位/航向/视觉/舵机门槛通过后自动解锁并起飞。'

if ! python3 "${WS_ROOT}/tools/route_preflight.py" "${ROUTE_FILE}"; then
  echo '[FATAL] 路线/GPS/罗盘起飞前检查未通过，拒绝启动任务。' >&2
  exit 3
fi

# Recheck after interactive confirmation, before starting the mission node.
if ! python3 "${WS_ROOT}/tools/ros_startup_guard.py" verify "${FCU_DEVICE}" \
    --mavros-pid "${MAVROS_MANAGED_PIDS[0]}"; then
  exit 3
fi

LAUNCH_CAMERA_ARGUMENT=()
if [[ -n "${CAMERA_SERIAL}" ]]; then
  LAUNCH_CAMERA_ARGUMENT+=("camera_serial:=${CAMERA_SERIAL}")
fi

ros2 launch "${PKG_NAME}" full_mission_real.launch.py \
  fcu_url:="${FCU_URL}" \
  start_mavros:=false \
  "${LAUNCH_CAMERA_ARGUMENT[@]}" \
  params_file:="${RUNTIME_PARAMS}" \
  model_path:="${MODEL_FILE}" \
  telemetry_csv_path:="${TELEMETRY_PATH}" &
MISSION_LAUNCH_PID=$!
start_postflight_report_watcher

wait "${MISSION_LAUNCH_PID}"
MISSION_STATUS=$?
exit "${MISSION_STATUS}"
