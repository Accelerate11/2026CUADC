#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
INSTALL_BASE="${CUADC_INSTALL_BASE:-${WS_ROOT}/install}"
FCU_URL="${CUADC_FCU_URL:-}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      printf "用法：bash servo_test/run_servo_test.sh --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200\n"
      printf "也可设置 CUADC_FCU_URL。保留卸桨、未解锁与输入 TEST 的确认要求。\n"
      exit 0 ;;
    --fcu-url)
      if [[ $# -lt 2 || -z "$2" ]]; then echo "--fcu-url 缺少参数" >&2; exit 2; fi
      FCU_URL="$2"; shift ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
  shift
done
FCU_PARTS=$(python3 "${WS_ROOT}/tools/deployment_config.py" "${FCU_URL}") || exit 2
IFS=$'\t' read -r FCU_DEVICE FCU_BAUD <<<"${FCU_PARTS}"
MAVROS_PID=""

cleanup() {
  if [[ -n "${MAVROS_PID}" ]] && kill -0 "${MAVROS_PID}" 2>/dev/null; then
    kill "${MAVROS_PID}" 2>/dev/null || true
    wait "${MAVROS_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble。" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "${INSTALL_BASE}/setup.bash"
set -u
exec 9>"${CUADC_SERIAL_LOCK:-${WS_ROOT}/.flight_serial.lock}"
if ! flock -n 9; then
  echo "飞行、标定或诊断程序正在使用飞控。" >&2
  exit 3
fi

if [[ ! -e "${FCU_DEVICE}" ]]; then
  echo "未找到飞控：${FCU_DEVICE}" >&2
  exit 2
fi
if pgrep -af '[m]avros_node' >/dev/null; then
  echo "检测到 MAVROS 已在运行。请先关闭它，避免串口和服务冲突。" >&2
  exit 2
fi

echo "请确认：飞机已卸桨、飞控未解锁、投放机构周围无人和物品。"
read -r -p "输入 TEST 开始舵机测试：" confirmation
if [[ "${confirmation}" != "TEST" ]]; then
  echo "已取消。"
  exit 1
fi

mkdir -p "${SCRIPT_DIR}/logs"
LOG_FILE="${SCRIPT_DIR}/logs/mavros_$(date +%Y%m%d_%H%M%S).log"
echo "启动 MAVROS，日志：${LOG_FILE}"
ros2 launch cuadc_full_mission_20260803 mission_mavros.launch.py \
  fcu_url:="${FCU_URL}" \
  namespace:=mavros >"${LOG_FILE}" 2>&1 &
MAVROS_PID=$!

sleep 2
if ! kill -0 "${MAVROS_PID}" 2>/dev/null; then
  echo "MAVROS 启动失败，日志如下：" >&2
  sed -n '1,120p' "${LOG_FILE}" >&2
  exit 2
fi

for message_id in 30 32; do
  ros2 service call /mavros/set_message_interval \
    mavros_msgs/srv/MessageInterval \
    "{message_id: ${message_id}, message_rate: 30.0}"
done

python3 "${SCRIPT_DIR}/servo_cycle_test.py"
