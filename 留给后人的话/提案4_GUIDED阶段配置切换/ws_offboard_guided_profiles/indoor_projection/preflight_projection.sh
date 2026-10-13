#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS_ROOT=$(cd -- "${SCRIPT_DIR}/.." && pwd)
INSTALL_BASE="${CUADC_INSTALL_BASE:-${WS_ROOT}/install}"
PKG_NAME="cuadc_full_mission_20260803"
PARAMS_FILE="${WS_ROOT}/src/${PKG_NAME}/config/mission_real.yaml"
EXPOSURE=""
RUN_STAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="${SCRIPT_DIR}/logs/preflight_${RUN_STAMP}"
VIDEO_PATH="${LOG_DIR}/vision/basket_annotated.avi"
DETECTIONS_PATH="${LOG_DIR}/vision/detections_camera.csv"

usage() {
  printf '用法: %s [曝光值]\n' "$0"
  printf '不指定曝光值使用自动曝光；末尾正数表示手动曝光（微秒）。\n'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    *)
      if [[ -z "${EXPOSURE}" && "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]]; then
        EXPOSURE="$1"
      else
        echo "未知参数或重复曝光值: $1" >&2
        usage >&2
        exit 2
      fi
      ;;
  esac
  shift
done

source /opt/ros/humble/setup.bash
source "${INSTALL_BASE}/setup.bash"
mkdir -p "${LOG_DIR}/ros" "${LOG_DIR}/vision"
export ROS_LOG_DIR="${LOG_DIR}/ros"
PACKAGE_PREFIX=$(ros2 pkg prefix "${PKG_NAME}")
MODEL_FILE="${PACKAGE_PREFIX}/share/${PKG_NAME}/models/basket_detect.onnx"

ARGS=(
  --ros-args
  --params-file "${PARAMS_FILE}"
  -p diagnostic_video_enabled:=true
  -p diagnostic_video_path:="${VIDEO_PATH}"
  -p diagnostic_detection_csv_path:="${DETECTIONS_PATH}"
  -p live_view_enabled:=true
  -p model_path:="${MODEL_FILE}"
)
if [[ -n "${CUADC_VISION_DEVICE:-}" ]]; then
  case "${CUADC_VISION_DEVICE}" in
    cpu|intel:gpu) ARGS+=( -p device:="${CUADC_VISION_DEVICE}" ) ;;
    *) echo '[FATAL] CUADC_VISION_DEVICE 必须是 cpu 或 intel:gpu' >&2; exit 2 ;;
  esac
fi
if [[ -n "${EXPOSURE}" ]]; then
  ARGS+=( -p exposure:="${EXPOSURE}" )
fi

echo '[INFO] 室内投放准星预检：仅启动视觉节点，不启动 MAVROS、飞控或任务状态机。'
echo '[INFO] 橙色十字：1号右前舵机（CH7）；洋红色十字：2号左后舵机（CH8）。'
echo "[INFO] 本次日志：${LOG_DIR}"
echo "[INFO] 标注视频：${VIDEO_PATH}"
echo "[INFO] 检测 CSV：${DETECTIONS_PATH}"
if [[ -n "${EXPOSURE}" ]]; then
  echo "[INFO] 手动曝光：${EXPOSURE}"
else
  echo '[INFO] 曝光：自动（YAML/相机自动曝光）'
fi
exec ros2 run "${PKG_NAME}" basket_vision_ros_node.py "${ARGS[@]}"
