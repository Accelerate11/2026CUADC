#!/usr/bin/env bash
# 统一定位工作区并加载 ROS 环境；此处不向飞机发送指令。
CUADC_WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

cuadc_source() {
  local restore_nounset=false
  [[ $- == *u* ]] && restore_nounset=true
  set +u
  # ROS 环境脚本可能引用尚未设置的环境变量。
  source "$1"
  if "$restore_nounset"; then set -u; fi
}

cuadc_source_ros() {
  local setup_file="${CUADC_ROS_SETUP:-/opt/ros/humble/setup.bash}"
  if [[ ! -r "$setup_file" ]]; then
    printf 'ROS setup not found: %s\n' "$setup_file" >&2
    return 2
  fi
  cuadc_source "$setup_file"
}

cuadc_source_workspace() {
  cuadc_source_ros
  if [[ ! -r "$CUADC_WORKSPACE/install/setup.bash" ]]; then
    printf 'Workspace is not built. Run: bash scripts/build_onboard.sh\n' >&2
    return 2
  fi
  cuadc_source "$CUADC_WORKSPACE/install/setup.bash"
}
