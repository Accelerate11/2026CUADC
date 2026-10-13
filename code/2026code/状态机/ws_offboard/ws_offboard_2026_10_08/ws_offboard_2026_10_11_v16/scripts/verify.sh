#!/usr/bin/env bash
# 离线校验与回归；不启动相机、MAVROS 或任务。
set -eo pipefail
WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd -- "$WORKSPACE"
python3 scripts/validate_source.py
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 src/cuadc_full_mission_20260803/test/test_capture_clock.py
python3 src/cuadc_full_mission_20260803/test/test_vision_backend.py
while IFS= read -r -d '' file; do bash -n "$file"; done < <(
  find scripts tools full_mission drop_only indoor_projection fixed_release_test servo_test \
    src/cuadc_full_mission_20260803/deploy -type f -name '*.sh' -print0)
if [[ "${1:-}" == "--source-only" ]]; then
  echo "源码与 Python 检查完成；未执行 ROS 编译测试。"
  exit 0
fi
if [[ ! -r install/setup.bash || ! -d build/cuadc_full_mission_20260803 ]]; then
  echo "请先运行 bash scripts/build_onboard.sh，或用 --source-only 只查源码。" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=187
export ROS_LOCALHOST_ONLY=1
mkdir -p log/offline_tests
export ROS_LOG_DIR="$WORKSPACE/log/offline_tests"
ctest --test-dir build/cuadc_full_mission_20260803 --output-on-failure
ros2 pkg executables cuadc_full_mission_20260803
ros2 launch cuadc_full_mission_20260803 full_mission_real.launch.py --show-args
ros2 launch cuadc_full_mission_20260803 mission_mavros.launch.py --show-args
echo "离线验证完成；本命令没有连接飞控、相机或执行任务。"
