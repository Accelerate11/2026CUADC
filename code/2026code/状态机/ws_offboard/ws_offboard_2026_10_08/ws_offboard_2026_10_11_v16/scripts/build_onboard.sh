#!/usr/bin/env bash
# 从源码编译 V16；不自动安装依赖，不连接设备。
set -eo pipefail
WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble，请先按 environment/README.md 配置 Ubuntu 22.04。" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
cd -- "$WORKSPACE"
# 避免用户级新版 setuptools 影响 Humble 的系统构建工具。
export PYTHONNOUSERSITE=${PYTHONNOUSERSITE:-1}
colcon --log-base log build --build-base build --install-base install \
  --packages-select cuadc_full_mission_20260803 \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  --event-handlers console_direct+ "$@"
source "$WORKSPACE/install/setup.bash"
ros2 pkg prefix cuadc_full_mission_20260803
echo "编译完成。测试命令：bash scripts/verify.sh"
