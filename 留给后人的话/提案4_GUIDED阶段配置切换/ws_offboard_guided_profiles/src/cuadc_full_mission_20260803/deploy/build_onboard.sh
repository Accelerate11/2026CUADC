#!/usr/bin/env bash
# 保留原 V16 构建入口，转发到可移植工作区脚本。
set -eo pipefail
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WS_ROOT=$(cd -- "${SCRIPT_DIR}/../../.." && pwd)
exec bash "${WS_ROOT}/scripts/build_onboard.sh" "$@"
