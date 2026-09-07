# Troubleshooting

## ROS setup + `set -u`

症状：

```text
AMENT_TRACE_SETUP_FILES: unbound variable
```

原因：脚本启用了 `set -u`，ROS Humble setup 访问未定义变量。本包脚本已在 source 前自动 `set +u`，source 后恢复。

## D435i busy

```text
VIDIOC_S_FMT ... errno=16
Device or resource busy
```

检查：

```bash
pgrep -af "drop_bucket_realsense|realsense2_camera|vision_provider_template"
sudo fuser -v /dev/video* 2>/dev/null
```

不要重复启动视觉节点。

## web 8080 占用

```text
bind: Address already in use
```

检查：

```bash
pgrep -af web_video_server
ss -lntp | grep ':8080'
```

若已经是 web_video_server，直接复用。

## 浏览器无图

```bash
ros2 topic hz /cuadc/calib/annotated
```

没有频率先修视觉节点；有频率再检查 SSH 隧道/web server。

## 重复 ROS 参数声明

症状：

```text
ParameterAlreadyDeclaredException
```

检查：

```bash
python3 - <<'PY'
from pathlib import Path
import re
from collections import Counter
p=Path('~/cuadc_mission/src/cuadc_visual_drop_flight/scripts/drop_bucket_realsense_node.py').expanduser()
s=p.read_text()
names=re.findall(r'self\.declare_parameter\(\s*["\']([^"\']+)["\']',s)
for k,v in Counter(names).items():
    if v>1: print(k,v)
PY
```
