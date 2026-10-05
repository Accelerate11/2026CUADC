"""读取航向标定工具保存的地理航向。"""
import math
from pathlib import Path

import yaml


def read_route_heading(route_file):
    path = Path(route_file).expanduser().resolve(strict=True)
    with path.open(encoding="utf-8") as stream:
        route = yaml.safe_load(stream)
    if not isinstance(route, dict):
        raise ValueError("Route YAML must contain a mapping")
    heading = route.get("heading_deg")
    if isinstance(heading, bool) or not isinstance(heading, (int, float)):
        raise ValueError("Route heading_deg must be a number")
    heading = float(heading)
    if not math.isfinite(heading) or not 0.0 <= heading < 360.0:
        raise ValueError("Route heading_deg must be finite and in [0, 360)")
    return heading
