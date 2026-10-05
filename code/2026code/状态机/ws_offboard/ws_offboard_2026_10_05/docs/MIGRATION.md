# 原压缩包与新工作区对照

来源是用户指定的 `cuadc_2.zip`。目录参考为 `2026CUADC/code/2026code/状态机/ws_offboard/ws_offboard_2026_8_20`；参考目录树的 Git SHA 记录在 `SOURCE_MANIFEST.json`。

| 原位置 | 新位置 |
| --- | --- |
| `aaa/cuadc_full_mission_20260803/src/cuadc_full_mission_node.cpp` | `src/cuadc_mission/src/cuadc_full_mission_node.cpp` |
| 原包 `scripts/basket_vision_ros_node.py` | `src/cuadc_perception/cuadc_perception/basket_vision_ros_node.py` |
| 原包 `scripts/basket_detect_seg_analysis.py` | `src/cuadc_perception/cuadc_perception/basket_detect_seg_analysis.py` |
| 原包 `scripts/preflight_camera_view.py` | `src/cuadc_perception/cuadc_perception/preflight_camera_view.py` |
| 原包 `scripts/flight_telemetry_recorder.py` | `src/cuadc_tools/cuadc_tools/flight_telemetry_recorder.py` |
| 原包 `scripts/mavlink_stream_configurator.py` | `src/cuadc_tools/cuadc_tools/mavlink_stream_configurator.py` |
| `aaa/routes/calibrate_route.py` | `src/cuadc_tools/cuadc_tools/calibrate_route.py` |
| 原包 `config/mission_real.yaml` | `src/cuadc_bringup/config/mission_real.yaml` |
| 原包 `models/*` | `src/cuadc_perception/models/*` |
| 原包 `deploy/build_onboard.sh` | 工作区 `scripts/build_onboard.sh` |
| 原包 `launch/full_mission_real.launch.py` | `src/cuadc_bringup/launch/flight.launch.py` |

## 保留与调整

任务 C++ 的非注释内容与原包一致。所有源代码注释和说明字符串译为中文；因此文本 SHA 会改变，不能再用整个 C++ 文件的逐字节一致来判断逻辑保留。三个模型的 SHA-256 与原压缩包一致，YAML 参数值和依赖版本也保留原值。

Python 视觉改用包内模块导入；模型通过 `cuadc_perception` 的 ament 索引查找，并保留源码目录回退。独立分析模块原来默认指向 `/home/nvidia/video/basket_new.engine`，现在默认查找随包 `basket_v3.pt`。未创建原包没有提供的 TensorRT 模型。

标定工具移除了固定用户目录和飞控序列号，改用 `--fcu-url`、`--output-dir`，以及可选的环境变量。两点距离和航向计算沿用原代码。

启动器明确读取本轮 `route_file` 并注入 `geographic_heading_deg`，修复原压缩包缺少上层航向启动脚本的问题。日志输出按架次组织。ROS 节点名和参数键保留原值，可执行入口改由新包提供。

## 未随发布包保留的内容

Python 缓存、旧构建清单和旧 launch 由对应的新文件替代；真实 `routes/test.yaml` 不发布，以合成 `routes/example.yaml` 展示格式。所有排除文件的名称记录在 `SOURCE_MANIFEST.json` 中。

原 `CMakeLists.txt` 安装一个压缩包没有提供的 `docs/` 目录，新构建清单只安装新包实际拥有的资源。原文档的绝对路径和不存在的 `run_full_mission.sh` 入口，替换为已提供的工作区脚本。

原 `LICENSES.md` 的中文历史译文保存在 `docs/ORIGINAL_LICENSES.md`，新项目许可按使用者选择统一声明为 AGPL-3.0-only。
