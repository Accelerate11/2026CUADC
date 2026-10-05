# 每轮中轴线航向

在工作区根目录运行 `bash scripts/calibrate_route.sh FCU_URL`，按提示保存本轮文件，例如 `round_01.yaml`，再将该文件传给 `scripts/run_flight.sh`。

`example.yaml` 使用合成坐标展示格式，不是实测航线。真实标定文件默认由 `.gitignore` 排除，不随开源仓库发布。

工具位于 `src/cuadc_tools/cuadc_tools/calibrate_route.py`；也可以使用安装后的 `ros2 run cuadc_tools calibrate_route --fcu-url FCU_URL --output-dir DIRECTORY`。
