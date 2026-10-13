# 固定坐标双投放测试（V16）

此入口打开视觉与录像，任务使用固定场地坐标 `(32.5, -2.6)`、`(32.5, 2.6)` 进行两次投放，然后继续原侦察航线、返航、降落和上锁。固定坐标与任务行为沿用 V16，适用场地必须自行核对。

从工作区根目录执行，`round_01` 必须是本轮真实标定文件：

```bash
bash fixed_release_test/run_fixed_release_test.sh round_01 --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 --preflight
bash fixed_release_test/run_fixed_release_test.sh round_01 --fcu-url serial:///dev/serial/by-id/YOUR_FCU:115200 expo=35
```

必须替换实际设备，或设置 `CUADC_FCU_URL`。读取 `full_mission/mission.yaml`，运行时启用固定投放测试模式；不修改参数源文件。日志在 `fixed_release_test/logs/flight_<时间>/`。此测试入口不自动生成后置报告；可按 `tools/README_flight_summary.md` 手动生成本地报告。
