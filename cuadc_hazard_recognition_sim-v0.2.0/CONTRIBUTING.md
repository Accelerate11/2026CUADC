# Contributing

感谢参与 CUADC 危险物侦察仿真项目。

1. 不要提交 `build/`、`install/`、`log/`、ONNX 权重或完整运行帧。
2. 保持“状态机不读取随机桶位真值”的隔离原则。
3. 新增识别策略必须具有类别通用性，禁止针对单个 seed 或单一类别硬编码。
4. RTK 修改必须注明坐标系、轴方向、单位和高度基准。
5. 相机修改必须注明图像轴、外参和投影参数来源。

提交前运行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cuadc_hazard_recognition_sim
python3 -m py_compile scripts/*.py launch/*.py
bash -n scripts/*.sh
```

功能修改至少用两个随机 seed 回归，并报告覆盖端点、分类结果、空桶误报、定位误差和两个可视化窗口状态。不要提交竞赛规则原文、未经授权的数据集、私有模型、密钥或本机绝对路径。
