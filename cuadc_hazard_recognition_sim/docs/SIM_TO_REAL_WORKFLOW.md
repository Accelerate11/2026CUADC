# Sim-to-Real 识别验证流程

该小场景用于把“危险品识别”从全流程任务中拆出来单独验证。

推荐流程：

1. 固定 `seed`，生成一组可复现实验场景。
2. 在 Gazebo 中启动识别场景。
3. 使用下视 RGB-D 相机图像训练或调试识别算法。
4. 将算法输出与 `config/generated_scene.yaml` 真值对比。
5. 改变 `seed`，重复验证泛化能力。
6. 将 ROS2 输入接口保持为 `/perception/*`。
7. 实机 D435i 也 remap 到 `/perception/*`，复用同一套识别代码。

当前场景只解决识别问题，不引入飞控、投放、航线等干扰因素。识别稳定后，再接入全流程仿真 `cuadc_rescue_sim`。

## 真值字段

```yaml
recon_targets:
  recon_1:
    x: 0.1
    y: 1.2
    marker: toxic
    yaw: 1.57079632679
```

含义：

- `x`, `y`: Gazebo 世界坐标
- `marker`: 标识类别，`none` 表示该桶内没有标识
- `yaw`: 标识在桶内的平面旋转角

## 识别算法建议输出

建议后续识别节点输出：

```text
/perception/hazard_detection
```

字段建议：

```text
recon_bucket_id
marker_class
confidence
center_u
center_v
```

第一版也可以直接输出 JSON 或 CSV，先把算法链路跑通。
