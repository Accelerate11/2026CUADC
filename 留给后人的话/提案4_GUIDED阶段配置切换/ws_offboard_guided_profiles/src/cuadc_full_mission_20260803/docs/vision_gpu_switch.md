# Vision inference backend

The `basket_vision_ros_node.device` parameter selects the inference path:

- `"intel:gpu"`: packaged FP32 OpenVINO copy of `basket_detect.onnx` on the
  Intel iGPU. This is the default in full mission, drop only, fixed release
  test (which uses the full mission YAML), and indoor projection preflight.
- `"cpu"`: original `basket_detect.onnx` on ONNX Runtime CPU. This is the
  manual rollback setting; no model conversion or rebuild is required.

Edit `device` in `full_mission/mission.yaml`, `drop_only/mission.yaml`, or the
package `config/mission_real.yaml` for indoor projection preflight. The
selection takes effect on the next launch, not during an active flight.
For a one-time CPU rollback without editing YAML, prefix a launch command
with `CUADC_VISION_DEVICE=cpu`; the full mission, drop-only, fixed-release,
and indoor projection launchers all accept this environment override.

If OpenVINO or its GPU is unavailable during startup, the vision node logs
an error and uses the original CPU model before the mission starts. If GPU
inference fails later, it switches once to CPU and continues publishing the
same detection topic. The active device and model path appear in vision
diagnostics. The main mission node and flight control logic are unchanged.
