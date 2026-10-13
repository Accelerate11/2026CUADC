> 历史基线文档：下文记录原公开 V16 的设计、配置或验证结果。本提案新增阶段配置切换，并将默认飞行开关关闭；当前说明以工作区 README 和上级提案 docs 为准。原 V16 的测试结果不表示本提案已经实飞验收。

# 常见问题

## 校验失败

先看报错文件名。刚解压就失败时重新核对ZIP外部SHA256；修改过参数或源码则会与发布基线不同。`SOURCE_MANIFEST.json` 是来源记录，`SHA256SUMS` 是本次发布内容，均在最终文档完成后统一生成。

## 找不到ROS包或launch

使用Ubuntu22.04和ROS2Humble，先编译，再依次source `/opt/ros/humble/setup.bash` 与工作区 `install/setup.bash`。原包名保留 `cuadc_full_mission_20260803`，不是目录名。复制到新机器后重新编译，不复制另一机器的install目录。

## PyTorch、OpenCV或NumPy冲突

按environment说明建立Python3.10环境，原V16要求NumPy1.x。ROS的cv_bridge与pip轮子需要匹配；不要在系统Python中混装多个OpenCV包。编译默认设置 `PYTHONNOUSERSITE=1`，避免用户级setuptools影响Humble；机载运行前仍需要检查实际解释器能够导入视觉依赖。

## IntelGPU不可用

先用环境工具检查OpenVINO和GPU驱动，再按原接口用 `CUADC_VISION_DEVICE=cpu` 做CPU对照。随包OpenVINO模型对应basket_detect.onnx；不要把basket_v3.pt直接当成该GPU模型。没有显卡驱动时仅安装Python包不能产生GPU设备。

## 找不到相机或串口

使用实际 `/dev/serial/by-id/` 路径和波特率。相机原序列号仍在V16参数中，运行时传本机 `--camera-serial`。Linux用户需要相应串口/视频设备访问权限。包装入口使用串口锁和原启动守护，先结束自己启动的旧任务；不要直接绕过独占检查。

## 没有READY_FOR_GUIDED

看 `CAMERA_TIME`、`FCU_TIME`、`VISION_TIME` 与就绪日志。时间同步、视觉采集、静止和舵机初始化必须满足原门槛。准备未完成就进入GUIDED时，需要按原提示切回其他模式后重新切入，不会绕过准备条件起飞。

## 参数看起来没生效

完整任务使用full_mission/mission.yaml，仅投放使用drop_only/mission.yaml，包内基础config是另一份文件。启动器将本轮航向、曝光和日志路径等写入mission_runtime.yaml，相机、模型和推理设备还会由launch参数或环境变量覆盖。需要结合运行副本、启动命令和环境记录核对。实际搜索固定两条线，配置中的search_lane_count不是本版动态路线开关。
