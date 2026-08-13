/*
 * CUADC 2026 视觉双投放状态机（救援优先 Rescue V2）
 *
 * 这个代码的开源
 * 核心流程：
 *   WAIT_FCU -> WAIT_NAV_STABLE -> PRESTREAM -> WAIT_GUIDED -> WAIT_ARM
 *   -> TAKEOFF(4.0 m) -> SEARCH(2.2 m) -> ALIGN_COARSE(1.7 m)
 *   -> RELEASE(A1/SERVO9) -> ALIGN_COARSE(1.7 m) -> RELEASE(A2/SERVO10)
 *   -> RECON_TRANSIT(2.5 m) -> RECON_DESCEND(1.2 m)
 *   -> RECON_SCAN(6 photo waypoints, 1.2 m)
 *   -> RECON_RETURN_CLIMB(4.0 m) -> RECON_RETURN(4.0 m)
 *   -> LAND -> DISARM -> DONE
 *
 * 设计原则：
 * 1. 飞手切入 GUIDED 作为最终启动确认；安全门禁通过后状态机自动解锁。
 * 2. 启动时锁定 mission_home 和 compass_hdg；全任务航向固定，不朝目标转头。
 * 3. 起飞使用 CommandTOL，起飞阶段不发布位置 setpoint，避免命令冲突。
 * 4. 2.2 m 搜索至少两个稳定、空间独立的桶；一旦两个成立立即停止搜索并冻结，不等待第三桶。若同一时刻已有更多稳定桶，只在现有候选中按直径升序选最小两个，不增加等待。
 * 5. 两个投放口使用独立机体 FLU 外参：
 *      A1/SERVO9:  [ 0.026, -0.065, -0.32 ]
 *      A2/SERVO10: [-0.026,  0.065, -0.32 ]
 * 6. 不再做为了A区而延时的厘米级精对准。目标锁定后下降到 1.7 m 做单阶段宽容对准，
 *    冻结补偿后的投放位姿；RELEASE 阶段不依赖后续视觉帧。
 * 7. 第一瓶完成后直接使用已冻结的第二目标位置，不重新搜索、不要求重新建 ID。
 * 8. 投放前仍满足目标身份、投放区边界、XY/高度、速度、姿态和角速度门限；
 *    XY门限放宽至0.35 m、稳定0.5 s。单次超时允许重新对准一次，仍失败才保瓶返航。
 * 9. 两瓶完成后不直接返航：先高速进入灾情侦察区，在1.2 m按6个固定航点拍摄RGB照片。
 *    侦察阶段不做危险物在线识别、不做YOLO/深度推理，只保存原始RGB照片。
 * 10. 六张照片触发完成后原地爬升至4.0 m，以更高速度返航；故障返航仍保留原3.0 m/s安全速度。
 * 11. 投放搜索航线横向内缩0.90 m，最外侧名义航线约为field_y=±3.10 m；
 *     SEARCH阶段加入field_y=±3.60 m主动回收边界，避免跟踪超调越出场地。
 * 12. field_lateral_offset_m用于统一横向标定：+Y为飞机左侧；负值可将投放/侦察整体右移。
 * 13. 落地采用相对高度和速度连续稳定确认后显式上锁。
 *
 * 位置源：/mavros/local_position/odom（ArduPilot EKF 融合后的本地位置）
 * 视觉源：/perception/drop_buckets_body
 * 舵机：MAV_CMD_DO_SET_SERVO，A1/A2=SERVO9/10，1200 us 挂载，1500 us 释放。
 */
