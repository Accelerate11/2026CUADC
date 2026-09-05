#pragma once

#include <cstdint>

namespace config {

constexpr uint32_t kControlRateHz = 500U;
constexpr uint32_t kServoRateHz = 50U;
constexpr uint32_t kTelemetryRateHz = 20U;
constexpr uint32_t kUartBaud = 115200U;

constexpr uint16_t kServoPeriodUs = 20000U;
constexpr float kServoMinDeg = -45.0F;
constexpr float kServoMaxDeg = 45.0F;

// 必须根据实际舵机和机构逐项标定。先拆下舵臂再修改。
constexpr uint16_t kRollMinPulseUs = 1000U;
constexpr uint16_t kRollCenterPulseUs = 1500U;
constexpr uint16_t kRollMaxPulseUs = 2000U;
constexpr uint16_t kPitchMinPulseUs = 1000U;
constexpr uint16_t kPitchCenterPulseUs = 1500U;
constexpr uint16_t kPitchMaxPulseUs = 2000U;

// +1 表示舵机命令增大使对应 IMU 正向角速度增大；若自激，立即 DISARM 并改为 -1。
constexpr float kRollServoDirection = 1.0F;
constexpr float kPitchServoDirection = 1.0F;

// ICM 模块相对托架的符号映射。更复杂的旋转应在 Icm42688::read() 后增加 3x3 旋转矩阵。
constexpr float kImuGyroXSign = 1.0F;
constexpr float kImuGyroYSign = 1.0F;
constexpr float kImuGyroZSign = 1.0F;
constexpr float kImuAccelXSign = 1.0F;
constexpr float kImuAccelYSign = 1.0F;
constexpr float kImuAccelZSign = 1.0F;

constexpr uint32_t kGyroCalibrationSamples = 1000U; // 2 s @ 500 Hz
constexpr float kCalibrationMaxGyroDps = 3.0F;
constexpr float kCalibrationAccelToleranceG = 0.12F;
constexpr uint32_t kMaxConsecutiveImuFailures = 10U;

} // namespace config
