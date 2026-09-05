#include "gimbal_controller.h"

#include "app_config.h"

float GimbalController::clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

void GimbalController::reset(float roll_servo_deg, float pitch_servo_deg) {
    servo_.roll_deg = clamp(roll_servo_deg, config::kServoMinDeg, config::kServoMaxDeg);
    servo_.pitch_deg = clamp(pitch_servo_deg, config::kServoMinDeg, config::kServoMaxDeg);
}

void GimbalController::setTarget(float roll_deg, float pitch_deg) {
    target_roll_deg_ = clamp(roll_deg, -20.0F, 20.0F);
    target_pitch_deg_ = clamp(pitch_deg, -30.0F, 30.0F);
}

void GimbalController::setRollGains(const AxisGains& gains) { roll_gains_ = gains; }
void GimbalController::setPitchGains(const AxisGains& gains) { pitch_gains_ = gains; }

float GimbalController::stepAxis(float angle_error_deg, float measured_rate_dps,
                                 float dt_s, float direction, const AxisGains& gains,
                                 float current_servo_deg) {
    const float target_rate = clamp(gains.attitude_kp * angle_error_deg,
                                    -gains.max_target_rate_dps, gains.max_target_rate_dps);
    const float rate_error = target_rate - measured_rate_dps;
    const float servo_slew = clamp(gains.rate_kp * rate_error,
                                   -gains.max_servo_slew_dps, gains.max_servo_slew_dps);
    return clamp(current_servo_deg + direction * servo_slew * dt_s,
                 config::kServoMinDeg, config::kServoMaxDeg);
}

ServoAngles GimbalController::update(const AttitudeState& state, float dt_s) {
    dt_s = clamp(dt_s, 0.0005F, 0.02F);
    servo_.roll_deg = stepAxis(target_roll_deg_ - state.roll_deg, state.roll_rate_dps,
                               dt_s, config::kRollServoDirection, roll_gains_, servo_.roll_deg);
    servo_.pitch_deg = stepAxis(target_pitch_deg_ - state.pitch_deg, state.pitch_rate_dps,
                                dt_s, config::kPitchServoDirection, pitch_gains_, servo_.pitch_deg);
    return servo_;
}
