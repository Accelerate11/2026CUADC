#pragma once

#include "types.h"

struct AxisGains {
    float attitude_kp{4.0F};
    float rate_kp{1.2F};
    float max_target_rate_dps{120.0F};
    float max_servo_slew_dps{160.0F};
};

struct ServoAngles {
    float roll_deg{0.0F};
    float pitch_deg{0.0F};
};

class GimbalController {
public:
    void reset(float roll_servo_deg = 0.0F, float pitch_servo_deg = 0.0F);
    ServoAngles update(const AttitudeState& state, float dt_s);
    void setTarget(float roll_deg, float pitch_deg);
    void setRollGains(const AxisGains& gains);
    void setPitchGains(const AxisGains& gains);
    AxisGains rollGains() const { return roll_gains_; }
    AxisGains pitchGains() const { return pitch_gains_; }
    float targetRollDeg() const { return target_roll_deg_; }
    float targetPitchDeg() const { return target_pitch_deg_; }

private:
    static float clamp(float value, float low, float high);
    static float stepAxis(float angle_error_deg, float measured_rate_dps,
                          float dt_s, float direction, const AxisGains& gains,
                          float current_servo_deg);

    float target_roll_deg_{0.0F};
    float target_pitch_deg_{0.0F};
    ServoAngles servo_{};
    AxisGains roll_gains_{};
    AxisGains pitch_gains_{};
};
