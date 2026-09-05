#include "attitude_filter.h"

#include <cmath>

namespace {
constexpr float kRadToDeg = 57.29577951308232F;
constexpr float kFilterTauS = 0.50F;

float clampValue(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

float wrap180(float angle_deg) {
    while (angle_deg > 180.0F) angle_deg -= 360.0F;
    while (angle_deg < -180.0F) angle_deg += 360.0F;
    return angle_deg;
}
} // namespace

void AttitudeFilter::reset(float roll_deg, float pitch_deg) {
    roll_deg_ = roll_deg;
    pitch_deg_ = pitch_deg;
    initialized_ = true;
}

void AttitudeFilter::setLevelOffsets(float roll_offset_deg, float pitch_offset_deg) {
    roll_offset_deg_ = roll_offset_deg;
    pitch_offset_deg_ = pitch_offset_deg;
}

void AttitudeFilter::setCurrentAsLevel() {
    roll_offset_deg_ = roll_deg_;
    pitch_offset_deg_ = pitch_deg_;
}

AttitudeState AttitudeFilter::update(const ImuSample& sample, float dt_s) {
    const float accel_norm = std::sqrt(sample.accel_x_g * sample.accel_x_g +
                                       sample.accel_y_g * sample.accel_y_g +
                                       sample.accel_z_g * sample.accel_z_g);
    const float roll_acc = std::atan2(sample.accel_y_g, sample.accel_z_g) * kRadToDeg;
    const float pitch_acc = std::atan2(-sample.accel_x_g,
                                      std::sqrt(sample.accel_y_g * sample.accel_y_g +
                                                sample.accel_z_g * sample.accel_z_g)) * kRadToDeg;
    if (!initialized_) reset(roll_acc, pitch_acc);

    dt_s = clampValue(dt_s, 0.0005F, 0.02F);
    const float roll_pred = wrap180(roll_deg_ + sample.gyro_x_dps * dt_s);
    const float pitch_pred = wrap180(pitch_deg_ + sample.gyro_y_dps * dt_s);
    const float accel_confidence = clampValue(1.0F - std::fabs(accel_norm - 1.0F) / 0.30F,
                                              0.0F, 1.0F);
    const float correction = dt_s / (kFilterTauS + dt_s) * accel_confidence;
    roll_deg_ = wrap180(roll_pred + correction * wrap180(roll_acc - roll_pred));
    pitch_deg_ = wrap180(pitch_pred + correction * wrap180(pitch_acc - pitch_pred));

    AttitudeState state{};
    state.roll_deg = wrap180(roll_deg_ - roll_offset_deg_);
    state.pitch_deg = wrap180(pitch_deg_ - pitch_offset_deg_);
    state.roll_rate_dps = sample.gyro_x_dps;
    state.pitch_rate_dps = sample.gyro_y_dps;
    state.accel_norm_g = accel_norm;
    return state;
}
