#pragma once

#include <cstdint>

struct ImuSample {
    float accel_x_g{0.0F};
    float accel_y_g{0.0F};
    float accel_z_g{0.0F};
    float gyro_x_dps{0.0F};
    float gyro_y_dps{0.0F};
    float gyro_z_dps{0.0F};
    float temperature_c{0.0F};
    uint32_t timestamp_us{0U};
};

struct AttitudeState {
    float roll_deg{0.0F};
    float pitch_deg{0.0F};
    float roll_rate_dps{0.0F};
    float pitch_rate_dps{0.0F};
    float accel_norm_g{1.0F};
};

enum class SystemState : uint8_t {
    Boot,
    Calibrating,
    Ready,
    Armed,
    Fault,
};
