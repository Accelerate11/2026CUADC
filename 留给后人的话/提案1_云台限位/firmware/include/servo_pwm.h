#pragma once

#include "stm32g4xx_hal.h"

class ServoPwm {
public:
    ServoPwm(TIM_HandleTypeDef& timer, uint32_t channel,
             uint16_t min_pulse_us, uint16_t center_pulse_us,
             uint16_t max_pulse_us);
    bool begin();
    void setAngle(float angle_deg);
    void hold();
    void disable();
    float angleDeg() const { return angle_deg_; }
    uint16_t pulseUs() const { return pulse_us_; }

private:
    static float clamp(float value, float low, float high);

    TIM_HandleTypeDef& timer_;
    uint32_t channel_;
    uint16_t min_pulse_us_;
    uint16_t center_pulse_us_;
    uint16_t max_pulse_us_;
    uint16_t pulse_us_{1500U};
    float angle_deg_{0.0F};
};
