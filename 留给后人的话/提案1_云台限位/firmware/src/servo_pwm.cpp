#include "servo_pwm.h"

#include "app_config.h"

ServoPwm::ServoPwm(TIM_HandleTypeDef& timer, uint32_t channel,
                   uint16_t min_pulse_us, uint16_t center_pulse_us,
                   uint16_t max_pulse_us)
    : timer_(timer), channel_(channel), min_pulse_us_(min_pulse_us),
      center_pulse_us_(center_pulse_us), max_pulse_us_(max_pulse_us),
      pulse_us_(center_pulse_us) {}

float ServoPwm::clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

bool ServoPwm::begin() {
    setAngle(0.0F);
    return HAL_TIM_PWM_Start(&timer_, channel_) == HAL_OK;
}

void ServoPwm::setAngle(float angle_deg) {
    angle_deg_ = clamp(angle_deg, config::kServoMinDeg, config::kServoMaxDeg);
    const float normalized = angle_deg_ / 45.0F;
    float pulse = static_cast<float>(center_pulse_us_);
    if (normalized >= 0.0F) {
        pulse += normalized * static_cast<float>(max_pulse_us_ - center_pulse_us_);
    } else {
        pulse += normalized * static_cast<float>(center_pulse_us_ - min_pulse_us_);
    }
    pulse_us_ = static_cast<uint16_t>(clamp(pulse,
        static_cast<float>(min_pulse_us_), static_cast<float>(max_pulse_us_)));
    __HAL_TIM_SET_COMPARE(&timer_, channel_, pulse_us_);
}

void ServoPwm::hold() { __HAL_TIM_SET_COMPARE(&timer_, channel_, pulse_us_); }
void ServoPwm::disable() { (void)HAL_TIM_PWM_Stop(&timer_, channel_); }
