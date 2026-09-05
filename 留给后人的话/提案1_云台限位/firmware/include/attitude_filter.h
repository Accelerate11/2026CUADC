#pragma once

#include "types.h"

class AttitudeFilter {
public:
    void reset(float roll_deg, float pitch_deg);
    AttitudeState update(const ImuSample& sample, float dt_s);
    void setLevelOffsets(float roll_offset_deg, float pitch_offset_deg);
    void setCurrentAsLevel();

private:
    bool initialized_{false};
    float roll_deg_{0.0F};
    float pitch_deg_{0.0F};
    float roll_offset_deg_{0.0F};
    float pitch_offset_deg_{0.0F};
};
