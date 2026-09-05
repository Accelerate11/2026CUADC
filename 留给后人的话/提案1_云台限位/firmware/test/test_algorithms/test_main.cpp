#include <unity.h>

#include "attitude_filter.h"
#include "gimbal_controller.h"

void setUp() {}
void tearDown() {}

namespace {
ImuSample levelSample() {
    ImuSample sample{};
    sample.accel_z_g = 1.0F;
    return sample;
}

void test_filter_converges_to_level() {
    AttitudeFilter filter;
    filter.reset(15.0F, -10.0F);
    AttitudeState state{};
    const ImuSample sample = levelSample();
    for (int i = 0; i < 2000; ++i) state = filter.update(sample, 0.002F);
    TEST_ASSERT_FLOAT_WITHIN(0.02F, 0.0F, state.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.02F, 0.0F, state.pitch_deg);
}

void test_filter_rejects_large_linear_acceleration() {
    AttitudeFilter filter;
    filter.reset(0.0F, 0.0F);
    ImuSample sample = levelSample();
    sample.accel_x_g = 1.0F;
    const AttitudeState state = filter.update(sample, 0.002F);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, state.pitch_deg);
}

void test_controller_moves_positive_for_positive_error() {
    GimbalController controller;
    controller.reset();
    controller.setTarget(10.0F, 0.0F);
    const ServoAngles output = controller.update(AttitudeState{}, 0.002F);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0F, output.roll_deg);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, output.pitch_deg);
}

void test_controller_obeys_position_limit() {
    GimbalController controller;
    controller.reset();
    controller.setTarget(20.0F, 0.0F);
    ServoAngles output{};
    for (int i = 0; i < 10000; ++i) output = controller.update(AttitudeState{}, 0.002F);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 45.0F, output.roll_deg);
}
} // namespace

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_filter_converges_to_level);
    RUN_TEST(test_filter_rejects_large_linear_acceleration);
    RUN_TEST(test_controller_moves_positive_for_positive_error);
    RUN_TEST(test_controller_obeys_position_limit);
    return UNITY_END();
}
