// SPDX-License-Identifier: AGPL-3.0-only
// Exercises the unchanged V16 production header; no ROS, camera, or FCU access.
#include "../src/release_gate.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using Decision = ReleaseGate::Decision;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void hold_requires_post_hold_exposures()
{
  ReleaseGate gate;
  require(gate.update(0.499, 100, true, 0.0, 0.0, true) == Decision::WAIT,
    "release before the 0.5 second hold");
  require(gate.count == 0, "pre-hold exposure counted");
  require(gate.update(0.5, 100, true, 0.0, 0.0, true) == Decision::WAIT &&
    gate.count == 0, "cached pre-hold exposure counted at hold boundary");
  gate.update(0.6, 101, true, 0.0, 0.0, false);
  require(gate.count == 0, "exposure captured before hold was accepted");
  gate.update(0.7, 102, true, 0.03, 0.09, true);
  require(gate.count == 1, "valid exposure could not begin the sequence");
}

void four_distinct_exposures_and_hysteresis()
{
  ReleaseGate gate;
  gate.update(1.0, 1, true, 0.031, 0.09, true);
  require(gate.count == 0, "sequence started beyond the 3 cm entry threshold");
  require(gate.update(1.1, 2, true, 0.03, 0.09, true) == Decision::WAIT,
    "one exposure released");
  for (int tick = 0; tick < 20; ++tick) {
    require(gate.update(1.2, 2, true, 0.01, 0.09, true) == Decision::WAIT &&
      gate.count == 1, "control ticks counted as camera exposures");
  }
  gate.update(1.3, 1, true, 0.01, 0.09, true);
  require(gate.count == 1 && gate.last_stamp == 2, "out-of-order exposure counted");
  require(gate.update(1.4, 3, true, 0.049, 0.09, true) == Decision::WAIT,
    "two exposures released");
  require(gate.update(1.5, 4, true, 0.04, 0.09, true) == Decision::WAIT,
    "three exposures released");
  require(gate.update(1.6, 5, true, 0.04, 0.09, true) == Decision::STABLE &&
    gate.count == 4, "four qualifying exposures did not release");
}

void invalid_ticks_break_the_stable_sequence()
{
  ReleaseGate gate;
  gate.update(1.0, 1, true, 0.02, 0.09, true);
  gate.update(1.1, 1, true, 0.02, 0.1, true);
  require(gate.count == 0, "speed at 0.1 m/s between frames did not reset count");
  gate.update(1.2, 2, true, 0.02, 0.09, true);
  gate.update(1.3, 2, false, 0.02, 0.09, true);
  require(gate.count == 0, "stale observation between frames did not reset count");
  gate.update(1.4, 3, true, 0.02, 0.09, true);
  gate.update(1.5, 3, true, 0.02, std::numeric_limits<double>::quiet_NaN(), true);
  require(gate.count == 0, "non-finite speed did not reset count");
  gate.update(1.6, 4, true, 0.02, 0.09, true);
  gate.update(1.7, 5, true, 0.05, 0.09, true);
  require(gate.count == 0, "5 cm continuation boundary was accepted");
  gate.update(1.8, 6, true, 0.04, 0.09, true);
  require(gate.count == 0, "reset sequence restarted without meeting entry threshold");
}

void relaxed_stage_still_requires_a_new_fresh_exposure()
{
  ReleaseGate gate;
  gate.update(3.9, 1, true, 0.02, 0.09, true);
  require(gate.update(4.0, 1, true, 0.02, 0.5, true) == Decision::WAIT &&
    gate.count == 0, "relaxed stage accepted a cached exposure");
  require(gate.update(4.1, 2, false, 0.02, 0.5, true) == Decision::WAIT,
    "relaxed stage accepted a stale exposure");
  require(gate.update(4.2, 3, true, 0.02, 0.5, false) == Decision::WAIT,
    "relaxed stage accepted an exposure from before the hold");
  require(gate.update(4.3, 4, true, 0.031, 0.5, true) == Decision::WAIT,
    "relaxed stage accepted error above 3 cm");
  require(gate.update(4.4, 5, true, 0.03, 0.5, true) == Decision::RELAXED,
    "V16 relaxed stage incorrectly retained the earlier speed/count requirement");
  require(gate.update(4.5, 5, true, 0.01, 0.0, true) == Decision::WAIT,
    "relaxed stage reused an exposure");
}

void six_second_deadline_is_unconditional()
{
  ReleaseGate gate;
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  require(gate.update(5.999, 0, false, invalid, invalid, false) == Decision::WAIT,
    "forced release happened before the deadline");
  // This records V16's existing forced-release behavior, not a safety guarantee.
  require(gate.update(6.0, 0, false, invalid, invalid, false) == Decision::FORCED,
    "V16 deadline was extended by invalid or missing observations");
  require(gate.update(20.0, 0, false, invalid, invalid, false) == Decision::FORCED,
    "elapsed deadline stopped applying");
}

void reset_clears_history_for_the_next_payload()
{
  ReleaseGate gate;
  gate.update(1.0, 500, true, 0.01, 0.0, true);
  gate.reset();
  require(gate.count == 0 && gate.last_stamp == 0, "payload reset retained history");
  require(gate.update(0.6, 1, true, 0.03, 0.0, true) == Decision::WAIT &&
    gate.count == 1, "new payload could not begin a fresh sequence");
}
}  // namespace

int main()
{
  try {
    hold_requires_post_hold_exposures();
    four_distinct_exposures_and_hysteresis();
    invalid_ticks_break_the_stable_sequence();
    relaxed_stage_still_requires_a_new_fresh_exposure();
    six_second_deadline_is_unconditional();
    reset_clears_history_for_the_next_payload();
  } catch (const std::exception & error) {
    std::cerr << "Release gate regression failure: " << error.what() << '\n';
    return 1;
  }
  std::cout << "Release gate regression checks passed (6 scenarios)\n";
  return 0;
}
