#pragma once
#include <cmath>
#include <cstdint>

// One call per control tick; stamp identifies a camera exposure, not a tick.
struct ReleaseGate
{
  enum class Decision { WAIT, STABLE, RELAXED, FORCED };
  int count = 0;
  int64_t last_stamp = 0;
  void reset() {count = 0; last_stamp = 0;}
  Decision update(double elapsed, int64_t stamp, bool fresh, double error,
    double speed, bool observation_after_hold)
  {
    if (elapsed >= 6.0) {return Decision::FORCED;}
    const bool new_frame = stamp > last_stamp;
    if (new_frame) {last_stamp = stamp;}
    if (elapsed < 0.5) {count = 0; return Decision::WAIT;}
    if (elapsed >= 4.0) {
      count = 0;
      return new_frame && fresh && observation_after_hold && error <= 0.03 ?
        Decision::RELAXED : Decision::WAIT;
    }
    // Speed is checked on EVERY tick, including between camera frames.
    if (!fresh || !std::isfinite(speed) || speed >= 0.1) {
      count = 0; return Decision::WAIT;
    }
    if (!new_frame || !observation_after_hold) {return Decision::WAIT;}
    if (count == 0) {count = error <= 0.03 ? 1 : 0;}
    else if (error < 0.05) {++count;}
    else {count = 0;}
    return count >= 4 ? Decision::STABLE : Decision::WAIT;
  }
};
