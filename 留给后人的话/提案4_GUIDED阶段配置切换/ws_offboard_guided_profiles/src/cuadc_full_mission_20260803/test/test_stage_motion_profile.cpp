#include "../src/stage_motion_profile.hpp"
#include <iostream>
#include <stdexcept>

void check(bool good, const char * message)
{
  if (!good) {throw std::runtime_error(message);}
}

int main()
{
  using namespace cuadc_guided;
  const auto profiles = demonstration_profiles();
  for (const auto & profile : profiles) {check(profile.valid(), "invalid example profile");}
  MotionProfile invalid = profiles[0]; invalid.vision_gain = 2.0;
  check(!invalid.valid(), "unsafe vision gain accepted");
  ContinuousReference reference;
  reference.reset({0.0, 0.0, 0.0});
  const auto verify_step = [&](ReferencePoint goal, const MotionProfile & profile, bool brake) {
    const auto old_position = reference.position();
    const auto old_velocity = reference.velocity();
    const auto next = reference.step(goal, profile, 0.05, brake);
    const auto velocity = reference.velocity();
    check(std::hypot(velocity.x-old_velocity.x, velocity.y-old_velocity.y) <=
      profile.acceleration_xy_m_s2*0.05+1e-8, "XY acceleration jump");
    check(std::abs(velocity.z-old_velocity.z) <=
      profile.acceleration_z_m_s2*0.05+1e-8, "Z acceleration jump");
    check(std::hypot(next.x-old_position.x, next.y-old_position.y) <=
      std::max(std::hypot(old_velocity.x,old_velocity.y),profile.speed_xy_m_s)*0.05+1e-8,
      "position step exceeds reference velocity");
    check(std::isfinite(next.x) && std::isfinite(next.y) && std::isfinite(next.z), "nonfinite reference");
  };
  for (int i=0; i<120; ++i) {verify_step({40.0,20.0,3.0}, profiles[0], false);}
  check(std::hypot(reference.velocity().x,reference.velocity().y)>2.9, "cruise not reached");
  // A new visual target behind the aircraft remains continuous.
  for (int i=0; i<20; ++i) {verify_step({-40.0,-20.0,1.0}, profiles[0], false);}
  for (int i=0; i<200 && !reference.stopped(); ++i) {
    verify_step({}, profiles[0], true);
  }
  check(reference.stopped(), "gate brake never stopped");
  const auto held = reference.position();
  for (int i=0; i<20; ++i) {verify_step({}, profiles[0], true);}
  check(std::hypot(reference.position().x-held.x,reference.position().y-held.y)<1e-10,
    "braked reference did not hold position");
  for (int i=0; i<200; ++i) {verify_step({held.x+0.8,held.y-0.5,1.0}, profiles[4], false);}
  check(std::hypot(reference.velocity().x,reference.velocity().y)<=profiles[4].speed_xy_m_s+1e-8,
    "release cap bypassed after cruise transition");
  SettledGate gate;
  check(!gate.update(0.0,true,0.0,0.0,true,0.25,0.15,0.5), "single sample opened gate");
  check(!gate.update(0.6,false,0.0,0.0,true,0.25,0.15,0.5), "stale nav opened gate");
  check(!gate.update(0.7,true,0.3,0.0,true,0.25,0.15,0.5), "moving nav opened gate");
  check(!gate.update(0.8,true,0.0,0.0,false,0.25,0.15,0.5), "moving reference opened gate");
  check(!gate.update(1.0,true,0.0,0.0,true,0.25,0.15,0.5), "unstable nav opened gate");
  check(gate.update(1.6,true,0.0,0.0,true,0.25,0.15,0.5), "settled gate stayed closed");
  std::cout << "continuous reference and physical settle gate passed\n";
}
