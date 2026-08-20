#pragma once

#include <atomic>

namespace cuadc_vehicle
{

enum class ControlAuthority
{
  IDLE,
  MISSION,
  PILOT_OVERRIDE,
  FAULT
};

class CommandAuthority
{
public:
  bool grant_mission() noexcept
  {
    ControlAuthority expected = ControlAuthority::IDLE;
    return authority_.compare_exchange_strong(expected, ControlAuthority::MISSION);
  }

  void revoke_for_pilot() noexcept
  {
    authority_.store(ControlAuthority::PILOT_OVERRIDE);
  }

  void revoke_for_fault() noexcept
  {
    // Pilot override is the highest-priority terminal state while armed. A
    // later health fault must not silently give command authority back to
    // software after the pilot has taken control.
    auto current = authority_.load();
    while (current != ControlAuthority::PILOT_OVERRIDE &&
      !authority_.compare_exchange_weak(current, ControlAuthority::FAULT))
    {
    }
  }
  void reset_disarmed() noexcept {authority_.store(ControlAuthority::IDLE);}
  bool mission_may_command() const noexcept
  {
    return authority_.load() == ControlAuthority::MISSION;
  }
  bool fault_landing_may_command() const noexcept
  {
    return authority_.load() == ControlAuthority::FAULT;
  }
  ControlAuthority current() const noexcept {return authority_.load();}

private:
  std::atomic<ControlAuthority> authority_{ControlAuthority::IDLE};
};

}  // namespace cuadc_vehicle
