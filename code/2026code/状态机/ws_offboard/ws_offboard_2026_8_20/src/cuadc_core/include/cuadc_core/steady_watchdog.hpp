#pragma once

#include <chrono>
#include <stdexcept>

namespace cuadc_core
{

class SteadyWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  explicit SteadyWatchdog(std::chrono::duration<double> timeout)
  : timeout_(timeout), last_kick_(Clock::now())
  {
    if (timeout_.count() <= 0.0) {
      throw std::invalid_argument("watchdog timeout must be positive");
    }
  }

  void kick() noexcept {last_kick_ = Clock::now();}

  bool expired() const noexcept
  {
    return std::chrono::duration<double>(Clock::now() - last_kick_) > timeout_;
  }

  double age_seconds() const noexcept
  {
    return std::chrono::duration<double>(Clock::now() - last_kick_).count();
  }

private:
  std::chrono::duration<double> timeout_;
  Clock::time_point last_kick_;
};

}  // namespace cuadc_core
