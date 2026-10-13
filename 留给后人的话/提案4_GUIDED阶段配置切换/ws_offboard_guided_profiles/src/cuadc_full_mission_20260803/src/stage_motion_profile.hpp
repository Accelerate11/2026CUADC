#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace cuadc_guided
{
enum class MotionStage : std::size_t
{
  CRUISE, SEARCH, APPROACH, ALIGN, RELEASE, RECON, RETURN, COUNT
};

inline const char * stage_name(MotionStage stage)
{
  constexpr std::array<const char *, 7> names{
    "cruise", "search", "approach", "align", "release", "recon", "return"};
  return names.at(static_cast<std::size_t>(stage));
}

struct ReferencePoint {double x = 0.0; double y = 0.0; double z = 0.0;};

struct MotionProfile
{
  double speed_xy_m_s = 1.0;
  double acceleration_xy_m_s2 = 0.5;
  double speed_z_m_s = 0.5;
  double acceleration_z_m_s2 = 0.5;
  double vision_gain = 1.0;

  bool valid() const
  {
    return std::isfinite(speed_xy_m_s) && speed_xy_m_s > 0.0 &&
      std::isfinite(acceleration_xy_m_s2) && acceleration_xy_m_s2 > 0.0 &&
      std::isfinite(speed_z_m_s) && speed_z_m_s > 0.0 &&
      std::isfinite(acceleration_z_m_s2) && acceleration_z_m_s2 > 0.0 &&
      std::isfinite(vision_gain) && vision_gain > 0.0 && vision_gain <= 1.0;
  }
};

inline std::array<MotionProfile, 7> demonstration_profiles()
{
  return {{{3.0, 1.0, 0.7, 0.5, 1.0}, {1.25, 0.6, 0.5, 0.4, 1.0},
    {1.5, 0.6, 0.5, 0.4, 0.8}, {0.375, 0.4, 0.16, 0.25, 0.6},
    {0.15, 0.25, 0.16, 0.25, 0.35}, {1.5, 0.8, 0.7, 0.5, 1.0},
    {3.0, 1.0, 0.7, 0.5, 1.0}}};
}

// A persistent reference generator, not an aircraft dynamics guarantee.
// Never reset position/velocity at a stage change. Brake with the old profile,
// then select the new limits at rest. Acceleration bounds apply to reference
// velocity; the autopilot determines actual acceleration and tracking error.
class ContinuousReference
{
public:
  void reset(ReferencePoint position)
  {
    position_ = position;
    velocity_ = {};
    initialized_ = true;
  }
  bool initialized() const {return initialized_;}
  const ReferencePoint & position() const {return position_;}
  const ReferencePoint & velocity() const {return velocity_;}
  bool stopped(double epsilon = 1.0e-6) const
  {
    return std::hypot(velocity_.x, velocity_.y) <= epsilon &&
      std::abs(velocity_.z) <= epsilon;
  }

  ReferencePoint step(ReferencePoint goal, const MotionProfile & limits, double dt, bool brake)
  {
    if (!initialized_ || !limits.valid() || !std::isfinite(dt) || dt <= 0.0) {
      return position_;
    }
    // Substeps preserve the bound during normal timer jitter. A long executor
    // stall advances at most 0.2 s; this slows reference time instead of jumping.
    double remaining = std::min(dt, 0.2);
    while (remaining > 1.0e-10) {
      const double h = std::min(remaining, 0.01);
      integrate(goal, limits, h, brake);
      remaining -= h;
    }
    return position_;
  }

private:
  void integrate(ReferencePoint goal, const MotionProfile & limits, double h, bool brake)
  {
    ReferencePoint desired;
    if (!brake) {
      const double dx = goal.x - position_.x;
      const double dy = goal.y - position_.y;
      const double distance = std::hypot(dx, dy);
      if (distance > 1.0e-10) {
        // The proportional term converges gently near the endpoint; the
        // stopping-distance term starts braking before arrival.
        const double speed = std::min({limits.speed_xy_m_s, distance * 2.0,
          std::sqrt(2.0 * limits.acceleration_xy_m_s2 * distance)});
        desired.x = dx / distance * speed;
        desired.y = dy / distance * speed;
      }
      const double dz = goal.z - position_.z;
      desired.z = std::copysign(std::min({limits.speed_z_m_s, std::abs(dz) * 2.0,
        std::sqrt(2.0 * limits.acceleration_z_m_s2 * std::abs(dz))}), dz);
    }
    const ReferencePoint old = velocity_;
    const double dvx = desired.x - old.x;
    const double dvy = desired.y - old.y;
    const double dv = std::hypot(dvx, dvy);
    const double ratio = dv > 1.0e-12 ?
      std::min(1.0, limits.acceleration_xy_m_s2 * h / dv) : 0.0;
    velocity_.x += dvx * ratio;
    velocity_.y += dvy * ratio;
    velocity_.z += std::clamp(desired.z - old.z,
      -limits.acceleration_z_m_s2 * h, limits.acceleration_z_m_s2 * h);
    position_.x += 0.5 * (old.x + velocity_.x) * h;
    position_.y += 0.5 * (old.y + velocity_.y) * h;
    position_.z += 0.5 * (old.z + velocity_.z) * h;
  }
  ReferencePoint position_;
  ReferencePoint velocity_;
  bool initialized_ = false;
};

// Keep physical telemetry and reference braking as separate gate conditions.
class SettledGate
{
public:
  void reset() {stable_since_ = -1.0;}
  bool update(double now_s, bool fresh, double xy_speed, double z_speed,
    bool reference_stopped, double xy_limit, double z_limit, double hold_s)
  {
    const bool good = fresh && reference_stopped && std::isfinite(xy_speed) &&
      std::isfinite(z_speed) && xy_speed <= xy_limit && std::abs(z_speed) <= z_limit;
    if (!good) {reset(); return false;}
    if (stable_since_ < 0.0) {stable_since_ = now_s;}
    return now_s - stable_since_ >= hold_s;
  }
private:
  double stable_since_ = -1.0;
};
}  // namespace cuadc_guided
