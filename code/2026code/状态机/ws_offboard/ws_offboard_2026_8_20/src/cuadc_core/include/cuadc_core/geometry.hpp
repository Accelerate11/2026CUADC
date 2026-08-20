#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cuadc_core
{

constexpr double kPi = 3.14159265358979323846;

struct Point3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline bool finite(const Point3 & point) noexcept
{
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

inline double distance_xy(const Point3 & lhs, const Point3 & rhs) noexcept
{
  return std::hypot(lhs.x - rhs.x, lhs.y - rhs.y);
}

inline double distance_xyz(const Point3 & lhs, const Point3 & rhs) noexcept
{
  return std::hypot(distance_xy(lhs, rhs), lhs.z - rhs.z);
}

inline double normalize_angle(double radians) noexcept
{
  return std::atan2(std::sin(radians), std::cos(radians));
}

class FieldFrame
{
public:
  FieldFrame(Point3 home, double yaw_enu_rad)
  : home_(home), cosine_(std::cos(yaw_enu_rad)), sine_(std::sin(yaw_enu_rad))
  {
    if (!finite(home_) || !std::isfinite(yaw_enu_rad)) {
      throw std::invalid_argument("FieldFrame requires finite home and yaw");
    }
  }

  Point3 field_to_local(const Point3 & field) const
  {
    if (!finite(field)) {
      throw std::invalid_argument("field point must be finite");
    }
    return Point3{
      home_.x + cosine_ * field.x - sine_ * field.y,
      home_.y + sine_ * field.x + cosine_ * field.y,
      home_.z + field.z};
  }

  Point3 local_to_field(const Point3 & local) const
  {
    if (!finite(local)) {
      throw std::invalid_argument("local point must be finite");
    }
    const double dx = local.x - home_.x;
    const double dy = local.y - home_.y;
    return Point3{
      cosine_ * dx + sine_ * dy,
      -sine_ * dx + cosine_ * dy,
      local.z - home_.z};
  }

private:
  Point3 home_;
  double cosine_;
  double sine_;
};

}  // namespace cuadc_core
