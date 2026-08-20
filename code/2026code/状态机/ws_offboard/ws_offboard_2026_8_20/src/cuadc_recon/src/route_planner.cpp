#include <cuadc_recon/route_planner.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace cuadc_recon
{

std::vector<cuadc_core::Point3> generate_waypoint_route(const RouteSpec & spec)
{
  const bool finite = std::isfinite(spec.center_x_m) &&
    std::isfinite(spec.center_y_m) && std::isfinite(spec.length_x_m) &&
    std::isfinite(spec.width_y_m) && std::isfinite(spec.altitude_m);
  if (!finite || spec.length_x_m <= 0.0 || spec.width_y_m <= 0.0 ||
    spec.altitude_m <= 0.0 || spec.lane_count < 1 || spec.lane_count > 20)
  {
    throw std::invalid_argument("invalid waypoint reconnaissance route spec");
  }

  std::vector<cuadc_core::Point3> route;
  route.reserve(static_cast<std::size_t>(spec.lane_count * 2));
  const double x_min = spec.center_x_m - spec.length_x_m * 0.5;
  const double y_min = spec.center_y_m - spec.width_y_m * 0.5;
  const double y_max = spec.center_y_m + spec.width_y_m * 0.5;
  const double lane_spacing = spec.lane_count == 1 ? 0.0 :
    spec.length_x_m / static_cast<double>(spec.lane_count - 1);

  for (int lane = 0; lane < spec.lane_count; ++lane) {
    const double x = x_min + lane_spacing * static_cast<double>(lane);
    if (lane % 2 == 0) {
      route.push_back({x, y_min, spec.altitude_m});
      route.push_back({x, y_max, spec.altitude_m});
    } else {
      route.push_back({x, y_max, spec.altitude_m});
      route.push_back({x, y_min, spec.altitude_m});
    }
  }
  return route;
}

}  // namespace cuadc_recon
