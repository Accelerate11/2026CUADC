#pragma once

#include <vector>

#include <cuadc_core/geometry.hpp>

namespace cuadc_recon
{

struct RouteSpec
{
  double center_x_m{0.0};
  double center_y_m{0.0};
  double length_x_m{0.0};
  double width_y_m{0.0};
  double altitude_m{0.0};
  int lane_count{0};
};

// Returns exactly two endpoints per lane in a continuous serpentine order.
// The route planner deliberately contains no image input, target detector, or
// ground-truth interface. Recon evidence recording is a separate passive class.
std::vector<cuadc_core::Point3> generate_waypoint_route(const RouteSpec & spec);

}  // namespace cuadc_recon
