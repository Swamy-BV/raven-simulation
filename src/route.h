// SPDX-License-Identifier: AGPL-3.0-only
#pragma once
#include <array>
#include <json.hpp>
#include <vector>

namespace raven::simulation {
using RoadPoint = std::array<double, 2>;
std::vector<RoadPoint> SegmentRoute(const nlohmann::json& route);
std::vector<RoadPoint> OffsetPath(const std::vector<RoadPoint>& points,
                                  double offset);
}  // namespace raven::simulation
