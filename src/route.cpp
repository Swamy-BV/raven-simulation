// SPDX-License-Identifier: AGPL-3.0-only
#include "route.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace raven::simulation {
namespace {
constexpr double kPi = 3.14159265358979323846;

double Distance(const RoadPoint& a, const RoadPoint& b) {
  return std::hypot(a[0] - b[0], a[1] - b[1]);
}

RoadPoint Unit(const RoadPoint& a, const RoadPoint& b) {
  const double length = Distance(a, b);
  if (length < 1e-6) {
    throw std::runtime_error("Repeated route point");
  }
  return {(b[0] - a[0]) / length, (b[1] - a[1]) / length};
}
}  // namespace

std::vector<RoadPoint> SegmentRoute(const nlohmann::json& route) {
  const auto first = route.at("start_xy_m").get<RoadPoint>();
  const double initial_heading =
      route.at("start_heading_deg").get<double>() * kPi / 180;
  double heading = initial_heading;
  std::vector<RoadPoint> points{first};
  for (const auto& segment : route.at("segments")) {
    const auto start = points.back();
    const auto type = segment.at("type").get<std::string>();
    if (type == "line") {
      const double length = segment.at("length_m").get<double>();
      if (!std::isfinite(length) || length <= 0 || length > 3000) {
        throw std::runtime_error("Invalid road line");
      }
      const int count = static_cast<int>(std::ceil(length));
      for (int i = 1; i <= count; ++i) {
        const double distance = length * i / count;
        points.push_back({start[0] + distance * std::cos(heading),
                          start[1] + distance * std::sin(heading)});
      }
    } else if (type == "arc") {
      const double radius = segment.at("radius_m").get<double>();
      const double turn = segment.at("turn_deg").get<double>() * kPi / 180;
      if (!std::isfinite(radius) || radius < 10 || radius > 1000 ||
          !std::isfinite(turn) || std::abs(turn) < .001 ||
          std::abs(turn) > 2 * kPi) {
        throw std::runtime_error("Invalid road arc");
      }
      const double direction = turn > 0 ? 1 : -1;
      const int count = static_cast<int>(std::ceil(radius * std::abs(turn)));
      for (int i = 1; i <= count; ++i) {
        const double theta = heading + turn * i / count;
        points.push_back(
            {start[0] +
                 radius / direction * (std::sin(theta) - std::sin(heading)),
             start[1] +
                 radius / direction * (std::cos(heading) - std::cos(theta))});
      }
      heading += turn;
    } else if (type == "bezier") {
      const auto a = segment.at("control1_xy_m").get<RoadPoint>();
      const auto b = segment.at("control2_xy_m").get<RoadPoint>();
      const auto end = segment.at("end_xy_m").get<RoadPoint>();
      const double entry = std::atan2(a[1] - start[1], a[0] - start[0]);
      if (std::abs(std::remainder(entry - heading, 2 * kPi)) > .001) {
        throw std::runtime_error("Road entry is not tangent");
      }
      const double length =
          Distance(start, a) + Distance(a, b) + Distance(b, end);
      if (!std::isfinite(length) || length < 1 || length > 3000) {
        throw std::runtime_error("Invalid road Bezier");
      }
      const int count = static_cast<int>(std::ceil(length));
      for (int i = 1; i <= count; ++i) {
        const double t = static_cast<double>(i) / count, u = 1 - t;
        points.push_back({u * u * u * start[0] + 3 * u * u * t * a[0] +
                              3 * u * t * t * b[0] + t * t * t * end[0],
                          u * u * u * start[1] + 3 * u * u * t * a[1] +
                              3 * u * t * t * b[1] + t * t * t * end[1]});
      }
      if (Distance(b, end) < 1) {
        throw std::runtime_error("Road exit needs a tangent");
      }
      heading = std::atan2(end[1] - b[1], end[0] - b[0]);
    } else {
      throw std::runtime_error("Unsupported road segment");
    }
    if (points.size() > 20000) {
      throw std::runtime_error("Road exceeds geometry budget");
    }
  }
  if (Distance(points.back(), first) > .01 ||
      std::abs(std::remainder(heading - initial_heading, 2 * kPi)) > .001) {
    throw std::runtime_error("Road must close with a continuous tangent");
  }
  for (const auto& point : points) {
    if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
        std::abs(point[0]) > 10000 || std::abs(point[1]) > 10000) {
      throw std::runtime_error("Invalid road coordinate");
    }
  }
  points.back() = first;
  return points;
}

std::vector<RoadPoint> OffsetPath(const std::vector<RoadPoint>& points,
                                  const double offset) {
  if (!std::isfinite(offset) || std::abs(offset) > 32 || points.size() < 4) {
    throw std::runtime_error("Invalid lane offset");
  }
  auto result = points;
  const std::size_t count = points.size() - 1;
  for (std::size_t i = 0; i < count; ++i) {
    const auto a = Unit(points[(i + count - 1) % count], points[i]);
    const auto b = Unit(points[i], points[(i + 1) % count]);
    const double nx = a[1] + b[1], ny = -a[0] - b[0],
                 length = std::hypot(nx, ny);
    if (length < .1) {
      throw std::runtime_error("Road reverses direction");
    }
    const double factor = nx / length * a[1] - ny / length * a[0];
    if (factor < .2) {
      throw std::runtime_error("Road corner is too sharp");
    }
    result[i] = {points[i][0] + offset * nx / length / factor,
                 points[i][1] + offset * ny / length / factor};
  }
  result.back() = result.front();
  return result;
}

std::vector<RoadPoint> ChangeLanes(const std::vector<RoadPoint>& points,
                                   const nlohmann::json& changes) {
  if (!changes.is_array() || changes.size() > 20) {
    throw std::runtime_error("Invalid lane-change schedule");
  }
  double previous_end = 0;
  for (const auto& change : changes) {
    const double start = change.at("start_m").get<double>();
    const double end = change.at("end_m").get<double>();
    const double offset = change.at("offset_m").get<double>();
    if (!std::isfinite(start) || !std::isfinite(end) ||
        !std::isfinite(offset) || start < previous_end || end - start < 20 ||
        std::abs(offset) > 5) {
      throw std::runtime_error(
          "Lane changes need ordered distances and bounded offsets");
    }
    previous_end = end;
  }
  if (changes.empty()) {
    return points;
  }
  if (std::abs(changes.back().at("offset_m").get<double>()) > 1e-6) {
    throw std::runtime_error("Repeated route must return to its original lane");
  }
  const auto right = OffsetPath(points, 1);
  auto result = points;
  double distance = 0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      distance += Distance(points[i - 1], points[i]);
    }
    double offset = 0;
    for (const auto& change : changes) {
      const double start = change.at("start_m").get<double>();
      const double end = change.at("end_m").get<double>();
      const double next = change.at("offset_m").get<double>();
      if (distance < start) {
        break;
      }
      if (distance >= end) {
        offset = next;
        continue;
      }
      const double t = (distance - start) / (end - start);
      const double blend = t * t * t * (10 + t * (-15 + 6 * t));
      offset += blend * (next - offset);
      break;
    }
    for (std::size_t axis = 0; axis < 2; ++axis) {
      result[i][axis] += offset * (right[i][axis] - points[i][axis]);
    }
  }
  if (previous_end > distance) {
    throw std::runtime_error("Lane change extends beyond route");
  }
  return result;
}

void StopAfterDistance(std::vector<RoadPoint>& points, const double distance) {
  if (!std::isfinite(distance) || distance < 10) {
    throw std::runtime_error("Invalid stopping distance");
  }
  double travelled = 0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double step = Distance(points[i - 1], points[i]);
    if (travelled + step >= distance) {
      const double fraction = (distance - travelled) / step;
      for (std::size_t axis = 0; axis < 2; ++axis) {
        points[i][axis] = points[i - 1][axis] +
                          fraction * (points[i][axis] - points[i - 1][axis]);
      }
      points.resize(i + 1);
      return;
    }
    travelled += step;
  }
  throw std::runtime_error("Stopping distance extends beyond trajectory");
}
}  // namespace raven::simulation
