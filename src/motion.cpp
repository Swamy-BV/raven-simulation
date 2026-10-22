// SPDX-License-Identifier: AGPL-3.0-only
#include "motion.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace raven::simulation {
Motion::Motion(std::vector<MotionPoint> points, const bool repeat)
    : points_(std::move(points)), repeat_(repeat) {
  if (points_.size() < 2 || points_.front().seconds != 0) {
    throw std::runtime_error("Motion needs a zero-based trajectory");
  }
  double previous = -1;
  for (const auto& point : points_) {
    if (!std::isfinite(point.seconds) || point.seconds <= previous ||
        !std::isfinite(point.yaw) ||
        !std::all_of(point.position.begin(), point.position.end(),
                     [](double value) { return std::isfinite(value); })) {
      throw std::runtime_error("Invalid fixture trajectory");
    }
    previous = point.seconds;
  }
}

MotionPoint Motion::At(double elapsed_seconds) const {
  if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0) {
    throw std::runtime_error("Invalid fixture elapsed time");
  }
  if (!repeat_ && elapsed_seconds >= points_.back().seconds) {
    return points_.back();
  }
  const double time = repeat_
                          ? std::fmod(elapsed_seconds, points_.back().seconds)
                          : elapsed_seconds;
  const auto upper =
      std::upper_bound(points_.begin(), points_.end(), time,
                       [](double value, const MotionPoint& point) {
                         return value < point.seconds;
                       });
  if (upper == points_.begin() || upper == points_.end()) {
    return points_.front();
  }
  const auto& lower = *(upper - 1);
  const double fraction =
      (time - lower.seconds) / (upper->seconds - lower.seconds);
  MotionPoint result{time, {}, lower.yaw + fraction * (upper->yaw - lower.yaw)};
  for (std::size_t axis = 0; axis < result.position.size(); ++axis) {
    result.position[axis] =
        lower.position[axis] +
        fraction * (upper->position[axis] - lower.position[axis]);
  }
  return result;
}
}  // namespace raven::simulation
